#include "kernel/yosys.h"
#include "kernel/sigtools.h"


USING_YOSYS_NAMESPACE
PRIVATE_NAMESPACE_BEGIN

bool is_unary_ops(RTLIL::Cell *cell){
  std::vector<RTLIL::IdString> unary_ops = {
    ID($not), ID($pos), ID($neg),
    ID($reduce_and), ID($reduce_or), ID($reduce_xor), ID($reduce_xnor), ID($reduce_bool),
    ID($logic_not), ID($slice), ID($lut), ID($sop)
  };

  for (auto type : unary_ops){
    if (cell->type == type)
      return true;
  }

  return false;
}

bool is_binary_ops(RTLIL::Cell *cell){
  std::vector<RTLIL::IdString> binary_ops = {
      ID($and), ID($or), ID($xor), ID($xnor),
      ID($shl), ID($shr), ID($sshl), ID($sshr), ID($shift), ID($shiftx),
      ID($lt), ID($le), ID($eq), ID($ne), ID($eqx), ID($nex), ID($ge), ID($gt),
      ID($add), ID($sub), ID($mul), ID($div), ID($mod), ID($divfloor), ID($modfloor), ID($pow),
      ID($logic_and), ID($logic_or), ID($concat), ID($macc)
    };

  for (auto type : binary_ops){
    if (cell->type == type)
      return true;
  }

  return false;

}

bool is_mux(RTLIL::Cell *cell){
  if (cell->type == ID($mux) or cell->type == ID($pmux))
    return true;

  return false;
}

bool is_reg(RTLIL::Cell *cell){
  std::vector<RTLIL::IdString> ff = {
    ID($sr), ID($ff), ID($dff), ID($dffe), ID($dffsr), ID($dffsre), ID($adff), ID($adffe),
    ID($aldff), ID($aldffe), ID($sdff), ID($sdffe), ID($sdffce), ID(dlatch), ID($adlatch),
    ID($dlatchsr) 
  };

  for (auto type : ff){
    if (cell->type == type)
      return true;
  }

  return false;
}

bool is_mem(RTLIL::Cell *cell){
  std::vector<RTLIL::IdString> dmem = {
    ID($memrd), ID($memrd_v2), ID($memwr), ID($memwr_v2),
    ID($meminit), ID($meminit_v2), ID($mem), ID($mem_v2)
  };

  for(auto type : dmem){
    if (cell->type == type)
      return true;
  }

  return false;
}

void print_ports(RTLIL::Cell *cell){
  //count how many times a single-bit signal is used
  std::map<RTLIL::SigBit, int> bit_usage_count;

  //print out the port information of unary ops
  if (is_unary_ops(cell) || is_binary_ops(cell)){
    RTLIL::SigSpec sig = cell->getPort(ID::A);
    log("the witdth of the cell %s is %d\n", log_id(cell), GetSize(sig));
  }
}

int get_width(RTLIL::Cell *cell){
  if (is_unary_ops(cell) || is_binary_ops(cell)){
    return GetSize(cell->getPort(ID::A));
  }
  if (is_reg(cell)){
    return GetSize(cell->getPort(ID::Q));
  }
  if (is_mux(cell)){
    return GetSize(cell->getPort(ID::Y));
  }
  if (is_mem(cell)){

    log("the cell name is %s, the addr width is %d and the data width is %d\n", log_id(cell),
      GetSize(cell->getPort(ID::ADDR)), GetSize(cell->getPort(ID::DATA)));
    return GetSize(cell->getPort(ID::ADDR)) * GetSize(cell->getPort(ID::DATA));
  }

  return 0;
}

struct PrintNumPass : public Pass {
  PrintNumPass() : Pass("print_num"){ }
  void execute(std::vector<string>, RTLIL::Design *design) override{
    int num_binary_ops = 0;
    int num_unary_ops = 0;
    int num_mux = 0;
    int num_reg = 0;
    int num_mem = 0;
    int total_cell = 0;

    for (auto module : design->modules()){
      log("Name of this module: %s\n", log_id(module));
      for (auto cell : module->cells()){
        total_cell ++;
        log("Cell type is %s\n", log_id(cell->type));
        if (is_binary_ops(cell))
          num_binary_ops += get_width(cell);
        else if(is_unary_ops(cell))
          num_unary_ops += get_width(cell);
        else if(is_mux(cell))
          num_mux += get_width(cell);
        else if(is_reg(cell))
          num_reg += get_width(cell);
        else if(is_mem(cell))
          num_mem += get_width(cell);
        print_ports(cell);
      }
    }

    log("The total number of cells is %d\n", total_cell);
    log("The number of binary_ops is %d\n", num_binary_ops);
    log("The number of unary_ops is %d\n", num_unary_ops);
    log("The number of registers is %d\n", num_reg);
    log("The number of muxes is %d\n", num_mux);
    log("The numeber of memories is %d\n", num_mem);
  }
} PrintNumPass;

struct CellType
{
  RTLIL::IdString type;
  pool<RTLIL::IdString> inputs, outputs;
  bool is_evaluable;
};

struct CellTypes
{
  dict<RTLIL::IdString, CellType> cell_types;

  CellTypes()
  {
  }

  CellTypes(RTLIL::Design *design)
  {
    setup(design);
  }

  void setup(RTLIL::Design *design = NULL)
  {
    if (design)
      setup_design(design);

    setup_internals();
    setup_internals_mem();
    setup_stdcells();
    setup_stdcells_mem();
  }

  void setup_type(RTLIL::IdString type, const pool<RTLIL::IdString> &inputs, const pool<RTLIL::IdString> &outputs, bool is_evaluable = false)
  {
    CellType ct = {type, inputs, outputs, is_evaluable};
    cell_types[ct.type] = ct;
  }

  void setup_module(RTLIL::Module *module)
  {
    pool<RTLIL::IdString> inputs, outputs;
    for (RTLIL::IdString wire_name : module->ports) {
      RTLIL::Wire *wire = module->wire(wire_name);
      if (wire->port_input)
        inputs.insert(wire->name);
      if (wire->port_output)
        outputs.insert(wire->name);
    }
    setup_type(module->name, inputs, outputs);
  }

  void setup_design(RTLIL::Design *design)
  {
    for (auto module : design->modules())
      setup_module(module);
  }

  void setup_internals()
  {
    setup_internals_eval();

    setup_type(ID($tribuf), {ID::A, ID::EN}, {ID::Y}, true);

    setup_type(ID($assert), {ID::A, ID::EN}, pool<RTLIL::IdString>(), true);
    setup_type(ID($assume), {ID::A, ID::EN}, pool<RTLIL::IdString>(), true);
    setup_type(ID($live), {ID::A, ID::EN}, pool<RTLIL::IdString>(), true);
    setup_type(ID($fair), {ID::A, ID::EN}, pool<RTLIL::IdString>(), true);
    setup_type(ID($cover), {ID::A, ID::EN}, pool<RTLIL::IdString>(), true);
    setup_type(ID($initstate), pool<RTLIL::IdString>(), {ID::Y}, true);
    setup_type(ID($anyconst), pool<RTLIL::IdString>(), {ID::Y}, true);
    setup_type(ID($anyseq), pool<RTLIL::IdString>(), {ID::Y}, true);
    setup_type(ID($allconst), pool<RTLIL::IdString>(), {ID::Y}, true);
    setup_type(ID($allseq), pool<RTLIL::IdString>(), {ID::Y}, true);
    setup_type(ID($equiv), {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($specify2), {ID::EN, ID::SRC, ID::DST}, pool<RTLIL::IdString>(), true);
    setup_type(ID($specify3), {ID::EN, ID::SRC, ID::DST, ID::DAT}, pool<RTLIL::IdString>(), true);
    setup_type(ID($specrule), {ID::EN_SRC, ID::EN_DST, ID::SRC, ID::DST}, pool<RTLIL::IdString>(), true);
  }

  void setup_internals_eval()
  {
    std::vector<RTLIL::IdString> unary_ops = {
      ID($not), ID($pos), ID($neg),
      ID($reduce_and), ID($reduce_or), ID($reduce_xor), ID($reduce_xnor), ID($reduce_bool),
      ID($logic_not), ID($slice), ID($lut), ID($sop)
    };

    std::vector<RTLIL::IdString> binary_ops = {
      ID($and), ID($or), ID($xor), ID($xnor),
      ID($shl), ID($shr), ID($sshl), ID($sshr), ID($shift), ID($shiftx),
      ID($lt), ID($le), ID($eq), ID($ne), ID($eqx), ID($nex), ID($ge), ID($gt),
      ID($add), ID($sub), ID($mul), ID($div), ID($mod), ID($divfloor), ID($modfloor), ID($pow),
      ID($logic_and), ID($logic_or), ID($concat), ID($macc)
    };

    for (auto type : unary_ops)
      setup_type(type, {ID::A}, {ID::Y}, true);

    for (auto type : binary_ops)
      setup_type(type, {ID::A, ID::B}, {ID::Y}, true);

    for (auto type : std::vector<RTLIL::IdString>({ID($mux), ID($pmux)}))
      setup_type(type, {ID::A, ID::B, ID::S}, {ID::Y}, true);

    setup_type(ID($lcu), {ID::P, ID::G, ID::CI}, {ID::CO}, true);
    setup_type(ID($alu), {ID::A, ID::B, ID::CI, ID::BI}, {ID::X, ID::Y, ID::CO}, true);
    setup_type(ID($fa), {ID::A, ID::B, ID::C}, {ID::X, ID::Y}, true);
  }

  void setup_internals_ff()
  {
    setup_type(ID($sr), {ID::SET, ID::CLR}, {ID::Q});
    setup_type(ID($ff), {ID::D}, {ID::Q});
    setup_type(ID($dff), {ID::CLK, ID::D}, {ID::Q});
    setup_type(ID($dffe), {ID::CLK, ID::EN, ID::D}, {ID::Q});
    setup_type(ID($dffsr), {ID::CLK, ID::SET, ID::CLR, ID::D}, {ID::Q});
    setup_type(ID($dffsre), {ID::CLK, ID::SET, ID::CLR, ID::D, ID::EN}, {ID::Q});
    setup_type(ID($adff), {ID::CLK, ID::ARST, ID::D}, {ID::Q});
    setup_type(ID($adffe), {ID::CLK, ID::ARST, ID::D, ID::EN}, {ID::Q});
    setup_type(ID($aldff), {ID::CLK, ID::ALOAD, ID::AD, ID::D}, {ID::Q});
    setup_type(ID($aldffe), {ID::CLK, ID::ALOAD, ID::AD, ID::D, ID::EN}, {ID::Q});
    setup_type(ID($sdff), {ID::CLK, ID::SRST, ID::D}, {ID::Q});
    setup_type(ID($sdffe), {ID::CLK, ID::SRST, ID::D, ID::EN}, {ID::Q});
    setup_type(ID($sdffce), {ID::CLK, ID::SRST, ID::D, ID::EN}, {ID::Q});
    setup_type(ID($dlatch), {ID::EN, ID::D}, {ID::Q});
    setup_type(ID($adlatch), {ID::EN, ID::D, ID::ARST}, {ID::Q});
    setup_type(ID($dlatchsr), {ID::EN, ID::SET, ID::CLR, ID::D}, {ID::Q});
  }

  void setup_internals_mem()
  {
    setup_internals_ff();

    setup_type(ID($memrd), {ID::CLK, ID::EN, ID::ADDR}, {ID::DATA});
    setup_type(ID($memrd_v2), {ID::CLK, ID::EN, ID::ARST, ID::SRST, ID::ADDR}, {ID::DATA});
    setup_type(ID($memwr), {ID::CLK, ID::EN, ID::ADDR, ID::DATA}, pool<RTLIL::IdString>());
    setup_type(ID($memwr_v2), {ID::CLK, ID::EN, ID::ADDR, ID::DATA}, pool<RTLIL::IdString>());
    setup_type(ID($meminit), {ID::ADDR, ID::DATA}, pool<RTLIL::IdString>());
    setup_type(ID($meminit_v2), {ID::ADDR, ID::DATA, ID::EN}, pool<RTLIL::IdString>());
    setup_type(ID($mem), {ID::RD_CLK, ID::RD_EN, ID::RD_ADDR, ID::WR_CLK, ID::WR_EN, ID::WR_ADDR, ID::WR_DATA}, {ID::RD_DATA});
    setup_type(ID($mem_v2), {ID::RD_CLK, ID::RD_EN, ID::RD_ARST, ID::RD_SRST, ID::RD_ADDR, ID::WR_CLK, ID::WR_EN, ID::WR_ADDR, ID::WR_DATA}, {ID::RD_DATA});

    setup_type(ID($fsm), {ID::CLK, ID::ARST, ID::CTRL_IN}, {ID::CTRL_OUT});
  }

  void setup_stdcells()
  {
    setup_stdcells_eval();

    setup_type(ID($_TBUF_), {ID::A, ID::E}, {ID::Y}, true);
  }

  void setup_stdcells_eval()
  {
    setup_type(ID($_BUF_), {ID::A}, {ID::Y}, true);
    setup_type(ID($_NOT_), {ID::A}, {ID::Y}, true);
    setup_type(ID($_AND_), {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_NAND_), {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_OR_),  {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_NOR_),  {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_XOR_), {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_XNOR_), {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_ANDNOT_), {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_ORNOT_), {ID::A, ID::B}, {ID::Y}, true);
    setup_type(ID($_MUX_), {ID::A, ID::B, ID::S}, {ID::Y}, true);
    setup_type(ID($_NMUX_), {ID::A, ID::B, ID::S}, {ID::Y}, true);
    setup_type(ID($_MUX4_), {ID::A, ID::B, ID::C, ID::D, ID::S, ID::T}, {ID::Y}, true);
    setup_type(ID($_MUX8_), {ID::A, ID::B, ID::C, ID::D, ID::E, ID::F, ID::G, ID::H, ID::S, ID::T, ID::U}, {ID::Y}, true);
    setup_type(ID($_MUX16_), {ID::A, ID::B, ID::C, ID::D, ID::E, ID::F, ID::G, ID::H, ID::I, ID::J, ID::K, ID::L, ID::M, ID::N, ID::O, ID::P, ID::S, ID::T, ID::U, ID::V}, {ID::Y}, true);
    setup_type(ID($_AOI3_), {ID::A, ID::B, ID::C}, {ID::Y}, true);
    setup_type(ID($_OAI3_), {ID::A, ID::B, ID::C}, {ID::Y}, true);
    setup_type(ID($_AOI4_), {ID::A, ID::B, ID::C, ID::D}, {ID::Y}, true);
    setup_type(ID($_OAI4_), {ID::A, ID::B, ID::C, ID::D}, {ID::Y}, true);
  }

  void setup_stdcells_mem()
  {
    std::vector<char> list_np = {'N', 'P'}, list_01 = {'0', '1'};

    for (auto c1 : list_np)
    for (auto c2 : list_np)
      setup_type(stringf("$_SR_%c%c_", c1, c2), {ID::S, ID::R}, {ID::Q});

    setup_type(ID($_FF_), {ID::D}, {ID::Q});

    for (auto c1 : list_np)
      setup_type(stringf("$_DFF_%c_", c1), {ID::C, ID::D}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
      setup_type(stringf("$_DFFE_%c%c_", c1, c2), {ID::C, ID::D, ID::E}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_01)
      setup_type(stringf("$_DFF_%c%c%c_", c1, c2, c3), {ID::C, ID::R, ID::D}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_01)
    for (auto c4 : list_np)
      setup_type(stringf("$_DFFE_%c%c%c%c_", c1, c2, c3, c4), {ID::C, ID::R, ID::D, ID::E}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
      setup_type(stringf("$_ALDFF_%c%c_", c1, c2), {ID::C, ID::L, ID::AD, ID::D}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_np)
      setup_type(stringf("$_ALDFFE_%c%c%c_", c1, c2, c3), {ID::C, ID::L, ID::AD, ID::D, ID::E}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_np)
      setup_type(stringf("$_DFFSR_%c%c%c_", c1, c2, c3), {ID::C, ID::S, ID::R, ID::D}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_np)
    for (auto c4 : list_np)
      setup_type(stringf("$_DFFSRE_%c%c%c%c_", c1, c2, c3, c4), {ID::C, ID::S, ID::R, ID::D, ID::E}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_01)
      setup_type(stringf("$_SDFF_%c%c%c_", c1, c2, c3), {ID::C, ID::R, ID::D}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_01)
    for (auto c4 : list_np)
      setup_type(stringf("$_SDFFE_%c%c%c%c_", c1, c2, c3, c4), {ID::C, ID::R, ID::D, ID::E}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_01)
    for (auto c4 : list_np)
      setup_type(stringf("$_SDFFCE_%c%c%c%c_", c1, c2, c3, c4), {ID::C, ID::R, ID::D, ID::E}, {ID::Q});

    for (auto c1 : list_np)
      setup_type(stringf("$_DLATCH_%c_", c1), {ID::E, ID::D}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_01)
      setup_type(stringf("$_DLATCH_%c%c%c_", c1, c2, c3), {ID::E, ID::R, ID::D}, {ID::Q});

    for (auto c1 : list_np)
    for (auto c2 : list_np)
    for (auto c3 : list_np)
      setup_type(stringf("$_DLATCHSR_%c%c%c_", c1, c2, c3), {ID::E, ID::S, ID::R, ID::D}, {ID::Q});
  }

  void clear()
  {
    cell_types.clear();
  }

  bool cell_known(RTLIL::IdString type) const
  {
    return cell_types.count(type) != 0;
  }

  bool cell_output(RTLIL::IdString type, RTLIL::IdString port) const
  {
    auto it = cell_types.find(type);
    return it != cell_types.end() && it->second.outputs.count(port) != 0;
  }

  bool cell_input(RTLIL::IdString type, RTLIL::IdString port) const
  {
    auto it = cell_types.find(type);
    return it != cell_types.end() && it->second.inputs.count(port) != 0;
  }

  bool cell_evaluable(RTLIL::IdString type) const
  {
    auto it = cell_types.find(type);
    return it != cell_types.end() && it->second.is_evaluable;
  }

  static RTLIL::Const eval_not(RTLIL::Const v)
  {
    for (auto &bit : v.bits)
      if (bit == State::S0) bit = State::S1;
      else if (bit == State::S1) bit = State::S0;
    return v;
  }

  static RTLIL::Const eval(RTLIL::IdString type, const RTLIL::Const &arg1, const RTLIL::Const &arg2, bool signed1, bool signed2, int result_len, bool *errp = nullptr)
  {
    if (type == ID($sshr) && !signed1)
      type = ID($shr);
    if (type == ID($sshl) && !signed1)
      type = ID($shl);

    if (type != ID($sshr) && type != ID($sshl) && type != ID($shr) && type != ID($shl) && type != ID($shift) && type != ID($shiftx) &&
        type != ID($pos) && type != ID($neg) && type != ID($not)) {
      if (!signed1 || !signed2)
        signed1 = false, signed2 = false;
    }

#define HANDLE_CELL_TYPE(_t) if (type == ID($##_t)) return const_ ## _t(arg1, arg2, signed1, signed2, result_len);
    HANDLE_CELL_TYPE(not)
    HANDLE_CELL_TYPE(and)
    HANDLE_CELL_TYPE(or)
    HANDLE_CELL_TYPE(xor)
    HANDLE_CELL_TYPE(xnor)
    HANDLE_CELL_TYPE(reduce_and)
    HANDLE_CELL_TYPE(reduce_or)
    HANDLE_CELL_TYPE(reduce_xor)
    HANDLE_CELL_TYPE(reduce_xnor)
    HANDLE_CELL_TYPE(reduce_bool)
    HANDLE_CELL_TYPE(logic_not)
    HANDLE_CELL_TYPE(logic_and)
    HANDLE_CELL_TYPE(logic_or)
    HANDLE_CELL_TYPE(shl)
    HANDLE_CELL_TYPE(shr)
    HANDLE_CELL_TYPE(sshl)
    HANDLE_CELL_TYPE(sshr)
    HANDLE_CELL_TYPE(shift)
    HANDLE_CELL_TYPE(shiftx)
    HANDLE_CELL_TYPE(lt)
    HANDLE_CELL_TYPE(le)
    HANDLE_CELL_TYPE(eq)
    HANDLE_CELL_TYPE(ne)
    HANDLE_CELL_TYPE(eqx)
    HANDLE_CELL_TYPE(nex)
    HANDLE_CELL_TYPE(ge)
    HANDLE_CELL_TYPE(gt)
    HANDLE_CELL_TYPE(add)
    HANDLE_CELL_TYPE(sub)
    HANDLE_CELL_TYPE(mul)
    HANDLE_CELL_TYPE(div)
    HANDLE_CELL_TYPE(mod)
    HANDLE_CELL_TYPE(divfloor)
    HANDLE_CELL_TYPE(modfloor)
    HANDLE_CELL_TYPE(pow)
    HANDLE_CELL_TYPE(pos)
    HANDLE_CELL_TYPE(neg)
#undef HANDLE_CELL_TYPE

    if (type == ID($_BUF_))
      return arg1;
    if (type == ID($_NOT_))
      return eval_not(arg1);
    if (type == ID($_AND_))
      return const_and(arg1, arg2, false, false, 1);
    if (type == ID($_NAND_))
      return eval_not(const_and(arg1, arg2, false, false, 1));
    if (type == ID($_OR_))
      return const_or(arg1, arg2, false, false, 1);
    if (type == ID($_NOR_))
      return eval_not(const_or(arg1, arg2, false, false, 1));
    if (type == ID($_XOR_))
      return const_xor(arg1, arg2, false, false, 1);
    if (type == ID($_XNOR_))
      return const_xnor(arg1, arg2, false, false, 1);
    if (type == ID($_ANDNOT_))
      return const_and(arg1, eval_not(arg2), false, false, 1);
    if (type == ID($_ORNOT_))
      return const_or(arg1, eval_not(arg2), false, false, 1);

    if (errp != nullptr) {
      *errp = true;
      return State::Sm;
    }

    log_abort();
  }

  static RTLIL::Const eval(RTLIL::Cell *cell, const RTLIL::Const &arg1, const RTLIL::Const &arg2, bool *errp = nullptr)
  {
    if (cell->type == ID($slice)) {
      RTLIL::Const ret;
      int width = cell->parameters.at(ID::Y_WIDTH).as_int();
      int offset = cell->parameters.at(ID::OFFSET).as_int();
      ret.bits.insert(ret.bits.end(), arg1.bits.begin()+offset, arg1.bits.begin()+offset+width);
      return ret;
    }

    if (cell->type == ID($concat)) {
      RTLIL::Const ret = arg1;
      ret.bits.insert(ret.bits.end(), arg2.bits.begin(), arg2.bits.end());
      return ret;
    }

    if (cell->type == ID($lut))
    {
      int width = cell->parameters.at(ID::WIDTH).as_int();

      std::vector<RTLIL::State> t = cell->parameters.at(ID::LUT).bits;
      while (GetSize(t) < (1 << width))
        t.push_back(State::S0);
      t.resize(1 << width);

      for (int i = width-1; i >= 0; i--) {
        RTLIL::State sel = arg1.bits.at(i);
        std::vector<RTLIL::State> new_t;
        if (sel == State::S0)
          new_t = std::vector<RTLIL::State>(t.begin(), t.begin() + GetSize(t)/2);
        else if (sel == State::S1)
          new_t = std::vector<RTLIL::State>(t.begin() + GetSize(t)/2, t.end());
        else
          for (int j = 0; j < GetSize(t)/2; j++)
            new_t.push_back(t[j] == t[j + GetSize(t)/2] ? t[j] : RTLIL::Sx);
        t.swap(new_t);
      }

      log_assert(GetSize(t) == 1);
      return t;
    }

    if (cell->type == ID($sop))
    {
      int width = cell->parameters.at(ID::WIDTH).as_int();
      int depth = cell->parameters.at(ID::DEPTH).as_int();
      std::vector<RTLIL::State> t = cell->parameters.at(ID::TABLE).bits;

      while (GetSize(t) < width*depth*2)
        t.push_back(State::S0);

      RTLIL::State default_ret = State::S0;

      for (int i = 0; i < depth; i++)
      {
        bool match = true;
        bool match_x = true;

        for (int j = 0; j < width; j++) {
          RTLIL::State a = arg1.bits.at(j);
          if (t.at(2*width*i + 2*j + 0) == State::S1) {
            if (a == State::S1) match_x = false;
            if (a != State::S0) match = false;
          }
          if (t.at(2*width*i + 2*j + 1) == State::S1) {
            if (a == State::S0) match_x = false;
            if (a != State::S1) match = false;
          }
        }

        if (match)
          return State::S1;

        if (match_x)
          default_ret = State::Sx;
      }

      return default_ret;
    }

    bool signed_a = cell->parameters.count(ID::A_SIGNED) > 0 && cell->parameters[ID::A_SIGNED].as_bool();
    bool signed_b = cell->parameters.count(ID::B_SIGNED) > 0 && cell->parameters[ID::B_SIGNED].as_bool();
    int result_len = cell->parameters.count(ID::Y_WIDTH) > 0 ? cell->parameters[ID::Y_WIDTH].as_int() : -1;
    return eval(cell->type, arg1, arg2, signed_a, signed_b, result_len, errp);
  }

  static RTLIL::Const eval(RTLIL::Cell *cell, const RTLIL::Const &arg1, const RTLIL::Const &arg2, const RTLIL::Const &arg3, bool *errp = nullptr)
  {
    if (cell->type.in(ID($mux), ID($pmux), ID($_MUX_))) {
      RTLIL::Const ret = arg1;
      for (size_t i = 0; i < arg3.bits.size(); i++)
        if (arg3.bits[i] == RTLIL::State::S1) {
          std::vector<RTLIL::State> bits(arg2.bits.begin() + i*arg1.bits.size(), arg2.bits.begin() + (i+1)*arg1.bits.size());
          ret = RTLIL::Const(bits);
        }
      return ret;
    }

    if (cell->type == ID($_AOI3_))
      return eval_not(const_or(const_and(arg1, arg2, false, false, 1), arg3, false, false, 1));
    if (cell->type == ID($_OAI3_))
      return eval_not(const_and(const_or(arg1, arg2, false, false, 1), arg3, false, false, 1));

    log_assert(arg3.bits.size() == 0);
    return eval(cell, arg1, arg2, errp);
  }

  static RTLIL::Const eval(RTLIL::Cell *cell, const RTLIL::Const &arg1, const RTLIL::Const &arg2, const RTLIL::Const &arg3, const RTLIL::Const &arg4, bool *errp = nullptr)
  {
    if (cell->type == ID($_AOI4_))
      return eval_not(const_or(const_and(arg1, arg2, false, false, 1), const_and(arg3, arg4, false, false, 1), false, false, 1));
    if (cell->type == ID($_OAI4_))
      return eval_not(const_and(const_or(arg1, arg2, false, false, 1), const_or(arg3, arg4, false, false, 1), false, false, 1));

    log_assert(arg4.bits.size() == 0);
    return eval(cell, arg1, arg2, arg3, errp);
  }
};

// initialized by yosys_setup()
extern CellTypes yosys_celltypes;

PRIVATE_NAMESPACE_END
