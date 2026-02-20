module alu_control_unit (
  input  wire [3:0] AluOp,
  input  wire [2:0] Func3,
  input  wire [6:0] Func7,
  input  wire [6:0] opcode,
  output wire [2:0] AluControl_opsel,
  output wire       AluControl_sub,
  output wire       AluControl_unsigned,
  output wire       AluControl_arith
);

endmodule