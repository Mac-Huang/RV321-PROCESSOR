module writeback (
  input  wire [31:0] AluResult,
  input  wire [31:0] LoadData,
  input  wire [31:0] pc_plus4,
  input  wire [31:0] Offset,
  input  wire        MemToReg,
  input  wire        lui,
  input  wire        Jump,
  output wire [31:0] WriteData
);

endmodule