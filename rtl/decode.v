module decode (
      parameter BYPASS_EN = 0   // passed down to rf
    ) (
      input  wire        i_clk,
      input  wire        i_rst,
      input  wire [31:0] Inst,
    
      // writeback feed-in (data only)
      // NOTICE!! The writedata in pipeline should be 3 stages later than the readdata! 
      input  wire [31:0] WriteData,
   
      // controls out
      output wire        lui,
      output wire        PcSrc,
      output wire [3:0]  AluOp,
      output wire        MemWrite,
      output wire        MemRead,
      output wire        MemToReg,
      output wire        AluSrc1,
      output wire        AluSrc2,
      output wire        RegWrite,
      output wire        Jump,
      output wire        Branch,

      // immediate out
      output wire [31:0] Offset,
    
      // retire (and also rf addressing) out
      output wire [4:0]  o_retire_rs1_raddr,
      output wire [4:0]  o_retire_rs2_raddr,
      output wire [4:0]  o_retire_rd_waddr,
      output wire [31:0] o_retire_rs1_rdata,
      output wire [31:0] o_retire_rs2_rdata,
    
      // exceptions out
      output wire        IllegalInst,
      output wire        EBreak
    );

    wire        Rs1Used;
    wire        Rs2Used;
    wire [4:0]  rs1_raddr_raw;
    wire [4:0]  rs2_raddr_raw;
    wire [4:0]  rd_waddr_raw;

    // TODO: Control, RF and Immediate



endmodule