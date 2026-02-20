module memory (
    input  wire [31:0] EffAddr,
    input  wire [31:0] StoreData,
    input  wire [2:0]  Func3,
    input  wire        MemRead,
    input  wire        MemWrite,
    input  wire [31:0] i_dmem_rdata,

    output wire [31:0] o_dmem_addr,
    output wire [3:0]  o_dmem_mask,
    output wire [31:0] o_dmem_wdata,
    output wire        o_dmem_ren,
    output wire        o_dmem_wen,

    output wire [31:0] LoadData,
    output wire        MisalignTrap
);



endmodule