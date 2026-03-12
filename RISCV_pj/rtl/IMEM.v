module IMEM (
  input  [31:0] A,
  output [31:0] RD,

  input         DBG_EN,
  input  [31:0] DBG_A,
  output [31:0] DBG_RD
);
  reg [31:0] RAM [0:191];

  initial begin 
    $readmemh("/home/dice14/PJ22/ww13/rtl/gemm8x8.hex", RAM);
  end

  assign RD = RAM[A[31:2]];

  assign DBG_RD = RAM[DBG_A[31:2]];

endmodule // IMEM
