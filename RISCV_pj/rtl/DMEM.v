module DMEM #(parameter DEPTH = 1024) (
  input         CLK,
  input         WE,
  input  [31:0] A,
  input  [31:0] WD,
  output [31:0] RD,

  input         DBG_EN,
  input         DBG_WE,
  input  [31:0] DBG_A,
  input  [31:0] DBG_WD,
  output [31:0] DBG_RD
);
  reg [31:0] RAM [0:DEPTH-1];

  assign RD = RAM[A[31:2]];

  assign DBG_RD = RAM[DBG_A[31:2]];

  always @(posedge CLK) begin
    if (DBG_EN && DBG_WE)
      RAM[DBG_A[31:2]] <= DBG_WD;
    else if (WE)
      RAM[A[31:2]] <= WD;
  end
endmodule
