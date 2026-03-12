module EXTEND (INSTRD, IMMSRCD, IMMEXTD);
  input  [31:7] INSTRD;
  input  [1:0]  IMMSRCD;
  output [31:0] IMMEXTD;

  reg [31:0] IMMEXTD;

  always @(*) begin
    case (IMMSRCD)
      // I-type 
      2'b00: IMMEXTD = {{20{INSTRD[31]}}, INSTRD[31:20]};
      // S-type (stores)
      2'b01: IMMEXTD = {{20{INSTRD[31]}}, INSTRD[31:25], INSTRD[11:7]};
      // B-type (branches)
      2'b10: IMMEXTD = {{20{INSTRD[31]}}, INSTRD[7], INSTRD[30:25], INSTRD[11:8], 1'b0};
      // J-type (jal)
      2'b11: IMMEXTD = {{12{INSTRD[31]}}, INSTRD[19:12], INSTRD[20], INSTRD[30:21], 1'b0};
      default: IMMEXTD = 32'bx;
    endcase
  end
endmodule // EXTEND
