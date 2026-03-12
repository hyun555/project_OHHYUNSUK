module SEVEN_SEGMENT (/*AUTOARG*/
   // Outputs
   O_SEG,
   // Inputs
   I_BIN
   );
   input [3:0] I_BIN;
   output reg [6:0] O_SEG;

   always @(*) begin
      case (I_BIN)
	4'd0:    O_SEG = 7'b1000000; // 0
        4'd1:    O_SEG = 7'b1111001; // 1
        4'd2:    O_SEG = 7'b0100100; // 2
        4'd3:    O_SEG = 7'b0110000; // 3
        4'd4:    O_SEG = 7'b0011001; // 4
        4'd5:    O_SEG = 7'b0010010; // 5
        4'd6:    O_SEG = 7'b0000010; // 6
        4'd7:    O_SEG = 7'b1111000; // 7
        4'd8:    O_SEG = 7'b0000000; // 8
        4'd9:    O_SEG = 7'b0010000; // 9
        default: O_SEG = 7'b1111111; // OFF
      endcase // case (I_BIN)
   end // always @ (*)
   
endmodule // Seven_Segment
