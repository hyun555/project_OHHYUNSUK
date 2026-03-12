module REGFILE(/*AUTOARG*/
   // Outputs
   RD1, RD2,
   // Inputs
   CLK, WE3, A1, A2, A3, WD3
   );
   input CLK; // Clock
   input WE3; // Write Enable
   input [4:0] A1, A2, A3; // Read/Read/Write Addresses
   input [31:0]	WD3; // Write Data

   output [31:0] RD1, RD2; // Read Data

   reg [31:0]	 RF [0:31]; // Storage Array: 32 Registers x 32 Bits

   assign RD1 = (A1 != 5'd0) ? RF[A1] : 32'd0;
   assign RD2 = (A2 != 5'd0) ? RF[A2] : 32'd0;

   always @(negedge CLK) begin
      if(WE3) begin
	 RF[A3] <= WD3;
      end
   end

endmodule // REGFILE
