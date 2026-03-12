module DEBOUNCER (/*AUTOARG*/
   // Outputs
   DOUT_H,
   // Inputs
   CLK, RST_N, DIN
   );
   input CLK, RST_N;
   input DIN;

   output reg DOUT_H;

   parameter  ACTIVE_LOW = 1;
   parameter  CNTR_BITS = 20;

   wire	      DIN_H = ACTIVE_LOW ? ~DIN : DIN;
   reg	      S0, S1, STABLE;
   reg [CNTR_BITS-1:0] CNT;

   always @(posedge CLK or negedge RST_N) begin
      if(!RST_N) begin
	 S0 <= 1'b0; S1 <= 1'b0; STABLE <= 1'b0; 
	 CNT <= {CNTR_BITS{1'b0}}; DOUT_H <= 1'b0;
      end 
      else begin
	 S0 <= DIN_H; S1 <= S0;
	 if (S1 == STABLE) begin
            CNT <= {CNTR_BITS{1'b0}};
	 end 
	 else begin
            CNT <= CNT + 1'b1;
            if(&CNT) begin
               STABLE <= S1;
               CNT    <= {CNTR_BITS{1'b0}};
            end
	 end
	 DOUT_H <= STABLE;
      end
   end // always @ (posedge CLK or negedge RST_N)
   
endmodule // DEBOUNCER

