module UART_RX (/*AUTOARG*/
   // Outputs
   O_DATA_AVAIL, O_DATA_BYTE,
   // Inputs
   CLK, RST_N, I_RX
   );
   input CLK, RST_N;
   input I_RX;
   
   output O_DATA_AVAIL;
   output [7:0]	O_DATA_BYTE;

   parameter	CLKS_PER_BIT = 434;

   localparam	IDLE_STATE = 2'b00;
   localparam	START_STATE = 2'b01;
   localparam	GET_BIT_STATE = 2'b10;
   localparam	STOP_STATE = 2'b11;

   reg		RX_BUFFER;
   reg		RX;

   reg [1:0]	STATE;
   reg [15:0]	COUNTER;
   reg [2:0]	BIT_INDEX;
   reg		DATA_AVAIL;
   reg [7:0]	DATA_BYTE;

   assign O_DATA_AVAIL = DATA_AVAIL;
   assign O_DATA_BYTE = DATA_BYTE;

   always @(posedge CLK, negedge RST_N) begin
      if (!RST_N) begin
	 STATE <= IDLE_STATE;
	 COUNTER <= 16'b0; BIT_INDEX <= 3'b0;
	 DATA_AVAIL <= 1'b0; DATA_BYTE <= 8'b0;
	 RX <= 1'b1; RX_BUFFER <= 1'b1;
      end
      else begin
	 RX_BUFFER <= I_RX;
	 RX <= RX_BUFFER;
	 case (STATE)
	   IDLE_STATE: begin
	      DATA_AVAIL <= 1'b0;
	      COUNTER <= 16'b0; BIT_INDEX <= 3'd0;
	      if (RX == 1'b0) begin
		 STATE <= START_STATE;
	      end
	      else begin
		 STATE <= IDLE_STATE;
	      end
	   end // case: IDLE_STATE

	   START_STATE: begin
	      if (COUNTER == (CLKS_PER_BIT - 1) / 2) begin
		 if (RX == 1'b0) begin
		    COUNTER <= 0;
		    STATE <= GET_BIT_STATE;
		 end
		 else begin
		    STATE <= IDLE_STATE;
		 end
	      end
	      else begin
		 COUNTER <= COUNTER + 16'b1;
		 STATE <= START_STATE;
	      end // else: !if(COUNTER == (CLKS_PER_BIT - 1) / 2)
	   end // case: START_STATE

	   GET_BIT_STATE: begin
	      if (COUNTER < CLKS_PER_BIT - 1) begin
		 COUNTER <= COUNTER + 16'b1;
		 STATE <= GET_BIT_STATE;
	      end
	      else begin
		 COUNTER <= 16'b0;
		 DATA_BYTE[BIT_INDEX] <= RX;
		 if (BIT_INDEX < 3'd7) begin
		    BIT_INDEX <= BIT_INDEX + 3'b1;
		    STATE <= GET_BIT_STATE;
		 end
		 else begin
		    BIT_INDEX <= 0;
		    STATE <= STOP_STATE;
		 end
	      end // else: !if(COUNTER < CLKS_PER_BIT - 1)
	   end // case: GET_BIT_STATE

	   STOP_STATE: begin
	      if (COUNTER < CLKS_PER_BIT - 1) begin
		 COUNTER <= COUNTER + 16'b1;
		 STATE <= STOP_STATE;
	      end
	      else begin
		 DATA_AVAIL <= 1'b1;
		 COUNTER <= 16'd0;
		 STATE <= IDLE_STATE;
	      end
	   end // case: STOP_STATE

	   default: begin
	      STATE <= IDLE_STATE;
	   end // case: default

	 endcase // case (STATE)
	 
      end // else: !if(!RST_N)

   end // always @ (posedge CLK, negedge RST_N)

endmodule // UART_RX
