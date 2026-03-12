module UART_TX (/*AUTOARG*/
   // Outputs
   O_ACTIVE, O_TX, O_DONE,
   // Inputs
   CLK, RST_N, I_DATA_AVAIL, I_DATA_BYTE
   );
   input CLK, RST_N;
   input I_DATA_AVAIL;
   input [7:0] I_DATA_BYTE;

   output reg  O_ACTIVE, O_TX, O_DONE;

   parameter   CLKS_PER_BIT = 434;

   localparam  IDLE_STATE = 2'b00;
   localparam  START_STATE = 2'b01;
   localparam  SEND_BIT_STATE = 2'b10;
   localparam  STOP_STATE = 2'b11;

   reg [1:0]   STATE;
   reg [15:0]  COUNTER;
   reg [2:0]   BIT_INDEX;
   reg [7:0]   DATA_BYTE;


   always @(posedge CLK or negedge RST_N) begin
      if (!RST_N) begin
	 STATE <= IDLE_STATE;
	 COUNTER <= 16'b0; BIT_INDEX <= 3'b0; DATA_BYTE <= 8'b0;
	 O_ACTIVE <= 1'b0; O_TX <= 1'b1; O_DONE <= 1'b0;
      end
      else begin
	 case (STATE)
	   IDLE_STATE: begin
	      O_TX <= 1'b1;
	      O_DONE <= 1'b0;
	      COUNTER <= 16'b0;
	      BIT_INDEX <= 3'b0;
	      if (I_DATA_AVAIL == 1'b1) begin
		 O_ACTIVE <= 1'b1;
		 DATA_BYTE <= I_DATA_BYTE;
		 STATE <= START_STATE;
	      end
	      else
		STATE <= IDLE_STATE;
	   end // case: IDLE_STATE

	   START_STATE: begin
	      O_TX <= 1'b0;
	      if (COUNTER < CLKS_PER_BIT - 1) begin
		 COUNTER <= COUNTER + 16'b1;
		 STATE <= START_STATE;
	      end
	      else begin
		 COUNTER <= 16'b0;
		 STATE <= SEND_BIT_STATE;
	      end
	   end // case: START_STATE

	   SEND_BIT_STATE: begin
	      O_TX <= DATA_BYTE[BIT_INDEX];
	      if (COUNTER < CLKS_PER_BIT - 1) begin
		 COUNTER <= COUNTER + 16'b1;
		 STATE <= SEND_BIT_STATE;
	      end
	      else begin
		 COUNTER <= 0;
		 if (BIT_INDEX < 3'd7) begin
		    BIT_INDEX <= BIT_INDEX + 3'b1;
		    STATE <= SEND_BIT_STATE;
		 end
		 else begin
		    BIT_INDEX <= 0;
		    STATE <= STOP_STATE;
		 end
	      end
	   end // case: SEND_BIT_STATE

	   STOP_STATE: begin
	      O_TX <= 1'b1;
	      if (COUNTER < CLKS_PER_BIT-1) begin
		 COUNTER <= COUNTER + 16'b1;
		 STATE <= STOP_STATE;
	      end
	      else begin
		 O_DONE <= 1'b1;
		 STATE <= IDLE_STATE;
		 O_ACTIVE <= 1'b0;
	      end
	   end // case: STOP_STATE

	   default: begin
	      STATE <= IDLE_STATE;
	   end // case: default
	   
	 endcase // case (STATE)
	 
      end // else: !if(!RST_N)

   end // always @ (posedge clock or negedge RST_N)

endmodule // UART_TX
