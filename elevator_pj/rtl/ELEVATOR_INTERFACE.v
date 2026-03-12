module ELEVATOR_INTERFACE(/*AUTOARG*/
   // Outputs
   UART_TXD, HEX0, HEX1, O_CALL_VALID, O_CALL_FLOOR, O_CALL_DIR,
   // Inputs
   CLOCK_50, KEY, UART_RXD, CUR_FLOOR, MOTOR_UP, MOTOR_DOWN
   );
   input CLOCK_50;
   input [0:0] KEY; // KEY[0] = RST_N (Active-Low)
   input       UART_RXD;
   output      UART_TXD;

   output [6:0]	HEX0, HEX1;

   //FSM State Monitor Input
   input [7:0]	CUR_FLOOR;
   input	MOTOR_UP, MOTOR_DOWN;

   //Call Interface
   output	O_CALL_VALID;
   output [5:0]	O_CALL_FLOOR;
   output	O_CALL_DIR;

   localparam	CLKS_PER_BIT = 434;
   localparam	MAX_FLOOR = 32;

   wire		RST_N = KEY[0];

   // UART RX
   wire		RX_DONE;
   wire [7:0]	RX_DATA;
   UART_RX #(.CLKS_PER_BIT(CLKS_PER_BIT))
   U_RX (/*AUTOINST*/
	 // Outputs
	 .O_DATA_AVAIL			(RX_DONE),
	 .O_DATA_BYTE			(RX_DATA),
	 // Inputs
	 .CLK				(CLOCK_50),
	 .RST_N				(RST_N),
	 .I_RX				(UART_RXD));

   // UART TX (ERROR/INFO)
   wire TX_ACTIVE_ERR, TX_DONE_ERR, TX_LINE_ERR;
   reg	TX_START_ERR;
   reg [7:0] TX_DATA_ERR;

   UART_TX #(.CLKS_PER_BIT(CLKS_PER_BIT))
   U_TX_ERR (/*AUTOINST*/
	     // Outputs
	     .O_ACTIVE			(TX_ACTIVE_ERR),
	     .O_TX			(TX_LINE_ERR),
	     .O_DONE			(TX_DONE_ERR),
	     // Inputs
	     .CLK			(CLOCK_50),
	     .RST_N			(RST_N),
	     .I_DATA_AVAIL		(TX_START_ERR),
	     .I_DATA_BYTE		(TX_DATA_ERR));

    wire TX_ACTIVE_INFO, TX_DONE_INFO, TX_LINE_INFO;
    reg  TX_START_INFO;
    reg  [7:0] TX_DATA_INFO;
   
   UART_TX #(.CLKS_PER_BIT(CLKS_PER_BIT))
   U_TX_INFO (/*AUTOINST*/
	      // Outputs
	      .O_ACTIVE			(TX_ACTIVE_INFO),
	      .O_TX			(TX_LINE_INFO),
	      .O_DONE			(TX_DONE_INFO),
	      // Inputs
	      .CLK			(CLOCK_50),
	      .RST_N			(RST_N),
	      .I_DATA_AVAIL		(TX_START_INFO),
	      .I_DATA_BYTE		(TX_DATA_INFO));

   assign UART_TXD = (TX_ACTIVE_ERR) ? TX_LINE_ERR : TX_LINE_INFO;

   // CMD PARSER
   wire HALL_UP_WE_I, HALL_DOWN_WE_I;
   wire [7:0] FLOOR_ADDR_I;
   wire	      SEND_ERROR_CMD_I, SEND_ERROR_FLOOR_I;

   CMD_PARSER #(.MAX_FLOOR(MAX_FLOOR))
   U_CMD (/*AUTOINST*/
	  // Outputs
	  .O_HALL_UP_WE			(HALL_UP_WE_I),
	  .O_HALL_DOWN_WE		(HALL_DOWN_WE_I),
	  .O_FLOOR_ADDR			(FLOOR_ADDR_I),
	  .O_SEND_ERROR_CMD		(SEND_ERROR_CMD_I),
	  .O_SEND_ERROR_FLOOR		(SEND_ERROR_FLOOR_I),
	  // Inputs
	  .CLK				(CLOCK_50),
	  .RST_N			(RST_N),
	  .RX_DATA_AVAIL		(RX_DONE),
	  .RX_DATA_BYTE			(RX_DATA));

   // FSM Call Signal
   reg CALL_VALID_R;
   reg [5:0] CALL_FLOOR_R;
   reg	     CALL_DIR_R; // 1: UP, 0:DOWN

   always @(posedge CLOCK_50 or negedge RST_N) begin
      if (!RST_N) begin
         CALL_VALID_R <= 1'b0;
         CALL_FLOOR_R <= 6'd0;
         CALL_DIR_R   <= 1'b0;
      end 
      else begin
         CALL_VALID_R <= 1'b0;
         if (HALL_UP_WE_I) begin
            CALL_VALID_R <= 1'b1;
            CALL_FLOOR_R <= FLOOR_ADDR_I[5:0];
            CALL_DIR_R   <= 1'b1; // UP
         end 
	 else if (HALL_DOWN_WE_I) begin
            CALL_VALID_R <= 1'b1;
            CALL_FLOOR_R <= FLOOR_ADDR_I[5:0];
            CALL_DIR_R   <= 1'b0; // DOWN
         end
      end
   end // always @ (posedge CLOCK_50 or negedge RST_N)
   assign O_CALL_VALID = CALL_VALID_R;
   assign O_CALL_FLOOR = CALL_FLOOR_R;
   assign O_CALL_DIR = CALL_DIR_R;

   // SEVEN SEGMENT
   wire [7:0] SEG_FLOOR;
   wire [3:0] ONES_DIGIT;
   wire [3:0] TENS_DIGIT;
   wire [7:0] ones_full;
   wire [7:0] tens_full;
	assign SEG_FLOOR = 8'd1;
   assign ones_full = SEG_FLOOR % 8'd10;
   assign tens_full = (SEG_FLOOR / 8'd10) % 8'd10;

   assign ONES_DIGIT = ones_full[3:0];
   assign TENS_DIGIT = tens_full[3:0];


   SEVEN_SEGMENT U_HEX0 (/*AUTOINST*/
			 // Outputs
			 .O_SEG			(HEX0),
			 // Inputs
			 .I_BIN			(ONES_DIGIT));
   SEVEN_SEGMENT U_HEX1 (/*AUTOINST*/
			 // Outputs
			 .O_SEG			(HEX1),
			 // Inputs
			 .I_BIN			(TENS_DIGIT));

   // 1Hz TICK
   reg [25:0] SEC_CNT;
   reg	      SEC_TICK;
   always @(posedge CLOCK_50 or negedge RST_N) begin
      if (!RST_N) begin
         SEC_CNT  <= 26'd0;
         SEC_TICK <= 1'b0;
      end 
      else if (SEC_CNT == 26'd49_999_999) begin
         SEC_CNT  <= 26'd0;
         SEC_TICK <= 1'b1;
      end 
      else begin
         SEC_CNT  <= SEC_CNT + 1'b1;
         SEC_TICK <= 1'b0;
      end
   end // always @ (posedge CLOCK_50 or negedge RST_N)

   // Quiet Mode: Toggle on Every Enter
   reg QUIET_MODE;
   always @(posedge CLOCK_50 or negedge RST_N) begin
      if (!RST_N) begin
	 QUIET_MODE <= 1'b0;
      end
      
      else if (RX_DONE && (RX_DATA == 8'h0D || RX_DATA == 8'h0A)) begin
        QUIET_MODE <= ~QUIET_MODE;
      end
   end // always @ (posedge CLOCK_50 or negedge RST_N)

   // Error Response Transmission
   localparam ERR_NONE = 2'd0, ERR_CMD = 2'd1, ERR_FLR = 2'd2;
   reg [1:0]  ERR_SEL;
   reg [3:0]  ERR_IDX;
   reg	      ERR_REQ_LATCHED;

   always @(posedge CLOCK_50 or negedge RST_N) begin
      if (!RST_N) begin
         ERR_SEL         <= ERR_NONE;
         ERR_IDX         <= 4'd0;
         ERR_REQ_LATCHED <= 1'b0;
         TX_START_ERR    <= 1'b0;
         TX_DATA_ERR     <= 8'h00;
      end 
      else begin
         TX_START_ERR <= 1'b0;

         if (SEND_ERROR_CMD_I) begin 
	    ERR_SEL <= ERR_CMD; ERR_REQ_LATCHED <= 1'b1; 
	 end
         if (SEND_ERROR_FLOOR_I) begin 
	    ERR_SEL <= ERR_FLR; ERR_REQ_LATCHED <= 1'b1; 
	 end

         if (ERR_REQ_LATCHED) begin
            case (ERR_IDX)
              4'd0:  TX_DATA_ERR <= "E";
              4'd1:  TX_DATA_ERR <= "R";
              4'd2:  TX_DATA_ERR <= "R";
              4'd3:  TX_DATA_ERR <= ":";
              4'd4:  TX_DATA_ERR <= (ERR_SEL == ERR_CMD) ? "C" : "F";
              4'd5:  TX_DATA_ERR <= (ERR_SEL == ERR_CMD) ? "M" : "L";
              4'd6:  TX_DATA_ERR <= (ERR_SEL == ERR_CMD) ? "D" : "R";
              4'd7:  TX_DATA_ERR <= 8'h0D;
              4'd8:  TX_DATA_ERR <= 8'h0A;
              default: TX_DATA_ERR <= 8'h00;
            endcase // case (ERR_IDX)

            if (!TX_ACTIVE_ERR && !QUIET_MODE) begin
	       TX_START_ERR <= 1'b1;
	    end
	    
            if (TX_DONE_ERR) begin
               if (ERR_IDX < 4'd8) begin
		  ERR_IDX <= ERR_IDX + 1'b1;
	       end
               else begin
                  ERR_IDX         <= 4'd0;
                  ERR_REQ_LATCHED <= 1'b0;
                  ERR_SEL         <= ERR_NONE;
               end
            end
         end
      end
   end // always @ (posedge CLOCK_50 or negedge RST_N)

   // Status Transmission (F:xx,DIR:UP|DOWN|STOP\r\n)
   localparam S_IDLE = 2'd0, S_SEND = 2'd1, S_WAIT = 2'd2;
   reg [1:0]  SEND_STATE;
   reg [5:0]  SEND_IDX;
   reg [1:0]  DIR_MODE;
   reg [7:0]  F_TENS, F_ONES;
   reg [2:0]  DIR_LEN;
   reg [7:0]  DIR_BYTE;
   wire	      ANY_ERROR_ACTIVE = ERR_REQ_LATCHED | TX_ACTIVE_ERR;
   wire [5:0] j = SEND_IDX - 6'd9;
	
	//DIR만 따로 조합 -> 이후 SEND 부분에서 사용
   always @(*) begin
      case (DIR_MODE)
        2'd0: begin
           case (j)
             6'd0: DIR_BYTE = "U";
             6'd1: DIR_BYTE = "P";
             default: DIR_BYTE = 8'h00;
           endcase
        end
        2'd1: begin
           case (j)
             6'd0: DIR_BYTE = "D";
             6'd1: DIR_BYTE = "O";
             6'd2: DIR_BYTE = "W";
             6'd3: DIR_BYTE = "N";
             default: DIR_BYTE = 8'h00;
           endcase
        end
        default: begin
           case (j)
             6'd0: DIR_BYTE = "S";
             6'd1: DIR_BYTE = "T";
             6'd2: DIR_BYTE = "O";
             6'd3: DIR_BYTE = "P";
             default: DIR_BYTE = 8'h00;
           endcase
        end
      endcase 
   end
	//SEND
   always @(posedge CLOCK_50 or negedge RST_N) begin
      if (!RST_N) begin
         SEND_STATE    <= S_IDLE;
         SEND_IDX      <= 6'd0;
         TX_START_INFO <= 1'b0;
         TX_DATA_INFO  <= 8'h00;
         DIR_MODE      <= 2'd2;
         F_TENS        <= 8'd0;
         F_ONES        <= 8'd0;
         DIR_LEN       <= 3'd0;
      end 
      else begin
         TX_START_INFO <= 1'b0;
         case (SEND_STATE)
           S_IDLE: begin
              if (SEC_TICK && !ANY_ERROR_ACTIVE && !QUIET_MODE) begin
                 F_TENS <= (CUR_FLOOR / 10) % 10;
                 F_ONES <=  CUR_FLOOR % 10;
                 if (MOTOR_UP && !MOTOR_DOWN)      DIR_MODE <= 2'd0;
                 else if (MOTOR_DOWN && !MOTOR_UP) DIR_MODE <= 2'd1;
                 else                               DIR_MODE <= 2'd2;
                 if (MOTOR_UP && !MOTOR_DOWN)      DIR_LEN <= 3'd2;
                 else if (MOTOR_DOWN && !MOTOR_UP) DIR_LEN <= 3'd4;
                 else                               DIR_LEN <= 3'd4;
                 SEND_IDX   <= 6'd0;
                 SEND_STATE <= S_SEND;
              end
           end // case: S_IDLE
	   
           S_SEND: begin
              case (SEND_IDX)
                6'd0:  TX_DATA_INFO <= "F";
                6'd1:  TX_DATA_INFO <= ":";
                6'd2:  TX_DATA_INFO <= "0" + F_TENS;
                6'd3:  TX_DATA_INFO <= "0" + F_ONES;
                6'd4:  TX_DATA_INFO <= ",";
                6'd5:  TX_DATA_INFO <= "D";
                6'd6:  TX_DATA_INFO <= "I";
                6'd7:  TX_DATA_INFO <= "R";
                6'd8:  TX_DATA_INFO <= ":";
                default: begin
                   if (SEND_IDX < (6'd9 + DIR_LEN))       TX_DATA_INFO <= DIR_BYTE;
                   else if (SEND_IDX == (6'd9 + DIR_LEN))  TX_DATA_INFO <= 8'h0D;
                   else if (SEND_IDX == (6'd10 + DIR_LEN)) TX_DATA_INFO <= 8'h0A;
                   else                                    TX_DATA_INFO <= 8'h00;
                end
              endcase
              if (!TX_ACTIVE_INFO) begin
                 TX_START_INFO <= 1'b1;
                 SEND_STATE    <= S_WAIT;
              end
           end // case: S_SEND
	   
           S_WAIT: begin
              if (TX_DONE_INFO) begin
                 if (SEND_IDX == (6'd10 + DIR_LEN)) begin
                    SEND_IDX   <= 6'd0;
                    SEND_STATE <= S_IDLE;
                 end 
		 else begin
                    SEND_IDX   <= SEND_IDX + 6'd1;
                    SEND_STATE <= S_SEND;
                 end
              end
           end // case: S_WAIT
	   
         endcase // case (SEND_STATE)
	 
         if ((ANY_ERROR_ACTIVE || QUIET_MODE) && (SEND_STATE != S_IDLE)) begin
            SEND_STATE <= S_IDLE;
            SEND_IDX   <= 6'd0;
         end
	 
      end // else: !if(!RST_N)
      
   end // always @ (posedge CLOCK_50 or negedge RST_N)
   
endmodule // ELEVATOR_INTERFACE
