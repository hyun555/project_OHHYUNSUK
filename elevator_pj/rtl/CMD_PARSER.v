module CMD_PARSER (/*AUTOARG*/
   // Outputs
   O_HALL_UP_WE, O_HALL_DOWN_WE, O_FLOOR_ADDR, O_SEND_ERROR_CMD,
   O_SEND_ERROR_FLOOR,
   // Inputs
   CLK, RST_N, RX_DATA_AVAIL, RX_DATA_BYTE
   );
   input CLK;
   input RST_N;
   input RX_DATA_AVAIL;
   input [7:0] RX_DATA_BYTE; //1byte 정보수신

   output reg  O_HALL_UP_WE;  //방향 결정
   output reg  O_HALL_DOWN_WE;
   output reg [7:0] O_FLOOR_ADDR; 
   output reg	    O_SEND_ERROR_CMD; // ERR 검출시 전달
   output reg	    O_SEND_ERROR_FLOOR;

   parameter	    MAX_FLOOR = 32;

   localparam	    S_IDLE     = 4'd0;
   localparam	    S_GET_DIGIT= 4'd1;
   localparam	    S_GET_U    = 4'd2;
   localparam	    S_GET_P    = 4'd3;
   localparam	    S_GET_D    = 4'd4;
   localparam	    S_GET_O    = 4'd5;
   localparam	    S_GET_W    = 4'd6;
   localparam	    S_GET_N    = 4'd7;

   localparam	    ASCII_0 = 8'h30;
   localparam	    ASCII_9 = 8'h39;
   localparam	    ASCII_U = 8'h55;
   localparam	    ASCII_P = 8'h50;
   localparam	    ASCII_D = 8'h44;
   localparam	    ASCII_O = 8'h4F;
   localparam	    ASCII_W = 8'h57;
   localparam	    ASCII_N = 8'h4E;
   localparam	    ASCII_CR = 8'h0D;
   localparam	    ASCII_LF = 8'h0A;

   reg [3:0]	    STATE;
   reg [7:0]	    FLOOR_REG; 
   //reg [1:0]	    DIR_REG;
	
	localparam [7:0] MAXF = MAX_FLOOR[7:0];
	wire valid_up   = (FLOOR_REG >= 8'd1) && (FLOOR_REG <  MAXF); // 허용 층수 범위 (UP은 1층에서 31층까지 32층은 UP 안됨)
	wire valid_down = (FLOOR_REG >  8'd1) && (FLOOR_REG <= MAXF); // 0DOWN 금지 32DOWN 허용
   wire [7:0]	    DIGIT_VAL = RX_DATA_BYTE - ASCII_0; //ASCII 코드 값을 내부적으로 정수형태로 만들기 위함
	
   wire		    IS_DIGIT = (RX_DATA_BYTE >= ASCII_0) && (RX_DATA_BYTE <= ASCII_9); //숫자 범위 내에 있는가?
   wire		    IS_ENTER = (RX_DATA_BYTE == ASCII_CR) || (RX_DATA_BYTE == ASCII_LF); // 엔터

   always @(posedge CLK or negedge RST_N) begin
      if (!RST_N) begin
	 STATE <= S_IDLE;
	 O_HALL_UP_WE <= 1'b0; O_HALL_DOWN_WE <= 1'b0;
	 O_SEND_ERROR_CMD <= 1'b0; O_SEND_ERROR_FLOOR <= 1'b0;
	 FLOOR_REG <= 8'b0; 
	 //DIR_REG <= 2'b0;
      end
      else begin
	 O_HALL_UP_WE <= 1'b0; O_HALL_DOWN_WE <= 1'b0;
	 O_SEND_ERROR_CMD <= 1'b0; O_SEND_ERROR_FLOOR <= 1'b0;
	 if (RX_DATA_AVAIL) begin
	    case (STATE)
	      S_IDLE: begin
		 if (IS_DIGIT) begin
		    FLOOR_REG <= DIGIT_VAL;
		    //DIR_REG <= 2'b0;
		    STATE <= S_GET_DIGIT;
		 end 
		 else if (IS_ENTER) begin 
		    STATE <= S_IDLE;			 
		 end 
		 else begin
		    O_SEND_ERROR_CMD <=1'b1;
		    STATE <= S_IDLE;
		 end
	      end // case: S_IDLE
 	      
	      S_GET_DIGIT: begin
		 if (IS_DIGIT) begin
		    FLOOR_REG = (FLOOR_REG * 8'd10) + DIGIT_VAL; //두개의 숫자 값 들어올 수도 있기에 
		    if (FLOOR_REG > MAX_FLOOR) begin
		       O_SEND_ERROR_CMD <=1'b1;
		    end
		    else begin 
		       STATE <= S_GET_DIGIT;
		    end 
		 end
		 else if (RX_DATA_BYTE == ASCII_U) begin
		    //DIR_REG <= 2'd1;
		    STATE <= S_GET_U;
		 end
		 else if (RX_DATA_BYTE == ASCII_D) begin
		    //DIR_REG <= 2'd2;
		    STATE <= S_GET_D;
		 end 
		 else if (IS_ENTER) begin
		    O_SEND_ERROR_CMD <= 1'b1;
		    STATE <= S_IDLE;
		 end 
		 else begin
		    O_SEND_ERROR_CMD <= 1'b1;
		    STATE <= S_IDLE;
		 end
	      end // case: S_GET_DIGIT
 	      
	      S_GET_U: begin
		 if (RX_DATA_BYTE == ASCII_P) begin
		    STATE <= S_GET_P;
		 end 
		 else begin
		    O_SEND_ERROR_CMD <= 1'b1;
		    STATE <= S_IDLE;
		 end
	      end // case: S_GET_U
 	      
S_GET_P: begin
   if (IS_ENTER) begin
      // UP은 1..MAX_FLOOR-1만 허용
      if (valid_up) begin
         O_HALL_UP_WE <= 1'b1;
         O_FLOOR_ADDR <= FLOOR_REG;
      end
      // else: 32UP, 0UP 등은 "조용히 무시" (ERR 미전송)
      STATE <= S_IDLE;
   end
   else begin
      O_SEND_ERROR_CMD <= 1'b1;
      STATE <= S_IDLE;
   end
end // case: S_GET_P

	      S_GET_D: begin
		 if (RX_DATA_BYTE == ASCII_O) begin
		    STATE <= S_GET_O;
		 end 
		 else begin
		    O_SEND_ERROR_CMD <= 1'b1;
		    STATE <= S_IDLE;
		 end
	      end // case: S_GET_D

	      S_GET_O: begin
		 if (RX_DATA_BYTE == ASCII_W) begin
		    STATE <= S_GET_W;
		 end 
		 else begin
		    O_SEND_ERROR_CMD <= 1'b1;
		    STATE <= S_IDLE;
		 end
	      end // case: S_GET_O

	      S_GET_W: begin
		 if (RX_DATA_BYTE == ASCII_N) begin
		    STATE <= S_GET_N;
		 end 
		 else begin
		    O_SEND_ERROR_CMD <= 1'b1;
		    STATE <= S_IDLE;
		 end
	      end // case: S_GET_W

	S_GET_N: begin
   if (IS_ENTER) begin
      // DOWN은 2..MAX_FLOOR만 허용
      if (valid_down) begin
         O_HALL_DOWN_WE <= 1'b1;
         O_FLOOR_ADDR   <= FLOOR_REG;
      end
      // else: 1DOWN, 0DOWN 등은 "조용히 무시" (ERR 미전송)
      STATE <= S_IDLE;
   end
   else begin
      O_SEND_ERROR_CMD <= 1'b1;
      STATE <= S_IDLE;
   end
end // case: S_GET_N
	      
	      default: begin
		 STATE <= S_IDLE;
	      end // case: default
	      
	    endcase // case (STATE)

	 end // if (RX_DATA_AVAIL)
	 
      end // else: !if(!RST_N)
      
   end // always @ (posedge CLK or negedge RST_N)
   
endmodule // CMD_PARSER





