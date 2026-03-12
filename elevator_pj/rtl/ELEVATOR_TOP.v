module ELEVATOR_TOP (/*AUTOARG*/
   // Outputs
   HEX0, HEX1, MOTOR_UP, MOTOR_DN, DOOR_OPEN_DRV, DOOR_CLOSE_DRV,
   // Inputs
   CLOCK_50, KEY, UART_RXD, UART_TXD, BTN_OPEN, BTN_CLOSE, EMG_STOP
   );
   input CLOCK_50;
   input [0:0] KEY;
   input       UART_RXD;
   input       BTN_OPEN, BTN_CLOSE;
   input       EMG_STOP;

   output      UART_TXD;
   output [6:0]	HEX0, HEX1;
   output	MOTOR_UP, MOTOR_DN;
   output	DOOR_OPEN_DRV, DOOR_CLOSE_DRV;

   wire		RST_N = KEY[0];

   parameter	MAX_FLOOR = 32;
   parameter	CLK_FREQ = 50_000_000;
   parameter	OPEN_ACTIVE_LOW = 1;
   parameter	CLOSE_ACTIVE_LOW = 1;

   localparam	TICK_DIV = CLK_FREQ - 1;

   reg [31:0]	TICK_CNT;
   reg		T_TICK;

   always @(posedge CLOCK_50 or negedge RST_N) begin
      if (!RST_N) begin
	 TICK_CNT <= 32'd0;
	 T_TICK <= 1'b0;
      end
      else if (TICK_CNT == TICK_DIV[31:0]) begin
	 TICK_CNT <= 32'd0;
	 T_TICK <= 1'b1;
      end
      else begin
	 TICK_CNT <= TICK_CNT + 1'b1;
	 T_TICK <= 1'b0;
      end
   end 

   wire BTN_OPEN_H, BTN_CLOSE_H;
   
   DEBOUNCER #(.ACTIVE_LOW(OPEN_ACTIVE_LOW), .CNTR_BITS(19)) 
   U_DB_OPEN  (/*AUTOINST*/
	       // Outputs
	       .DOUT_H			(BTN_OPEN_H),
	       // Inputs
	       .CLK			(CLOCK_50),
	       .RST_N			(RST_N),
	       .DIN			(BTN_OPEN));
   DEBOUNCER #(.ACTIVE_LOW(CLOSE_ACTIVE_LOW), .CNTR_BITS(19)) 
   U_DB_CLOSE (/*AUTOINST*/
	       // Outputs
	       .DOUT_H			(BTN_CLOSE_H),
	       // Inputs
	       .CLK			(CLOCK_50),
	       .RST_N			(RST_N),
	       .DIN			(BTN_CLOSE));

  wire        CALL_VALID;
  wire [5:0]  CALL_FLOOR;
  wire	      CALL_DIR;
  wire [5:0]  CURRENT_FLOOR;
   wire [1:0] CURRENT_DIR;

   ELEVATOR_FSM #(.MAX_FLOOR(MAX_FLOOR)) 
   U_FSM (/*AUTOINST*/
	  // Outputs
	  .CURRENT_FLOOR		(CURRENT_FLOOR),
	  .CURRENT_DIR			(CURRENT_DIR),
	  .MOTOR_UP			(MOTOR_UP),
	  .MOTOR_DN			(MOTOR_DN),
	  .DOOR_OPEN_DRV		(DOOR_OPEN_DRV),
	  .DOOR_CLOSE_DRV		(DOOR_CLOSE_DRV),
	  // Inputs
	  .CLK				(CLOCK_50),
	  .RST_N			(RST_N),
	  .T_TICK			(T_TICK),
	  .EMG_STOP			(EMG_STOP),
	  .CALL_FLOOR			(CALL_FLOOR[5:0]),
	  .CALL_DIR			(CALL_DIR),
	  .CALL_VALID			(CALL_VALID),
	  .BTN_OPEN			(BTN_OPEN_H),
	  .BTN_CLOSE			(BTN_CLOSE_H));

   ELEVATOR_INTERFACE 
     U_INTERFACE (/*AUTOINST*/
		  // Outputs
		  .UART_TXD		(UART_TXD),
		  .HEX0			(HEX0[6:0]),
		  .HEX1			(HEX1[6:0]),
		  .O_CALL_VALID		(CALL_VALID),
		  .O_CALL_FLOOR		(CALL_FLOOR),
		  .O_CALL_DIR		(CALL_DIR),
		  // Inputs
		  .CLOCK_50		(CLOCK_50),
		  .KEY			(KEY[0:0]),
		  .UART_RXD		(UART_RXD),
		  .CUR_FLOOR		({2'b00, CURRENT_FLOOR}),
		  .MOTOR_UP		(MOTOR_UP),
		  .MOTOR_DOWN		(MOTOR_DN));

endmodule // ELEVATOR_TOP
