module ELEVATOR_FSM(/*AUTOARG*/
   // Outputs
   CURRENT_FLOOR, CURRENT_DIR, MOTOR_UP, MOTOR_DN, DOOR_OPEN_DRV,
   DOOR_CLOSE_DRV,
   // Inputs
   CLK, RST_N, T_TICK, EMG_STOP, CALL_FLOOR, CALL_DIR, CALL_VALID,
   BTN_OPEN, BTN_CLOSE
   );
   input		   CLK, RST_N, T_TICK;
   input		   EMG_STOP;
   input [5:0]		   CALL_FLOOR;
   input		   CALL_DIR, CALL_VALID;
   input		   BTN_OPEN, BTN_CLOSE;
   
   output reg [5:0]	   CURRENT_FLOOR;
   output reg [1:0]	   CURRENT_DIR;
   output reg		   MOTOR_UP, MOTOR_DN, DOOR_OPEN_DRV, DOOR_CLOSE_DRV;

   parameter		   MAX_FLOOR = 32;
   parameter		   START_FLOOR = 1;
   
   localparam		   DWELL_BASE = 3;
   localparam		   FLOOR_TRAVEL_TIME = 2;
   localparam		   DOOR_OPEN_TIME = 1;
   localparam		   DOOR_CLOSE_TIME = 1;
   localparam		   OPEN_MAX_TIME = 10;
   localparam		   DIR_DOWN = 2'd0;
   localparam		   DIR_UP = 2'd1;
   localparam		   DIR_STOP = 2'd2;
   localparam		   S_IDLE_STOP = 3'd0;
   localparam		   S_DOOR_OPENING = 3'd1;
   localparam		   S_DWELL = 3'd2;
   localparam		   S_DOOR_CLOSING = 3'd3;
   localparam		   S_MOVING_UP = 3'd4;
   localparam		   S_MOVING_DOWN = 3'd5;
   localparam		   S_EMG_STOP = 3'd6;
   
   reg [2:0]		   state, recover_state;
   reg [MAX_FLOOR:0]	   hall_up, hall_down, car_call;
   reg [3:0]		   T_TICK_CNT, recover_T_TICK_CNT;
   reg			   recovering;
   reg [3:0]		   door_timer;
   
   wire [MAX_FLOOR:0]	   all_requests;
   wire [MAX_FLOOR:0]	   up_priority_requests;
   wire [MAX_FLOOR:0]	   down_priority_requests;
   assign all_requests = car_call | hall_up | hall_down;
   assign up_priority_requests = car_call | hall_up;
   assign down_priority_requests = car_call | hall_down;
   
   wire req_at_floor;
   assign req_at_floor = all_requests[CURRENT_FLOOR];
   
   wire has_req_above, has_req_below;
   assign has_req_above = |(all_requests >> (CURRENT_FLOOR + 1'b1));
   assign has_req_below = (CURRENT_FLOOR <= START_FLOOR) ? 1'b0 : |(all_requests & ((1'b1 << CURRENT_FLOOR) - 1'b1));
   
   wire has_up_req_above, has_down_req_below;
   assign has_up_req_above = |(up_priority_requests >> (CURRENT_FLOOR + 1'b1));
   assign has_down_req_below = (CURRENT_FLOOR <= START_FLOOR) ? 1'b0 : |(down_priority_requests & ((1'b1 << CURRENT_FLOOR) - 1'b1));
   
   wire [5:0] next_up;
   assign next_up = (CURRENT_FLOOR == MAX_FLOOR)    ? CURRENT_FLOOR : (CURRENT_FLOOR + 1'b1);
   wire [5:0] next_down; 
   assign next_down = (CURRENT_FLOOR == START_FLOOR) ? CURRENT_FLOOR : (CURRENT_FLOOR - 1'b1);
   wire	      has_any_req_above_next_up;
   assign has_any_req_above_next_up  = (next_up == MAX_FLOOR)    ? 1'b0 : |(all_requests >> (next_up + 1'b1));
   wire	      has_any_req_below_next_down;
   assign has_any_req_below_next_down = (next_down <= START_FLOOR) ? 1'b0 : |(all_requests & ((1'b1 << next_down) - 1'b1));
   wire	      stop_at_next_up;
   assign stop_at_next_up = up_priority_requests[next_up] |
			    (next_up == MAX_FLOOR) |
			    (!has_up_req_above & !has_any_req_above_next_up & hall_down[next_up]);
   wire	      stop_at_next_down;
   assign stop_at_next_down = down_priority_requests[next_down] |
			      (next_down == START_FLOOR) |
			      (!has_down_req_below & !has_any_req_below_next_down & hall_up[next_down]);
   
   always @(posedge CLK or negedge RST_N) begin
      if (!RST_N) begin
         state <= S_IDLE_STOP;
         recover_state <= S_IDLE_STOP;
         CURRENT_FLOOR <= START_FLOOR;
         CURRENT_DIR <= DIR_STOP;
         hall_up <= {MAX_FLOOR+1{1'b0}};
         hall_down <= {MAX_FLOOR+1{1'b0}};
         car_call <= {MAX_FLOOR+1{1'b0}};
         MOTOR_UP <= 1'b0;
         MOTOR_DN <= 1'b0;
         DOOR_OPEN_DRV <= 1'b0;
         DOOR_CLOSE_DRV <= 1'b0;
         T_TICK_CNT <= 4'd0;
         recover_T_TICK_CNT <= 4'd0;
         recovering <= 1'b0;
      end
      else begin
         if (CALL_VALID && (CALL_FLOOR >= START_FLOOR) && (CALL_FLOOR <= MAX_FLOOR)) begin
            if (CALL_DIR == DIR_UP) begin
               hall_up[CALL_FLOOR] <= 1'b1;
            end
            else if (CALL_DIR == DIR_DOWN) begin
               hall_down[CALL_FLOOR] <= 1'b1;
            end
         end
         if (EMG_STOP) begin
            if (state != S_EMG_STOP) begin
               recover_state <= state;
               if (state == S_DWELL || state == S_MOVING_UP || state == S_MOVING_DOWN) begin
                  recover_T_TICK_CNT <= T_TICK_CNT; 
                  recovering <= 1'b1;
               end
               else if (state == S_DOOR_OPENING || state == S_DOOR_CLOSING) begin
                  recover_T_TICK_CNT <= 4'd0; 
                  recovering <= 1'b1;
               end
               else begin 
                  recover_T_TICK_CNT <= 4'd0;
                  recovering <= 1'b0; 
               end
            end
            state <= S_EMG_STOP;
            MOTOR_UP <= 1'b0;
            MOTOR_DN <= 1'b0;
            DOOR_OPEN_DRV <= 1'b0;
            DOOR_CLOSE_DRV <= 1'b0;
            CURRENT_DIR <= DIR_STOP;
         end
         else begin
            MOTOR_UP <= 1'b0;
            MOTOR_DN <= 1'b0;
            DOOR_OPEN_DRV <= 1'b0;
            DOOR_CLOSE_DRV <= 1'b0;
            case (state)
              S_IDLE_STOP: begin
                 CURRENT_DIR <= DIR_STOP;
                 if (recovering) begin
                    recovering <= 1'b0;
                    state <= recover_state;
                    T_TICK_CNT <= recover_T_TICK_CNT; 
                 end
                 else if (req_at_floor) begin
                    state <= S_DOOR_OPENING;
                    T_TICK_CNT <= 4'd0;
                 end
                 else if (has_req_above) begin
                    CURRENT_DIR <= DIR_UP;
                    state <= S_MOVING_UP;
                    T_TICK_CNT <= 4'd0;
                 end
                 else if (has_req_below) begin
                    CURRENT_DIR <= DIR_DOWN;
                    state <= S_MOVING_DOWN;
                    T_TICK_CNT <= 4'd0;
                 end
              end
              S_DOOR_OPENING: begin
                 DOOR_OPEN_DRV <= 1'b1;
		 if (BTN_OPEN) begin
		    T_TICK_CNT <= 4'd0;
		 end
                 else if (T_TICK) begin
                    if (T_TICK_CNT == (DOOR_OPEN_TIME - 1'b1)) begin
                       state <= S_DWELL;
                       T_TICK_CNT <= 4'd0;
                    end else begin
                       T_TICK_CNT <= T_TICK_CNT + 1'b1;
                    end
                 end
              end
              S_DWELL: begin
                 car_call[CURRENT_FLOOR] <= 1'b0;
                 if (CURRENT_DIR == DIR_UP) begin
                    hall_up[CURRENT_FLOOR] <= 1'b0;
                    if ((CURRENT_FLOOR == MAX_FLOOR) || !has_up_req_above) begin
                       hall_down[CURRENT_FLOOR] <= 1'b0;
                    end
                 end else if (CURRENT_DIR == DIR_DOWN) begin
                    hall_down[CURRENT_FLOOR] <= 1'b0;
                    if ((CURRENT_FLOOR == START_FLOOR) || !has_down_req_below) begin
                       hall_up[CURRENT_FLOOR] <= 1'b0;
                    end
                 end else begin 
                    hall_up[CURRENT_FLOOR] <= 1'b0;
                    hall_down[CURRENT_FLOOR] <= 1'b0;
                 end
                 if (BTN_CLOSE) begin
                    state <= S_DOOR_CLOSING;
                    T_TICK_CNT <= 4'd0;
                 end
                 else if (BTN_OPEN) begin
		    T_TICK_CNT <= 4'd0;
		    if (T_TICK) begin
		       if (door_timer == OPEN_MAX_TIME - 1'b1) begin
			  state <= S_DOOR_CLOSING;
			  door_timer <= 4'd0;
		       end
		       else begin
			  door_timer <= door_timer + 1'b1;
		       end
		    end			  
                 end
                 else if (T_TICK) begin
                    if (T_TICK_CNT == (DWELL_BASE - 1'b1 + (BTN_OPEN ? 4'd1 : 4'd0))) begin
                       state <= S_DOOR_CLOSING;
                       T_TICK_CNT <= 4'd0;
                    end else begin
                       T_TICK_CNT <= T_TICK_CNT + 1'b1;
                    end
                 end
              end
              S_DOOR_CLOSING: begin
                 DOOR_CLOSE_DRV <= 1'b1;
                 if (BTN_OPEN) begin
                    state <= S_DOOR_OPENING;
                    T_TICK_CNT <= 4'd0;
                 end
                 else if (T_TICK) begin
                    if (T_TICK_CNT == (DOOR_CLOSE_TIME - 1'b1)) begin
                       if ((CURRENT_DIR == DIR_UP || CURRENT_DIR == DIR_STOP) && has_req_above) begin
                          CURRENT_DIR <= DIR_UP;
                          state <= S_MOVING_UP;
                          T_TICK_CNT <= 4'd0;
                       end
                       else if ((CURRENT_DIR == DIR_DOWN || CURRENT_DIR == DIR_STOP) && has_req_below) begin
                          CURRENT_DIR <= DIR_DOWN;
                          state <= S_MOVING_DOWN;
                          T_TICK_CNT <= 4'd0;
                       end
                       else if (has_req_above) begin
                          CURRENT_DIR <= DIR_UP;
                          state <= S_MOVING_UP;
                          T_TICK_CNT <= 4'd0;
                       end
                       else if (has_req_below) begin
                          CURRENT_DIR <= DIR_DOWN;
                          state <= S_MOVING_DOWN;
                          T_TICK_CNT <= 4'd0;
                       end
		       else if (req_at_floor) begin
                          state <= S_IDLE_STOP; 
                       end
                       else begin
                          state <= S_IDLE_STOP;
                          T_TICK_CNT <= 4'd0;
                       end
                    end else begin
                       T_TICK_CNT <= T_TICK_CNT + 1'b1;
                    end
                 end
              end
              S_MOVING_UP: begin
                 MOTOR_UP <= 1'b1;
                 CURRENT_DIR <= DIR_UP;
                 if (T_TICK) begin
                    if (T_TICK_CNT == (FLOOR_TRAVEL_TIME - 1'b1)) begin
                       CURRENT_FLOOR <= next_up;
                       if (stop_at_next_up) begin
                          state <= S_DOOR_OPENING;
                          T_TICK_CNT <= 4'd0;
                       end else begin
                          state <= S_MOVING_UP;
                          T_TICK_CNT <= 4'd0;
                       end
                    end else begin
                       T_TICK_CNT <= T_TICK_CNT + 1'b1;
                    end
                 end
              end
              S_MOVING_DOWN: begin
                 MOTOR_DN <= 1'b1;
                 CURRENT_DIR <= DIR_DOWN;
                 if (T_TICK) begin
                    if (T_TICK_CNT == (FLOOR_TRAVEL_TIME - 1'b1)) begin
                       CURRENT_FLOOR <= next_down;
                       if (stop_at_next_down) begin
                          state <= S_DOOR_OPENING;
                          T_TICK_CNT <= 4'd0;
                       end else begin
                          state <= S_MOVING_DOWN;
                          T_TICK_CNT <= 4'd0;
                       end
                    end else begin
                       T_TICK_CNT <= T_TICK_CNT + 1'b1;
                    end
                 end
              end
              S_EMG_STOP: begin
                 if (!EMG_STOP) begin
                    state <= S_IDLE_STOP;
                 end
              end
              default: begin
                 state <= S_IDLE_STOP;
                 T_TICK_CNT <= 4'd0;
              end
            endcase // case (state)
	    
         end // else: !if(EMG_STOP)
	 
      end // else: !if(!RST_N)
      
   end // always @ (posedge CLK or negedge RST_N)
   
endmodule // ELEVATOR_FSM
