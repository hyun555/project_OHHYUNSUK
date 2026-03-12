module PE_array_top (
		     input wire               clk,
		     input wire               rst_n,

		     // --------- RUN 모드용 입력 ---------
                     // ARRAY 별로 5개씩, 총 20개 입력
		     input wire [7:0]         din0_0, din0_1, din0_2, din0_3, din0_4, // ARRAY 0
		     input wire [7:0]         din1_0, din1_1, din1_2, din1_3, din1_4, // ARRAY 1
		     input wire [7:0]         din2_0, din2_1, din2_2, din2_3, din2_4, // ARRAY 2
		     input wire [7:0]         din3_0, din3_1, din3_2, din3_3, din3_4, // ARRAY 3

		     input wire               valid_in,

		     // --------- WEIGHT 로딩용 입력 ---------
                     // 40비트 한 번에 들어오는 weight (5 x 8bit)
		     input wire [39:0]        weight_in,     // {w4,w3,w2,w1,w0} 형태
		     input wire               weight_valid,  // weight 유효 신호
		     input wire               change_weight,

		     // --------- 출력 (5x5 배열 4개) : 포트는 다 나눠서 ----------
		     output wire              valid_out_0,
		     output wire              valid_out_1,
		     output wire              valid_out_2,
		     output wire              valid_out_3,

		     output wire signed [21:0] final_result_0,
		     output wire signed [21:0] final_result_1,
		     output wire signed [21:0] final_result_2,
		     output wire signed [21:0] final_result_3,

                     // 5개짜리 묶음 카운터 (0~19)
		     output reg [6:0]        weight_cnt
		     );

   // ================================
   //   상태 머신 정의
   // ================================
   localparam                       S_LOAD = 2'd0;   // weight 로딩 상태
   localparam                       S_RUN  = 2'd1;   // 계산 수행 상태

   reg [1:0]                        state;

   // RUN 상태일 때만 valid_in 사용
   wire                             run_valid = (state == S_RUN) ? valid_in : 1'b0;

   // valid 파이프라인 레지스터 (adder_tree로 전달)
   reg                              pe_valid;

   // ================================
   //   내부 신호 정의
   // ================================

   // 40비트 weight_in을 내부에서 8비트 5개로 슬라이스
   wire [7:0] weight_in_0 = weight_in[7:0];    // 첫 번째 weight
   wire [7:0] weight_in_1 = weight_in[15:8];   // 두 번째
   wire [7:0] weight_in_2 = weight_in[23:16];  // 세 번째
   wire [7:0] weight_in_3 = weight_in[31:24];  // 네 번째
   wire [7:0] weight_in_4 = weight_in[39:32];  // 다섯 번째

   // pe_din[행][열]
   // 총 4개 array * 5행 = 20행 → 0~19
   // 열은 입력 + 5개 PE 통과해서 0~5
   wire [7:0]                       pe_din [0:19][0:5];

   // mul_res[PE index] : 총 100개 (4 * 25)
   wire signed [15:0]               mul_res [0:99];

   // 각 PE의 weight (1차원: 0~99)
   // index = array*25 + row*5 + col
   reg signed [7:0]                 weight_flat [0:99];

   integer                          i;

   // adder_tree 내부 연결용
   wire [3:0]                       valid_out_bus;
   wire signed [21:0]               final_result_bus [0:3];

   // 포트로 나눠서 연결
   assign valid_out_0    = valid_out_bus[0];
   assign valid_out_1    = valid_out_bus[1];
   assign valid_out_2    = valid_out_bus[2];
   assign valid_out_3    = valid_out_bus[3];

   assign final_result_0 = final_result_bus[0];
   assign final_result_1 = final_result_bus[1];
   assign final_result_2 = final_result_bus[2];
   assign final_result_3 = final_result_bus[3];

   // ================================
   //   상태 머신 + weight 로딩 로직
   // ================================
   always @(posedge clk or negedge rst_n) begin
      if (!rst_n) begin
         state      <= S_LOAD;  // 리셋 후에는 weight부터 받음
         weight_cnt <= 7'd0;    // 0~19: 5개짜리 그룹 인덱스
         pe_valid   <= 1'b0;

         // (선택) weight 초기화
         for (i = 0; i < 100; i = i + 1) begin
            weight_flat[i] <= 8'sd0;
         end
      end
      else begin
         pe_valid <= run_valid; // RUN 상태일 때만 valid_in 반영

         case (state)
           S_LOAD: begin
              if (weight_valid) begin
                 // base index = weight_cnt * 5
                  weight_flat[weight_cnt*5 + 0] <= weight_in_4; // MSB → (row, col0)
      weight_flat[weight_cnt*5 + 1] <= weight_in_3; //        (row, col1)
      weight_flat[weight_cnt*5 + 2] <= weight_in_2; //        (row, col2)
      weight_flat[weight_cnt*5 + 3] <= weight_in_1; //        (row, col3)
      weight_flat[weight_cnt*5 + 4] <= weight_in_0; // LSB → (row, col4)

                 if (weight_cnt == 7'd19) begin
                    // 20번(5개씩) 다 채웠으면 RUN 상태로 전환
                    weight_cnt <= 7'd0;
                    state      <= S_RUN;
                 end
                 else begin
                    weight_cnt <= weight_cnt + 1'b1;
                 end
              end
           end

           S_RUN: begin
	      if (change_weight) begin
		 state <= S_LOAD;   // 다시 weight 로딩
	      end
	      else begin
                 state <= S_RUN;
	      end
           end

           default: begin
              state <= S_LOAD;
           end
         endcase
      end
   end

   // ================================
   //   ARRAY별 입력 버스 매핑
   // ================================
   // din_bus[ARRAY index][ROW index]
   wire [7:0] din_bus [0:3][0:4];

   // ARRAY 0
   assign din_bus[0][0] = din0_0;
   assign din_bus[0][1] = din0_1;
   assign din_bus[0][2] = din0_2;
   assign din_bus[0][3] = din0_3;
   assign din_bus[0][4] = din0_4;

   // ARRAY 1
   assign din_bus[1][0] = din1_0;
   assign din_bus[1][1] = din1_1;
   assign din_bus[1][2] = din1_2;
   assign din_bus[1][3] = din1_3;
   assign din_bus[1][4] = din1_4;

   // ARRAY 2
   assign din_bus[2][0] = din2_0;
   assign din_bus[2][1] = din2_1;
   assign din_bus[2][2] = din2_2;
   assign din_bus[2][3] = din2_3;
   assign din_bus[2][4] = din2_4;

   // ARRAY 3
   assign din_bus[3][0] = din3_0;
   assign din_bus[3][1] = din3_1;
   assign din_bus[3][2] = din3_2;
   assign din_bus[3][3] = din3_3;
   assign din_bus[3][4] = din3_4;

   // ================================
   //   입력 -> 각 ARRAY로 연결
   // ================================
   genvar a, r, c;
   generate
      for (a = 0; a < 4; a = a + 1) begin : ARRAY
         // 각 ARRAY는 5개의 row를 가짐
         // global_row = a*5 + r (0~19 중 일부)
         for (r = 0; r < 5; r = r + 1) begin : ROW

            // 각 row의 0번째 열에 ARRAY별/row별 입력 연결
            assign pe_din[a*5 + r][0] = din_bus[a][r];

            for (c = 0; c < 5; c = c + 1) begin : COL
               // global index 계산
               // weight / mul_res index: a*25 + r*5 + c (0~99)
               // pe_din index: [a*5 + r][c]
               PE u_pe (
                        .clk      (clk),
                        .rst_n    (rst_n),
                        .valid_in (run_valid),                 // LOAD 상태일 땐 0
                        .din      (pe_din[a*5 + r][c]),
                        .weight   (weight_flat[a*25 + r*5 + c]),
                        .dout     (pe_din[a*5 + r][c+1]),
                        .p_out    (mul_res[a*25 + r*5 + c])
			);
            end
         end
      end
   endgenerate

   // ================================
   //   Adder Tree 4개 (ARRAY당 1개)
   // ================================
   reg pe_valid_1, pe_valid_2;
   always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
	 pe_valid_1 <= 0;
	 pe_valid_2 <= 0;
      end
      else begin
	 pe_valid_1 <= pe_valid;
	 pe_valid_2 <= pe_valid_1;
      end
   end
   
   genvar at;
   generate
      for (at = 0; at < 4; at = at + 1) begin : ADDER_TREE_BLOCK
         adder_tree u_adder_tree (
				  .clk       (clk),
				  .rst_n     (rst_n),
				  .valid_in  (pe_valid_2),   // RUN 상태에서만 유효

				  .p_in_00   (mul_res[at*25 + 0 ]), 
				  .p_in_01   (mul_res[at*25 + 1 ]), 
				  .p_in_02   (mul_res[at*25 + 2 ]), 
				  .p_in_03   (mul_res[at*25 + 3 ]), 
				  .p_in_04   (mul_res[at*25 + 4 ]),
				  .p_in_05   (mul_res[at*25 + 5 ]), 
				  .p_in_06   (mul_res[at*25 + 6 ]), 
				  .p_in_07   (mul_res[at*25 + 7 ]), 
				  .p_in_08   (mul_res[at*25 + 8 ]), 
				  .p_in_09   (mul_res[at*25 + 9 ]),
				  .p_in_10   (mul_res[at*25 + 10]), 
				  .p_in_11   (mul_res[at*25 + 11]), 
				  .p_in_12   (mul_res[at*25 + 12]), 
				  .p_in_13   (mul_res[at*25 + 13]), 
				  .p_in_14   (mul_res[at*25 + 14]),
				  .p_in_15   (mul_res[at*25 + 15]), 
				  .p_in_16   (mul_res[at*25 + 16]), 
				  .p_in_17   (mul_res[at*25 + 17]), 
				  .p_in_18   (mul_res[at*25 + 18]), 
				  .p_in_19   (mul_res[at*25 + 19]),
				  .p_in_20   (mul_res[at*25 + 20]), 
				  .p_in_21   (mul_res[at*25 + 21]), 
				  .p_in_22   (mul_res[at*25 + 22]), 
				  .p_in_23   (mul_res[at*25 + 23]), 
				  .p_in_24   (mul_res[at*25 + 24]),

				  .valid_out (valid_out_bus[at]),
				  .final_sum (final_result_bus[at])
				  );
      end
   endgenerate

endmodule
