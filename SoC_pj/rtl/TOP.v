module TOP  #(
	      parameter	S_DATA_WIDTH = 8, // AXIS 입력 폭
	      parameter	B_DATA_WIDTH = 40, // BRAM 데이터 폭 (5 x 8bit)
	      parameter	ADDR_WIDTH = 11, // BRAM depth = 2048 → addr 11bit
	      parameter	CONV_WEIGHTS = 1300, // conv 전체 weight 개수 (개)
	      parameter	FC_WEIGHTS = 1920  // fc  전체 weight 개수 (개)
	      )(
		input wire	   clk,
		input wire	   rst_n,
		input wire	   start,

		//input wire	   weight_count, // if 25 -> conv1_w

		input wire [7:0]   s_axis_tdata,
		input wire	   s_axis_tvalid,
		input wire	   s_axis_tlast, // (옵션)
		output reg	   s_axis_tready, // 항상 High (Reset 아닐 때)

		input wire [39:0]  weight_in,
		output wire	   weight_bram_en,
		output wire [8:0]  weight_bram_addr,

		output wire [7:0]  conv_bram_addr,
		output wire [31:0] conv_bram_din,
		output wire	   conv_bram_we, conv_bram_en,

		output reg [7:0]   conv2_in_bram_addr,
		input wire [31:0]  conv2_in_bram_din,
		output reg	   conv2_in_bram_en,

		output reg [8:0]   conv_addr,
		output reg [39:0]  conv_din,
		output reg	   conv_we,
		output reg	   conv_en,

		// fc BRAM Port A (40bit)
		output reg [8:0]   fc_addr,
		output reg [39:0]  fc_din,
		output reg	   fc_we,
		output reg	   fc_en,

		//
		output wire [8:0]  fc_w_addr,
		output wire [3:0]  decision,
		output wire	   decision_valid_out,
		output wire	   fc_w_bram_en,
		output wire [9:0]  decision_addr,// = image_count

		input wire [39:0]  fc_w_data_40,

		input wire [12:0]  axi_lite_addr_in,
		output wire [9:0]  axi_lite_addr_out,

		output wire [31:0] axi_lite_data,
		input wire [3:0]   axi_lite_bram


		);

   assign axi_lite_data = axi_lite_bram;

   assign axi_lite_addr_out = axi_lite_addr_in >> 2;
   

   localparam integer		   PACK_SIZE   = 5;
   localparam integer		   CONV_WORDS  = CONV_WEIGHTS / PACK_SIZE;   // 1300 / 5 = 260
   localparam integer		   FC_WORDS    = FC_WEIGHTS   / PACK_SIZE;   // 1920 / 5 = 384
   localparam integer		   TOTAL_WORDS = CONV_WORDS + FC_WORDS;      // 644


   localparam			   IDLE = 3'b000, CONV1_W = 3'b001, IMAGE_IN = 3'b010, CONV1_OUT = 3'b011, CONV2_W = 3'b100, CONV2_IN = 3'b101, DONE = 3'b110, WAIT_START = 3'b111;

   wire				   line_buffer_axis_tvalid;

   wire [7:0]			   c0_out_row0;
   wire [7:0]			   c0_out_row1;
   wire [7:0]			   c0_out_row2;
   wire [7:0]			   c0_out_row3;
   wire [7:0]			   c0_out_row4;

   wire [7:0]			   c2_out_row00;
   wire [7:0]			   c2_out_row01;
   wire [7:0]			   c2_out_row02;
   wire [7:0]			   c2_out_row03;
   wire [7:0]			   c2_out_row04;

   wire [7:0]			   c2_out_row10;
   wire [7:0]			   c2_out_row11;
   wire [7:0]			   c2_out_row12;
   wire [7:0]			   c2_out_row13;
   wire [7:0]			   c2_out_row14;

   wire [7:0]			   c2_out_row20;
   wire [7:0]			   c2_out_row21;
   wire [7:0]			   c2_out_row22;
   wire [7:0]			   c2_out_row23;
   wire [7:0]			   c2_out_row24;

   wire [7:0]			   c2_out_row30;
   wire [7:0]			   c2_out_row31;
   wire [7:0]			   c2_out_row32;
   wire [7:0]			   c2_out_row33;
   wire [7:0]			   c2_out_row34;

   wire [7:0]			   out_row00;
   wire [7:0]			   out_row01;
   wire [7:0]			   out_row02;
   wire [7:0]			   out_row03;
   wire [7:0]			   out_row04;

   wire [7:0]			   out_row10;
   wire [7:0]			   out_row11;
   wire [7:0]			   out_row12;
   wire [7:0]			   out_row13;
   wire [7:0]			   out_row14;

   wire [7:0]			   out_row20;
   wire [7:0]			   out_row21;
   wire [7:0]			   out_row22;
   wire [7:0]			   out_row23;
   wire [7:0]			   out_row24;

   wire [7:0]			   out_row30;
   wire [7:0]			   out_row31;
   wire [7:0]			   out_row32;
   wire [7:0]			   out_row33;
   wire [7:0]			   out_row34;

   wire				   conv2_line_valid0;
   wire				   conv2_line_valid1;
   wire				   conv2_line_valid2;
   wire				   conv2_line_valid3;

   wire				   out_valid;
   wire				   valid_out_0;

   wire [21:0]			   final_result_0;
   wire [21:0]			   final_result_1;
   wire [21:0]			   final_result_2;
   wire [21:0]			   final_result_3;

   wire				   weight_valid;
   wire				   done, busy;

   reg [2:0]			   state;
   //reg [10:0]			   weight_cnt;
   reg [9:0]			   image_cnt;
   reg				   weight_change;
   wire [39:0]			   weight_out;

   reg [3:0]			   wea_bram;
   reg				   rstn_bram;
   wire [6:0]			   weight_cnt_in_bram;

   wire [7:0]			   out0, out1, out2, out3;
   wire				   valid_out_requ;

   wire [7:0]			   out00_0, out01_0, out10_0, out11_0;
   wire				   pool_out_valid_0;
   wire [7:0]			   out00_1, out01_1, out10_1, out11_1;
   wire				   pool_out_valid_1;
   wire [7:0]			   out00_2, out01_2, out10_2, out11_2;
   wire				   pool_out_valid_2;
   wire [7:0]			   out00_3, out01_3, out10_3, out11_3;
   wire				   pool_out_valid_3;

   wire [7:0]			   max_dout_0, max_dout_1, max_dout_2, max_dout_3;
   wire				   max_out_0, max_out_1, max_out_2, max_out_3;

   wire				   frame_done;

   reg [3:0]			   kernel_cnt;
   reg [7:0]			   conv2_in_cnt;
   reg				   conv2_in_valid;

   reg				   buffer_sel;
   wire				   pe_valid;

   reg				   pooling_size_sel;

   wire				   conv1_to_bram_valid;
   wire				   valid_conv2_adder_in;
   wire				   requ_valid_in;

   wire				   conv2_add_valid;

   wire [21:0]			   conv2_add_out;

   wire [21:0]			   final_result_00;

   reg [9:0]			   image_counter;

   reg [39:0]			   word_data;
   reg [39:0]			   pack_reg;
   reg [2:0]			   pack_cnt;
   reg [9:0]			   word_cnt;
   reg [10:0]			   conv_addr_cnt;
   reg [10:0]			   fc_addr_cnt;
   reg				   load_done;

   
   wire				   valid_conv2_to_fc;

   
   
   assign line_buffer_axis_tvalid = (state == IMAGE_IN) ? s_axis_tvalid : 0;

   always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
	 wea_bram <= 0;
	 rstn_bram <= 0;
      end
      else begin
      end
   end

   always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
	 state <= IDLE;
	 //weight_cnt <= 0;
	 image_cnt <= 0;
	 weight_change <= 0;
	 s_axis_tready <= 0;
	 kernel_cnt <= 0;
	 conv2_in_cnt <= 0;
	 image_counter <= 0;

	 load_done     <= 0;

         pack_cnt      <= 0;
         pack_reg      <= 0;
         word_cnt      <= 0;

         conv_addr_cnt <= 0;
         fc_addr_cnt   <= 0;
         conv_addr     <= 0;
         fc_addr       <= 0;

         conv_din      <= 0;
         fc_din        <= 0;

         conv_we       <= 0;
         conv_en       <= 0;
         fc_we         <= 0;
         fc_en         <= 0;

	 conv2_in_bram_addr <= 0;
	 conv2_in_bram_en <= 0;
      end
      else begin
	 if(conv2_in_bram_en) begin
	    conv2_in_valid <= 1;
	 end
	 else begin
	    conv2_in_valid <= 0;
	 end
	 
	 case(state)
	   IDLE : begin
	      s_axis_tready <= 1'b1;
              if (s_axis_tvalid && s_axis_tready) begin
                 // 새 weight를 pack_reg의 MSB 쪽으로 밀어 넣어서
                 // [w4,w3,w2,w1,w0] 형태를 만들기 위한 시프트
                 word_data = {s_axis_tdata, pack_reg[B_DATA_WIDTH-1:S_DATA_WIDTH]};
                 pack_reg  <= word_data;

                 if (pack_cnt == PACK_SIZE-1) begin
                    // 5개 모였으니 BRAM에 한 word write
                    if (word_cnt < CONV_WORDS) begin
                       conv_en       <= 1'b1;
                       conv_we       <= 1'b1;
                       conv_din      <= word_data;      // ★ 이번 클럭에 완성된 40bit
                       conv_addr     <= conv_addr_cnt;
                       conv_addr_cnt <= conv_addr_cnt + 1'b1;
                    end
                    else begin
                       fc_en       <= 1'b1;
                       fc_we       <= 1'b1;
                       fc_din      <= word_data;
                       fc_addr     <= fc_addr_cnt;
                       fc_addr_cnt <= fc_addr_cnt + 1'b1;
                    end

                    word_cnt <= word_cnt + 1'b1;
                    pack_cnt <= 3'd0;

                    if (word_cnt == TOTAL_WORDS-1) begin
                       s_axis_tready <= 1'b0;
                       load_done     <= 1'b1;
                       state         <= WAIT_START;
                    end
                 end
                 else begin
                    // 아직 5개 안 모였으면 카운터만 증가
                    pack_cnt <= pack_cnt + 1'b1;
                 end
              end
	   end // case: IDLE
	   WAIT_START : begin
	      if(start) begin
		 state <= CONV1_W;
	      end
	   end
	   CONV1_W : begin
	      weight_change <= 1;
	      state <= IMAGE_IN;
	   end
	   IMAGE_IN : begin
	      weight_change <= 0;
	      buffer_sel <= 0;
	      pooling_size_sel <= 1;	      
	      if(image_cnt != 783) begin
		 s_axis_tready <= 1;
		 if(s_axis_tvalid && s_axis_tready) begin
		    image_cnt <= image_cnt + 1;
		 end		 
	      end
	      else begin
		 s_axis_tready <= 0;
		 image_cnt <= 0;
		 state <= CONV1_OUT;
	      end
	   end
	   CONV1_OUT : begin
	      if(frame_done) begin
		 state <= CONV2_W;
	      end
	      else begin
		 state <= state;
	      end
	   end
	   CONV2_W : begin
	      state <= CONV2_IN;
	      weight_change <= 1;
	      conv2_in_bram_en <= 1;
	   end
	   CONV2_IN : begin
	      pooling_size_sel <= 0;
	      //conv2_in_valid <= 1;
	      buffer_sel <= 1;
	      weight_change= 0;
	      if(conv2_in_cnt != 143) begin
		 conv2_in_bram_addr <= conv2_in_bram_addr + 1;
		 conv2_in_cnt <= conv2_in_cnt + 1;
	      end
	      else begin
		 conv2_in_bram_addr <= 0;
		 conv2_in_cnt <= 0;
		 conv2_in_bram_en <= 0;
		 //conv2_in_valid <= 0;
		 if(kernel_cnt != 11) begin
		    kernel_cnt <= kernel_cnt + 1;
		    state <= CONV2_W;
		 end
		 else begin
		    state <= DONE;
		    //conv2_in_bram_en <= 0;
		    //conv2_in_valid <= 0;
		    kernel_cnt <= 0;
		 end
	      end	      
	   end // case: CONV2_IN
	   DONE : begin
	      state <= WAIT_START;
	      if(image_counter != 999) begin
		 image_counter <= image_counter + 1;
	      end
	      else begin
		 image_counter <= 0;
	      end
	   end
	 endcase
      end
   end // always @ (posedge clk or negedge rst_n)

   reg pool_sel_0;
   reg pool_sel_1;
   reg pool_sel_2;
   reg pool_sel_3;
   reg pool_sel_4;
   reg pool_sel_5;
   reg pool_sel_6;
   reg pool_sel_7;
   reg pool_sel_8;
   reg pool_sel_9;
   reg pool_sel_10;
   reg pool_sel_11;
   reg pool_sel_12;
   reg pool_sel_13;
   reg pool_sel_14;
   reg pool_sel_15;
   reg pool_sel_16;
   reg pool_sel_17;
   

   always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
	 pool_sel_0 <= 1'b0;
         pool_sel_1 <= 1'b0;
         pool_sel_2 <= 1'b0;
         pool_sel_3 <= 1'b0;
         pool_sel_4 <= 1'b0;
         pool_sel_5 <= 1'b0;
         pool_sel_6 <= 1'b0;
         pool_sel_7 <= 1'b0;
         pool_sel_8 <= 1'b0;
         pool_sel_9 <= 1'b0;
	 pool_sel_10 <= 1'b0;
	 pool_sel_11 <= 1'b0;
	 pool_sel_12 <= 1'b0;
	 pool_sel_13 <= 1'b0;
	 pool_sel_14 <= 1'b0;
	 pool_sel_15 <= 1'b0;
	 pool_sel_16 <= 1'b0;
         pool_sel_17 <= 1'b0;
      end
      else begin
	 pool_sel_0 <= pooling_size_sel;
	 pool_sel_1 <= pool_sel_0;  // 0 → 1
         pool_sel_2 <= pool_sel_1;  // 1 → 2
         pool_sel_3 <= pool_sel_2;  // 2 → 3
         pool_sel_4 <= pool_sel_3;  // 3 → 4
         pool_sel_5 <= pool_sel_4;  // 4 → 5
         pool_sel_6 <= pool_sel_5;  // 5 → 6
         pool_sel_7 <= pool_sel_6;  // 6 → 7
         pool_sel_8 <= pool_sel_7;  // 7 → 8
         pool_sel_9 <= pool_sel_8;  // 8 → 9
	 pool_sel_10 <= pool_sel_9;  // 8 → 9
	 pool_sel_11 <= pool_sel_10;  // 8 → 9
	 pool_sel_12 <= pool_sel_11;  // 8 → 9
	 pool_sel_13 <= pool_sel_12;  // 8 → 9
	 pool_sel_14 <= pool_sel_13;  // 8 → 9
	 pool_sel_15 <= pool_sel_14;  // 8 → 9
	 pool_sel_16 <= pool_sel_15;  // 8 → 9
	 pool_sel_17 <= pool_sel_16;  // 8 → 9
      end
   end
   
   weight_bram_reader weight_bram_reader_0(
					   // Outputs
					   .bram_addr		(weight_bram_addr[8:0]),
					   .bram_en		(weight_bram_en),
					   .weight_out		(weight_out[39:0]),
					   .weight_valid	(weight_valid),
					   .done		(done),
					   .busy		(busy),
					   // Inputs
					   .clk			(clk),
					   .rst_n		(rst_n),
					   .start		(weight_change),
					   .bram_dout		(weight_in[39:0])
					   );
   
   
   
   linebuffer linebuffer_0(
			   // Outputs
			   //.s_axis_tready	(s_axis_tready),
			   .out_row0		(c0_out_row0[7:0]),
			   .out_row1		(c0_out_row1[7:0]),
			   .out_row2		(c0_out_row2[7:0]),
			   .out_row3		(c0_out_row3[7:0]),
			   .out_row4		(c0_out_row4[7:0]),
			   .out_valid		(out_valid),
			   // Inputs
			   .clk			(clk),
			   .rst_n		(rst_n),
			   .s_axis_tdata	(s_axis_tdata[7:0]),
			   .s_axis_tvalid	(line_buffer_axis_tvalid),
			   .s_axis_tlast	(s_axis_tlast),
			   .s_axis_tready(s_axis_tready));

   wire [7:0] conv2_in_bram_din_0;
   wire [7:0] conv2_in_bram_din_1;
   wire [7:0] conv2_in_bram_din_2;
   wire [7:0] conv2_in_bram_din_3;

   assign conv2_in_bram_din_0 = conv2_in_bram_din[7:0];
   assign conv2_in_bram_din_1 = conv2_in_bram_din[15:8];
   assign conv2_in_bram_din_2 = conv2_in_bram_din[23:16];
   assign conv2_in_bram_din_3 = conv2_in_bram_din[31:24];
   
   linebuffer_conv2 line_conv2_0(
				 // Outputs
				 .out_row0		(c2_out_row00[7:0]),
				 .out_row1		(c2_out_row01[7:0]),
				 .out_row2		(c2_out_row02[7:0]),
				 .out_row3		(c2_out_row03[7:0]),
				 .out_row4		(c2_out_row04[7:0]),
				 .out_valid		(conv2_line_valid0),
				 // Inputs
				 .clk			(clk),
				 .rst_n		(rst_n),
				 .in_data		(conv2_in_bram_din_0[7:0]),
				 .in_valid		(conv2_in_valid));
   linebuffer_conv2 line_conv2_1(
				 // Outputs
				 .out_row0		(c2_out_row10[7:0]),
				 .out_row1		(c2_out_row11[7:0]),
				 .out_row2		(c2_out_row12[7:0]),
				 .out_row3		(c2_out_row13[7:0]),
				 .out_row4		(c2_out_row14[7:0]),
				 .out_valid		(conv2_line_valid1),
				 // Inputs
				 .clk			(clk),
				 .rst_n			(rst_n),
				 .in_data		(conv2_in_bram_din_1[7:0]),
				 .in_valid		(conv2_in_valid));
   linebuffer_conv2 line_conv2_2(
				 // Outputs
				 .out_row0		(c2_out_row20[7:0]),
				 .out_row1		(c2_out_row21[7:0]),
				 .out_row2		(c2_out_row22[7:0]),
				 .out_row3		(c2_out_row23[7:0]),
				 .out_row4		(c2_out_row24[7:0]),
				 .out_valid		(conv2_line_valid2),
				 // Inputs
				 .clk			(clk),
				 .rst_n			(rst_n),
				 .in_data		(conv2_in_bram_din_2[7:0]),
				 .in_valid		(conv2_in_valid));
   linebuffer_conv2 line_conv2_3(
				 // Outputs
				 .out_row0		(c2_out_row30[7:0]),
				 .out_row1		(c2_out_row31[7:0]),
				 .out_row2		(c2_out_row32[7:0]),
				 .out_row3		(c2_out_row33[7:0]),
				 .out_row4		(c2_out_row34[7:0]),
				 .out_valid		(conv2_line_valid3),
				 // Inputs
				 .clk			(clk),
				 .rst_n			(rst_n),
				 .in_data		(conv2_in_bram_din_3[7:0]),
				 .in_valid		(conv2_in_valid));


   assign out_row00 = (!pool_sel_13) ? c2_out_row00 : c0_out_row0;
   assign out_row01 = (!pool_sel_13) ? c2_out_row01 : c0_out_row1;
   assign out_row02 = (!pool_sel_13) ? c2_out_row02 : c0_out_row2;
   assign out_row03 = (!pool_sel_13) ? c2_out_row03 : c0_out_row3;
   assign out_row04 = (!pool_sel_13) ? c2_out_row04 : c0_out_row4;

   assign out_row10 = (!pool_sel_13) ? c2_out_row10 : c0_out_row0;
   assign out_row11 = (!pool_sel_13) ? c2_out_row11 : c0_out_row1;
   assign out_row12 = (!pool_sel_13) ? c2_out_row12 : c0_out_row2;
   assign out_row13 = (!pool_sel_13) ? c2_out_row13 : c0_out_row3;
   assign out_row14 = (!pool_sel_13) ? c2_out_row14 : c0_out_row4;

   assign out_row20 = (!pool_sel_13) ? c2_out_row20 : c0_out_row0;
   assign out_row21 = (!pool_sel_13) ? c2_out_row21 : c0_out_row1;
   assign out_row22 = (!pool_sel_13) ? c2_out_row22 : c0_out_row2;
   assign out_row23 = (!pool_sel_13) ? c2_out_row23 : c0_out_row3;
   assign out_row24 = (!pool_sel_13) ? c2_out_row24 : c0_out_row4;

   assign out_row30 = (!pool_sel_13) ? c2_out_row30 : c0_out_row0;
   assign out_row31 = (!pool_sel_13) ? c2_out_row31 : c0_out_row1;
   assign out_row32 = (!pool_sel_13) ? c2_out_row32 : c0_out_row2;
   assign out_row33 = (!pool_sel_13) ? c2_out_row33 : c0_out_row3;
   assign out_row34 = (!pool_sel_13) ? c2_out_row34 : c0_out_row4;

   assign pe_valid = (!pool_sel_13) ? conv2_line_valid0 : out_valid;

   PE_array_top PE_array_top_0(
			       // Outputs
			       .valid_out_0	(valid_out_0),
			       .valid_out_1	(valid_out_1),
			       .valid_out_2	(valid_out_2),
			       .valid_out_3	(valid_out_3),
			       .final_result_0	(final_result_0[21:0]),
			       .final_result_1	(final_result_1[21:0]),
			       .final_result_2	(final_result_2[21:0]),
			       .final_result_3	(final_result_3[21:0]),
			       // Inputs
			       .clk		(clk),
			       .rst_n		(rst_n),
			       .valid_in	(pe_valid),
			       .din0_0		(out_row04[7:0]),
			       .din0_1		(out_row03[7:0]),
			       .din0_2		(out_row02[7:0]),
			       .din0_3		(out_row01[7:0]),
			       .din0_4		(out_row00[7:0]),
			       .din1_0		(out_row14[7:0]),
			       .din1_1		(out_row13[7:0]),
			       .din1_2		(out_row12[7:0]),
			       .din1_3		(out_row11[7:0]),
			       .din1_4		(out_row10[7:0]),
			       .din2_0		(out_row24[7:0]),
			       .din2_1		(out_row23[7:0]),
			       .din2_2		(out_row22[7:0]),
			       .din2_3		(out_row21[7:0]),
			       .din2_4		(out_row20[7:0]),
			       .din3_0		(out_row34[7:0]),
			       .din3_1		(out_row33[7:0]),
			       .din3_2		(out_row32[7:0]),
			       .din3_3		(out_row31[7:0]),
			       .din3_4		(out_row30[7:0]),
			       .weight_in	(weight_out[39:0]),
			       .weight_valid	(weight_valid),
			       .change_weight	(weight_change),
			       .weight_cnt(weight_cnt_in_bram[6:0]));


   assign valid_conv2_adder_in = (!pool_sel_13) && valid_out_0; //conv2

   assign requ_valid_in = (!pool_sel_13) ? conv2_add_valid : valid_out_0;

   assign final_result_00 = (!pool_sel_13) ? conv2_add_out : final_result_0;

   conv2_adder_tree conv2_adder_tree_0(
				       // Outputs
				       .conv2_add_out	(conv2_add_out[21:0]),
				       .conv2_add_valid	(conv2_add_valid),
				       // Inputs
				       .clk		(clk),
				       .rst_n		(rst_n),
				       .in_valid	(valid_conv2_adder_in),
				       .in0		(final_result_0[21:0]),
				       .in1		(final_result_1[21:0]),
				       .in2		(final_result_2[21:0]),
				       .in3		(final_result_3[21:0]));

   requantization requantization_0(
				   // Outputs
				   .valid_out		(valid_out_requ),
				   .out0		(out0[7:0]),
				   .out1		(out1[7:0]),
				   .out2		(out2[7:0]),
				   .out3		(out3[7:0]),
				   // Inputs
				   .clk			(clk),
				   .rst_n		(rst_n),
				   .valid_in		(requ_valid_in), //from pe_array_top
				   .in0			(final_result_00[21:0]),
				   .in1			(final_result_1[21:0]),
				   .in2			(final_result_2[21:0]),
				   .in3			(final_result_3[21:0]));
   
   pooling_line_buffer pooling_0(
				 // Outputs
				 .out00			(out00_0[7:0]),
				 .out01			(out01_0[7:0]),
				 .out10			(out10_0[7:0]),
				 .out11			(out11_0[7:0]),
				 .out_valid		(pool_out_valid_0),
				 // Inputs
				 .clk			(clk),
				 .rst_n			(rst_n),
				 .size_sel              (pooling_size_sel),
				 .in_data		(out0[7:0]),
				 .in_valid		(valid_out_requ));
   
   pooling_line_buffer pooling_1(
				 // Outputs
				 .out00			(out00_1[7:0]),
				 .out01			(out01_1[7:0]),
				 .out10			(out10_1[7:0]),
				 .out11			(out11_1[7:0]),
				 .out_valid		(pool_out_valid_1),
				 // Inputs
				 .clk			(clk),
				 .rst_n			(rst_n),
				 .size_sel              (pooling_size_sel),
				 .in_data		(out1[7:0]),
				 .in_valid		(valid_out_requ));
   pooling_line_buffer pooling_2(
				 // Outputs
				 .out00			(out00_2[7:0]),
				 .out01			(out01_2[7:0]),
				 .out10			(out10_2[7:0]),
				 .out11			(out11_2[7:0]),
				 .out_valid		(pool_out_valid_2),
				 // Inputs
				 .clk			(clk),
				 .rst_n			(rst_n),
				 .size_sel              (pooling_size_sel),
				 .in_data		(out2[7:0]),
				 .in_valid		(valid_out_requ));
   pooling_line_buffer pooling_3(
				 // Outputs
				 .out00			(out00_3[7:0]),
				 .out01			(out01_3[7:0]),
				 .out10			(out10_3[7:0]),
				 .out11			(out11_3[7:0]),
				 .out_valid		(pool_out_valid_3),
				 // Inputs
				 .clk			(clk),
				 .rst_n			(rst_n),
				 .size_sel              (pooling_size_sel),
				 .in_data		(out3[7:0]),
				 .in_valid		(valid_out_requ));

   maxpooling maxpooling_0(
			   // Outputs
			   .dout		(max_dout_0[7:0]),
			   .out_valid		(max_out_0),
			   // Inputs
			   .clk			(clk),
			   .rst_n		(rst_n),
			   .din0		(out00_0[7:0]),
			   .din1		(out01_0[7:0]),
			   .din2		(out10_0[7:0]),
			   .din3		(out11_0[7:0]),
			   .in_valid		(pool_out_valid_0));
   maxpooling maxpooling_1(
			   // Outputs
			   .dout		(max_dout_1[7:0]),
			   .out_valid		(max_out_1),
			   // Inputs
			   .clk			(clk),
			   .rst_n		(rst_n),
			   .din0		(out00_1[7:0]),
			   .din1		(out01_1[7:0]),
			   .din2		(out10_1[7:0]),
			   .din3		(out11_1[7:0]),
			   .in_valid		(pool_out_valid_1));
   maxpooling maxpooling_2(
			   // Outputs
			   .dout		(max_dout_2[7:0]),
			   .out_valid		(max_out_2),
			   // Inputs
			   .clk			(clk),
			   .rst_n		(rst_n),
			   .din0		(out00_2[7:0]),
			   .din1		(out01_2[7:0]),
			   .din2		(out10_2[7:0]),
			   .din3		(out11_2[7:0]),
			   .in_valid		(pool_out_valid_2));
   maxpooling maxpooling_3(
			   // Outputs
			   .dout		(max_dout_3[7:0]),
			   .out_valid		(max_out_3),
			   // Inputs
			   .clk			(clk),
			   .rst_n		(rst_n),
			   .din0		(out00_3[7:0]),
			   .din1		(out01_3[7:0]),
			   .din2		(out10_3[7:0]),
			   .din3		(out11_3[7:0]),
			   .in_valid		(pool_out_valid_3));

   assign valid_conv2_to_fc = max_out_0 && (~pool_sel_17);


   //assign conv1_to_bram_valid = max_out_0 && pooling_size_sel;
   assign conv1_to_bram_valid = max_out_0 && pool_sel_13;

   wire [7:0] conv_bram_din_0;
   wire [7:0] conv_bram_din_1;
   wire [7:0] conv_bram_din_2;
   wire [7:0] conv_bram_din_3;

   assign conv_bram_din[ 7: 0] = conv_bram_din_0[7:0];
assign conv_bram_din[15: 8] = conv_bram_din_1[7:0];
assign conv_bram_din[23:16] = conv_bram_din_2[7:0];
assign conv_bram_din[31:24] = conv_bram_din_3[7:0];

   conv1_out_to_bram conv1_bram_0(
				  // Outputs
				  .bram_addr		(conv_bram_addr[7:0]),
				  .bram_din_0		(conv_bram_din_0[7:0]),
				  .bram_din_1		(conv_bram_din_1[7:0]),
				  .bram_din_2		(conv_bram_din_2[7:0]),
				  .bram_din_3		(conv_bram_din_3[7:0]),
				  .bram_we		(conv_bram_we),
				  .bram_en		(conv_bram_en),
				  .frame_done		(frame_done),
				  // Inputs
				  .clk			(clk),
				  .rst_n		(rst_n),
				  .din_0		(max_dout_0[7:0]),
				  .din_1		(max_dout_1[7:0]),
				  .din_2		(max_dout_2[7:0]),
				  .din_3		(max_dout_3[7:0]),
				  .in_valid		(conv1_to_bram_valid));

   FC_comparator_top fc_comparator(
				   // Outputs
				   .w_addr		(fc_w_addr[8:0]),
				   .decision		(decision[3:0]),
				   .valid_out		(decision_valid_out),
				   .fc_w_bram_en	(fc_w_bram_en),
				   .decision_addr	(decision_addr[9:0]),
				   // Inputs
				   .clk			(clk),
				   .rst_n		(rst_n),
				   .fc_valid_in		(valid_conv2_to_fc),
				   .data_in		(max_dout_0[7:0]),
				   .w_data_40		(fc_w_data_40[39:0]));

   

   
   

   
   
   
endmodule // TOP
