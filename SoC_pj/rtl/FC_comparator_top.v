module FC_comparator_top #(
		       parameter INPUT_NUM = 192,
		       parameter OUTPUT_NUM = 10,
		       parameter IN_BITS = 8,
		       parameter W_BITS = 8,
		       parameter ACC_BITS = 32,
		       parameter OUT_BITS = 32, // comparator? ?? ??
		       parameter WE_ADDR_WIDTH = 9, // depth 384 ? addr 0~383
		       parameter DATA_BITS = 32, // FC ?? ??? 32
		       parameter NUM_CLASS = 10
		       )(
			 input				 clk, rst_n,
			 input				 fc_valid_in,
			 input wire [IN_BITS-1:0]	 data_in,
			 input wire [39:0]		 w_data_40,

			 output wire [WE_ADDR_WIDTH-1:0] w_addr,
			 output wire [3:0]		 decision,
			 output wire			 valid_out,
			 output reg			 fc_w_bram_en,
			 output wire [9:0]		 decision_addr
			 );

   wire							 res_valid;
   wire [31:0]						 res_data;

   always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
	 fc_w_bram_en <= 0;
      end
      else begin
	 fc_w_bram_en <= 1;
      end
   end

   
   
   FC FC_0(
	   // Outputs
	   .w_addr			(w_addr[WE_ADDR_WIDTH-1:0]),
	   .res_valid			(res_valid),
	   .res_data			(res_data[OUT_BITS-1:0]),
	   .fc_done			(fc_done),
	   // Inputs
	   .clk				(clk),
	   .rst_n			(rst_n),
	   .valid_in			(fc_valid_in),
	   .data_in			(data_in[IN_BITS-1:0]),
	   .w_data_40			(w_data_40[39:0]));
   
   comparator com_0(
		    // Outputs
		    .decision		(decision[3:0]),
		    .valid_out		(valid_out),
		    .decision_addr (decision_addr),
		    // Inputs
		    .clk		(clk),
		    .rst_n		(rst_n),
		    .valid_in		(res_valid),
		    .data_in		(res_data[DATA_BITS-1:0]));

endmodule
