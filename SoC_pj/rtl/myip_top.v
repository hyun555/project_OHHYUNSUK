`timescale 1ns/1ps

module myip_v1_0 #(
		   // Parameters of Axi Slave Bus Interface S00_AXI
		   parameter integer C_S00_AXI_DATA_WIDTH = 32,
		   parameter integer C_S00_AXI_ADDR_WIDTH = 6,
		   parameter [31:0]  OUTPUT_ADDRESS = 32'h0000_0000,

		   
	     parameter DATA_WIDTH = 16, // mul_vec 한 lane 폭 (16비트)
	     parameter NUM_LANE = 16,
	     parameter W_DATA_WIDTH = 256,
	     parameter DIM_WIDTH = 32,
		   parameter ADDR_WIDTH = 10
	     
		   )(
		     // Ports of Axi Slave Bus Interface S00_AXI
		     input wire				       s00_axi_aclk,
		     input wire				       s00_axi_aresetn,
		     input wire [C_S00_AXI_ADDR_WIDTH-1:0]     s00_axi_awaddr,
		     input wire [2:0]			       s00_axi_awprot,
		     input wire				       s00_axi_awvalid,
		     output wire			       s00_axi_awready,
		     input wire [C_S00_AXI_DATA_WIDTH-1:0]     s00_axi_wdata,
		     input wire [(C_S00_AXI_DATA_WIDTH/8)-1:0] s00_axi_wstrb,
		     input wire				       s00_axi_wvalid,
		     output wire			       s00_axi_wready,
		     output wire [1:0]			       s00_axi_bresp,
		     output wire			       s00_axi_bvalid,
		     input wire				       s00_axi_bready,
		     input wire [C_S00_AXI_ADDR_WIDTH-1:0]     s00_axi_araddr,
		     input wire [2:0]			       s00_axi_arprot,
		     input wire				       s00_axi_arvalid,
		     output wire			       s00_axi_arready,
		     output wire [C_S00_AXI_DATA_WIDTH-1:0]    s00_axi_rdata,
		     output wire [1:0]			       s00_axi_rresp,
		     output wire			       s00_axi_rvalid,
		     input wire				       s00_axi_rready,

		     
		//input wire	   weight_count, // if 25 -> conv1_w

		input wire [7:0]   s_axis_tdata,
		input wire	   s_axis_tvalid,
		input wire	   s_axis_tlast, // (옵션)
		output wire	   s_axis_tready, // 항상 High (Reset 아닐 때)

		input wire [39:0]  weight_in,
		output wire	   weight_bram_en,
		output wire [8:0] weight_bram_addr,

		output wire [7:0]  conv_bram_addr,
		output wire [31:0]   conv_bram_din,
		output wire	   conv_bram_we, conv_bram_en,

		output wire [7:0]   conv2_in_bram_addr,
		input wire [31:0]   conv2_in_bram_din,
		output wire	   conv2_in_bram_en,

		output wire [8:0]  conv_addr,
		output wire [39:0]  conv_din,
		output wire	   conv_we,
		output wire	   conv_en,

		// fc BRAM Port A (40bit)
		output wire [8:0]  fc_addr,
		output wire [39:0]  fc_din,
		output wire	   fc_we,
		output wire	   fc_en,

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

   // ============================================================================
   // AXI-Lite slave wrapper
   // ============================================================================
   wire							       start;
   //wire [9:0]						       decision_addr;

   myip_v1_0_S00_AXI #(
		       .C_S_AXI_DATA_WIDTH(C_S00_AXI_DATA_WIDTH),
		       .C_S_AXI_ADDR_WIDTH(C_S00_AXI_ADDR_WIDTH)
		       ) myip_v1_0_S00_AXI_inst (
						 .S_AXI_ACLK   (s00_axi_aclk),
						 .S_AXI_ARESETN(s00_axi_aresetn),
						 .S_AXI_AWADDR (s00_axi_awaddr),
						 .S_AXI_AWPROT (s00_axi_awprot),
						 .S_AXI_AWVALID(s00_axi_awvalid),
						 .S_AXI_AWREADY(s00_axi_awready),
						 .S_AXI_WDATA  (s00_axi_wdata),
						 .S_AXI_WSTRB  (s00_axi_wstrb),
						 .S_AXI_WVALID (s00_axi_wvalid),
						 .S_AXI_WREADY (s00_axi_wready),
						 .S_AXI_BRESP  (s00_axi_bresp),
						 .S_AXI_BVALID (s00_axi_bvalid),
						 .S_AXI_BREADY (s00_axi_bready),
						 .S_AXI_ARADDR (s00_axi_araddr),
						 .S_AXI_ARPROT (s00_axi_arprot),
						 .S_AXI_ARVALID(s00_axi_arvalid),
						 .S_AXI_ARREADY(s00_axi_arready),
						 .S_AXI_RDATA  (s00_axi_rdata),
						 .S_AXI_RRESP  (s00_axi_rresp),
						 .S_AXI_RVALID (s00_axi_rvalid),
						 .S_AXI_RREADY (s00_axi_rready),

						 // custom control
						 .start(start),
						 .decision_addr(decision_addr[9:0])   
						 );

   TOP TOP_0(
	     // Outputs
	     .s_axis_tready		(s_axis_tready),
	     .weight_bram_en		(weight_bram_en),
	     .weight_bram_addr		(weight_bram_addr[8:0]),
	     .conv_bram_addr		(conv_bram_addr[7:0]),
	     .conv_bram_din		(conv_bram_din[31:0]),
	     .conv_bram_we		(conv_bram_we),
	     .conv_bram_en		(conv_bram_en),
	     .conv2_in_bram_addr	(conv2_in_bram_addr[7:0]),
	     .conv2_in_bram_en		(conv2_in_bram_en),
	     .conv_addr			(conv_addr[8:0]),
	     .conv_din			(conv_din[39:0]),
	     .conv_we			(conv_we),
	     .conv_en			(conv_en),
	     .fc_addr			(fc_addr[8:0]),
	     .fc_din			(fc_din[39:0]),
	     .fc_we			(fc_we),
	     .fc_en			(fc_en),
	     .fc_w_addr			(fc_w_addr[8:0]),
	     .decision			(decision[3:0]),
	     .decision_valid_out	(decision_valid_out),
	     .fc_w_bram_en		(fc_w_bram_en),
	     .decision_addr		(decision_addr[9:0]),
	     .axi_lite_addr_out(axi_lite_addr_out),
	     .axi_lite_data(axi_lite_data),
	     // Inputs
	     .clk			(s00_axi_aclk),
	     .rst_n			(s00_axi_aresetn),
	     .start			(start),
	     //.weight_count		(weight_count),
	     .s_axis_tdata		(s_axis_tdata[7:0]),
	     .s_axis_tvalid		(s_axis_tvalid),
	     .s_axis_tlast		(s_axis_tlast),
	     .weight_in			(weight_in[39:0]),
	     .conv2_in_bram_din	(conv2_in_bram_din[31:0]),
	     .fc_w_data_40		(fc_w_data_40[39:0]),
	     .axi_lite_addr_in(axi_lite_addr_in),
	     .axi_lite_bram(axi_lite_bram));
   // Outputs
   

endmodule
