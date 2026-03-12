`timescale 1ns/1ps

module TOP_GEMM (
		  input		CLK,
		  input		RESET,

		  output [31:0]	PC,
		  output [31:0]	DATAADR,
		  output [31:0]	WRITEDATA,
		  output	MEMWRITE,

		  input		IMEM_DBG_EN,
		  input [31:0]	IMEM_DBG_A,
		  output [31:0]	IMEM_DBG_RD,

		  input		DMEM_DBG_EN,
		  input		DMEM_DBG_WE,
		  input [31:0]	DMEM_DBG_A,
		  input [31:0]	DMEM_DBG_WD,
		  output [31:0]	DMEM_DBG_RD
		 );
   wire [31:0] PCF;
   wire [31:0] INSTRF;
   wire [31:0] ALURESULTM;
   wire [31:0] WRITEDATAM;
   wire	       MEMWRITEM;
   wire [31:0] READDATAM;

   RISCV_CPU CPU(// Outputs
		 .PCF			(PCF[31:0]),
		 .ALURESULTM		(ALURESULTM[31:0]),
		 .WRITEDATAM		(WRITEDATAM[31:0]),
		 .MEMWRITEM		(MEMWRITEM),
		 // Inputs
		 .CLK			(CLK),
		 .RESET			(RESET),
		 .INSTRF		(INSTRF[31:0]),
		 .READDATAM		(READDATAM[31:0]));
   IMEM IMEM(// Outputs
	     .RD			(INSTRF[31:0]),
	     .DBG_RD			(IMEM_DBG_RD[31:0]),
	     // Inputs
	     .A				(PCF[31:0]),
	     .DBG_EN			(IMEM_DBG_EN),
	     .DBG_A			(IMEM_DBG_A[31:0]));
   DMEM DMEM(// Outputs
	     .RD			(READDATAM[31:0]),
	     .DBG_RD			(DMEM_DBG_RD[31:0]),
	     // Inputs
	     .CLK			(CLK),
	     .WE			(MEMWRITEM),
	     .A				(ALURESULTM[31:0]),
	     .WD			(WRITEDATAM[31:0]),
	     .DBG_EN			(DMEM_DBG_EN),
	     .DBG_WE			(DMEM_DBG_WE),
	     .DBG_A			(DMEM_DBG_A[31:0]),
	     .DBG_WD			(DMEM_DBG_WD[31:0]));

   assign PC = PCF;
   assign DATAADR = ALURESULTM;
   assign WRITEDATA = WRITEDATAM;
   assign MEMWRITE = MEMWRITEM;
   
endmodule // TOP_GEMM
