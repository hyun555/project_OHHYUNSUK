module RISCV_CPU (/*AUTOARG*/
   // Outputs
   PCF, ALURESULTM, WRITEDATAM, MEMWRITEM,
   // Inputs
   CLK, RESET, INSTRF, READDATAM
   );
   input  CLK, RESET;

   // Instruction Memory
   output [31:0] PCF;
   input [31:0]	 INSTRF;

   // Data Memory
   output [31:0] ALURESULTM;
   output [31:0] WRITEDATAM;
   output	 MEMWRITEM;
   input [31:0]	 READDATAM;

   wire		 REGWRITED, MEMWRITED, JUMPD, BRANCHD, ALUSRCD;
   wire [1:0]	 RESULTSRCD, IMMSRCD;
   wire [1:0]	 ALUCONTROLD;

   wire [31:0]	 INSTRD;
   wire		 ZEROE;

   wire		 STALLF, STALLD;
   wire		 FLUSHD, FLUSHE;
   wire [1:0]	 FORWARDAE, FORWARDBE;

   wire [4:0]	 RS1D, RS2D, RS1E, RS2E, RDE, RDM, RDW;
   
   wire		 REGWRITEM, REGWRITEW;
   wire		 RESULTSRCE0;
   wire		 PCSRCE;

   CONTROLLER CTRL(// Outputs
		   .REGWRITED		(REGWRITED),
		   .MEMWRITED		(MEMWRITED),
		   .JUMPD		(JUMPD),
		   .BRANCHD		(BRANCHD),
		   .ALUSRCD		(ALUSRCD),
		   .RESULTSRCD		(RESULTSRCD[1:0]),
		   .IMMSRCD		(IMMSRCD[1:0]),
		   .ALUCONTROLD		(ALUCONTROLD[1:0]),
		   // Inputs
		   .OP			(INSTRD[6:0]),
		   .FUNCT3		(INSTRD[14:12]),
		   .FUNCT7B0		(INSTRD[25]),
		   .FUNCT7B5		(INSTRD[30]));
		   
   DATAPATH DP(// Outputs
	       .PCF			(PCF[31:0]),
	       .ALURESULTM		(ALURESULTM[31:0]),
	       .WRITEDATAM		(WRITEDATAM[31:0]),
	       .MEMWRITEM		(MEMWRITEM),
	       .INSTRD			(INSTRD[31:0]),
	       .ZEROE			(ZEROE),
	       .RS1D			(RS1D[4:0]),
	       .RS2D			(RS2D[4:0]),
	       .RS1E			(RS1E[4:0]),
	       .RS2E			(RS2E[4:0]),
	       .RDE			(RDE[4:0]),
	       .RDM			(RDM[4:0]),
	       .RDW			(RDW[4:0]),
	       .REGWRITEM		(REGWRITEM),
	       .REGWRITEW		(REGWRITEW),
	       .RESULTSRCE0		(RESULTSRCE0),
	       .PCSRCE			(PCSRCE),
	       // Inputs
	       .CLK			(CLK),
	       .RESET			(RESET),
	       .INSTRF			(INSTRF[31:0]),
	       .READDATAM		(READDATAM[31:0]),
	       .REGWRITED		(REGWRITED),
	       .RESULTSRCD		(RESULTSRCD[1:0]),
	       .MEMWRITED		(MEMWRITED),
	       .JUMPD			(JUMPD),
	       .BRANCHD			(BRANCHD),
	       .ALUCONTROLD		(ALUCONTROLD[1:0]),
	       .ALUSRCD			(ALUSRCD),
	       .IMMSRCD			(IMMSRCD[1:0]),
	       .STALLF			(STALLF),
	       .STALLD			(STALLD),
	       .FLUSHD			(FLUSHD),
	       .FLUSHE			(FLUSHE),
	       .FORWARDAE		(FORWARDAE[1:0]),
	       .FORWARDBE		(FORWARDBE[1:0]));
   HAZARD_UNIT HU(/*AUTOINST*/
		  // Outputs
		  .FORWARDAE		(FORWARDAE[1:0]),
		  .FORWARDBE		(FORWARDBE[1:0]),
		  .STALLF		(STALLF),
		  .STALLD		(STALLD),
		  .FLUSHD		(FLUSHD),
		  .FLUSHE		(FLUSHE),
		  // Inputs
		  .RS1E			(RS1E[4:0]),
		  .RS2E			(RS2E[4:0]),
		  .RDM			(RDM[4:0]),
		  .RDW			(RDW[4:0]),
		  .REGWRITEM		(REGWRITEM),
		  .REGWRITEW		(REGWRITEW),
		  .RS1D			(RS1D),
		  .RS2D			(RS2D),
		  .RDE			(RDE[4:0]),
		  .RESULTSRCE0		(RESULTSRCE0),
		  .PCSRCE		(PCSRCE));
   
endmodule // RISCV_CPU
