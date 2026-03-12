module CONTROLLER (/*AUTOARG*/
   // Outputs
   REGWRITED, MEMWRITED, JUMPD, BRANCHD, ALUSRCD, RESULTSRCD, IMMSRCD,
   ALUCONTROLD,
   // Inputs
   OP, FUNCT3, FUNCT7B0, FUNCT7B5
   );
   input [6:0] OP; // Instruction Opcode
   input [2:0] FUNCT3; // Instruction Funct3
   input       FUNCT7B0, FUNCT7B5; // Bit 5 of Funct7 (Used for SUB Detection + *MUL Detection*)

   output      REGWRITED, MEMWRITED,
         JUMPD, BRANCHD, ALUSRCD;
   output [1:0]	RESULTSRCD, IMMSRCD;
   output [1:0] ALUCONTROLD;

   wire [1:0]   ALUOP;

   // Main Decoder: Generates High-Level Control Signalgs from Opcode
   MAINDEC MD(/*AUTOINST*/
	      // Outputs
	      .REGWRITED		(REGWRITED),
	      .MEMWRITED		(MEMWRITED),
	      .JUMPD			(JUMPD),
	      .BRANCHD			(BRANCHD),
	      .ALUSRCD			(ALUSRCD),
	      .RESULTSRCD		(RESULTSRCD[1:0]),
	      .IMMSRCD			(IMMSRCD[1:0]),
	      .ALUOP			(ALUOP[1:0]),
	      // Inputs
	      .OP			(OP[6:0])
	      );

   // ALU Decoder: Generates Exact ALU Operation from Funct3/Funct7 and ALUOP
   ALUDEC AD(// Outputs
	     .ALUCONTROLD		(ALUCONTROLD[1:0]),
	     // Inputs
	     .OPB5			(OP[5]),
	     .FUNCT3			(FUNCT3[2:0]),
	     .FUNCT7B0			(FUNCT7B0),
	     .FUNCT7B5			(FUNCT7B5),
	     .ALUOP			(ALUOP[1:0])
	     );
   
endmodule // CONTROLLER
