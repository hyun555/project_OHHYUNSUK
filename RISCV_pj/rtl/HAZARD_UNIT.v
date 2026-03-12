module HAZARD_UNIT (/*AUTOARG*/
		    // Outputs
		    FORWARDAE, FORWARDBE, STALLF, STALLD, FLUSHD, FLUSHE,
		    // Inputs
		    RS1E, RS2E, RDM, RDW, REGWRITEM, REGWRITEW, RS1D, RS2D, RDE,
		    RESULTSRCE0, PCSRCE
		    );
   // Forwarding Logic Input
   input [4:0] RS1E, RS2E;
   input [4:0] RDM, RDW;
   input       REGWRITEM, REGWRITEW;

   // Stall Logic Input
   input [4:0] RS1D, RS2D;
   input [4:0] RDE;
   input       RESULTSRCE0;

   // Control Hazard Input
   input       PCSRCE;

   output [1:0]	FORWARDAE, FORWARDBE;
   output	STALLF, STALLD;
   output	FLUSHD, FLUSHE;

   // Forwarding
   wire		MATCH_MEM_A, MATCH_WB_A, MATCH_MEM_B, MATCH_WB_B, LWSTALL;
   assign MATCH_MEM_A = (RS1E == RDM) & REGWRITEM & (|RS1E);
   assign MATCH_WB_A = (RS1E == RDW) & REGWRITEW & (|RS1E);
   
   assign MATCH_MEM_B = (RS2E == RDM) & REGWRITEM & (|RS2E);
   assign MATCH_WB_B = (RS2E == RDW) & REGWRITEW & (|RS2E);

   assign FORWARDAE = MATCH_MEM_A ? 2'b10 :
		      MATCH_WB_A ? 2'b01 :
		      2'b00;
   
   assign FORWARDBE = MATCH_MEM_B ? 2'b10 :
		      MATCH_WB_B ? 2'b01 :
		      2'b00;

   // Load-Use Hazard Detection
   assign LWSTALL = RESULTSRCE0 & ((RS1D == RDE) | (RS2D == RDE));

   // Pipeline Control Signal
   assign STALLF = LWSTALL;
   assign STALLD = LWSTALL;

   assign FLUSHD = PCSRCE;
   assign FLUSHE = LWSTALL | PCSRCE;

endmodule // HAZARD_UNIT

