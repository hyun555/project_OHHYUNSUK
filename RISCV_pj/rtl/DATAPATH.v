module DATAPATH(/*AUTOARG*/
		// Outputs
		PCF, ALURESULTM, WRITEDATAM, MEMWRITEM, INSTRD, ZEROE, RS1D, RS2D,
		RS1E, RS2E, RDE, RDM, RDW, REGWRITEM, REGWRITEW, RESULTSRCE0,
		PCSRCE,
		// Inputs
		CLK, RESET, INSTRF, READDATAM, REGWRITED, RESULTSRCD, MEMWRITED,
		JUMPD, BRANCHD, ALUCONTROLD, ALUSRCD, IMMSRCD, STALLF, STALLD,
		FLUSHD, FLUSHE, FORWARDAE, FORWARDBE
		);
   input CLK, RESET;

   // Instruction Memory
   output [31:0] PCF;
   input [31:0]	 INSTRF;
   // Data Memory
   output [31:0] ALURESULTM;
   output [31:0] WRITEDATAM;
   output	 MEMWRITEM;
   input [31:0]	 READDATAM;
   // Controller
   input	 REGWRITED;
   input [1:0]	 RESULTSRCD;
   input	 MEMWRITED;
   input	 JUMPD;
   input	 BRANCHD;
   input [1:0]	 ALUCONTROLD;
   input	 ALUSRCD;
   input [1:0]	 IMMSRCD;
   output [31:0] INSTRD;
   output	 ZEROE;

   // Hazard Unit
   input	 STALLF, STALLD;
   input	 FLUSHD, FLUSHE;
   input [1:0]	 FORWARDAE, FORWARDBE;
   output [4:0]	 RS1D, RS2D;
   output [4:0]	 RS1E, RS2E, RDE;
   output [4:0]	 RDM, RDW;
   output	 REGWRITEM, REGWRITEW;
   output	 RESULTSRCE0;
   output	 PCSRCE;

   // MOD: LBU Pipeline Registers (funct3 propagation)
   reg [2:0] FUNCT3E_REG, FUNCT3M_REG, FUNCT3W_REG;

   // MOD: Pre-calculated Hazard Detection Registers
   reg match_1e_m_reg, match_2e_m_reg;
   reg match_1e_w_reg, match_2e_w_reg;

   // 1. Fetch State
   reg [31:0]	 RD1E, RD2E, IMMEXTE, PCE, PCPLUS4E;

   wire [31:0]	 PCPLUS4F;
   wire		 PCSRCE;

   PC PC(/*AUTOINST*/
	 // Outputs
	 .PCF				(PCF[31:0]),
	 .PCPLUS4F			(PCPLUS4F[31:0]),
	 // Inputs
	 .CLK				(CLK),
	 .RESET				(RESET),
	 .STALLF			(STALLF),
	 .PCE				(PCE[31:0]),
	 .IMMEXTE			(IMMEXTE[31:0]),
	 .PCSRCE			(PCSRCE));
   // 2. Decode Stage
   reg [31:0] INSTRD_REG, PCPLUS4D, PCD;
   always @(posedge CLK or posedge RESET) begin
      if (RESET) begin
	 INSTRD_REG <= 32'd0;
	 PCPLUS4D <= 32'd0; PCD <= 32'd0;
      end
      else if (FLUSHD) begin
	 INSTRD_REG <= 32'd0;
	 PCPLUS4D <= 32'd0; PCD <= 32'd0;
      end
      else if (!STALLD) begin
	 INSTRD_REG <= INSTRF;
	 PCPLUS4D <= PCPLUS4F; PCD <= PCF;
      end
   end // always @ (posedge CLK or posedge RESET)

   assign INSTRD = INSTRD_REG;
   wire [31:0] RD1D, RD2D, IMMEXTD;
   wire [31:0] RESULTW;

   // MOD: Needed for Pre-calculated Hazard (Early Access to Pipeline Regs)
   reg [4:0] RDM_REG, RDW_REG; 

   REGFILE RF(// Outputs
	      .RD1			(RD1D[31:0]),
	      .RD2			(RD2D[31:0]),
	      // Inputs
	      .CLK			(CLK),
	      .WE3			(REGWRITEW),
	      .A1			(INSTRD_REG[19:15]),
	      .A2			(INSTRD_REG[24:20]),
	      .A3			(RDW_REG), // MOD: Use Internal RDW_REG
	      .WD3			(RESULTW[31:0]));
   EXTEND EXT(// Outputs
	      .IMMEXTD			(IMMEXTD[31:0]),
	      // Inputs
	      .INSTRD			(INSTRD_REG[31:7]),
	      .IMMSRCD			(IMMSRCD[1:0]));
   assign RS1D = INSTRD_REG[19:15];
   assign RS2D = INSTRD_REG[24:20];

   // MOD: Hazard Pre-calculation Logic (Decode Stage)
   wire [4:0] RDE_REG_WIRE; // Forward declaration
   wire match_1d_e = (INSTRD_REG[19:15] == RDE_REG_WIRE);
   wire match_2d_e = (INSTRD_REG[24:20] == RDE_REG_WIRE);
   wire match_1d_m = (INSTRD_REG[19:15] == RDM_REG);
   wire match_2d_m = (INSTRD_REG[24:20] == RDM_REG);


   // 3. Execute Stage
   reg [4:0]  RS1E_REG, RS2E_REG, RDE_REG;
   assign RDE_REG_WIRE = RDE_REG; // MOD: Wire connection

   reg	      REGWRITEE, MEMWRITEE, JUMPE, BRANCHE, ALUSRCE;
   reg [1:0]  RESULTSRCE;
   reg [1:0]  ALUCONTROLE;
   always @(posedge CLK or posedge RESET) begin
      if (RESET) begin
	 REGWRITEE <= 1'b0;
	 MEMWRITEE <= 1'b0; JUMPE <= 1'b0;
	 BRANCHE <= 1'b0; ALUSRCE <= 1'b0; RESULTSRCE <= 2'b00;
	 ALUCONTROLE <= 2'b00;
	 RD1E <= 32'd0; RD2E <= 32'd0; IMMEXTE <= 32'd0;
	 PCE <= 32'd0; PCPLUS4E <= 32'd0;
	 RS1E_REG <= 5'd0;
	 RS2E_REG <= 5'd0; RDE_REG <= 5'd0;
     FUNCT3E_REG <= 3'd0; // MOD: Reset funct3
     match_1e_m_reg <= 1'b0; match_2e_m_reg <= 1'b0; // MOD: Reset Hazard Flags
     match_1e_w_reg <= 1'b0; match_2e_w_reg <= 1'b0; // MOD: Reset Hazard Flags
      end
      else if (FLUSHE) begin
	 REGWRITEE <= 1'b0;
	 MEMWRITEE <= 1'b0; JUMPE <= 1'b0;
	 BRANCHE <= 1'b0; ALUSRCE <= 1'b0; RESULTSRCE <= 2'b00;
	 ALUCONTROLE <= 2'b00;
	 RD1E <= 32'd0; RD2E <= 32'd0; IMMEXTE <= 32'd0;
	 PCE <= 32'd0; PCPLUS4E <= 32'd0;
	 RS1E_REG <= 5'd0;
	 RS2E_REG <= 5'd0; RDE_REG <= 5'd0;
     FUNCT3E_REG <= 3'd0; // MOD: Flush funct3
     match_1e_m_reg <= 1'b0; match_2e_m_reg <= 1'b0; // MOD: Flush Hazard Flags
     match_1e_w_reg <= 1'b0; match_2e_w_reg <= 1'b0; // MOD: Flush Hazard Flags
      end
      else begin
	 REGWRITEE <= REGWRITED; MEMWRITEE <= MEMWRITED;
	 JUMPE <= JUMPD; BRANCHE <= BRANCHD;
	 ALUSRCE <= ALUSRCD; RESULTSRCE <= RESULTSRCD;
	 ALUCONTROLE <= ALUCONTROLD;

	 RD1E <= RD1D;
	 RD2E <= RD2D; IMMEXTE <= IMMEXTD;
	 PCE <= PCD; PCPLUS4E <= PCPLUS4D;
	 RS1E_REG <= INSTRD_REG[19:15];
	 RS2E_REG <= INSTRD_REG[24:20];
	 RDE_REG <= INSTRD_REG[11:7];
     
     FUNCT3E_REG <= INSTRD_REG[14:12]; // MOD: Capture funct3 for LBU
     match_1e_m_reg <= match_1d_e; // MOD: Propagate Hazard Flag
     match_2e_m_reg <= match_2d_e; // MOD
     match_1e_w_reg <= match_1d_m; // MOD
     match_2e_w_reg <= match_2d_m; // MOD
      end // else: !if(RESET || FLUSHE)
   end // always @ (posedge CLK or posedge RESET)

   // MOD: Internal Fast Forwarding Logic (Replacing HAZARD_UNIT inputs)
   reg REGWRITEM_REG, REGWRITEW_REG; // Forward declaration
   wire fwd_ae_mem = match_1e_m_reg & REGWRITEM_REG & (|RDM_REG); // MOD
   wire fwd_be_mem = match_2e_m_reg & REGWRITEM_REG & (|RDM_REG); // MOD
   wire fwd_ae_wb  = match_1e_w_reg & REGWRITEW_REG & (|RDW_REG); // MOD
   wire fwd_be_wb  = match_2e_w_reg & REGWRITEW_REG & (|RDW_REG); // MOD

   // MOD: Mux Select Signals
   wire [1:0] fast_forwardae = fwd_ae_mem ? 2'b10 : (fwd_ae_wb ? 2'b01 : 2'b00);
   wire [1:0] fast_forwardbe = fwd_be_mem ? 2'b10 : (fwd_be_wb ? 2'b01 : 2'b00);

   wire [31:0] SRCAE, SRCBE, WRITEDATAE, ALURESULTE;
   
   // MOD: Use fast_forward signals instead of FORWARDAE/BE
   assign SRCAE = (fast_forwardae == 2'b00) ? RD1E :
		  (fast_forwardae == 2'b01) ? RESULTW :
		  ALURESULTM;
   assign WRITEDATAE = (fast_forwardbe == 2'b00) ? RD2E :
		       (fast_forwardbe == 2'b01) ? RESULTW :
		       ALURESULTM;

   assign SRCBE = (ALUSRCE) ? IMMEXTE : WRITEDATAE;
   ALU ALU (// Outputs
	    .RESULT			(ALURESULTE[31:0]),
	    .ZEROE			(ZEROE),
	    // Inputs
	    .SRCAE			(SRCAE[31:0]),
	    .SRCBE			(SRCBE[31:0]),
	    .ALUCONTROLE		(ALUCONTROLE[1:0]));
   wire [31:0] PCTARGETE;
   
   assign PCTARGETE = PCE + IMMEXTE;
   wire PC_wire;
   assign PC_wire=(SRCAE==SRCBE);
   assign PCSRCE = (BRANCHE & PC_wire) |
		   JUMPE;

   assign RS1E = RS1E_REG;
   assign RS2E = RS2E_REG;
   assign RDE = RDE_REG;
   assign RESULTSRCE0 = RESULTSRCE[0];
   // 4. Memory Stage
   reg [31:0] ALURESULTM_REG, WRITEDATAM_REG, PCPLUS4M;
   // MOD: RDM_REG and REGWRITEM_REG declared earlier
   reg	      MEMWRITEM_REG;
   reg [1:0]  RESULTSRCM;
   always @(posedge CLK or posedge RESET) begin
      if(RESET) begin
	 REGWRITEM_REG <= 1'b0; MEMWRITEM_REG <= 1'b0;
	 RESULTSRCM <= 2'b00; ALURESULTM_REG <= 32'd0;
	 WRITEDATAM_REG <= 32'd0; RDM_REG <= 5'd0;
	 PCPLUS4M <= 32'd0;
     FUNCT3M_REG <= 3'd0; // MOD: Reset funct3
      end
      else begin
	 REGWRITEM_REG <= REGWRITEE; MEMWRITEM_REG <= MEMWRITEE;
	 RESULTSRCM <= RESULTSRCE; ALURESULTM_REG <= ALURESULTE;
	 WRITEDATAM_REG <= WRITEDATAE; RDM_REG <= RDE_REG;
	 PCPLUS4M <= PCPLUS4E;
     FUNCT3M_REG <= FUNCT3E_REG; // MOD: E -> M
      end // else: !if(RESET)
   end // always @ (posedge CLK or posedge RESET)

   assign ALURESULTM = ALURESULTM_REG;
   assign WRITEDATAM = WRITEDATAM_REG;
   assign MEMWRITEM = MEMWRITEM_REG;

   assign RDM = RDM_REG;
   assign REGWRITEM = REGWRITEM_REG;
   // 5. Writeback Stage
   reg[31:0] ALURESULTW_REG, READDATAW_REG, PCPLUS4W;
   // MOD: RDW_REG and REGWRITEW_REG declared earlier
   reg [1:0] RESULTSRCW;

   always @(posedge CLK or posedge RESET) begin
      if (RESET) begin
	 REGWRITEW_REG <= 1'b0;
	 RESULTSRCW <= 2'b00;
	 ALURESULTW_REG <= 32'd0; READDATAW_REG <= 32'd0;
	 RDW_REG <= 5'd0; PCPLUS4W <= 32'd0;
     FUNCT3W_REG <= 3'd0; // MOD: Reset funct3
      end
      else begin
	 REGWRITEW_REG <= REGWRITEM_REG;
	 RESULTSRCW <= RESULTSRCM;

	 ALURESULTW_REG <= ALURESULTM_REG; READDATAW_REG <= READDATAM;
	 RDW_REG <= RDM_REG; PCPLUS4W <= PCPLUS4M;
     FUNCT3W_REG <= FUNCT3M_REG; // MOD: M -> W
      end // else: !if(RESET)
   end // always @ (posedge CLK or posedge RESET)

   // MOD: LBU Logic Implementation
   // Determine byte offset from the calculated address
   wire [1:0] ByteOffset = ALURESULTW_REG[1:0];
   reg [7:0] SelectedByte;
   
   // Byte Selector Mux
   always @(*) begin
       case (ByteOffset)
           2'b00: SelectedByte = READDATAW_REG[7:0];
           2'b01: SelectedByte = READDATAW_REG[15:8];
           2'b10: SelectedByte = READDATAW_REG[23:16];
           2'b11: SelectedByte = READDATAW_REG[31:24];
       endcase
   end

   // If LBU (funct3 == 100), Zero-Extend SelectedByte. Else, use Full Word.
   wire [31:0] MEM_DATA_FINAL;
   assign MEM_DATA_FINAL = (FUNCT3W_REG == 3'b100) ? {24'd0, SelectedByte} : READDATAW_REG;

   // MOD: Updated RESULTW to use MEM_DATA_FINAL
   assign RESULTW = (RESULTSRCW == 2'b00) ?
		    ALURESULTW_REG :
		    (RESULTSRCW == 2'b01) ? MEM_DATA_FINAL : // MOD
		    PCPLUS4W;

   assign REGWRITEW = REGWRITEW_REG;
   assign RDW = RDW_REG;

endmodule // DATAPATH
