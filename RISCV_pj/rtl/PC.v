module PC(/*AUTOARG*/
   // Outputs
   PCF, PCPLUS4F,
   // Inputs
   CLK, RESET, STALLF, PCE, IMMEXTE, PCSRCE
   );
   input CLK; // Clock Signal
   input RESET; // Active High
   input STALLF;
   
   input [31:0]	PCE;
   input [31:0]	IMMEXTE; // Immediate offset for branches
   input	PCSRCE; // Selects Next PC Srouce (0: PC + 4, 1: PC + ImmExt)
   output [31:0] PCF;
   output [31:0] PCPLUS4F;

   reg [31:0]	 PC_REG;
   wire [31:0]	 PCNEXTF, PCTARGETE;

   assign PCF = PC_REG;
   assign PCPLUS4F = PCF + 32'd4;
   assign PCTARGETE = PCE + IMMEXTE;
   assign PCNEXTF = (PCSRCE == 1) ? PCTARGETE : PCPLUS4F;

   always @ (posedge CLK or posedge RESET) begin
      if (RESET) begin
	 PC_REG <= 32'd0;
      end
      else if (!STALLF) begin
	 PC_REG <= PCNEXTF;
      end
   end

endmodule // PC
