module ALUDEC(/*AUTOARG*/
   // Outputs
   ALUCONTROLD,
   // Inputs
   OPB5, FUNCT3, FUNCT7B0, FUNCT7B5, ALUOP
   ); // Decodes ALU Operation based on Funct3/Funct7 and ALUOP
   input OPB5; // opcode[5]
   input [2:0] FUNCT3; // Funct3 Field 
   input       FUNCT7B0, FUNCT7B5; // *funct7[0]: MUL | funct7[5]: SUB
   input [1:0] ALUOP; // ALU Operation Category
   output reg [1:0] ALUCONTROLD;

   wire		    RTYPESUB, RTYPEMUL;
   assign RTYPESUB = FUNCT7B5 & OPB5; // Detect SUB Instruction
   assign RTYPEMUL = FUNCT7B0 & OPB5; // Detect MUL Instruction


   always @(*) begin
      case (ALUOP)
	2'b00: ALUCONTROLD = 2'b00; // ADD (used in LW, SW Address Calculation)
	2'b01: ALUCONTROLD = 2'b01; // SUB (used in BEQ Comparison)
	default: begin
	   case(FUNCT3)
	     3'b000: begin
		if (RTYPEMUL) begin
		   ALUCONTROLD = 2'b11;
		end
		else if (RTYPESUB) begin
		   ALUCONTROLD = 2'b01;
		end
		else begin
		   ALUCONTROLD = 2'b00;
		end
		end
	     3'b111: ALUCONTROLD = 2'b10; // AND, ANDI
	     default: ALUCONTROLD = 2'b00;
	   endcase // case (FUNCT3)
	end // case: default
	endcase
end

   
endmodule // ALUDEC
