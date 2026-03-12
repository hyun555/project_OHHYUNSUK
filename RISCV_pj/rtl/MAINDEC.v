module MAINDEC (/*AUTOARG*/
   // Outputs
   REGWRITED, MEMWRITED, JUMPD, BRANCHD, ALUSRCD, RESULTSRCD, IMMSRCD,
   ALUOP,
   // Inputs
   OP
   );
   input  [6:0] OP;
   output reg	REGWRITED, MEMWRITED, JUMPD, BRANCHD, ALUSRCD;
   output reg [1:0] RESULTSRCD, IMMSRCD, ALUOP;
   
   always @(*) begin
      // Default values
      REGWRITED = 1'b0; MEMWRITED = 1'b0; JUMPD = 1'b0;
      BRANCHD = 1'b0; ALUSRCD = 1'b0; RESULTSRCD = 2'b00;
      IMMSRCD = 2'b00; ALUOP = 2'b00;

      case (OP)
	7'b0000011: begin // LW
	   REGWRITED = 1'b1; IMMSRCD = 2'b00; ALUSRCD = 1'b1; 
	   MEMWRITED = 1'b0; RESULTSRCD = 2'b01; ALUOP = 2'b00;
	end
	7'b0100011: begin // SW
	   REGWRITED = 1'b0; IMMSRCD = 2'b01; ALUSRCD = 1'b1;
	   MEMWRITED = 1'b1; RESULTSRCD = 2'bxx; ALUOP = 2'b00;
	end
	7'b0110011: begin // R-type
	   REGWRITED = 1'b1; IMMSRCD = 2'bxx; ALUSRCD = 1'b0;
	   MEMWRITED = 1'b0; RESULTSRCD = 2'b00; ALUOP = 2'b10;
	end
	7'b0010011: begin // I-type
	   REGWRITED = 1'b1; IMMSRCD = 2'b00; ALUSRCD = 1'b1;
	   MEMWRITED = 1'b0; RESULTSRCD = 2'b00; ALUOP = 2'b10;
	end
	7'b1100011: begin // BEQ
	   REGWRITED = 1'b0; IMMSRCD = 2'b10; ALUSRCD = 1'b0;
	   MEMWRITED = 1'b0; RESULTSRCD = 2'bxx; ALUOP = 2'b01;
	   BRANCHD = 1'b1;
	end
	7'b1101111: begin // JAL
	   REGWRITED = 1'b1; IMMSRCD = 2'b11; ALUSRCD = 1'bx;
	   MEMWRITED = 1'b0; RESULTSRCD = 2'b10; ALUOP = 2'bxx;
	   JUMPD = 1'b1;
	end
	default: begin
	   REGWRITED = 1'b0; MEMWRITED = 1'b0;
	end
      endcase
   end
endmodule // MAINDEC
