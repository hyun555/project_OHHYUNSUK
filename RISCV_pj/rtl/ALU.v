module ALU(
    output reg [31:0] RESULT,
    output reg        ZEROE,
    input [31:0]      SRCAE, SRCBE,
    input [1:0]       ALUCONTROLE
);

    wire [31:0] mul_srcA = (ALUCONTROLE == 2'b11) ? SRCAE : 32'd0;
    wire [31:0] mul_srcB = (ALUCONTROLE == 2'b11) ? SRCBE : 32'd0;
    wire [31:0] RES_MUL = mul_srcA * mul_srcB;

    wire [31:0] RES_ADD = SRCAE + SRCBE;
    wire [31:0] RES_SUB = SRCAE - SRCBE;
    wire [31:0] RES_AND = SRCAE & SRCBE;

    always @(*) begin
        case (ALUCONTROLE)
            2'b00: RESULT = RES_ADD;
            2'b01: RESULT = RES_SUB;
            2'b10: RESULT = RES_AND;
            2'b11: RESULT = RES_MUL;
            default: RESULT = 32'd0;
        endcase
    end

    always @(*) begin
        ZEROE = (RESULT == 32'd0) ? 1'b1 : 1'b0;
    end

endmodule