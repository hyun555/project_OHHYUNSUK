module maxpooling (
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  din0,
    input  wire [7:0]  din1,
    input  wire [7:0]  din2,
    input  wire [7:0]  din3,
    input  wire        in_valid,

    output reg  [7:0]  dout,
    output reg         out_valid
);

    // -----------------------
    // Stage 1: pair-wise max
    // -----------------------
    wire [7:0] max01_w = (din0 > din1) ? din0 : din1;
    wire [7:0] max23_w = (din2 > din3) ? din2 : din3;

    reg  [7:0] max01_r, max23_r;
    reg        v1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            max01_r <= 8'd0;
            max23_r <= 8'd0;
            v1      <= 1'b0;
        end else begin
            // 유효 데이터만 의미있게 통과(버블 허용)
            if (in_valid) begin
                max01_r <= max01_w;
                max23_r <= max23_w;
            end else begin
                max01_r <= 8'd0;
                max23_r <= 8'd0;
            end
            v1 <= in_valid;
        end
    end

    // -----------------------
    // Stage 2: final max + output reg
    // -----------------------
    wire [7:0] max_all_w = (max01_r > max23_r) ? max01_r : max23_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout      <= 8'd0;
            out_valid <= 1'b0;
        end else begin
            // Stage1 유효일 때만 결과가 의미있음
            if (v1)
                dout <= max_all_w;
            else
                dout <= 8'd0;

            out_valid <= v1; // in_valid에서 2클럭 지연된 valid
        end
    end

endmodule
