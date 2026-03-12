// 4 x 22-bit signed input adder tree
// - clk, rst_n (active low)
// - 입력 valid: in_valid
// - 출력 valid: conv2_add_valid
// - 입력/출력 모두 signed 22bit
// - 2-stage 파이프라인 (tree 구조)

module conv2_adder_tree (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 in_valid,          // 입력 valid
    input  wire signed [21:0]   in0,
    input  wire signed [21:0]   in1,
    input  wire signed [21:0]   in2,
    input  wire signed [21:0]   in3,

    output wire signed [21:0]   conv2_add_out,    // 결과 22bit (truncate)
    output reg                  conv2_add_valid   // 출력 valid
);

    // Stage 1: (in0 + in1), (in2 + in3)
    // 22bit + 22bit -> 최대 23bit 필요하므로 23bit로 정의
    reg signed [22:0] s1_sum0;
    reg signed [22:0] s1_sum1;
    reg               s1_valid;

    // Stage 2: s1_sum0 + s1_sum1
    // 23bit + 23bit -> 최대 24bit 필요
    reg signed [23:0] s2_sum;

    // -------------------------
    // Stage 1 파이프라인
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_sum0  <= 23'sd0;
            s1_sum1  <= 23'sd0;
            s1_valid <= 1'b0;
        end else begin
            if (in_valid) begin
                // sign extend 해서 23bit로 맞춤
                s1_sum0  <= $signed({in0[21], in0}) + $signed({in1[21], in1});
                s1_sum1  <= $signed({in2[21], in2}) + $signed({in3[21], in3});
                s1_valid <= 1'b1;
            end else begin
                s1_valid <= 1'b0;
            end
        end
    end

    // -------------------------
    // Stage 2 파이프라인
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s2_sum          <= 24'sd0;
            conv2_add_valid <= 1'b0;
        end else begin
            if (s1_valid) begin
                // stage1 결과 sign extend 해서 24bit로 맞춤
                s2_sum <= $signed({s1_sum0[22], s1_sum0}) +
                          $signed({s1_sum1[22], s1_sum1});
                conv2_add_valid <= 1'b1;
            end else begin
                conv2_add_valid <= 1'b0;
            end
        end
    end

    // 최종 출력은 22bit로 truncate (오버플로우 처리 X, 단순 잘라냄)
    assign conv2_add_out = s2_sum[21:0];

endmodule
