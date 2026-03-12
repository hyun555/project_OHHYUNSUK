// ReLU + quantization (/128, round-to-nearest) - 2-stage pipelined
// 22-bit signed input x4 -> 8-bit unsigned output x4
module requantization (
    input  wire               clk,
    input  wire               rst_n,        // active-low reset

    input  wire               valid_in,     // input valid (4채널 모두 유효)
    input  wire signed [21:0] in0,          // 22-bit signed input 0
    input  wire signed [21:0] in1,          // 22-bit signed input 1
    input  wire signed [21:0] in2,          // 22-bit signed input 2
    input  wire signed [21:0] in3,          // 22-bit signed input 3

    output reg                valid_out,    // output valid (out0~3 유효)
    output reg        [7:0]   out0,         // 8-bit unsigned output 0
    output reg        [7:0]   out1,         // 8-bit unsigned output 1
    output reg        [7:0]   out2,         // 8-bit unsigned output 2
    output reg        [7:0]   out3          // 8-bit unsigned output 3
);

    // ----------------------------
    // Stage 1: ReLU + rounding(+64)
    // ----------------------------
    reg              valid_s1;
    reg signed [21:0] s1_0, s1_1, s1_2, s1_3;

    // ----------------------------
    // Stage 2: shift(>>>7) + clamp
    // ----------------------------
    wire signed [21:0] q0_w = (s1_0 >>> 7);
    wire signed [21:0] q1_w = (s1_1 >>> 7);
    wire signed [21:0] q2_w = (s1_2 >>> 7);
    wire signed [21:0] q3_w = (s1_3 >>> 7);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // pipeline regs
            valid_s1  <= 1'b0;
            s1_0      <= 22'sd0;
            s1_1      <= 22'sd0;
            s1_2      <= 22'sd0;
            s1_3      <= 22'sd0;

            // outputs
            valid_out <= 1'b0;
            out0      <= 8'd0;
            out1      <= 8'd0;
            out2      <= 8'd0;
            out3      <= 8'd0;
        end else begin
            // ----------------------------
            // Stage 1 register
            // ----------------------------
            valid_s1 <= valid_in;

            if (valid_in) begin
                // ReLU 후 rounding(+64)
                s1_0 <= (in0 <= 0) ? 22'sd0 : (in0 + 22'sd64);
                s1_1 <= (in1 <= 0) ? 22'sd0 : (in1 + 22'sd64);
                s1_2 <= (in2 <= 0) ? 22'sd0 : (in2 + 22'sd64);
                s1_3 <= (in3 <= 0) ? 22'sd0 : (in3 + 22'sd64);
            end
            // valid_in==0이면 s1_*는 don't care/유지 (valid로 구분)

            // ----------------------------
            // Stage 2 register (output)
            // ----------------------------
            valid_out <= valid_s1;

            if (valid_s1) begin
                // 채널 0
                if (q0_w > 22'sd255)      out0 <= 8'd255;
                else if (q0_w < 22'sd0)   out0 <= 8'd0;    // 안전장치(실제로는 거의 불필요)
                else                      out0 <= q0_w[7:0];

                // 채널 1
                if (q1_w > 22'sd255)      out1 <= 8'd255;
                else if (q1_w < 22'sd0)   out1 <= 8'd0;
                else                      out1 <= q1_w[7:0];

                // 채널 2
                if (q2_w > 22'sd255)      out2 <= 8'd255;
                else if (q2_w < 22'sd0)   out2 <= 8'd0;
                else                      out2 <= q2_w[7:0];

                // 채널 3
                if (q3_w > 22'sd255)      out3 <= 8'd255;
                else if (q3_w < 22'sd0)   out3 <= 8'd0;
                else                      out3 <= q3_w[7:0];
            end
            // valid_s1==0이면 out*는 이전 값 유지, valid_out만 0
        end
    end

endmodule
