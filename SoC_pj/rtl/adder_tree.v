module adder_tree (
    input wire clk,
    input wire rst_n,
    input wire valid_in, // PE에서 나오는 valid 신호 (모두 동일한 타이밍이라 가정)
    input wire signed [15:0] p_in_00, p_in_01, p_in_02, p_in_03, p_in_04,
    input wire signed [15:0] p_in_05, p_in_06, p_in_07, p_in_08, p_in_09,
    input wire signed [15:0] p_in_10, p_in_11, p_in_12, p_in_13, p_in_14,
    input wire signed [15:0] p_in_15, p_in_16, p_in_17, p_in_18, p_in_19,
    input wire signed [15:0] p_in_20, p_in_21, p_in_22, p_in_23, p_in_24,

    output reg valid_out,
    output reg signed [21:0] final_sum // 16bit + log2(25) bits approx 21 bits
);

    // Pipeline Stages
    // Stage 1: 25 inputs -> 13 sums (Pairwise addition)
    reg signed [16:0] stage1_sum [0:12];
    reg valid_s1;
    
    // Stage 2: 13 inputs -> 7 sums
    reg signed [17:0] stage2_sum [0:6];
    reg valid_s2;

    // Stage 3: 7 inputs -> 4 sums
    reg signed [18:0] stage3_sum [0:3];
    reg valid_s3;

    // Stage 4: 4 inputs -> 2 sums
    reg signed [19:0] stage4_sum [0:1];
    reg valid_s4;

    // Stage 5: 2 inputs -> 1 final output
    // This is directly connected to output reg 'final_sum'

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1 <= 0; valid_s2 <= 0; valid_s3 <= 0; valid_s4 <= 0; valid_out <= 0;
            final_sum <= 0;
            // Reset regs (omitted loop for brevity, assume 0)
        end else begin
            // --- Stage 1 (25 -> 13) ---
            // 0~23번까지 12쌍 더하기, 24번은 그냥 통과
            stage1_sum[0] <= p_in_00 + p_in_01; stage1_sum[1] <= p_in_02 + p_in_03;
            stage1_sum[2] <= p_in_04 + p_in_05; stage1_sum[3] <= p_in_06 + p_in_07;
            stage1_sum[4] <= p_in_08 + p_in_09; stage1_sum[5] <= p_in_10 + p_in_11;
            stage1_sum[6] <= p_in_12 + p_in_13; stage1_sum[7] <= p_in_14 + p_in_15;
            stage1_sum[8] <= p_in_16 + p_in_17; stage1_sum[9] <= p_in_18 + p_in_19;
            stage1_sum[10]<= p_in_20 + p_in_21; stage1_sum[11]<= p_in_22 + p_in_23;
            stage1_sum[12]<= p_in_24; 
            valid_s1 <= valid_in;

            // --- Stage 2 (13 -> 7) ---
            stage2_sum[0] <= stage1_sum[0] + stage1_sum[1];
            stage2_sum[1] <= stage1_sum[2] + stage1_sum[3];
            stage2_sum[2] <= stage1_sum[4] + stage1_sum[5];
            stage2_sum[3] <= stage1_sum[6] + stage1_sum[7];
            stage2_sum[4] <= stage1_sum[8] + stage1_sum[9];
            stage2_sum[5] <= stage1_sum[10] + stage1_sum[11];
            stage2_sum[6] <= stage1_sum[12];
            valid_s2 <= valid_s1;

            // --- Stage 3 (7 -> 4) ---
            stage3_sum[0] <= stage2_sum[0] + stage2_sum[1];
            stage3_sum[1] <= stage2_sum[2] + stage2_sum[3];
            stage3_sum[2] <= stage2_sum[4] + stage2_sum[5];
            stage3_sum[3] <= stage2_sum[6];
            valid_s3 <= valid_s2;

            // --- Stage 4 (4 -> 2) ---
            stage4_sum[0] <= stage3_sum[0] + stage3_sum[1];
            stage4_sum[1] <= stage3_sum[2] + stage3_sum[3];
            valid_s4 <= valid_s3;

            // --- Stage 5 (2 -> 1 Final) ---
            final_sum <= stage4_sum[0] + stage4_sum[1];
            valid_out <= valid_s4;
        end
    end

endmodule
