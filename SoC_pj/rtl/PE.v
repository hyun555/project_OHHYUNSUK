module PE (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_in,
    input  wire [7:0]  din,
    input  wire signed [7:0] weight,

    output reg  [7:0]  dout,
    output reg  signed [15:0] p_out
);

    // 1단계: 입력 래치 (+ din 0-extend를 여기서 수행)
    reg signed [8:0]   din_ext_r;   // {1'b0, din} = 9bit (항상 양수)
    reg signed [7:0]   weight_r;
    reg               valid_r1;

    // 2단계: DSP에서 나오는 조합 곱셈 결과
    // 9bit(signed, MSB=0) * signed 8bit = 17bit
    (* use_dsp = "yes" *)
    wire signed [16:0] mul_res;

    // 3단계: 파이프라인 레지스터
    reg  signed [16:0] mul_res_r;
    reg                valid_r2;

    // ------------ Stage 1 : 입력 파이프라인 (0-extend 여기서) ------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            din_ext_r <= 9'sd0;
            weight_r  <= 8'sd0;
            valid_r1  <= 1'b0;
            dout      <= 8'd0;
        end else begin
            din_ext_r <= {1'b0, din}; // 여기서 앞에 0 붙임
            weight_r  <= weight;
            valid_r1  <= valid_in;
            dout      <= din;         // 슬라이딩용 (1cycle 지연)
        end
    end

    // ------------ Stage 2 : 조합 곱셈 (그냥 곱하기만) ------------
    assign mul_res = din_ext_r * weight_r;

    // ------------ Stage 3 : 곱셈 결과 레지스터 + p_out ------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_res_r <= 17'sd0;
            valid_r2  <= 1'b0;
            p_out     <= 16'sd0;
        end else begin
            mul_res_r <= mul_res;
            valid_r2  <= valid_r1;

            if (valid_r2)
                p_out <= mul_res_r[15:0];
            else
                p_out <= 16'sd0;
        end
    end

endmodule
