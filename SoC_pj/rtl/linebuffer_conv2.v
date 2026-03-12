module linebuffer_conv2 (
    input  wire       clk,
    input  wire       rst_n,

    // 입력: 1픽셀씩 들어오는 8비트 데이터 + valid
    input  wire [7:0] in_data,
    input  wire       in_valid,

    // 5x5 커널용 5개 라인 데이터
    output wire [7:0] out_row0, // 현재 들어온 값
    output wire [7:0] out_row1,
    output wire [7:0] out_row2,
    output wire [7:0] out_row3,
    output wire [7:0] out_row4, // 가장 과거 값
    
    // Output Valid 신호
    output wire       out_valid
);

    //----------------------------------------------------------------
    // 파라미터 정의 (12x12 이미지)
    //----------------------------------------------------------------
    localparam IMG_WIDTH    = 12;
    localparam IMG_HEIGHT   = 12;
    localparam TOTAL_PIXELS = IMG_WIDTH * IMG_HEIGHT; // 144
    localparam BUFFER_SIZE  = IMG_WIDTH * 4;          // 48 bytes (4라인)

    //----------------------------------------------------------------
    // 내부 레지스터
    //----------------------------------------------------------------
    // Line Buffer (4라인 분량)
    reg [7:0] shift_reg [0:BUFFER_SIZE-1];
    integer i;

    // 카운터
    reg [9:0] pixel_cnt; // 0 ~ TOTAL_PIXELS-1 (0~143)
    reg [4:0] col_cnt;   // 0 ~ IMG_WIDTH-1   (0~11)

    //----------------------------------------------------------------
    // 데이터 처리 및 카운터 로직
    //----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_cnt <= 0;
            col_cnt   <= 0;
            for (i = 0; i < BUFFER_SIZE; i = i + 1)
                shift_reg[i] <= 8'b0;
        end else begin
            // Valid 데이터가 들어오면 동작
            if (in_valid) begin
                
                // 1) Shift Register 업데이트
                shift_reg[0] <= in_data;
                for (i = 1; i < BUFFER_SIZE; i = i + 1) begin
                    shift_reg[i] <= shift_reg[i-1];
                end

                // 2) 카운터 관리 (자동 순환)
                if (pixel_cnt == TOTAL_PIXELS - 1) begin
                    // 한 프레임(12x12) 다 받으면 0으로 리셋 (다음 프레임 시작)
                    pixel_cnt <= 0;
                    col_cnt   <= 0;
                end else begin
                    pixel_cnt <= pixel_cnt + 1;

                    // 컬럼 카운터 (0~IMG_WIDTH-1 반복)
                    if (col_cnt == IMG_WIDTH - 1)
                        col_cnt <= 0;
                    else
                        col_cnt <= col_cnt + 1;
                end
            end
        end
    end

    //----------------------------------------------------------------
    // Output Data Assign
    //----------------------------------------------------------------
    assign out_row0 = in_data;                // 현재 입력
    assign out_row1 = shift_reg[IMG_WIDTH-1]; // 1 Line 전
    assign out_row2 = shift_reg[IMG_WIDTH*2-1]; // 2 Line 전
    assign out_row3 = shift_reg[IMG_WIDTH*3-1]; // 3 Line 전
    assign out_row4 = shift_reg[IMG_WIDTH*4-1]; // 4 Line 전

    //----------------------------------------------------------------
    // Output Valid Logic
    //----------------------------------------------------------------
    // 조건:
    // 1. 버퍼가 채워졌는가?       (pixel_cnt >= BUFFER_SIZE == 48)
    // 2. 유효한 컬럼 위치인가?    (col_cnt >= 4) → 가로로도 최소 5픽셀 확보
    //
    // 한 프레임 끝나고 pixel_cnt가 다시 0이 되면 out_valid는 자연스럽게 Low됨.
    assign out_valid = in_valid &&
                       (pixel_cnt >= BUFFER_SIZE) &&
                       (col_cnt   >= 4);

endmodule
