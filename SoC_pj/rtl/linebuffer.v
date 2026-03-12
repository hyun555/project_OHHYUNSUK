module linebuffer (
    input wire	      clk,
    input wire	      rst_n,

    // 1. AXI Stream Slave Interface (데이터 수신부)
    // 외부 start 신호 없이 항상 데이터를 기다립니다.
    input wire [7:0]  s_axis_tdata,
    input wire	      s_axis_tvalid,
    input wire	      s_axis_tlast, // (옵션)
    input wire s_axis_tready,

    // 7. Output Interface (5x5 커널용 5개 라인 데이터)
    output wire [7:0] out_row0, // 현재 들어온 값
    output wire [7:0] out_row1,
    output wire [7:0] out_row2,
    output wire [7:0] out_row3,
    output wire [7:0] out_row4, // 가장 과거 값
    
    // 9. Output Valid 신호
    output wire	      out_valid
);

    //----------------------------------------------------------------
    // 파라미터 정의
    //----------------------------------------------------------------
    localparam IMG_WIDTH  = 28;
    localparam IMG_HEIGHT = 28;
    localparam TOTAL_PIXELS = IMG_WIDTH * IMG_HEIGHT; // 784
    localparam BUFFER_SIZE = IMG_WIDTH * 4; // 112 bytes

    //----------------------------------------------------------------
    // 내부 레지스터
    //----------------------------------------------------------------
    // Line Buffer
    reg [7:0] shift_reg [0:BUFFER_SIZE-1];
    integer i;

    // 카운터
    reg [9:0] pixel_cnt; // 0 ~ 783
    reg [4:0] col_cnt;   // 0 ~ 27

    //----------------------------------------------------------------
    // 1. Ready 신호 제어
    //----------------------------------------------------------------
    // 리셋이 풀리면 무조건 데이터를 받을 준비를 합니다.
    //assign s_axis_tready = rst_n; 

    //----------------------------------------------------------------
    // 2. 데이터 처리 및 카운터 로직
    //----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_cnt <= 0;
            col_cnt   <= 0;
            for (i=0; i<BUFFER_SIZE; i=i+1) shift_reg[i] <= 8'b0;
        end else begin
            // Valid 데이터가 들어오면 동작 (Ready는 항상 1이므로 Valid만 체크)
            if (s_axis_tvalid && s_axis_tready) begin
                
                // 2-1. Shift Register 업데이트
                shift_reg[0] <= s_axis_tdata;
                for (i=1; i<BUFFER_SIZE; i=i+1) begin
                    shift_reg[i] <= shift_reg[i-1];
                end

                // 2-2. 카운터 관리 (자동 순환)
                if (pixel_cnt == TOTAL_PIXELS - 1) begin
                    // 11. 784개를 다 받으면 0으로 리셋 (다음 프레임 시작)
                    pixel_cnt <= 0;
                    col_cnt   <= 0;
                end else begin
                    pixel_cnt <= pixel_cnt + 1;

                    // 컬럼 카운터 (0~27 반복)
                    if (col_cnt == IMG_WIDTH - 1) 
                        col_cnt <= 0;
                    else 
                        col_cnt <= col_cnt + 1;
                end
            end
        end
    end

    //----------------------------------------------------------------
    // 7, 8. Output Data Assign
    //----------------------------------------------------------------
    assign out_row0 = s_axis_tdata;             // 현재 입력
    assign out_row1 = shift_reg[IMG_WIDTH-1];   // 1 Line 전
    assign out_row2 = shift_reg[IMG_WIDTH*2-1]; // 2 Line 전
    assign out_row3 = shift_reg[IMG_WIDTH*3-1]; // 3 Line 전
    assign out_row4 = shift_reg[IMG_WIDTH*4-1]; // 4 Line 전

    //----------------------------------------------------------------
    // 6, 10. Output Valid Logic
    //----------------------------------------------------------------
    // 조건: 
    // 1. 버퍼가 채워졌는가? (pixel_cnt >= 112)
    // 2. 유효한 컬럼 위치인가? (col_cnt >= 4)
    //
    // 주의: 한 이미지가 끝나고(pixel_cnt가 0이 되고) 다음 이미지가 시작되면
    // pixel_cnt < 112가 되므로 out_valid는 자동으로 Low가 되어 
    // 다음 이미지의 버퍼가 찰 때까지 기다립니다.
    
    assign out_valid = s_axis_tvalid && 
                       (pixel_cnt >= BUFFER_SIZE) && 
                       (col_cnt >= 4);

endmodule 
