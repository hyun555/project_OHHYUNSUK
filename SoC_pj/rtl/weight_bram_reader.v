module weight_bram_reader #(
    parameter ADDR_WIDTH     = 13,    // 0~5196 까지 표현 가능 (1300*4)
    parameter TOTAL_WEIGHTS  = 260,   // 전체 weight 개수
    parameter BLOCK_WEIGHTS  = 20     // 한 번 start 당 읽을 weight 개수
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // 읽기 시작 트리거 (start) : 1사이클 펄스 가정
    input  wire                   start,

    // BRAM 인터페이스 (read-only)
    output reg  [ADDR_WIDTH-1:0]  bram_addr,
    output reg                    bram_en,
    input  wire [39:0]            bram_dout,   // ★ 8비트 -> 40비트

    // weight 출력 (PE_array_top의 weight_in 쪽으로 연결)
    output wire [39:0]            weight_out,  // ★ 8비트 -> 40비트
    output reg                    weight_valid,

    // 상태 정보
    output reg                    done,   // BLOCK_WEIGHTS개 다 읽은 순간 1사이클 펄스
    output wire                   busy    // READ 중이면 1
);

    // 상태 정의
    localparam S_IDLE = 2'd0;
    localparam S_READ = 2'd1;

    reg [1:0] state, state_next;

    // 전체 weight 중에서 지금 읽고 있는 index
    reg [10:0] ptr_reg, ptr_next;   // TOTAL_WEIGHTS 최대 개수에 맞게 여유 비트

    // 이번 start에서 몇 개 "내보냈는지"
    reg [6:0]  cnt_reg, cnt_next;   // BLOCK_WEIGHTS 최대 개수에 맞게 여유 비트

    // BRAM read가 한 클럭 늦게 데이터가 나오므로,
    // 이전 클럭에 read를 걸었는지 표시하는 플래그
    reg data_valid_reg, data_valid_next;

    // next 값/출력용
    reg                   done_next;
    reg                   weight_valid_next;
    reg [39:0]            weight_out_next;     // ★ 8비트 -> 40비트 (현재는 사용 안 함)
    reg [ADDR_WIDTH-1:0]  bram_addr_next;
    reg                   bram_en_next;

    assign busy       = (state != S_IDLE);
    assign weight_out = bram_dout;  // ★ BRAM 40비트 데이터를 그대로 출력으로

    // ===========================
    //   Sequential (레지스터 갱신)
    // ===========================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            ptr_reg        <= 11'd0;                  // 처음엔 index 0에서 시작
            cnt_reg        <= 7'd0;
            data_valid_reg <= 1'b0;

            done           <= 1'b0;
            weight_valid   <= 1'b0;
            //weight_out     <= 40'd0;               // ★ 주석이지만 40비트로 변경
            bram_addr      <= {ADDR_WIDTH{1'b0}};
            bram_en        <= 1'b0;
        end else begin
            state          <= state_next;
            ptr_reg        <= ptr_next;
            cnt_reg        <= cnt_next;
            data_valid_reg <= data_valid_next;

            done           <= done_next;
            weight_valid   <= weight_valid_next;
            //weight_out     <= weight_out_next;      // ★ 사용하려면 40비트로 맞춰둔 상태
            bram_addr      <= bram_addr_next;
            bram_en        <= bram_en_next;
        end
    end

    // ===========================
    //   Combinational (다음 상태/출력)
    // ===========================
    always @* begin
        // 기본값들
        state_next         = state;
        ptr_next           = ptr_reg;
        cnt_next           = cnt_reg;

        done_next          = 1'b0;
        bram_en_next       = 1'b0;
        bram_addr_next     = bram_addr;

        weight_valid_next  = 1'b0;
        weight_out_next    = weight_out;   // ★ 타입만 40비트로 맞춰둠

        data_valid_next    = data_valid_reg;

        case (state)
            // -----------------------
            // IDLE : 대기 상태
            // -----------------------
            S_IDLE: begin
                cnt_next        = 7'd0;
                data_valid_next = 1'b0;   // 새로 시작이니까 "출력 유효" 플래그 0

                if (start) begin
                    // READ 모드 진입
                    state_next      = S_READ;
                    // 첫 주소 요청 (이 클럭에 addr/en 걸고)
                    bram_en_next    = 1'b1;
                    bram_addr_next  = ptr_reg;
                    // 아직 데이터는 안 나왔으니 data_valid_reg는 다음 클럭부터 1
                    data_valid_next = 1'b1;
                end
            end

            // -----------------------
            // READ : weight BLOCK_WEIGHTS개 읽기
            // -----------------------
            S_READ: begin
                // 기본적으로 계속 읽기 요청 (파이프라인)
                bram_en_next = 1'b1;

                // 직전 클럭에서 read 요청이 있었다면,
                // 지금 클럭의 bram_dout은 유효한 데이터라고 가정
                if (data_valid_reg) begin
                    weight_valid_next = 1'b1;
                    //weight_out_next   = bram_dout;  // ★ 40비트로 동작 (현재는 assign 방식 사용)

                    // 이번이 마지막인가?
                    if (cnt_reg == (BLOCK_WEIGHTS-1)) begin
                        // 마지막 weight를 지금 내보내는 사이클
                        state_next       = S_IDLE;
                        cnt_next         = 7'd0;
                        done_next        = 1'b1;   // 블록 완료 펄스
                        bram_en_next     = 1'b0;   // 다음 read 없음
                        data_valid_next  = 1'b0;   // 다음 클럭부터는 데이터 없음
                        // ptr_next는 여기서 한 번만 증가
                        if (ptr_reg == (TOTAL_WEIGHTS-1))
                            ptr_next = 11'd0;
                        else
                            ptr_next = ptr_reg + 1'b1;
                    end else begin
                        // 아직 BLOCK_WEIGHTS개 안 채웠으면 카운터, 포인터 증가
                        cnt_next = cnt_reg + 1'b1;

                        if (ptr_reg == (TOTAL_WEIGHTS-1))
                            ptr_next = 11'd0;
                        else
                            ptr_next = ptr_reg + 1'b1;

                        // 다음 클럭에 나올 데이터를 위해 다음 주소 걸어줌
                        bram_addr_next  = ptr_next;
                        data_valid_next = 1'b1; // 계속 유효 데이터 나옴
                    end
                end
                else begin
                    // data_valid_reg == 0 이면,
                    // 아직 첫 데이터가 안 나온 상태 (IDLE→READ 첫 사이클)
                    // 이 사이클에서는 단지 addr/en 유지만 하면 됨.
                    // (IDLE에서 이미 첫 addr/en을 걸어놓았기 때문에
                    //  여기서는 bram_addr 유지 정도만 해줘도 됨)
                    bram_addr_next  = bram_addr;
                    data_valid_next = 1'b1; // 다음 클럭부터 데이터 유효
                end
            end

            default: begin
                state_next      = S_IDLE;
                data_valid_next = 1'b0;
            end
        endcase
    end

endmodule
