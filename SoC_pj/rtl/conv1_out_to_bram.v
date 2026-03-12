// 8비트 입력을 BRAM에 12x12(144개) 저장
// in_valid 뜰 때마다 바로 그 클럭에 쓰기
module conv1_out_to_bram (
    input wire	      clk,
    input wire	      rst_n,

    input wire [7:0]  din_0,
    input wire [7:0]  din_1,
    input wire [7:0]  din_2,
    input wire [7:0]  din_3,
			  
    input wire	      in_valid,

    // BRAM write port 인터페이스
    output wire [7:0] bram_addr, // 0 ~ 143
    output wire [7:0] bram_din_0,
    output wire [7:0] bram_din_1,
    output wire [7:0] bram_din_2,
    output wire [7:0] bram_din_3,
			  
    output wire	      bram_we, // wea
    output wire	      bram_en,

    output reg	      frame_done  // 144개 다 쓰면 1클럭 펄스
);

    // 현재 "쓰고 있는" 주소 (그 클럭에서 write 되는 주소)
    reg [7:0] addr;

    // BRAM 쪽은 전부 바로 assign
    assign bram_addr = addr;
    assign bram_din_0  = din_0;
   assign bram_din_1  = din_1;
   assign bram_din_2  = din_2;
   assign bram_din_3  = din_3;
    assign bram_we   = in_valid;
    assign bram_en   = in_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr       <= 8'd0;
            frame_done <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            if (in_valid) begin
                // 지금 클럭에 addr 주소로 write 되고,
                // 다음 클럭을 위해 addr 업데이트
                if (addr == 8'd143) begin
                    addr       <= 8'd0;   // 다음 valid부터 다시 0부터
                    frame_done <= 1'b1;   // 이번에 143까지 채웠다는 의미
                end else begin
                    addr <= addr + 8'd1;
                end
            end
        end
    end

endmodule
