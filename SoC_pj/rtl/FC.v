// 11_30 ???? ?? + pe_valid / pe_phase ??
// 2 clk ?? ??

module FC #(
    parameter INPUT_NUM     = 192,
    parameter OUTPUT_NUM    = 10,
    parameter IN_BITS       = 8,
    parameter W_BITS        = 8,
    parameter ACC_BITS      = 32,
    parameter OUT_BITS      = 32,   // comparator? ?? ??
    parameter WE_ADDR_WIDTH = 9     // depth 384 ? addr 0~383
)(
    input  wire                         clk,
    input  wire                         rst_n,

    // MAX pooling ?? ???
    input  wire                         valid_in,
    input  wire  [IN_BITS-1:0]    data_in,

    // BRAM weight ???
    output reg  [WE_ADDR_WIDTH-1:0]     w_addr,      // addra
    input  wire [39:0]                  w_data_40,   // weight 5????? / BRAM width 40?? ??

    // FC -> comparator
    output reg                          res_valid,
    output reg signed [OUT_BITS-1:0]    res_data,
    output reg                          fc_done
);

    localparam S_LOW   = 3'd0;  // ??? ??? low(0~4) MAC
    localparam S_HIGH  = 3'd1;  // ?? ??? high(5~9) MAC
   localparam  S_WAIT_1 = 3'd2;
   localparam  S_WAIT_2 = 3'd3;
    localparam S_WRITE = 3'd4;  // acc0~9 ?? ??

    reg [2:0] state;

    reg [$clog2(INPUT_NUM)-1:0]   in_idx;   // 0~191
    reg [$clog2(OUTPUT_NUM)-1:0]  out_idx;  // 0~9

    reg  [IN_BITS-1:0] act_reg;       // ?? ???

    reg pe_clear;
    reg pe_valid;
    reg pe_phase;                           // 0: low, 1: high

    reg frame_clear_req;    // comparator? ? ?? ? ?? ??? ?? ??? clear

    // weight 5??
    wire signed [W_BITS-1:0] w0 = w_data_40[ 7: 0];
    wire signed [W_BITS-1:0] w1 = w_data_40[15: 8];
    wire signed [W_BITS-1:0] w2 = w_data_40[23:16];
    wire signed [W_BITS-1:0] w3 = w_data_40[31:24];
    wire signed [W_BITS-1:0] w4 = w_data_40[39:32];

    // PE 5?: (0,5), (1,6), (2,7), (3,8), (4,9) <= weight ?? ??? low ???? high? PE? ??
    wire signed [ACC_BITS-1:0] acc0_low, acc0_high;
    wire signed [ACC_BITS-1:0] acc1_low, acc1_high;
    wire signed [ACC_BITS-1:0] acc2_low, acc2_high;
    wire signed [ACC_BITS-1:0] acc3_low, acc3_high;
    wire signed [ACC_BITS-1:0] acc4_low, acc4_high;

    PE_ONE #(.IN_BITS(IN_BITS), .W_BITS(W_BITS), .ACC_BITS(ACC_BITS)) u_pe0 (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (pe_clear),
        .valid    (pe_valid),
        .phase    (pe_phase),
        .in_act   (act_reg),
        .in_w     (w0),
        .acc_low  (acc0_low),
        .acc_high (acc0_high)
    );

    PE_ONE #(.IN_BITS(IN_BITS), .W_BITS(W_BITS), .ACC_BITS(ACC_BITS)) u_pe1 (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (pe_clear),
        .valid    (pe_valid),
        .phase    (pe_phase),
        .in_act   (act_reg),
        .in_w     (w1),
        .acc_low  (acc1_low),
        .acc_high (acc1_high)
    );

    PE_ONE #(.IN_BITS(IN_BITS), .W_BITS(W_BITS), .ACC_BITS(ACC_BITS)) u_pe2 (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (pe_clear),
        .valid    (pe_valid),
        .phase    (pe_phase),
        .in_act   (act_reg),
        .in_w     (w2),
        .acc_low  (acc2_low),
        .acc_high (acc2_high)
    );

    PE_ONE #(.IN_BITS(IN_BITS), .W_BITS(W_BITS), .ACC_BITS(ACC_BITS)) u_pe3 (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (pe_clear),
        .valid    (pe_valid),
        .phase    (pe_phase),
        .in_act   (act_reg),
        .in_w     (w3),
        .acc_low  (acc3_low),
        .acc_high (acc3_high)
    );

    PE_ONE #(.IN_BITS(IN_BITS), .W_BITS(W_BITS), .ACC_BITS(ACC_BITS)) u_pe4 (
        .clk      (clk),
        .rst_n    (rst_n),
        .clear    (pe_clear),
        .valid    (pe_valid),
        .phase    (pe_phase),
        .in_act   (act_reg),
        .in_w     (w4),
        .acc_low  (acc4_low),
        .acc_high (acc4_high)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_LOW;
            in_idx         <= 0;
            out_idx        <= 0;
            act_reg        <= 0;
            w_addr         <= {WE_ADDR_WIDTH{1'b0}}; // reset ? ? clk?? addr 0 ? prefetch
            res_valid      <= 1'b0;
            res_data       <= {OUT_BITS{1'b0}};
            fc_done        <= 1'b0;
            pe_clear       <= 1'b0;
            pe_valid       <= 1'b0;
            pe_phase       <= 1'b0;
            frame_clear_req<= 1'b0;
        end else begin

            res_valid <= 1'b0;
            fc_done   <= 1'b0;
            pe_valid  <= 1'b0;
            pe_clear  <= 1'b0;

            case (state)
                S_LOW: begin
                    if (valid_in && (in_idx < INPUT_NUM)) begin
                        act_reg <= data_in;

                        // ? ??? ? ????? acc ???
                        if (frame_clear_req) begin
                            pe_clear       <= 1'b1;
                            frame_clear_req<= 1'b0;
                        end

                        // low ?? ??
                        pe_phase <= 1'b0;
                        pe_valid <= 1'b1;

                        // ?? ??? high word(5~9) ????
                        w_addr   <= {in_idx, 1'b1};

                        state    <= S_HIGH;
                    end
                end

                S_HIGH: begin
                    // high ?? ??
                   pe_phase <= 1'b1;
                   pe_valid <= 1'b1;

                   if (in_idx == INPUT_NUM-1) begin
                      // ??? ?? ?? ?? ? ?? ??
                      out_idx <= 0;
                      state   <= S_WAIT_1;
                   end else begin
                      in_idx  <= in_idx + 1'b1;
                      // ?? ??? even word ????
                      w_addr  <= {in_idx + 1'b1, 1'b0};
                      state   <= S_LOW;
                   end
                end // case: S_HIGH

	      S_WAIT_1: begin
		 state <= S_WAIT_2;
	      end
	      S_WAIT_2: begin
		 state <= S_WRITE;
	      end

              S_WRITE: begin
                 res_valid <= 1'b1;

                 case (out_idx)
                   4'd0: res_data <= acc0_low;
                        4'd1: res_data <= acc1_low;
                        4'd2: res_data <= acc2_low;
                        4'd3: res_data <= acc3_low;
                        4'd4: res_data <= acc4_low;
                        4'd5: res_data <= acc0_high;
                        4'd6: res_data <= acc1_high;
                        4'd7: res_data <= acc2_high;
                        4'd8: res_data <= acc3_high;
                        4'd9: res_data <= acc4_high;
                        default: res_data <= {OUT_BITS{1'b0}};
                    endcase

                    if (out_idx == OUTPUT_NUM-1) begin
                        fc_done        <= 1'b1;
                        in_idx         <= 0;
                        frame_clear_req<= 1'b1;  // ?? ??? ? ???? clear
                        w_addr         <= {WE_ADDR_WIDTH{1'b0}};
                        state          <= S_LOW;
                        pe_clear       <= 1'b1;
                    end else begin
                        out_idx <= out_idx + 1'b1;
                    end
                end

                default: state <= S_LOW;
            endcase
        end
    end

endmodule
