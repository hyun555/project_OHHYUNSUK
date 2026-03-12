module comparator #(
    parameter DATA_BITS  = 32,   // FC ?? ??? 32
    parameter NUM_CLASS  = 10    
)(
    input                        clk,
    input                        rst_n,

    input                        valid_in,           
    input  signed [DATA_BITS-1:0] data_in,           

    output reg  [3:0]            decision,           // max? ?? 
    output reg                   valid_out,
output reg [9:0] decision_addr           
);

    // ???? ? ?? ? ???
    reg signed [DATA_BITS-1:0] max_val;
    // ? ???? ?? ??? ????? ????
    reg [3:0]                  max_idx;

    // ???? ? ?? ???? ???? ??? (0~NUM_CLASS-1)
    reg [3:0]                  count;

    // ??? ????? ??? ? ?? ?, ?? ???? ??? ???? ?? ???
    reg                        done_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            max_val      <= {DATA_BITS{1'b0}};
            max_idx      <= 4'd0;
            count        <= 4'd0;
            decision     <= 4'd0;
            valid_out    <= 1'b0;
            done_pending <= 1'b0;
        end else begin
            valid_out <= 1'b0; 

            if (valid_in) begin
                if (count == 4'd0) begin
                    // ? ?? ??? ??? max ? ???
                    max_val <= data_in;
                    max_idx <= 4'd0;
                end else begin
                    // ? ?? ???? ?? max ? ??
                    if (data_in >= max_val) begin
                        max_val <= data_in;
                        max_idx <= count;  // ?? ??? ?? ??? ???
                    end
                end

                // ???(10??) ??? ?? ??
                if (count == NUM_CLASS-1) begin
                    // ???? ??/?? ??? max_idx ? ?? ???? decision ?? ??? ??
                    done_pending <= 1'b1;
                    count        <= 4'd0;    // ?? digit ? ?? ??? ??
                end else begin
                    // ?? 10? ? ? ???? ??? ??
                    count <= count + 1'b1;
                end
            end

           // 10? ?? ? ?? ?? ???? ?? ??
           if (done_pending) begin
              decision     <= max_idx;
              valid_out    <= 1'b1;  // ? ???? decision ??
              done_pending <= 1'b0;
              // ?? digit ? ?? max ????
              // ?? valid_in==1 ???? ?? count==0? ? ?? ???
           end
        end
    end // always @ (posedge clk or negedge rst_n)

   always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin
	 decision_addr <= 0;
      end
      else begin
	 if(valid_out) begin
	    decision_addr <= decision_addr + 1;
	 end
      end
   end

endmodule

