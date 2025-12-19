module fifo #(
    
    parameter D_WIDTH = 8,              // data width -> default 8 bits
    parameter A_WIDTH = 4              // address width -> default 4 bits (2^4 locations)
) (
    input clk,                      // clock
    input rst,                      // reset
    input wr,                    // write control line   
    input rd,                     // read control line
    input [D_WIDTH-1:0] w_data,     // write data in
    output empty,              // FIFO empty status
    output full,               // FIFO full status
    output [D_WIDTH-1:0] r_data     // read data out
);


    reg [D_WIDTH-1:0] buff[2**A_WIDTH-1:0];    // FIFO buffer of size 2^address_width x data_width

    reg [A_WIDTH-1:0] w_ptr, w_ptr_next;         // write pointer and next state write pointer
    reg [A_WIDTH-1:0] r_ptr, r_ptr_next;         // read pointer and next state read pointer

    // FIFO full and empty states
    reg empty_reg, empty_next;
    reg full_reg, full_next;

    assign empty = empty_reg;
    assign full = full_reg;

    assign r_data = buff[r_ptr];        // write buffer contents out continuously

    wire wr_en;
    assign wr_en = wr & ~full_reg;

    always @(posedge clk)               // write to buffer only when write is enabled and the buffer is not filled
        if (wr_en)  
            buff[w_ptr] <= w_data;


    always @* begin

        if (rst) begin
            // on reset, reinitiliaze buffer
            w_ptr <= 0;
            r_ptr <= 0;
            empty_reg <= 1'b1;
            full_reg <= 1'b0;
        end
        else begin
            // assign new state to current state
            w_ptr <= w_ptr_next;
            r_ptr <= r_ptr_next;
            full_reg <= full_next;
            empty_reg <= empty_next;
        end
    end

    always @(posedge clk) begin

        // reassign state by default
        r_ptr_next = r_ptr;
        w_ptr_next = w_ptr;
        full_next = full_reg;
        empty_next = empty_reg;

        case ({wr, rd})
            // 2'b00:
                // no op

            2'b01:
                // read
                if (~empty_reg) begin

                    r_ptr_next = r_ptr + 1'b1;          // move read pointer to next location
                    full_next = 1'b0;                   // not full after read
                    if (r_ptr_next == w_ptr) begin      // if new read pointer equals write pointer...
                        empty_next = 1'b1;              // FIFO is empty
                    end
                end

            2'b10:
                // write
                if (~full_reg) begin

                    w_ptr_next = w_ptr + 1'b1;          // move write pointer to next location
                    empty_next = 1'b0;                  // not empty after write
                    if (w_ptr_next == r_ptr) begin      // if new write pointer equals read pointer...
                        full_next = 1'b1;               // FIFO is full
                    end
                end

            2'b11:
                // read and write
                begin
                    w_ptr_next = w_ptr + 1'b1;          // move write pointer
                    r_ptr_next = r_ptr + 1'b1;          // move read pointer
                    // empty and full states remain the same as before
                end
        endcase

    end


endmodule