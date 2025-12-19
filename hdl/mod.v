module mod #(

    parameter MOD = 256         // counter counts from 0 to MOD-1
)(

    input clk,                  // clock
    input rst,                  // reset
    input cen,                  // count enable
    output [WIDTH-1:0] q,       // counter output
    output sync_ovf             // synchronous overflow enable (for chaining)
);

    localparam WIDTH = $clog2(MOD);
    
    reg [WIDTH:0] q_reg;

    always @(posedge clk or posedge rst) begin
        if (rst)
            q_reg <= {WIDTH{1'b0}};         // reset counter and overflow
        else begin
            q_reg <= q_reg;     // default state

            if (cen) begin
                q_reg <= q_reg + 1'b1;      // increment when counter enabled

                if (q_reg == MOD-1)         // if reached MOD, reset
                    q_reg <= {WIDTH{1'b0}};
            end
        end
    end

    assign q = q_reg;
    assign sync_ovf = (q_reg == MOD-1) ? cen : 1'b0;      // only enable synchronous overflow when counter is at MOD-1 and count enabled

endmodule