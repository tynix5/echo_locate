module synchronizer #(
    parameter SYNC_STAGES = 2
)(
    input clk,
    input rst,
    input async_in,
    output rise_edge_tick,
    output fall_edge_tick,
    output sync_out
);

    reg [SYNC_STAGES-1:0] sync;     // synchronizer stages
    reg sync_reg, rise_reg, fall_reg;

    assign sync_out = sync_reg;      // output at end of synchronizer
    assign rise_edge_tick = rise_reg;
    assign fall_edge_tick = fall_reg;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // reset stages
            sync <= {SYNC_STAGES{1'b0}};
            sync_reg <= 1'b0;
            rise_reg <= 1'b0;
            fall_reg <= 1'b0;
        end
        else begin
            sync_reg <= sync[0];    // feed output
            sync <= {async_in, sync[SYNC_STAGES-1:1]};      // new input to beginning of synchronizer, shift contents right
            rise_reg <= ~sync_reg & sync[0];    // rising edge when current state is low and next state is high
            fall_reg <= sync_reg & ~sync[0];    // falling edge when current state is high and next state is low
        end
    end

endmodule