module adc_sync_top(
    input clk,
    input rst_n
);

    localparam F_SAMPLE = 40000;

    localparam S_IDLE = 1'b0;
    localparam S_SAMPLE = 1'b1;

    localparam N_MICS = 3;
    localparam BUFF_SIZE = 480;             // samples taken before transfer to STM32 (per mic)

    // synchronize raw external reset
    wire rst, rsync;
    synchronizer #(.SYNC_STAGES(2)) reset_synchronizer(.clk(clk), .rst(1'b0), .async_in(rst_n), .sync_out(rsync));

    // sample ADCs at 40kHz
    wire sample_trigger;
    mod #(.MOD(F_SAMPLE)) sample_tim(.clk(clk), .rst(rst), .cen(1'b1), .q(), .sync_ovf(sample_trigger));

    reg state;

    // sampling logic
    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
        end
        else begin
            
            case (state)
                S_IDLE: begin

                    if (sample_trigger) begin

                        state <= S_SAMPLE;
                    end
                end

                S_SAMPLE: begin

                    if (sample_done) begin

                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end


    // STM32 transfer logic
    always @(posedge clk) begin

    end

    assign rst = ~rsync;
endmodule