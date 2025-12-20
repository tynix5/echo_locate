module sync_adc (
    input clk,                      // 100 MHz
    input rst_n,                    // reset button
    input [2:0] adc_miso,           // MISO lines for sample streams
    output adc_sclk,                // universal CLK line for ADCs
    output adc_mosi,                // universal MOSI line for ADCs
    output [2:0] adc_cs,            // ADC chip select
    output stm_sclk,                // STM32 CLK
    output stm_mosi,                // STM32 MOSI
    output stm_cs                   // STM32 chip select
);

    // STM32 SPI must be fast enough to transfer all data between consecutive sample_done signals
    // At 1MHz SPI, t = 48 bits / 5,000,000 bits/s = 0.0000096 s
    // At 40kHz sample speed, t = 1 / 400000 Hz = 0.000025 s

    localparam F_CLK = 100_000_000;
    localparam F_SAMPLE = 40_000;                       // sampling frequency (Hz)
    localparam ADC_MOD = F_CLK / F_SAMPLE;

    localparam S_IDLE = 1'b0;
    localparam S_SAMPLE = 1'b1;

    localparam N_ADC = 3;                               // # of ADCs
    localparam W_ADC = 12;                              // 12-bit ADC

    localparam MCP_PAYLOAD_SIZE = 2 ** $clog2(W_ADC);               // 16 bit space for each sample
    localparam STM_PAYLOAD_SIZE = N_ADC * MCP_PAYLOAD_SIZE;         // 3 samples * 16 bits each

    localparam CMD_SPI = {4'b1101, {12{1'b0}}};         // single-ended mode, channel 0, MSB first


    // synchronize raw external reset
    wire rst, rsync;
    synchronizer #(.SYNC_STAGES(2)) reset_synchronizer(.clk(clk), .rst(1'b0), .async_in(rst_n), .sync_out(rsync));

    // sample ADCs at 40kHz
    wire sample_trigger;
    mod #(.MOD(ADC_MOD)) sample_tim(.clk(clk), .rst(rst), .cen(1'b1), .q(), .sync_ovf(sample_trigger));

    wire sample_done;
    reg [STM_PAYLOAD_SIZE-1:0] samples_reg;         // current samples from all 3 microphones

    // MCP3202 ADC SPI units (may need to change order of samples_reg for STM32 DMA)
    spi_master #(.F_SPI(1_000_000), .CPHA(1'b0), .CPOL(1'b0), .N(MCP_PAYLOAD_SIZE)) adc0(.clk(clk), .rst(rst), .start(sample_trigger), .data_in(CMD_SPI), 
                                                                                        .miso(adc_miso[0]), .sclk(adc_sclk), .mosi(adc_mosi), 
                                                                                        .cs(adc_cs[0]), .done(sample_done), 
                                                                                        .data_out(samples_reg[MCP_PAYLOAD_SIZE-1:0]));

    spi_master #(.F_SPI(1_000_000), .CPHA(1'b0), .CPOL(1'b0), .N(MCP_PAYLOAD_SIZE)) adc1(.clk(clk), .rst(rst), .start(sample_trigger), .data_in(CMD_SPI), 
                                                                                        .miso(adc_miso[1]), .sclk(), .mosi(), .cs(adc_cs[1]), .done(), 
                                                                                        .data_out(samples_reg[2*MCP_PAYLOAD_SIZE-1:MCP_PAYLOAD_SIZE]));
    
    spi_master #(.F_SPI(1_000_000), .CPHA(1'b0), .CPOL(1'b0), .N(MCP_PAYLOAD_SIZE)) adc2(.clk(clk), .rst(rst), .start(sample_trigger), .data_in(CMD_SPI), 
                                                                                        .miso(adc_miso[2]), .sclk(), .mosi(), .cs(adc_cs[2]), .done(), 
                                                                                        .data_out(samples_reg[STM_PAYLOAD_SIZE-1:2*MCP_PAYLOAD_SIZE]));

    reg stm_trigger;

    // STM32 SPI unit
    spi_master #(.F_SPI(5_000_000), .CPHA(1'b0), .CPOL(1'b0), .N(STM_PAYLOAD_SIZE)) stm(.clk(clk), .rst(rst), .start(stm_trigger), .data_in(samples_reg), 
                                                                                       .miso(), .sclk(stm_sclk), .mosi(stm_mosi), .cs(stm_cs), .done(), 
                                                                                       .data_out());

    
    reg state;

    // sampling logic
    always @(posedge clk) begin

        // initialize all components to known-state
        if (rst) begin
            stm_trigger <= 1'b0;
            state <= S_IDLE;
        end
        else begin
            
            case (state)

                // wait for sample trigger (every 40kHz)
                S_IDLE: begin

                    stm_trigger <= 1'b0;

                    if (sample_trigger) begin

                        state <= S_SAMPLE;
                    end
                end

                // wait for sample to complete, then signal data ready for STM32 SPI
                S_SAMPLE: begin

                    if (sample_done) begin

                        stm_trigger <= 1'b1;
                        state <= S_IDLE;
                    end
                end
            endcase
        end
    end

    assign rst = ~rsync;

endmodule