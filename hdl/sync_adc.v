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

    // STM32 SPI must be fast enough to transfer all data between consecutive adc_sample_done signals
    // STM32 SPI expects 16 bit words, so for each group of ADC samples, there must be 3 separate transfers
    // There is a small delay between these transfers (~2 SPI clock cycles)
    // At 5MHz SPI, t = (48 bits data + 2 bits delay + 2 bit delay + 2 bits delay) / 5,000,000 bits/s = 0.0000108 s
    // At 40kHz sample speed, t = 1 / 400000 Hz = 0.000025 s

    localparam F_CLK = 100_000_000;                     // FPGA fabric speed
    localparam F_SAMPLE = 40_000;                       // sampling frequency (Hz)
    localparam ADC_MOD = F_CLK / F_SAMPLE;              // required 100 MHz counts to reach equivalent 40kHz clock

    localparam F_STM_SPI = 5_000_000;                   // STM32 SPI frequency
    localparam F_ADC_SPI = 1_000_000;                   // MCP3202 SPI frequency

    localparam CH_TRANS_DELAY = 2 * F_CLK / F_STM_SPI;  // channel transfer delay --> delay between sending ADCn sample and ADC(n+1) sample
                                                        // here, this is 2 SPI cycles to ensure STM32 detects CS release before beginning of next transfer

    localparam N_ADC = 3;                               // # of ADCs
    localparam W_ADC = 12;                              // 12-bit ADC

    localparam W_ADC_WORD = 2 ** $clog2(W_ADC);         // 16 bit space for each sample

    localparam MCP_CMD = {4'b1101, {12{1'b0}}};         // single-ended mode, channel 0, MSB first

    /************** FSM States ************/
    localparam S_ADC_IDLE = 1'b0;                       // waiting for 40kHz timer
    localparam S_ADC_BUSY = 1'b1;                       // simultaneously sample ADCs

    localparam S_STM_IDLE = 1'b0;                      // wait here until ADCs have been sampled
    localparam S_STM_BUSY = 1'b1;                      // transferring ADC samples
    /************ End FSM States **********/


    // synchronize raw external reset
    wire rst, rsync;
    synchronizer #(.SYNC_STAGES(2)) reset_synchronizer(.clk(clk), .rst(1'b0), .async_in(rst_n), .sync_out(rsync));

    // sample ADCs at 40kHz
    wire adc_trigger;
    mod #(.MOD(ADC_MOD)) sample_tim(.clk(clk), .rst(rst), .cen(1'b1), .q(), .sync_ovf(adc_trigger));

    // timer to control delay between consecutive SPI transfers (per group sample) with STM32
    reg delay_en;
    wire delay_done;
    mod #(.MOD(CH_TRANS_DELAY)) delay_tim(.clk(clk), .rst(rst), .cen(delay_en), .q(), .sync_ovf(delay_done));

    wire adc_sample_done;
    wire [W_ADC_WORD-1:0] samples_reg [N_ADC-1:0];         // current samples from all 3 microphones

    // MCP3202 ADC SPI units (may need to change order of samples_reg for STM32 DMA)
    spi_master #(.F_SPI(F_ADC_SPI), .CPHA(1'b0), .CPOL(1'b0), .N(W_ADC_WORD)) adc0(.clk(clk), .rst(rst), .start(adc_trigger), .data_in(MCP_CMD), 
                                                                                        .miso(adc_miso[0]), .sclk(adc_sclk), .mosi(adc_mosi), 
                                                                                        .cs(adc_cs[0]), .done(adc_sample_done), 
                                                                                        .data_out(samples_reg[0]));

    spi_master #(.F_SPI(F_ADC_SPI), .CPHA(1'b0), .CPOL(1'b0), .N(W_ADC_WORD)) adc1(.clk(clk), .rst(rst), .start(adc_trigger), .data_in(MCP_CMD), 
                                                                                        .miso(adc_miso[1]), .sclk(), .mosi(), .cs(adc_cs[1]), .done(), 
                                                                                        .data_out(samples_reg[1]));
    
    spi_master #(.F_SPI(F_ADC_SPI), .CPHA(1'b0), .CPOL(1'b0), .N(W_ADC_WORD)) adc2(.clk(clk), .rst(rst), .start(adc_trigger), .data_in(MCP_CMD), 
                                                                                        .miso(adc_miso[2]), .sclk(), .mosi(), .cs(adc_cs[2]), .done(), 
                                                                                        .data_out(samples_reg[2]));

    reg stm_trigger, stm_start;
    wire stm_sample_done;
    reg [1:0] stm_curr_sample;

    // STM32 SPI unit
    spi_master #(.F_SPI(F_STM_SPI), .CPHA(1'b0), .CPOL(1'b0), .N(W_ADC_WORD)) stm(.clk(clk), .rst(rst), .start(stm_trigger), .data_in(samples_reg[stm_curr_sample]), 
                                                                                       .miso(), .sclk(stm_sclk), .mosi(stm_mosi), .cs(stm_cs), .done(stm_sample_done), 
                                                                                       .data_out());

    reg state_adc;
    reg state_stm;

    // sampling logic
    always @(posedge clk) begin

        // initialize all components to known-state
        if (rst) begin
            stm_start <= 1'b0;
            state_adc <= S_ADC_IDLE;
        end
        else begin
            
            case (state_adc)

                // wait for sample trigger (every 40kHz)
                S_ADC_IDLE: begin

                    stm_start <= 1'b0;

                    if (adc_trigger) begin
                        state_adc <= S_ADC_BUSY;
                    end
                end

                // wait for sample to complete, then signal data ready for STM32 SPI
                S_ADC_BUSY: begin

                    if (adc_sample_done) begin

                        stm_start <= 1'b1;
                        state_adc <= S_ADC_IDLE;
                    end
                end
            endcase
        end
    end

    always @(posedge clk) begin

        if (rst) begin

            stm_trigger <= 1'b0;
            stm_curr_sample <= 2'b0;
            delay_en <= 1'b0;
            state_stm <= S_STM_IDLE;
        end
        else begin


            case (state_stm)

                S_STM_IDLE: begin

                    if (stm_start) begin            // if ADC samples are ready

                        stm_trigger <= 1'b1;        // start first transfer
                        stm_curr_sample <= 2'b0;    // using ADC0 sample
                        state_stm <= S_STM_BUSY;
                    end

                end

                S_STM_BUSY: begin
                    
                    stm_trigger <= 1'b0;                            // wait until this transfer is complete

                    if (stm_sample_done) begin                      // if transfer has completed...

                        if (stm_curr_sample == 2'b10)               // if last sample was transferred...
                            state_stm <= S_STM_IDLE;                // idle until next samples are ready
                        else
                            delay_en <= 1'b1;                       // start delay counter
                    end

                    if (delay_done) begin                           // if there has been a 2 SPI clock cycle delay

                        delay_en <= 1'b0;                           // disable delay timer
                        stm_trigger <= 1'b1;                        // start next transfer
                        stm_curr_sample <= stm_curr_sample + 2'b1;  // move to next sample
                    end
                end

            endcase
        end

    end


    assign rst = ~rsync;

endmodule