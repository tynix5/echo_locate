`timescale 10ns/100ps                   // 100 MHz clock

module sync_adc_tb();

    localparam DURATION = 100_000;       // ns

    reg clk, rst;

    wire stm_sclk, stm_mosi, stm_cs;

    wire [2:0] adc_miso;
    wire [2:0] adc_cs;
    wire adc_sclk, adc_mosi;


    // Create 3 ADCs
    spi_slave #(.CPOL(1'b0), .CPHA(1'b0)) adc0(.clk(clk), .rst(rst), .sclk(adc_sclk), .mosi(adc_mosi), .cs(adc_cs[0]), .miso(adc_miso[0]));
    spi_slave #(.CPOL(1'b0), .CPHA(1'b0)) adc1(.clk(clk), .rst(rst), .sclk(adc_sclk), .mosi(adc_mosi), .cs(adc_cs[1]), .miso(adc_miso[1]));
    spi_slave #(.CPOL(1'b0), .CPHA(1'b0)) adc2(.clk(clk), .rst(rst), .sclk(adc_sclk), .mosi(adc_mosi), .cs(adc_cs[2]), .miso(adc_miso[2]));

    // Create master controller
    sync_adc uut(.clk(clk), .rst_n(~rst), .adc_miso(adc_miso), .adc_sclk(adc_sclk), .adc_mosi(adc_mosi), .adc_cs(adc_cs), .stm_sclk(stm_sclk), 
                .stm_mosi(stm_mosi), .stm_cs(stm_cs));

    // Create STM32 slave
    spi_slave #(.CPOL(1'b0), .CPHA(1'b0)) stm(.clk(clk), .rst(rst), .sclk(stm_sclk), .mosi(stm_mosi), .cs(stm_cs), .miso());


    initial begin

        // Create simulation variables
        $dumpvars(0, sync_adc_tb);

        // Run for DURATION
        #(DURATION)

        // Simulation complete
        $display("Done");
        $finish;
    end

    initial begin
        // reset
        clk = 0;
        rst = 1;
        #5
        clk = 1;
        #5
        clk = 0;
        rst = 0;

        // stream of clock pulses (100 MHz)
        forever begin
            #5
            clk = ~clk;
        end
    end

endmodule