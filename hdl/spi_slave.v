// DEMO SPI slave for sync_adc
// returns a 16-bit running count
module spi_slave #(

    parameter CPOL = 1'b0,      // clock polarity
    parameter CPHA = 1'b0       // clock phase
)(
    input clk,                  // clock
    input rst,                  // reset
    input sclk,                 // spi clock
    input mosi,                 // spi mosi line
    input cs,                   // chip select
    output miso                 // spi miso line
);

    // FSM states
    localparam STATE_IDLE = 1'b0;
    localparam STATE_BUSY = 1'b1;
    // SPI modes
    localparam SPI_MODE0 = 2'b00;
    localparam SPI_MODE1 = 2'b01;
    localparam SPI_MODE2 = 2'b10;
    localparam SPI_MODE3 = 2'b11;

    localparam ADC_SAMPLE = 17'b00000101010101010;

    localparam N = 17;
    localparam BITS = $clog2(N);

    reg state;                  // FSM state register
    wire [1:0] spi_mode;        // CPOL and CPHA state

    reg [N-1:0] di_reg;         // data in register

    reg miso_reg;               // slave output line
    reg last_sclk;              // last sclk state

    reg first_edge;             // first rising/falling edge detected register
    reg [BITS-1:0] bit_cnt;     // current bit counter

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // initialize slave on reset
            last_sclk <= CPOL;      // clock idle
            miso_reg <= 1'bz;       // tri state

            // reset registers
            di_reg <= {N{1'b0}};

            bit_cnt <= {BITS{1'b0}};    // reset bit counter
            first_edge <= 1'b1;     // first edge not yet detected

            state <= STATE_IDLE;
        end
        else begin

            // default states
            last_sclk <= sclk;
            miso_reg <= 1'bz;

            case (state)
                STATE_IDLE: begin

                    first_edge <= 1'b1;

                    if (~cs) begin               
                        bit_cnt <= {BITS{1'b0}};      // reset counter
                        state <= STATE_BUSY;    // if device is selected, transmission in progress
                    end
                end
                STATE_BUSY: begin

                    miso_reg <= ADC_SAMPLE[N - bit_cnt - 1];    // Update MISO line when bit shifts on falling edge (MSb first)

                    case (spi_mode)
                        SPI_MODE0, SPI_MODE3: begin        // data sample on rising edge and shifted on falling edge
                            if (last_sclk && ~sclk && ~first_edge) begin    // if falling edge detected and first rising edge has already been detected (SPI_MODE3)...
                                bit_cnt <= bit_cnt + 1'b1;                  // shift to next bit
                            end
                            else if (~last_sclk && sclk) begin      // if rising edge...
                                di_reg <= {di_reg[N-2:0], mosi};    // sample bit
                                first_edge <= 1'b0;                 // indicate rising edge detected
                            end
                            
                        end
                        SPI_MODE1, SPI_MODE2: begin        // data sampled on falling edge and shifted on rising edge
                            if (last_sclk && ~sclk) begin           // if falling edge...
                                di_reg <= {di_reg[N-2:0], mosi};    // sample bit
                                first_edge <= 1'b0;                 // indicate falling edge detected
                            end
                            else if (~last_sclk && sclk && ~first_edge)     // if rising edge detected and first falling edge has already been detected (SPI_MODE1)..
                                bit_cnt <= bit_cnt + 1'b1;                  // shift to next bit
                        end
                    endcase

                    if (cs) begin       // when device is released...
                        state <= STATE_IDLE;            // idle
                    end

                end
            endcase
        end
    end

    assign miso = miso_reg;
    assign spi_mode = {CPOL, CPHA};

endmodule