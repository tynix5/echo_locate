module spi_master #(

    parameter F_SPI = 1_000_000,   
    parameter CPOL = 1'b0,      // clock polarity
    parameter CPHA = 1'b0,      // clock phase
    parameter N = 8             // number of bits in transmission
)(

    input clk,                  // clock
    input rst,                  // reset
    input start,                // initiate transmission
    input [N-1:0] data_in,      // data to be sent
    input miso,                 // spi line
    output sclk,                // spi clock
    output mosi,                // spi line
    output cs,                  // chip select
    output done,                // transmission complete
    output [N-1:0] data_out     // data received
);

    localparam F_CLK = 100_000_000;
    localparam SPI_TICK = F_CLK / F_SPI / 2;        // frequency spi clock should be toggled

    localparam N_BITS = $clog2(N);

    // FSM states
    localparam STATE_IDLE = 1'b0;
    localparam STATE_BUSY = 1'b1;
    // SPI modes
    localparam SPI_MODE0 = 2'b00;
    localparam SPI_MODE1 = 2'b01;
    localparam SPI_MODE2 = 2'b10;
    localparam SPI_MODE3 = 2'b11;

    reg state;              // FSM state reg
    wire [1:0] spi_mode;    // CPOL and CPHA state

    reg sclk_reg, mosi_reg, cs_reg; // spi output lines

    reg [N-1:0] do_reg, do_latch;     // data out register and latch
    reg [N-1:0] di_latch;             // data in latch

    reg spi_en;                 // spi clock enable
    wire sclk_edge;             // spi clock edge detection (transitions)
    reg done_reg;               // bit counter overflow

    reg first_edge;
    reg [N_BITS:0] bit_cnt;     // current bit count

    mod #(.MOD(SPI_TICK)) spi_sclk_mod(.clk(clk), .rst(rst), .cen(spi_en), .sync_ovf(sclk_edge));
    
    always @(posedge clk) begin
        if (rst) begin
            // initialize master spi lines
            sclk_reg <= CPOL;
            mosi_reg <= 1'b0;
            cs_reg <= 1'b1;         // device not selected
            spi_en <= 1'b0;         // disable spi clock

            // reset output registers
            do_reg <= 8'b0;
            do_latch <= 8'b0;

            first_edge <= 1'b1;     // first edge not yet detected
            bit_cnt <= 'b0;         // reset bit counter

            done_reg <= 1'b0;

            state <= STATE_IDLE;    // default idle state
        end
        else begin

            case (state)
                STATE_IDLE: begin

                    first_edge <= 1'b1;
                    sclk_reg <= CPOL;       // clock idle
                    spi_en <= 1'b0;         // disable spi clock
                    cs_reg <= 1'b1;         // device not selected
                    done_reg <= 1'b0;       // no transmission complete
                    state <= STATE_IDLE;    // transition states

                    if (start) begin            // start transmission...
                        spi_en <= 1'b1;         // enable spi clock
                        cs_reg <= 1'b0;         // device selected
                        di_latch <= data_in;    // latch data
                        bit_cnt <= 'b0;         // reset counter
                        state <= STATE_BUSY;    // transition states
                    end
                end
                STATE_BUSY: begin

                    if (sclk_edge)      sclk_reg <= ~sclk_reg;      // toggle clock on edge

                    mosi_reg <= di_latch[N - 1 - bit_cnt];         // put current bit onto line (MSb first)

                    case (spi_mode)
                        SPI_MODE0, SPI_MODE3: begin        // data sample on rising edge and shifted on falling edge

                            if (sclk_edge && sclk_reg && ~first_edge)   // if transitioning to falling edge and first rising edge has already been detected (SPI_MODE3)...
                                bit_cnt <= bit_cnt + 'b1;               // move to next bit
                            else if (sclk_edge && ~sclk_reg) begin      // if transitioning to rising edge...
                                do_reg <= {do_reg[N-2:0], miso};        // sample miso line
                                first_edge <= 1'b0;                     // indicate rising edge has been seen
                            end

                            if (bit_cnt == N-1 && sclk_edge && sclk_reg) begin  // when all bits have been exchanged
                                done_reg <= 1'b1;       // signal done
                                do_latch <= do_reg;     // latch output data
                                state <= STATE_IDLE;    // idle
                            end
                        end
                        SPI_MODE1, SPI_MODE2: begin        // data sampled on falling edge and shifted on rising edge
                            
                            if (sclk_edge && sclk_reg) begin        // if transitioning to falling edge...
                                do_reg <= {do_reg[N-2:0], miso};    // sample miso line
                                first_edge <= 1'b0;                 // indicate falling edge has been seen
                            end
                            else if (sclk_edge && ~sclk_reg && ~first_edge)     // if transitioning to rising edge and falling edge has already been detected (SPI_MODE1)......
                                bit_cnt <= bit_cnt + 'b1;                       // move to next bit (only when not first rising edge)

                            if (bit_cnt == N-1 && sclk_edge && ~sclk_reg) begin  // when all bits have been exchanged
                                done_reg <= 1'b1;       // signal done
                                do_latch <= do_reg;     // latch output data
                                state <= STATE_IDLE;    // idle
                            end
                        end
                    endcase

                end

            endcase
        end
    end

    assign sclk = sclk_reg;
    assign mosi = mosi_reg;
    assign cs = cs_reg;
    assign done = done_reg;
    assign data_out = do_latch;
    assign spi_mode = {CPOL, CPHA};
    
endmodule