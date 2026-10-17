`timescale 1ns/1ps
// ============================================================================
// Multi-row feed-forward Sobel filter, Spartan-7 (Real Digital Boolean).
//
// UART protocol (unchanged): PC -> A5 + IMAGE_W*IMAGE_W bytes
//                            FPGA -> 5A + IMAGE_W*IMAGE_W bytes
//
// Architecture: the frame is processed in passes of ROWS output rows each.
//   LOAD  : the ROWS+2 input rows (ROWS output rows + 1 halo row above/below)
//           are copied from the frame BRAM into ROWS+2 row buffers.
//   SWEEP : one image COLUMN per clock is read from all ROWS+2 buffers (one
//           read port each) and shifted into a 3-column window. ROWS parallel
//           3x3 Sobel lanes produce ROWS output pixels per clock.
//   TX    : the ROWS x IMAGE_W result block is sent row-major over the UART.
// Larger ROWS => more parallel lanes and fewer re-loaded halo rows
// (halo overhead = (ROWS+2)/ROWS). Zero padding at all image borders.
//
// Requirements: IMAGE_W >= 4, ROWS >= 1.
// ============================================================================
module image_filter_top #(
    parameter integer IMAGE_W      = 128,
    parameter integer ROWS         = 4,     // output rows computed in parallel
    parameter integer CLKS_PER_BIT = 868    // 100 MHz / 115200
)(
    input  logic        clk,
    input  logic        btn_reset,
    input  logic        uart_rxd,
    output logic        uart_txd,
    output logic [15:0] led
);
    localparam integer NUM_PIXELS = IMAGE_W*IMAGE_W;
    localparam integer NUM_BUF    = ROWS + 2;
    localparam integer ADDR_W     = $clog2(NUM_PIXELS);
    localparam integer COL_W      = $clog2(IMAGE_W);
    localparam integer WR_LAT     = 7;   // sweep clock (relative to column 0) at which column 0 is written to the result RAM
    localparam integer SW_W       = $clog2(IMAGE_W + WR_LAT + 1);
    localparam integer LDR_W      = $clog2(NUM_BUF);
    localparam integer TXR_W      = (ROWS < 2) ? 1 : $clog2(ROWS);
    localparam integer BASE_W     = $clog2(IMAGE_W + ROWS + 1);

    // ---------------------------------------------------------------- reset
    // Synchronise the push-button; power-up value is "reset asserted".
    logic rst_meta = 1'b1;
    logic rst      = 1'b1;
    always_ff @(posedge clk) begin
        rst_meta <= btn_reset;
        rst      <= rst_meta;
    end

    // ----------------------------------------------------------------- UART
    logic [7:0] rx_data;
    logic [7:0] tx_data;
    logic       rx_valid, tx_start, tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .reset(rst), .rx_serial(uart_rxd),
        .rx_data(rx_data), .rx_valid(rx_valid));

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .reset(rst), .tx_data(tx_data),
        .tx_start(tx_start), .tx_serial(uart_txd), .tx_busy(tx_busy));

    // ------------------------------------------------------------ FSM state
    typedef enum logic [3:0] {
        S_IDLE, S_RECV, S_HDR_START, S_HDR_BUSY, S_HDR_DONE,
        S_LOAD, S_SWEEP,
        S_TX_FETCH, S_TX_LATCH, S_TX_START, S_TX_BUSY, S_TX_DONE
    } state_t;
    state_t state = S_IDLE;

    logic [ADDR_W-1:0] receive_count = '0;
    logic [BASE_W-1:0] base_row      = '0;   // first output row of this pass
    logic [LDR_W-1:0]  ld_row        = '0;   // row-buffer index being loaded
    logic [COL_W-1:0]  ld_col        = '0;
    logic [SW_W-1:0]   sw_cnt        = '0;   // sweep cycle counter
    logic [TXR_W-1:0]  tx_row        = '0;
    logic [COL_W-1:0]  tx_col        = '0;

    // ------------------------------------------- combinational address logic
    integer       src_row, rows_left, rows_this;
    logic         ld_in_range;
    logic [31:0]  ld_addr_full;
    logic [ADDR_W-1:0] ld_addr;
    logic         tx_last_row, last_pass;
    logic [COL_W-1:0]  rb_raddr;

    always_comb begin
        src_row      = integer'(base_row) - 1 + integer'(ld_row);
        ld_in_range  = (src_row >= 0) && (src_row < IMAGE_W);
        ld_addr_full = ld_in_range ? (src_row*IMAGE_W + integer'(ld_col)) : 0;
        ld_addr      = ld_addr_full[ADDR_W-1:0];

        rows_left    = IMAGE_W - integer'(base_row);
        rows_this    = (rows_left >= ROWS) ? ROWS : rows_left;
        tx_last_row  = (integer'(tx_row) == rows_this - 1);
        last_pass    = (integer'(base_row) + ROWS >= IMAGE_W);

        // column read address for the row buffers (pad cycle re-reads col 0)
        rb_raddr     = (sw_cnt >= IMAGE_W) ? '0 : sw_cnt[COL_W-1:0];
    end

    // ------------------------------------------- frame memory (block RAM)
    // Synchronous read => maps to block RAM (the old async read did not).
    (* ram_style = "block" *) logic [7:0] image_mem [0:NUM_PIXELS-1];
    logic [7:0] img_q;
    always_ff @(posedge clk) begin
        if (state == S_RECV && rx_valid)
            image_mem[receive_count] <= rx_data;
        img_q <= image_mem[ld_addr];
    end

    // LOAD pipeline stage: BRAM data arrives one clock after the address.
    logic             s1_v   = 1'b0;
    logic [LDR_W-1:0] s1_row = '0;
    logic [COL_W-1:0] s1_col = '0;
    logic             s1_in  = 1'b0;
    always_ff @(posedge clk) begin
        s1_v   <= (state == S_LOAD);
        s1_row <= ld_row;
        s1_col <= ld_col;
        s1_in  <= ld_in_range;
    end

    // ------------------------------------ row buffers + 3-column window
    logic [NUM_BUF*8-1:0] rb_q_flat;     // column read data of every buffer
    logic [NUM_BUF*8-1:0] win_a_flat;    // oldest column  (x-1)
    logic [NUM_BUF*8-1:0] win_b_flat;    // centre column  (x)
    logic [NUM_BUF*8-1:0] win_c_flat;    // newest column  (x+1)

    genvar gi;
    generate
        for (gi = 0; gi < NUM_BUF; gi = gi + 1) begin : g_buf
            logic [7:0] mem [0:IMAGE_W-1];
            logic [7:0] q;
            logic [7:0] wa, wb, wc;
            always_ff @(posedge clk) begin
                // write (LOAD): out-of-frame rows are stored as zeros
                if (s1_v && (s1_row == gi))
                    mem[s1_col] <= s1_in ? img_q : 8'd0;
                // synchronous read (SWEEP)
                q <= mem[rb_raddr];
            end
            // Column window. sw_cnt==0 clears it (left zero border);
            // sw_cnt 1..IMAGE_W shifts in column sw_cnt-1;
            // sw_cnt==IMAGE_W+1 shifts in zeros (right zero border).
            always_ff @(posedge clk) begin
                if (state == S_SWEEP) begin
                    if (sw_cnt == 0) begin
                        wa <= 8'd0; wb <= 8'd0; wc <= 8'd0;
                    end else if (sw_cnt <= IMAGE_W) begin
                        wa <= wb; wb <= wc; wc <= q;
                    end else if (sw_cnt == IMAGE_W + 1) begin
                        wa <= wb; wb <= wc; wc <= 8'd0;
                    end
                end
            end
            assign rb_q_flat[gi*8 +: 8]  = q;
            assign win_a_flat[gi*8 +: 8] = wa;
            assign win_b_flat[gi*8 +: 8] = wb;
            assign win_c_flat[gi*8 +: 8] = wc;
        end
    endgenerate

    // --------------------------------- parallel Sobel lanes + result buffers
    // Lane g uses row buffers g, g+1, g+2. The Sobel arithmetic is split into
    // four register stages so each stage is only a few logic levels deep:
    //   S1: weighted partial sums   gxp/gxn/gyp/gyn
    //   S2: signed differences      sx, sy
    //   S3: absolute values         ax, ay
    //   S4: add + saturate          res   -> result RAM
    logic                 ob_we;
    logic [SW_W-1:0]      ob_wfull;
    logic [COL_W-1:0]     ob_wcol;
    logic [ROWS*8-1:0]    ob_q_flat;

    assign ob_we    = (state == S_SWEEP) && (sw_cnt >= WR_LAT);
    assign ob_wfull = sw_cnt - WR_LAT;
    assign ob_wcol  = ob_wfull[COL_W-1:0];

    generate
        for (gi = 0; gi < ROWS; gi = gi + 1) begin : g_lane
            logic [7:0] p00, p01, p02, p10, p12, p20, p21, p22;
            assign p00 = win_a_flat[ gi   *8 +: 8];
            assign p01 = win_b_flat[ gi   *8 +: 8];
            assign p02 = win_c_flat[ gi   *8 +: 8];
            assign p10 = win_a_flat[(gi+1)*8 +: 8];
            assign p12 = win_c_flat[(gi+1)*8 +: 8];
            assign p20 = win_a_flat[(gi+2)*8 +: 8];
            assign p21 = win_b_flat[(gi+2)*8 +: 8];
            assign p22 = win_c_flat[(gi+2)*8 +: 8];

            logic [9:0]         gxp, gxn, gyp, gyn;   // S1
            logic signed [11:0] sx, sy;               // S2
            logic [10:0]        ax, ay;               // S3
            logic [11:0]        mag;
            logic [7:0]         res;                  // S4
            logic [7:0]         ob_mem [0:IMAGE_W-1];
            logic [7:0]         ob_q;

            assign mag = {1'b0, ax} + {1'b0, ay};

            always_ff @(posedge clk) begin
                // S1 (operands are zero-extended to 10 bits by the LHS width)
                gxp <= p02 + {p12, 1'b0} + p22;
                gxn <= p00 + {p10, 1'b0} + p20;
                gyp <= p20 + {p21, 1'b0} + p22;
                gyn <= p00 + {p01, 1'b0} + p02;
                // S2
                sx  <= $signed({2'b00, gxp}) - $signed({2'b00, gxn});
                sy  <= $signed({2'b00, gyp}) - $signed({2'b00, gyn});
                // S3 (|value| <= 1020 fits in 11 bits)
                ax  <= sx[11] ? (~sx[10:0] + 11'd1) : sx[10:0];
                ay  <= sy[11] ? (~sy[10:0] + 11'd1) : sy[10:0];
                // S4
                res <= (mag > 12'd255) ? 8'hFF : mag[7:0];

                if (ob_we) ob_mem[ob_wcol] <= res;
                ob_q <= ob_mem[tx_col];
            end
            assign ob_q_flat[gi*8 +: 8] = ob_q;
        end
    endgenerate

    // ----------------------------------------------------------- main FSM
    always_ff @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            receive_count <= '0; base_row <= '0;
            ld_row <= '0; ld_col <= '0; sw_cnt <= '0;
            tx_row <= '0; tx_col <= '0;
            tx_data <= 8'd0; tx_start <= 1'b0;
        end else begin
            tx_start <= 1'b0;
            case (state)
                S_IDLE: if (rx_valid && rx_data == 8'hA5) begin
                    receive_count <= '0;
                    state <= S_RECV;
                end

                S_RECV: if (rx_valid) begin           // image_mem written above
                    if (receive_count == NUM_PIXELS-1) state <= S_HDR_START;
                    else receive_count <= receive_count + 1'b1;
                end

                S_HDR_START: if (!tx_busy) begin
                    tx_data  <= 8'h5A;
                    tx_start <= 1'b1;
                    state    <= S_HDR_BUSY;
                end
                S_HDR_BUSY: if (tx_busy) state <= S_HDR_DONE;
                S_HDR_DONE: if (!tx_busy) begin
                    base_row <= '0; ld_row <= '0; ld_col <= '0;
                    state <= S_LOAD;
                end

                // Issue (ROWS+2)*IMAGE_W BRAM reads, one per clock.
                S_LOAD: begin
                    if (ld_col == IMAGE_W-1) begin
                        ld_col <= '0;
                        if (ld_row == NUM_BUF-1) begin
                            ld_row <= '0; sw_cnt <= '0;
                            state <= S_SWEEP;
                        end else ld_row <= ld_row + 1'b1;
                    end else ld_col <= ld_col + 1'b1;
                end

                // IMAGE_W+WR_LAT clocks: read, window shift, Sobel pipeline, result write.
                S_SWEEP: begin
                    if (sw_cnt == IMAGE_W + WR_LAT - 1) begin
                        tx_row <= '0; tx_col <= '0;
                        state  <= S_TX_FETCH;
                    end else sw_cnt <= sw_cnt + 1'b1;
                end

                S_TX_FETCH: state <= S_TX_LATCH;       // result RAM read address settles
                S_TX_LATCH: begin
                    tx_data <= ob_q_flat[tx_row*8 +: 8];
                    state   <= S_TX_START;
                end
                S_TX_START: if (!tx_busy) begin
                    tx_start <= 1'b1;
                    state    <= S_TX_BUSY;
                end
                S_TX_BUSY: if (tx_busy) state <= S_TX_DONE;
                S_TX_DONE: if (!tx_busy) begin
                    if (tx_col == IMAGE_W-1) begin
                        tx_col <= '0;
                        if (tx_last_row) begin
                            if (last_pass) state <= S_IDLE;
                            else begin
                                base_row <= base_row + ROWS;
                                ld_row <= '0; ld_col <= '0;
                                state <= S_LOAD;
                            end
                        end else begin
                            tx_row <= tx_row + 1'b1;
                            state  <= S_TX_FETCH;
                        end
                    end else begin
                        tx_col <= tx_col + 1'b1;
                        state  <= S_TX_FETCH;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ----------------------------------------------------------------- LEDs
    logic [15:0] rc16, br16;
    assign rc16 = receive_count;
    assign br16 = base_row;
    always_comb begin
        led[7:0]   = tx_data;
        led[11:8]  = state;
        led[15:12] = (state == S_RECV) ? rc16[3:0] : br16[3:0];
    end
endmodule

// ============================================================================
// UART receiver, 8-N-1
// ============================================================================
module uart_rx #(parameter integer CLKS_PER_BIT = 868)(
    input  logic       clk, reset, rx_serial,
    output logic [7:0] rx_data,
    output logic       rx_valid
);
    localparam integer CNT_W = (CLKS_PER_BIT < 2) ? 1 : $clog2(CLKS_PER_BIT + 1);
    typedef enum logic [1:0] {RX_IDLE, RX_START, RX_DATA, RX_STOP} rx_state_t;
    rx_state_t state = RX_IDLE;
    logic rx_meta = 1'b1, rx_sync = 1'b1;
    logic [CNT_W-1:0] clk_count = '0;
    logic [2:0] bit_index = 0;
    logic [7:0] shift_reg = 0;
    always_ff @(posedge clk) begin
        if (reset) begin
            rx_meta <= 1; rx_sync <= 1; state <= RX_IDLE; clk_count <= '0;
            bit_index <= 0; shift_reg <= 0; rx_data <= 0; rx_valid <= 0;
        end else begin
            rx_meta <= rx_serial; rx_sync <= rx_meta; rx_valid <= 0;
            case (state)
                RX_IDLE: if (!rx_sync) begin clk_count <= CLKS_PER_BIT/2; state <= RX_START; end
                RX_START: if (clk_count == 0) begin
                    if (!rx_sync) begin clk_count <= CLKS_PER_BIT-1; bit_index <= 0; state <= RX_DATA; end
                    else state <= RX_IDLE;
                end else clk_count <= clk_count - 1'b1;
                RX_DATA: if (clk_count == 0) begin
                    shift_reg[bit_index] <= rx_sync; clk_count <= CLKS_PER_BIT-1;
                    if (bit_index == 7) state <= RX_STOP; else bit_index <= bit_index + 1'b1;
                end else clk_count <= clk_count - 1'b1;
                RX_STOP: if (clk_count == 0) begin
                    if (rx_sync) begin rx_data <= shift_reg; rx_valid <= 1; end
                    state <= RX_IDLE;
                end else clk_count <= clk_count - 1'b1;
                default: state <= RX_IDLE;
            endcase
        end
    end
endmodule

// ============================================================================
// UART transmitter, 8-N-1
// ============================================================================
module uart_tx #(parameter integer CLKS_PER_BIT = 868)(
    input  logic       clk, reset,
    input  logic [7:0] tx_data,
    input  logic       tx_start,
    output logic       tx_serial,
    output logic       tx_busy
);
    localparam integer CNT_W = (CLKS_PER_BIT < 2) ? 1 : $clog2(CLKS_PER_BIT + 1);
    logic [CNT_W-1:0] clk_count = '0;
    logic [3:0] bit_index = 0;
    logic [9:0] shift_reg = 10'h3ff;
    always_ff @(posedge clk) begin
        if (reset) begin clk_count <= '0; bit_index <= 0; shift_reg <= 10'h3ff; tx_serial <= 1; tx_busy <= 0; end
        else if (!tx_busy) begin
            tx_serial <= 1;
            if (tx_start) begin
                shift_reg <= {1'b1, tx_data, 1'b0}; tx_serial <= 0;
                tx_busy <= 1; clk_count <= CLKS_PER_BIT-1; bit_index <= 0;
            end
        end else if (clk_count == 0) begin
            if (bit_index == 9) begin tx_serial <= 1; tx_busy <= 0; end
            else begin
                tx_serial <= shift_reg[1]; shift_reg <= {1'b1, shift_reg[9:1]};
                bit_index <= bit_index + 1'b1; clk_count <= CLKS_PER_BIT-1;
            end
        end else clk_count <= clk_count - 1'b1;
    end
endmodule
