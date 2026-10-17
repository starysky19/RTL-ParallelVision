`timescale 1ns / 1ps
// Self-checking testbench for the multi-row Sobel filter.
// Default: 8x8 image, ROWS=3  -> 3 passes, last pass has only 2 valid rows,
// so the partial-pass path is exercised. Override IMAGE_W / ROWS to test others
// (Vivado: Simulation Settings -> xsim.simulate.xsim.more_options
//          -generic_top IMAGE_W=5 -generic_top ROWS=2).
module image_filter_tb #(
    parameter integer IMAGE_W = 8,
    parameter integer ROWS    = 3
);
    localparam integer NUM_PIXELS   = IMAGE_W * IMAGE_W;
    localparam integer CLKS_PER_BIT = 16;
    localparam integer NUM_FRAMES   = 3;

    logic clk = 1'b0;
    logic btn_reset = 1'b1;
    logic uart_rxd = 1'b1;
    logic uart_txd;
    logic [15:0] led;

    logic [7:0] img [0:NUM_PIXELS-1];
    logic [7:0] received_byte;
    integer i, r, c, frame, seed, errors;

    image_filter_top #(.IMAGE_W(IMAGE_W), .ROWS(ROWS), .CLKS_PER_BIT(CLKS_PER_BIT)) dut (
        .clk(clk), .btn_reset(btn_reset), .uart_rxd(uart_rxd),
        .uart_txd(uart_txd), .led(led));

    always #5 clk = ~clk;

    // ---- host -> FPGA byte, 8-N-1
    task automatic send_uart_byte(input logic [7:0] value);
        integer k;
        begin
            uart_rxd = 1'b0; repeat (CLKS_PER_BIT) @(posedge clk);
            for (k = 0; k < 8; k = k + 1) begin
                uart_rxd = value[k]; repeat (CLKS_PER_BIT) @(posedge clk);
            end
            uart_rxd = 1'b1; repeat (CLKS_PER_BIT) @(posedge clk);
        end
    endtask

    // ---- FPGA -> host byte. Returns in the MIDDLE of the stop bit, so the
    // next call can never miss the following start bit (the previous version
    // returned half a bit after the stop bit and could miss it).
    task automatic receive_uart_byte(output logic [7:0] value);
        integer k;
        begin
            @(negedge uart_txd);
            repeat (CLKS_PER_BIT + CLKS_PER_BIT/2) @(posedge clk);   // middle of bit 0
            for (k = 0; k < 8; k = k + 1) begin
                value[k] = uart_txd;
                repeat (CLKS_PER_BIT) @(posedge clk);
            end
            if (uart_txd !== 1'b1) begin
                $display("FAIL: framing error (stop bit not 1) on DUT TX line");
                $fatal(1);
            end
        end
    endtask

    // ---- golden 3x3 Sobel, |Gx|+|Gy| saturated to 255, zero padding
    function automatic [7:0] expected_pixel(input integer rr, input integer cc);
        integer a, b, yy, xx, v, gxx, gyy, mag;
        begin
            gxx = 0; gyy = 0;
            for (a = -1; a <= 1; a = a + 1)
                for (b = -1; b <= 1; b = b + 1) begin
                    yy = rr + a; xx = cc + b; v = 0;
                    if (yy >= 0 && yy < IMAGE_W && xx >= 0 && xx < IMAGE_W)
                        v = img[yy*IMAGE_W + xx];
                    case ((a+1)*3 + (b+1))
                        0: begin gxx = gxx - v;     gyy = gyy - v;     end
                        1: begin                    gyy = gyy - 2*v;   end
                        2: begin gxx = gxx + v;     gyy = gyy - v;     end
                        3: begin gxx = gxx - 2*v;                      end
                        5: begin gxx = gxx + 2*v;                      end
                        6: begin gxx = gxx - v;     gyy = gyy + v;     end
                        7: begin                    gyy = gyy + 2*v;   end
                        8: begin gxx = gxx + v;     gyy = gyy + v;     end
                        default: ;
                    endcase
                end
            mag = ((gxx < 0) ? -gxx : gxx) + ((gyy < 0) ? -gyy : gyy);
            expected_pixel = (mag > 255) ? 8'd255 : mag[7:0];
        end
    endfunction

    initial begin
        errors = 0; seed = 32'h1234_5678;
        $display("Starting %0dx%0d UART Sobel test, ROWS=%0d, %0d frames.",
                 IMAGE_W, IMAGE_W, ROWS, NUM_FRAMES);
        repeat (8) @(posedge clk);
        btn_reset = 1'b0;
        repeat (8) @(posedge clk);

        for (frame = 0; frame < NUM_FRAMES; frame = frame + 1) begin
            // frame 0: random, frame 1: checkerboard 0/255 (saturation), frame 2: random
            for (i = 0; i < NUM_PIXELS; i = i + 1) begin
                r = i / IMAGE_W; c = i % IMAGE_W;
                if (frame == 1) img[i] = ((r + c) % 2) ? 8'd255 : 8'd0;
                else            img[i] = $random(seed);
            end

            send_uart_byte(8'hA5);
            for (i = 0; i < NUM_PIXELS - 1; i = i + 1) send_uart_byte(img[i]);
            // The DUT starts the 0x5A header while the last byte's stop bit is
            // still on the wire, so arm the receiver before sending it.
            fork
                receive_uart_byte(received_byte);
                send_uart_byte(img[NUM_PIXELS-1]);
            join
            if (received_byte !== 8'h5A) begin
                $display("FAIL: frame %0d expected header 0x5A, got 0x%02h", frame, received_byte);
                $fatal(1);
            end

            for (i = 0; i < NUM_PIXELS; i = i + 1) begin
                receive_uart_byte(received_byte);
                r = i / IMAGE_W; c = i % IMAGE_W;
                if (received_byte !== expected_pixel(r, c)) begin
                    $display("MISMATCH frame %0d (%0d,%0d): got %0d expected %0d",
                             frame, r, c, received_byte, expected_pixel(r, c));
                    errors = errors + 1;
                end
            end
            repeat (4 * CLKS_PER_BIT) @(posedge clk);
        end

        if (errors == 0)
            $display("PASS: %0d frames x %0d pixels match the 3x3 Sobel reference (ROWS=%0d).",
                     NUM_FRAMES, NUM_PIXELS, ROWS);
        else begin
            $display("FAIL: %0d pixel mismatches.", errors);
            $fatal(1);
        end
        $finish;
    end

    initial begin
        #50000000;
        $display("TIMEOUT: testbench did not complete.");
        $fatal(1);
    end
endmodule
