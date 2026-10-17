# Spartan-7 Multi-Row Feed-Forward Sobel Filter (corrected)

Target: Real Digital Boolean (XC7S50-CSGA324-1), 100 MHz, UART 115200 8-N-1.
PC protocol is unchanged: send `A5` + `IMAGE_W*IMAGE_W` bytes, receive `5A` + the same number of bytes.
Output = `min(255, |Gx|+|Gy|)`, zero padding at all borders. The Python GUI needs no changes.

## Architecture (`ROWS` = output rows per pass)
| Phase | What happens | Clocks per pass |
|-------|--------------|-----------------|
| LOAD  | `ROWS+2` input rows (incl. 1 halo row above and below) copied from frame BRAM into `ROWS+2` row buffers | `(ROWS+2)*IMAGE_W` |
| SWEEP | one *column* per clock is read from every row buffer into a 3-column window; `ROWS` Sobel lanes work in parallel, `ROWS` pixels/clock | `IMAGE_W+7` |
| TX    | the `ROWS x IMAGE_W` result block is sent row-major | UART-limited |

Compute cycles for a 128x128 frame (verified with `sim/datapath_model.py`):

| ROWS | 1 | 2 | 4 | 8 | 16 |
|------|---|---|---|---|----|
| cycles | 66,432 | 41,408 | 28,896 | 22,640 | 19,512 |
| time @100 MHz | 0.66 ms | 0.41 ms | 0.29 ms | 0.23 ms | 0.19 ms |

Resources scale with `ROWS` (`ROWS` Sobel lanes, `2*ROWS+2` small RAMs). `ROWS=4` is a good default; try 8 if you want to push it.

**Honest note on latency:** a 128x128 frame takes ~1.42 s to send and ~1.42 s to return at 115200 baud, so the end-to-end time is UART-bound
(~2.85 s) no matter how many rows you process. More rows reduce on-chip compute time and re-load overhead, not the serial transfer.
To cut end-to-end time: (a) raise the baud rate (e.g. 921600 -> `CLKS_PER_BIT=109`, and `BAUD` in the GUI), or (b) stream rows out while
later rows are still arriving (UART is full duplex; the GUI would then need a reader thread).

## What was fixed vs. the previous version
1. **Memories now synthesize as RAM.** The old async-read `image_mem` / `line0..2` (nine combinational taps) became huge LUT-RAM + muxes.
   Now: synchronous-read block RAM for the frame, one read port per row buffer, and a shift window instead of 9 random reads.
2. **Multi-row.** `ROWS` parallel lanes; the old code reloaded 3 rows for every single output row and had one lane.
3. **Testbench bug.** `receive_uart_byte` returned half a bit after the stop bit, but the DUT starts the next byte within a few clocks,
   so at `CLKS_PER_BIT=16` the TB could miss start bits. It now returns mid-stop-bit and checks the stop bit.
4. **Stronger tests.** Random data, a 0/255 checkerboard (saturation), 3 back-to-back frames, and a size where the last pass is partial.
5. Reset button is now synchronised.
6. **Timing fix (v2):** the Sobel arithmetic is split into 4 register stages (partial sums, differences, absolute values, add+saturate).
   A single-stage version failed 100 MHz timing (WNS -1.179 ns on the 32 `res` endpoints for `ROWS=4`). This adds 3 clocks per pass.

## Vivado steps
1. Create an RTL project for `xc7s50csga324-1`; add `rtl/image_filter_top.sv` (top = `image_filter_top`) and `constraints/boolean_board.xdc`.
2. Parameters: `IMAGE_W=128`, `ROWS=4`, `CLKS_PER_BIT=868` (defaults). `IMAGE_W` must be >= 4.
3. Run Behavioral Simulation first (see `sim/README_SIM.md`), then Synthesis -> Implementation -> Bitstream.
4. `python -m pip install pillow pyserial`, then `python python/fpga_image_filter_gui.py`.

## Validation status
The datapath algorithm (load, sweep, window, borders, partial passes) is verified cycle-by-cycle in Python against a NumPy-style Sobel
reference for IMAGE_W in {4,5,8,13,16,128} and ROWS in {1,2,3,4,8,16}. The SystemVerilog itself has **not** been run through Vivado
in this environment, so run the testbench and check the synthesis log (and confirm the XDC pins against your board revision) before relying on it.
