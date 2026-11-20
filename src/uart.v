/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART with automatic baud-rate detection and fractional (1/16 clock) bit timing.
 *
 * Autobaud: after reset (or a line break, or an explicit unlock) the receiver
 * waits for 0x55 characters.  0x55 sent LSB first produces falling edges at
 * 0, 2, 4, 6 and 8 bit times; the receiver times 8 bit periods, checks that the
 * edges are evenly spaced, and averages four such characters before locking
 * (the edge quantization of one clock is then spread over 32 bit times).  divisor = clocks per bit in
 * Q12.4, so e.g. 50 MHz / 12 Mbaud = 4.1667 is represented exactly enough.
 *
 * The receiver samples the middle of every bit (3-sample majority vote when a bit
 * lasts at least 8 clocks).
 * A line held low for 2^17 clocks (2.6 ms at 50 MHz) is a "break": it unlocks the
 * baud rate.  Supported rates: >= ~9600 baud at 50 MHz.
 */
`default_nettype none

module uart_rx (
    input  wire        clk,
    input  wire        rst,
    input  wire        rxd,
    input  wire        div_set,     // load div_val and lock
    input  wire [15:0] div_val,
    input  wire        unlock,
    output reg  [15:0] div_fx,
    output reg         locked,
    output reg  [7:0]  data,
    output reg         valid,
    output reg         ferr,
    output reg         brk
);
  reg        r1, r2, r3, r4;
  wire       falling = r4 & ~r3;
  wire       maj3    = (r2 & r3) | (r2 & r4) | (r3 & r4);

  // ---------------- autobaud ----------------
  reg [17:0] t;
  reg [2:0]  nedge;
  reg [17:0] f1, f2;
  reg        meas;
  reg [1:0]  nchar;
  reg [17:0] tsum;
  wire [17:0] tsum_n = tsum + t;
  wire [18:0] two_f1 = {f1, 1'b0};
  wire [18:0] two_f2 = {f2, 1'b0};
  wire [18:0] d1 = ({1'b0, f2} > two_f1) ? ({1'b0, f2} - two_f1) : (two_f1 - {1'b0, f2});
  wire [18:0] d2 = ({1'b0, t}  > two_f2) ? ({1'b0, t}  - two_f2) : (two_f2 - {1'b0, t});

  // Majority vote over 3 clocks only when a bit lasts >= 8 clocks; at high baud
  // rates the window would reach into the neighbouring bits.
  wire       maj     = (div_fx >= 16'd128) ? maj3 : r3;

  // ---------------- receiver ----------------
  localparam [1:0] R_IDLE = 2'd0, R_START = 2'd1, R_DATA = 2'd2, R_STOP = 2'd3;
  reg [1:0]  rs;
  reg [16:0] acc;
  reg [2:0]  bitn;
  reg [7:0]  sh;
  reg [16:0] lowcnt;

  always @(posedge clk) begin
    if (rst) begin
      r1 <= 1'b1; r2 <= 1'b1; r3 <= 1'b1; r4 <= 1'b1;
      div_fx <= 16'd0; locked <= 1'b0; data <= 8'd0; valid <= 1'b0; ferr <= 1'b0; brk <= 1'b0;
      t <= 18'd0; nedge <= 3'd0; f1 <= 18'd0; f2 <= 18'd0; meas <= 1'b0;
      nchar <= 2'd0; tsum <= 18'd0;
      rs <= R_IDLE; acc <= 17'd0; bitn <= 3'd0; sh <= 8'd0; lowcnt <= 17'd0;
    end else begin
      r1 <= rxd; r2 <= r1; r3 <= r2; r4 <= r3;
      valid <= 1'b0;
      ferr  <= 1'b0;
      brk   <= 1'b0;

      // break detection (line low for a long time)
      if (r3) begin
        lowcnt <= 17'd0;
      end else if (lowcnt != 17'h1FFFF) begin
        lowcnt <= lowcnt + 17'd1;
      end else begin
        lowcnt <= 17'd0;
        brk    <= 1'b1;
        locked <= 1'b0;
        meas   <= 1'b0;
        nchar  <= 2'd0;
        tsum   <= 18'd0;
        rs     <= R_IDLE;
      end

      if (div_set) begin
        div_fx <= div_val;
        locked <= (div_val >= 16'd48);  // at least 3 clocks per bit
        meas   <= 1'b0;
      end else if (unlock) begin
        locked <= 1'b0;
        meas   <= 1'b0;
        nchar  <= 2'd0;
        tsum   <= 18'd0;
        rs     <= R_IDLE;
      end else if (!locked) begin
        // ------------- baud measurement -------------
        if (!meas) begin
          if (falling) begin
            meas  <= 1'b1;
            t     <= 18'd1;
            nedge <= 3'd0;
          end
        end else begin
          t <= t + 18'd1;
          if (t == 18'h3FFFF) meas <= 1'b0;
          if (falling) begin
            nedge <= nedge + 3'd1;
            case (nedge)
              3'd0: f1 <= t;                         // 2 bit times
              3'd1: begin                            // 4 bit times
                f2 <= t;
                if (d1 > {3'b000, f1[17:2]}) begin   // restart from this edge
                  t <= 18'd1; nedge <= 3'd0;
                end
              end
              3'd2: ;                                // 6 bit times
              default: begin                         // 8 bit times
                meas <= 1'b0;
                if (d2 <= {3'b000, f2[17:2]} && t[17:15] == 3'b000 && t >= 18'd24) begin
                  if (nchar == 2'd3) begin
                    div_fx <= tsum_n[16:1];          // (sum / 32) * 16
                    locked <= 1'b1;
                    nchar  <= 2'd0;
                    tsum   <= 18'd0;
                  end else begin
                    nchar <= nchar + 2'd1;
                    tsum  <= tsum_n;
                  end
                end else begin
                  nchar <= 2'd0;
                  tsum  <= 18'd0;
                end
              end
            endcase
          end
        end
      end else begin
        // ------------- normal reception -------------
        case (rs)
          R_IDLE: if (falling) begin
            acc  <= {2'b00, div_fx[15:1]};          // middle of the start bit
            rs   <= R_START;
          end
          default: begin
            if (acc < 17'd16) begin
              acc <= acc + {1'b0, div_fx} - 17'd16;
              case (rs)
                R_START: begin
                  if (maj) rs <= R_IDLE;   // glitch, not a start bit
                  else begin rs <= R_DATA; bitn <= 3'd0; end
                end
                R_DATA: begin
                  sh   <= {maj, sh[7:1]};
                  bitn <= bitn + 3'd1;
                  if (bitn == 3'd7) rs <= R_STOP;
                end
                default: begin // R_STOP
                  if (maj) begin data <= sh; valid <= 1'b1; end
                  else     ferr <= 1'b1;
                  rs <= R_IDLE;
                end
              endcase
            end else begin
              acc <= acc - 17'd16;
            end
          end
        endcase
      end
    end
  end
endmodule

module uart_tx (
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] div_fx,
    input  wire        start,
    input  wire [7:0]  data,
    output wire        busy,
    output reg         txd
);
  reg [9:0]  sh;
  reg [3:0]  n;
  reg [16:0] acc;
  assign busy = (n != 4'd0);

  always @(posedge clk) begin
    if (rst) begin
      sh <= 10'h3FF; n <= 4'd0; acc <= 17'd0; txd <= 1'b1;
    end else begin
      if (n == 4'd0) begin
        txd <= 1'b1;
        if (start) begin
          sh  <= {1'b1, data, 1'b0};
          n   <= 4'd10;
          acc <= {1'b0, div_fx};
          txd <= 1'b0;
        end
      end else begin
        if (acc <= 17'd16) begin
          acc <= acc + {1'b0, div_fx} - 17'd16;
          sh  <= {1'b1, sh[9:1]};
          n   <= n - 4'd1;
          txd <= (n == 4'd1) ? 1'b1 : sh[1];
        end else begin
          acc <= acc - 17'd16;
        end
      end
    end
  end
endmodule
