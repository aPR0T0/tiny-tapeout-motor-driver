/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * Small sequential arithmetic units shared by the FOC engine.
 */
`default_nettype none

// -----------------------------------------------------------------------------
// mul16: 16x16 -> 32 multiplier, one bit per clock (17 clocks).
//   a is unsigned, b is signed.
// -----------------------------------------------------------------------------
module mul16 (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [15:0] a,         // unsigned
    input  wire [15:0] b,         // signed
    output wire [31:0] p,         // valid from `done` until the next start
    output reg         done
);
  reg [15:0] ua;
  reg [31:0] acc;
  reg        neg;
  reg [4:0]  n;
  reg        busy;

  wire [15:0] b_abs = b[15] ? (~b + 16'd1) : b;
  wire [16:0] sum   = {1'b0, acc[31:16]} + (acc[0] ? {1'b0, ua} : 17'd0);
  assign p = neg ? (~acc + 32'd1) : acc;

  always @(posedge clk) begin
    if (rst) begin
      ua <= 16'd0; acc <= 32'd0; neg <= 1'b0; n <= 5'd0; busy <= 1'b0; done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start) begin
        ua   <= a;
        acc  <= {16'd0, b_abs};
        neg  <= b[15];
        n    <= 5'd16;
        busy <= 1'b1;
      end else if (busy) begin
        acc <= {sum, acc[15:1]};
        n   <= n - 5'd1;
        if (n == 5'd1) begin
          busy <= 1'b0;
          done <= 1'b1;
        end
      end
    end
  end
endmodule

// -----------------------------------------------------------------------------
// cordic_rot: rotate (x, y) by angle `ang` (65536 = 360 deg), 16 iterations.
//   Output is scaled by the CORDIC gain K = 1.64676 (callers pre-scale by 1/K)
//   and saturated to 16 bits.
// -----------------------------------------------------------------------------
module cordic_rot (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [15:0] xin,
    input  wire [15:0] yin,
    input  wire [15:0] ang,
    output wire [15:0] xout,      // valid from `done` until the next start
    output wire [15:0] yout,
    output reg         done
);
  reg signed [17:0] x, y;
  reg signed [16:0] z;
  reg [4:0]         i;
  reg               busy;

  function [15:0] atan_tab;
    input [3:0] k;
    case (k)
      4'd0:  atan_tab = 16'd8192;
      4'd1:  atan_tab = 16'd4836;
      4'd2:  atan_tab = 16'd2555;
      4'd3:  atan_tab = 16'd1297;
      4'd4:  atan_tab = 16'd651;
      4'd5:  atan_tab = 16'd326;
      4'd6:  atan_tab = 16'd163;
      4'd7:  atan_tab = 16'd81;
      4'd8:  atan_tab = 16'd41;
      4'd9:  atan_tab = 16'd20;
      4'd10: atan_tab = 16'd10;
      4'd11: atan_tab = 16'd5;
      4'd12: atan_tab = 16'd3;
      4'd13: atan_tab = 16'd1;
      4'd14: atan_tab = 16'd1;
      default: atan_tab = 16'd0;
    endcase
  endfunction

  function [15:0] sat16;
    input signed [17:0] v;
    if (v > 18'sd32767)       sat16 = 16'h7FFF;
    else if (v < -18'sd32768) sat16 = 16'h8000;
    else                      sat16 = v[15:0];
  endfunction

  // quadrant pre-rotation: angles in [90, 270) deg are rotated by 180 first
  wire               flip = (ang[15:14] == 2'b01) || (ang[15:14] == 2'b10);
  wire signed [17:0] xs   = {{2{xin[15]}}, xin};
  wire signed [17:0] ys   = {{2{yin[15]}}, yin};
  wire [15:0]        za   = flip ? (ang + 16'h8000) : ang;

  wire signed [17:0] xsh  = x >>> i[3:0];
  wire signed [17:0] ysh  = y >>> i[3:0];
  wire signed [16:0] at   = {1'b0, atan_tab(i[3:0])};
  wire               pos  = !z[16];

  assign xout = sat16(x);
  assign yout = sat16(y);

  always @(posedge clk) begin
    if (rst) begin
      x <= 0; y <= 0; z <= 0; i <= 0; busy <= 1'b0; done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start) begin
        x    <= flip ? -xs : xs;
        y    <= flip ? -ys : ys;
        z    <= {za[15], za};
        i    <= 5'd0;
        busy <= 1'b1;
      end else if (busy) begin
        x <= pos ? x - ysh : x + ysh;
        y <= pos ? y + xsh : y - xsh;
        z <= pos ? z - at  : z + at;
        i <= i + 5'd1;
        if (i == 5'd15) begin
          busy <= 1'b0;
          done <= 1'b1;
        end
      end
    end
  end
endmodule
