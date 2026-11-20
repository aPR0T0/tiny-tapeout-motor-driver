/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * hall_sensor - 3-channel hall input conditioning and rotor angle estimation.
 *   - 2-FF synchronizer + programmable glitch filter (code must be stable N clocks)
 *   - programmable code -> sector map (3 bits per code, codes 0..7)
 *   - direction detection from the sector sequence
 *   - period measurement between edges (24-bit, saturating = standstill)
 *   - speed:  step = (60deg << 16) / period  (angle units per clock, 16.16)
 *   - angle:  at standstill the sector center, while moving the last edge angle
 *             plus interpolated travel (clamped to one sector)
 * Angle units: 65536 = 360 electrical degrees.
 */
`default_nettype none

module hall_sensor (
    input  wire        clk,
    input  wire        rst,
    input  wire [2:0]  hall_in,
    input  wire [7:0]  filt_len,
    input  wire [23:0] hall_map,
    output reg  [2:0]  code,
    output reg  [2:0]  sector,
    output reg         dir,          // 1 = forward (increasing angle)
    output reg         moving,
    output wire        invalid,      // current code is 0 or 7
    output reg         bad_seq,      // 1-cycle pulse: sector jumped
    output wire [15:0] angle,        // estimated angle (no offset applied)
    output wire [15:0] angle_center, // center of the current sector
    output reg  [23:0] period,
    output reg  [29:0] step
);

  // ---------------- input conditioning ----------------
  reg [2:0] s1, s2, cand;
  reg [7:0] fcnt;
  reg       code_valid_once;
  always @(posedge clk) begin
    if (rst) begin
      s1 <= 3'd0; s2 <= 3'd0; cand <= 3'd0; fcnt <= 8'd0;
    end else begin
      s1 <= hall_in;
      s2 <= s1;
      if (s2 != cand) begin
        cand <= s2;
        fcnt <= 8'd0;
      end else if (fcnt != 8'hFF) begin
        fcnt <= fcnt + 8'd1;
      end
    end
  end
  wire       accept   = (s2 == cand) && (fcnt >= filt_len);
  wire [2:0] new_sec  = hall_map[cand*3 +: 3];
  wire       cand_inv = (cand == 3'd0) || (cand == 3'd7);
  assign invalid = (code == 3'd0) || (code == 3'd7);

  // ---------------- sector angle tables ----------------
  function [15:0] sec_start;
    input [2:0] s;
    case (s)
      3'd0: sec_start = 16'd0;
      3'd1: sec_start = 16'd10923;
      3'd2: sec_start = 16'd21845;
      3'd3: sec_start = 16'd32768;
      3'd4: sec_start = 16'd43691;
      default: sec_start = 16'd54613;
    endcase
  endfunction

  function [2:0] sec_inc;
    input [2:0] s;
    sec_inc = (s >= 3'd5) ? 3'd0 : s + 3'd1;
  endfunction

  function [2:0] sec_dec;
    input [2:0] s;
    sec_dec = (s == 3'd0 || s > 3'd5) ? 3'd5 : s - 3'd1;
  endfunction

  // ---------------- divider for the step (speed) ----------------
  localparam [29:0] SIXTY_FP = 30'd715849728;  // 10923 << 16
  reg         div_start;
  wire [29:0] div_q;
  wire        div_done;

  divu #(.NW(30), .DW(24)) u_div (
      .clk(clk), .rst(rst), .start(div_start),
      .num(SIXTY_FP), .den(period), .quo(div_q), .done(div_done)
  );

  // ---------------- edge processing ----------------
  reg [23:0] tcnt;
  reg        last_ok;      // previous edge was a valid +-1 step
  reg [15:0] edge_ang;
  reg [31:0] travel;
  wire       tsat = (tcnt == 24'hFFFFFF);

  wire [31:0] travel_sum = travel + {2'b00, step};
  wire [31:0] travel_max = {2'b00, SIXTY_FP};

  always @(posedge clk) begin
    if (rst) begin
      code      <= 3'd0;
      sector    <= 3'd0;
      dir       <= 1'b1;
      moving    <= 1'b0;
      bad_seq   <= 1'b0;
      period    <= 24'hFFFFFF;
      step      <= 30'd0;
      tcnt      <= 24'hFFFFFF;
      last_ok   <= 1'b0;
      edge_ang  <= 16'd0;
      travel    <= 32'd0;
      div_start <= 1'b0;
      code_valid_once <= 1'b0;
    end else begin
      div_start <= 1'b0;
      bad_seq   <= 1'b0;
      if (!tsat) tcnt <= tcnt + 24'd1;

      if (div_done) begin
        step   <= div_q;
        moving <= 1'b1;
      end

      if (accept && (cand != code)) begin
        code <= cand;
        if (!cand_inv) begin
          sector <= new_sec;
          tcnt   <= 24'd1;
          travel <= 32'd0;
          if (code_valid_once && (new_sec == sec_inc(sector) || new_sec == sec_dec(sector))) begin
            // valid single-sector step
            dir      <= (new_sec == sec_inc(sector));
            edge_ang <= (new_sec == sec_inc(sector)) ? sec_start(new_sec) : sec_start(sector);
            if (last_ok && !tsat && (dir == (new_sec == sec_inc(sector)))) begin
              period    <= tcnt;
              div_start <= 1'b1;
            end else begin
              moving <= 1'b0;
              step   <= 30'd0;
            end
            last_ok <= 1'b1;
          end else begin
            if (code_valid_once) bad_seq <= 1'b1;
            moving  <= 1'b0;
            step    <= 30'd0;
            last_ok <= 1'b0;
          end
          code_valid_once <= 1'b1;
        end else begin
          moving  <= 1'b0;
          step    <= 30'd0;
          last_ok <= 1'b0;
        end
      end else begin
        if (tsat) begin
          moving  <= 1'b0;
          step    <= 30'd0;
          last_ok <= 1'b0;
          period  <= 24'hFFFFFF;
        end
        if (moving) travel <= (travel_sum > travel_max) ? travel_max : travel_sum;
      end
    end
  end

  assign angle_center = sec_start(sector) + 16'd5461;
  assign angle = moving ? (dir ? edge_ang + travel[31:16] : edge_ang - travel[31:16])
                        : angle_center;

endmodule

// Sequential restoring unsigned divider: quo = num / den  (NW cycles)
module divu #(
    parameter NW = 30,
    parameter DW = 24
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          start,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,
    output reg  [NW-1:0] quo,
    output reg           done
);
  reg [DW:0]   rem;
  reg [5:0]    n;
  reg          busy;
  wire [DW:0]  rsh  = {rem[DW-1:0], quo[NW-1]};
  wire [DW+1:0] diff = {1'b0, rsh} - {2'b00, den};   // den must stay stable while busy

  always @(posedge clk) begin
    if (rst) begin
      rem <= 0; quo <= 0; n <= 0; busy <= 1'b0; done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start) begin
        rem  <= 0;
        quo  <= num;
        n    <= NW[5:0];
        busy <= 1'b1;
      end else if (busy) begin
        if (!diff[DW+1]) begin
          rem <= diff[DW:0];
          quo <= {quo[NW-2:0], 1'b1};
        end else begin
          rem <= rsh;
          quo <= {quo[NW-2:0], 1'b0};
        end
        n <= n - 6'd1;
        if (n == 6'd1) begin
          busy <= 1'b0;
          done <= 1'b1;
        end
      end
    end
  end
endmodule
