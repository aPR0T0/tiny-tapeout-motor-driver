/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * pwm3 - 3-phase center-aligned PWM with the digital half of a gate driver:
 *   - up/down counter 0..P..0  (PWM frequency = f_clk / (2*P))
 *   - double-buffered duties, latched at the counter valley
 *   - per-phase dead-time insertion (both switches off for `deadtime` clocks)
 *   - hard shoot-through interlock on the registered outputs
 *   - instant shutdown when out_en drops (fault / disarm)
 *   - brake mode (all low-side switches on)
 *   - programmable output polarity for high and low side gate inputs
 *   - ADC trigger pulse `adc_adv` clocks before the counter peak
 *     (all low-side switches are on around the peak -> low-side shunt sampling)
 *
 * High-side switch x is on while (cnt < act_x), so high-side pulses are centered
 * on the valley and low-side pulses on the peak.
 */
`default_nettype none

module pwm3 (
    input  wire        clk,
    input  wire        rst,
    input  wire [11:0] period,
    input  wire [7:0]  deadtime,
    input  wire [11:0] adc_adv,
    input  wire        out_en,
    input  wire        brake,
    input  wire        pol_h_inv,
    input  wire        pol_l_inv,
    input  wire [11:0] duty_a,
    input  wire [11:0] duty_b,
    input  wire [11:0] duty_c,
    output reg  [11:0] act_a,
    output reg  [11:0] act_b,
    output reg  [11:0] act_c,
    output reg  [5:0]  gate,       // {CL, CH, BL, BH, AL, AH} after polarity
    output reg         valley,     // 1-cycle pulse at the counter valley
    output reg         adc_trig,   // 1-cycle pulse before the peak
    output wire [11:0] cnt_o
);

  reg [11:0] cnt;
  reg        up;
  assign cnt_o = cnt;

  wire turn_up = !up && (cnt == 12'd0);

  always @(posedge clk) begin
    if (rst) begin
      cnt      <= 12'd0;
      up       <= 1'b1;
      valley   <= 1'b0;
      adc_trig <= 1'b0;
      act_a    <= 12'd0;
      act_b    <= 12'd0;
      act_c    <= 12'd0;
    end else begin
      valley   <= turn_up;
      adc_trig <= up && (cnt == period - adc_adv) && (adc_adv <= period);
      if (up) begin
        if (cnt >= period) begin
          up  <= 1'b0;
          cnt <= (cnt == 12'd0) ? 12'd0 : cnt - 12'd1;
        end else begin
          cnt <= cnt + 12'd1;
        end
      end else begin
        if (cnt == 12'd0) begin
          up  <= 1'b1;
          cnt <= (period == 12'd0) ? 12'd0 : 12'd1;
          // shadow -> active at the valley
          act_a <= duty_a;
          act_b <= duty_b;
          act_c <= duty_c;
        end else begin
          cnt <= cnt - 12'd1;
        end
      end
    end
  end

  // Desired switch state per phase: 1 = high side on, 0 = low side on
  wire s_a = !brake && (act_a > cnt);
  wire s_b = !brake && (act_b > cnt);
  wire s_c = !brake && (act_c > cnt);

  reg       q_a, q_b, q_c;
  reg [7:0] dt_a, dt_b, dt_c;

  always @(posedge clk) begin
    if (rst || !out_en) begin
      q_a  <= 1'b0;
      q_b  <= 1'b0;
      q_c  <= 1'b0;
      dt_a <= deadtime;
      dt_b <= deadtime;
      dt_c <= deadtime;
    end else begin
      if (s_a != q_a) begin q_a <= s_a; dt_a <= deadtime; end
      else if (dt_a != 8'd0) dt_a <= dt_a - 8'd1;
      if (s_b != q_b) begin q_b <= s_b; dt_b <= deadtime; end
      else if (dt_b != 8'd0) dt_b <= dt_b - 8'd1;
      if (s_c != q_c) begin q_c <= s_c; dt_c <= deadtime; end
      else if (dt_c != 8'd0) dt_c <= dt_c - 8'd1;
    end
  end

  // Raw (active-high) gate commands; a switch is only on after its dead time.
  wire ah = out_en &&  q_a && (dt_a == 8'd0);
  wire al = out_en && !q_a && (dt_a == 8'd0);
  wire bh = out_en &&  q_b && (dt_b == 8'd0);
  wire bl = out_en && !q_b && (dt_b == 8'd0);
  wire ch = out_en &&  q_c && (dt_c == 8'd0);
  wire cl = out_en && !q_c && (dt_c == 8'd0);

  // Interlock (belt and braces): never both switches of a leg.
  wire [5:0] raw = {cl & ~ch, ch & ~cl, bl & ~bh, bh & ~bl, al & ~ah, ah & ~al};
  wire [5:0] pol = {pol_l_inv, pol_h_inv, pol_l_inv, pol_h_inv, pol_l_inv, pol_h_inv};

  always @(posedge clk) begin
    if (rst) gate <= 6'b000000;
    else     gate <= raw ^ pol;
  end

endmodule
