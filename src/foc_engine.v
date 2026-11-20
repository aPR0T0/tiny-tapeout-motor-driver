/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * foc_engine - field oriented control datapath, run once per PWM period.
 *
 *   ia, ib    = sign * (raw - offset)                         (Q15)
 *   Clarke    : ialpha = ia, ibeta = (ia + 2 ib) / sqrt(3)
 *   Park      : (id, iq) = rot(-theta) (ialpha, ibeta)       (CORDIC)
 *   speed PI  : iq_ref = PI(speed_ref - speed)               (every spd_div+1 updates)
 *   current PI: vd = PI(id_ref - id), vq = PI(iq_ref - iq)   (or vd/vq commands)
 *   inv. Park : (valpha, vbeta) = rot(+theta) (vd, vq)        (CORDIC)
 *   inv. Clarke + min/max zero-sequence injection (SVPWM equivalent)
 *   duty      = P/2 + v * P / 2^15,  clamped to [dmin, dmax]
 *
 * Voltage scale: 32768 = Vbus.  Linear modulation limit = Vbus/sqrt(3) = 18918.
 * PI:  out = sat( (kp*e) >>> kp_sh  +  integ >>> 16 ),
 *      integ += (ki*e) >>> ki_sh,  clamped to +-lim << 16  (anti-windup)
 *
 * One 16x16 sequential multiplier and one CORDIC are time shared.  Their
 * operands are selected by the state that waits for the result.
 * A full update takes ~ 330 clocks.
 */
`default_nettype none

module foc_engine (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        cur_valid,    // ADC values are real (else currents = 0)
    input  wire [15:0] raw_a,
    input  wire [15:0] raw_b,
    input  wire [15:0] off_a,
    input  wire [15:0] off_b,
    input  wire        sign_a,
    input  wire        sign_b,
    input  wire [15:0] theta,
    input  wire        cur_loop,
    input  wire        spd_loop,
    input  wire        int_reset,
    input  wire [15:0] id_ref,
    input  wire [15:0] iq_ref_ext,
    input  wire [15:0] vd_cmd,
    input  wire [15:0] vq_cmd,
    input  wire [15:0] kp,
    input  wire [3:0]  kp_sh,
    input  wire [15:0] ki,
    input  wire [3:0]  ki_sh,
    input  wire [15:0] vlim,
    input  wire [15:0] oc_lim,
    input  wire [15:0] speed,
    input  wire [15:0] spd_ref,
    input  wire [15:0] kps,
    input  wire [3:0]  kps_sh,
    input  wire [15:0] kis,
    input  wire [3:0]  kis_sh,
    input  wire [15:0] iq_lim,
    input  wire [7:0]  spd_div,
    input  wire [11:0] period,
    input  wire [11:0] dmin,
    input  wire [11:0] dmax,
    output reg  [15:0] ia,
    output reg  [15:0] ib,
    output reg  [15:0] id,
    output reg  [15:0] iq,
    output reg  [15:0] vd,
    output reg  [15:0] vq,
    output reg  [15:0] iq_ref_int,
    output reg  [11:0] duty_a,
    output reg  [11:0] duty_b,
    output reg  [11:0] duty_c,
    output reg         done,
    output reg         oc_fault,
    output reg  [15:0] count,
    output wire        busy
);

  localparam [15:0] C_INVK    = 16'd19898;  // 1/K (CORDIC gain), Q15
  localparam [15:0] C_BETA    = 16'd45954;  // 4/(sqrt(3)*K), Q15, used on (ia+2ib)/4
  localparam [15:0] C_SQRT3_2 = 16'd28378;  // sqrt(3)/2, Q15

  localparam [4:0] S_IDLE  = 5'd0,  S_CUR   = 5'd1,  S_OC    = 5'd2,  S_MA    = 5'd3,
                   S_MB    = 5'd4,  S_PARK  = 5'd5,  S_DQ    = 5'd6,  S_PI_E  = 5'd7,
                   S_PI_P  = 5'd8,  S_PI_I  = 5'd9,  S_PI_O  = 5'd10, S_IK0   = 5'd11,
                   S_IK1   = 5'd12, S_IK2   = 5'd13, S_IPARK = 5'd14, S_ICL   = 5'd15,
                   S_SVM   = 5'd16, S_DA    = 5'd17, S_DB    = 5'd18, S_DC    = 5'd19,
                   S_DONE  = 5'd20, S_PI_J  = 5'd21;

  reg [4:0] st;
  assign busy = (st != S_IDLE);

  // ---------------- helpers ----------------
  function [15:0] sat16_17;  // signed 17 -> 16
    input [16:0] v;
    if (!v[16] && v[15])      sat16_17 = 16'h7FFF;
    else if (v[16] && !v[15]) sat16_17 = 16'h8000;
    else                      sat16_17 = v[15:0];
  endfunction

  function [15:0] sat16_18;  // signed 18 -> 16
    input [17:0] v;
    if (!v[17] && (v[16:15] != 2'b00))      sat16_18 = 16'h7FFF;
    else if (v[17] && (v[16:15] != 2'b11))  sat16_18 = 16'h8000;
    else                                    sat16_18 = v[15:0];
  endfunction

  function [15:0] sat16_32;  // signed 32 -> 16
    input [31:0] v;
    if (!v[31] && (v[30:15] != 16'h0000))      sat16_32 = 16'h7FFF;
    else if (v[31] && (v[30:15] != 16'hFFFF))  sat16_32 = 16'h8000;
    else                                       sat16_32 = v[15:0];
  endfunction

  function [15:0] clamp_lim;  // clamp signed 17-bit value to +-lim (0 <= lim <= 32767)
    input [16:0] v;
    input [15:0] lim;
    reg signed [16:0] sv, sl;
    begin
      sv = v;
      sl = {2'b00, lim[14:0]};
      if (sv > sl)       clamp_lim = {1'b0, lim[14:0]};
      else if (sv < -sl) clamp_lim = -{1'b0, lim[14:0]};
      else               clamp_lim = v[15:0];
    end
  endfunction

  function [15:0] abs16;
    input [15:0] v;
    abs16 = v[15] ? (~v + 16'd1) : v;
  endfunction

  // ---------------- state ----------------
  reg  [15:0] theta_r;
  reg  [15:0] t0, t1;            // temporaries
  reg  [16:0] va, vb, vc;
  reg  [15:0] err, ptmp;
  reg  [31:0] pr;                // registered (ki * e) >>> ki_sh
  reg  [31:0] integ_d, integ_q, integ_s;
  reg  [1:0]  pi_sel;            // 0 speed, 1 d, 2 q
  reg  [7:0]  spd_cnt;
  reg         spd_tick;
  reg         mul_start, cor_start;

  // PI operand selection
  reg  [15:0] pi_ref, pi_meas, pi_kp, pi_ki, pi_lim;
  reg  [3:0]  pi_kpsh, pi_kish;
  reg  [31:0] pi_integ;
  always @(*) begin
    case (pi_sel)
      2'd0: begin
        pi_ref = spd_ref;  pi_meas = speed; pi_kp = kps; pi_kpsh = kps_sh;
        pi_ki = kis; pi_kish = kis_sh; pi_lim = iq_lim; pi_integ = integ_s;
      end
      2'd1: begin
        pi_ref = id_ref;   pi_meas = id;    pi_kp = kp;  pi_kpsh = kp_sh;
        pi_ki = ki;  pi_kish = ki_sh;  pi_lim = vlim;   pi_integ = integ_d;
      end
      default: begin
        pi_ref = spd_loop ? iq_ref_int : iq_ref_ext;  pi_meas = iq;
        pi_kp = kp;  pi_kpsh = kp_sh; pi_ki = ki; pi_kish = ki_sh; pi_lim = vlim;
        pi_integ = integ_q;
      end
    endcase
  end

  wire [15:0] cor_xo, cor_yo;

  // ---------------- shared multiplier (operands chosen by the waiting state) ----------------
  wire [17:0] ab2 = {{2{ia[15]}}, ia} + {ib[15], ib, 1'b0};   // ia + 2 ib
  reg  [15:0] mul_a, mul_b;
  always @(*) begin
    case (st)
      S_MA:    begin mul_a = C_INVK;            mul_b = ia;          end
      S_MB:    begin mul_a = C_BETA;            mul_b = ab2[17:2];   end
      S_PI_P:  begin mul_a = pi_kp;             mul_b = err;         end
      S_PI_I:  begin mul_a = pi_ki;             mul_b = err;         end
      S_IK1:   begin mul_a = C_INVK;            mul_b = vd;          end
      S_IK2:   begin mul_a = C_INVK;            mul_b = vq;          end
      S_ICL:   begin mul_a = C_SQRT3_2;         mul_b = cor_yo;      end
      S_DB:    begin mul_a = {4'd0, period};    mul_b = va[15:0];    end
      S_DC:    begin mul_a = {4'd0, period};    mul_b = vb[15:0];    end
      default: begin mul_a = {4'd0, period};    mul_b = vc[15:0];    end
    endcase
  end

  wire [31:0] mul_p;
  wire        mul_done;
  mul16 u_mul (
      .clk(clk), .rst(rst), .start(mul_start), .a(mul_a), .b(mul_b),
      .p(mul_p), .done(mul_done)
  );

  wire [3:0]  sh_amt = (st == S_PI_P) ? pi_kpsh : pi_kish;
  wire [31:0] p_shr  = $signed(mul_p) >>> sh_amt;
  wire [15:0] p_q15  = sat16_32({{15{mul_p[31]}}, mul_p[31:15]});  // p >>> 15, saturated

  // ---------------- shared CORDIC ----------------
  wire        park = (st == S_PARK);
  wire        cor_done;
  cordic_rot u_cordic (
      .clk(clk), .rst(rst), .start(cor_start),
      .xin(park ? t0 : t1), .yin(park ? t1 : t0), .ang(park ? -theta_r : theta_r),
      .xout(cor_xo), .yout(cor_yo), .done(cor_done)
  );

  // ---------------- datapath ----------------
  wire [16:0] pi_e17     = {pi_ref[15], pi_ref} - {pi_meas[15], pi_meas};
  wire [32:0] integ_sum  = {pi_integ[31], pi_integ} + {pr[31], pr};
  wire [32:0] integ_max  = {2'b00, pi_lim[14:0], 16'h0000};
  wire [32:0] integ_min  = -integ_max;
  wire [31:0] integ_next = ($signed(integ_sum) > $signed(integ_max)) ? integ_max[31:0] :
                           ($signed(integ_sum) < $signed(integ_min)) ? integ_min[31:0] :
                           integ_sum[31:0];
  wire [16:0] pi_out17   = {ptmp[15], ptmp} + {pi_integ[31], pi_integ[31:16]};
  wire [15:0] pi_out     = clamp_lim(pi_out17, pi_lim);

  wire [16:0] da = sign_a ? ({1'b0, off_a} - {1'b0, raw_a}) : ({1'b0, raw_a} - {1'b0, off_a});
  wire [16:0] db = sign_b ? ({1'b0, off_b} - {1'b0, raw_b}) : ({1'b0, raw_b} - {1'b0, off_b});
  wire [16:0] ic17   = {ia[15], ia} + {ib[15], ib};
  wire [16:0] ic_abs = ic17[16] ? (~ic17 + 17'd1) : ic17;

  // SVM
  wire signed [16:0] sva = va, svb = vb, svc = vc;
  wire signed [16:0] vmax = (sva > svb) ? ((sva > svc) ? sva : svc) : ((svb > svc) ? svb : svc);
  wire signed [16:0] vmin = (sva < svb) ? ((sva < svc) ? sva : svc) : ((svb < svc) ? svb : svc);
  wire signed [17:0] vsum = vmax + vmin;
  wire        [16:0] vmid = vsum[17:1];
  wire        [17:0] ea   = {va[16], va} - {vmid[16], vmid};
  wire        [17:0] eb   = {vb[16], vb} - {vmid[16], vmid};
  wire        [17:0] ec   = {vc[16], vc} - {vmid[16], vmid};

  // duty from product (P * v) >>> 15
  wire [16:0] dv    = mul_p[31:15];                       // signed, |dv| <= P
  wire [16:0] dsum  = {6'd0, period[11:1]} + dv;          // P/2 + dv
  wire [11:0] dhi   = (dmax > period) ? period : dmax;
  wire [11:0] duty_new = dsum[16] ? dmin :
                         (dsum[15:0] > {4'd0, dhi}) ? dhi :
                         (dsum[15:0] < {4'd0, dmin}) ? dmin : dsum[11:0];

  wire [16:0] vtq   = {t0[15], t0};
  wire [16:0] vhalf = {cor_xo[15], cor_xo[15], cor_xo[15:1]};  // valpha / 2

  always @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;
      ia <= 16'd0; ib <= 16'd0; id <= 16'd0; iq <= 16'd0; vd <= 16'd0; vq <= 16'd0;
      iq_ref_int <= 16'd0;
      duty_a <= 12'd0; duty_b <= 12'd0; duty_c <= 12'd0;
      done <= 1'b0; oc_fault <= 1'b0; count <= 16'd0;
      mul_start <= 1'b0; cor_start <= 1'b0;
      theta_r <= 16'd0; t0 <= 16'd0; t1 <= 16'd0;
      va <= 17'd0; vb <= 17'd0; vc <= 17'd0; err <= 16'd0; ptmp <= 16'd0; pr <= 32'd0;
      integ_d <= 32'd0; integ_q <= 32'd0; integ_s <= 32'd0;
      pi_sel <= 2'd0; spd_cnt <= 8'd0; spd_tick <= 1'b0;
    end else begin
      mul_start <= 1'b0;
      cor_start <= 1'b0;
      done      <= 1'b0;
      oc_fault  <= 1'b0;

      if (int_reset || !cur_loop) begin
        integ_d <= 32'd0;
        integ_q <= 32'd0;
      end
      if (int_reset || !spd_loop) begin
        // bumpless transfer: the speed integrator follows the external iq
        // reference while the speed loop is off
        integ_s    <= {iq_ref_ext, 16'h0000};
        iq_ref_int <= iq_ref_ext;
      end

      case (st)
        S_IDLE: if (start) begin
          theta_r <= theta;
          st      <= S_CUR;
        end

        S_CUR: begin
          ia <= cur_valid ? sat16_17(da) : 16'd0;
          ib <= cur_valid ? sat16_17(db) : 16'd0;
          if (spd_cnt >= spd_div) begin
            spd_cnt  <= 8'd0;
            spd_tick <= 1'b1;
          end else begin
            spd_cnt  <= spd_cnt + 8'd1;
            spd_tick <= 1'b0;
          end
          st <= S_OC;
        end

        S_OC: begin
          if ((abs16(ia) > oc_lim) || (abs16(ib) > oc_lim) || (ic_abs > {1'b0, oc_lim}))
            oc_fault <= 1'b1;
          mul_start <= 1'b1;                 // ialpha / K
          st <= S_MA;
        end

        S_MA: if (mul_done) begin
          t0        <= p_q15;
          mul_start <= 1'b1;                 // ibeta / K
          st        <= S_MB;
        end

        S_MB: if (mul_done) begin
          t1        <= p_q15;
          cor_start <= 1'b1;                 // Park
          st        <= S_PARK;
        end

        S_PARK: if (cor_done) begin
          id <= cor_xo;
          iq <= cor_yo;
          if (spd_loop && spd_tick && !int_reset) begin
            pi_sel <= 2'd0;
            st     <= S_PI_E;
          end else begin
            st <= S_DQ;
          end
        end

        S_DQ: begin
          if (cur_loop && !int_reset) begin
            pi_sel <= 2'd1;
            st     <= S_PI_E;
          end else begin
            vd <= clamp_lim({vd_cmd[15], vd_cmd}, vlim);
            vq <= clamp_lim({vq_cmd[15], vq_cmd}, vlim);
            st <= S_IK0;
          end
        end

        // ---- generic PI ----
        S_PI_E: begin
          err       <= sat16_17(pi_e17);
          mul_start <= 1'b1;                 // kp * e
          st        <= S_PI_P;
        end

        S_PI_P: if (mul_done) begin
          ptmp      <= sat16_32(p_shr);
          mul_start <= 1'b1;                 // ki * e
          st        <= S_PI_I;
        end

        S_PI_I: if (mul_done) begin
          pr <= p_shr;
          st <= S_PI_J;
        end

        S_PI_J: begin
          case (pi_sel)
            2'd0:    integ_s <= integ_next;
            2'd1:    integ_d <= integ_next;
            default: integ_q <= integ_next;
          endcase
          st <= S_PI_O;
        end

        S_PI_O: begin
          case (pi_sel)
            2'd0: begin
              iq_ref_int <= pi_out;
              st <= S_DQ;
            end
            2'd1: begin
              vd     <= pi_out;
              pi_sel <= 2'd2;
              st     <= S_PI_E;
            end
            default: begin
              vq <= pi_out;
              st <= S_IK0;
            end
          endcase
        end

        // ---- inverse Park ----
        S_IK0: begin
          mul_start <= 1'b1;                 // vd / K
          st <= S_IK1;
        end

        S_IK1: if (mul_done) begin
          t1        <= p_q15;
          mul_start <= 1'b1;                 // vq / K
          st        <= S_IK2;
        end

        S_IK2: if (mul_done) begin
          t0        <= p_q15;
          cor_start <= 1'b1;                 // inverse Park
          st        <= S_IPARK;
        end

        S_IPARK: if (cor_done) begin
          mul_start <= 1'b1;                 // vbeta * sqrt(3)/2
          st <= S_ICL;
        end

        // ---- inverse Clarke ----
        S_ICL: if (mul_done) begin
          t0 <= p_q15;
          va <= {cor_xo[15], cor_xo};
          st <= S_SVM;
        end

        S_SVM: begin
          vb <= 17'd0 - vhalf + vtq;
          vc <= 17'd0 - vhalf - vtq;
          st <= S_DA;
        end

        // ---- zero sequence injection, duties ----
        S_DA: begin
          va <= {ea[17], sat16_18(ea)};
          vb <= {eb[17], sat16_18(eb)};
          vc <= {ec[17], sat16_18(ec)};
          mul_start <= 1'b1;                 // P * va
          st <= S_DB;
        end

        S_DB: if (mul_done) begin
          duty_a    <= duty_new;
          mul_start <= 1'b1;                 // P * vb
          st        <= S_DC;
        end

        S_DC: if (mul_done) begin
          duty_b    <= duty_new;
          mul_start <= 1'b1;                 // P * vc
          st        <= S_DONE;
        end

        S_DONE: if (mul_done) begin
          duty_c <= duty_new;
          done   <= 1'b1;
          count  <= count + 16'd1;
          st     <= S_IDLE;
        end

        default: st <= S_IDLE;
      endcase
    end
  end

endmodule
