/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * foc_soc - the complete FOC motor controller / microcontroller.
 *
 *   bus masters : host_link (priority), rv32e_core
 *   memory map  : 0x0000_0000  QSPI flash, execute in place (16 MB, read only)
 *                 0x1000_0000  on-chip RAM (RAM_WORDS x 32 bit)
 *                 0x2000_0000  peripheral registers (see docs/register_map.md)
 *   peripherals : pwm3 (gate drive), hall_sensor, foc_engine, spi_bus
 *                 (flash + ADC + aux SPI), host_link (UART), fault logic
 */
`default_nettype none

module foc_soc #(
    parameter RAM_WORDS = 32,
    parameter RAM_AW    = 5
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        boot,          // CPU runs from flash after reset
    // motor side
    input  wire [2:0]  hall_in,
    input  wire        nfault_in,
    input  wire        nestop_in,
    output wire [5:0]  gate,          // {CL, CH, BL, BH, AL, AH}
    output wire        drv_en,
    // host
    input  wire        rxd,
    output wire        txd,
    // SPI bus
    output wire        sck,
    output wire [2:0]  cs_n,
    output wire [3:0]  sd_o,
    output wire [3:0]  sd_oe,
    input  wire [3:0]  sd_i,
    input  wire        sdob_i
);

  localparam [31:0] ID_WORD = 32'h464F_4301;   // "FOC" v1

  // ================================================================ registers
  reg  [10:0] ctrl;
  wire        c_arm      = ctrl[0];
  wire        c_drv_en   = ctrl[1];
  wire        c_adc_en   = ctrl[2];
  wire        c_cur_loop = ctrl[4];
  wire        c_spd_loop = ctrl[5];
  wire [1:0]  c_amode    = ctrl[7:6];
  wire        c_brake    = ctrl[8];
  wire        c_wdt_en   = ctrl[9];
  wire        c_spi_sync = ctrl[10];

  reg  [5:0]  faults, fmask;
  reg  [11:0] pwm_period;
  reg  [7:0]  pwm_dead;
  reg         pol_h, pol_l;
  reg  [11:0] adc_adv;
  reg  [11:0] dmin, dmax;
  reg  [4:0]  adc_nbits;
  reg  [3:0]  adc_lsh, adc_div;
  reg         adc_cpol, adc_late, adc_sign_a, adc_sign_b;
  reg  [7:0]  adc_csdly;
  reg  [15:0] off_a, off_b;
  reg  [15:0] id_ref, iq_ref, vd_cmd, vq_cmd;
  reg  [15:0] kp, ki, kps, kis;
  reg  [3:0]  kp_sh, ki_sh, kps_sh, kis_sh;
  reg  [15:0] vlim, oc_lim, spd_ref, iq_lim;
  reg  [7:0]  spd_div;
  reg  [15:0] ang_off;
  reg  [3:0]  spd_shift;
  reg  [23:0] ol_step;
  reg  [15:0] forced_ang;
  reg  [23:0] hall_map;
  reg  [7:0]  hall_filt;
  reg  [1:0]  spi_cs;
  reg         spi_hold;
  reg  [3:0]  spi_div;
  reg         spi_auto;
  reg         fl_quad, fl_late;
  reg  [2:0]  fl_dummy;
  reg  [3:0]  fl_div;
  reg         cpu_run, cpu_pause, cpu_step, cpu_bkhit;
  reg  [31:2] cpu_boot;          // reset vector (boot address)
  reg  [31:2] bkpt_addr;
  reg         bkpt_en;
  reg  [31:0] mbox_h2c, mbox_c2h;
  reg  [31:0] time_cnt;
  reg  [15:0] wdt_lim;
  reg         tel_en;
  reg  [3:0]  tel_n;
  reg  [7:0]  tel_div;
  reg  [7:2]  tel_addr;
  reg  [31:0] ol_acc;

  // ================================================================ reset tree
  // One registered reset copy per block keeps every reset net's fanout to a
  // few hundred flops (a single 4000-load net was the slow-corner critical
  // path).  `keep` stops synthesis from merging the identical registers.
  (* keep *) reg rst_hall, rst_pwm, rst_foc, rst_spi, rst_link, rst_cpu, rst_bus, rst_regs;
  always @(posedge clk) begin
    rst_hall <= rst; rst_pwm  <= rst; rst_foc  <= rst; rst_spi  <= rst;
    rst_link <= rst; rst_cpu  <= rst; rst_bus  <= rst; rst_regs <= rst;
  end

  // ================================================================ blocks
  // ---- input synchronizers ----
  reg [1:0] nf_s, ne_s;
  always @(posedge clk) begin
    nf_s <= {nf_s[0], nfault_in};
    ne_s <= {ne_s[0], nestop_in};
  end
  wire nfault_pin = nf_s[1];
  wire nestop_pin = ne_s[1];

  // ---- fault logic ----
  wire fault_active = |(faults & fmask);
  wire out_en       = c_arm && !fault_active;
  assign drv_en     = c_drv_en && !fault_active;

  // ---- hall ----
  wire [2:0]  h_code, h_sector;
  wire        h_dir, h_moving, h_invalid, h_bad;
  wire [15:0] h_angle, h_center;
  wire [23:0] h_period;
  wire [29:0] h_step;

  hall_sensor u_hall (
      .clk(clk), .rst(rst_hall), .hall_in(hall_in), .filt_len(hall_filt), .hall_map(hall_map),
      .code(h_code), .sector(h_sector), .dir(h_dir), .moving(h_moving),
      .invalid(h_invalid), .bad_seq(h_bad), .angle(h_angle), .angle_center(h_center),
      .period(h_period), .step(h_step)
  );

  // ---- angle selection + speed ----
  reg [15:0] theta;
  always @(*) begin
    case (c_amode)
      2'd0:    theta = h_angle + ang_off;
      2'd1:    theta = h_center + ang_off;
      2'd2:    theta = ol_acc[31:16];
      default: theta = forced_ang;
    endcase
  end

  wire [31:0] spd_raw = (c_amode == 2'd2) ? {{8{ol_step[23]}}, ol_step} :
                        (h_dir ? {2'b00, h_step} : (32'd0 - {2'b00, h_step}));
  wire [31:0] spd_shr = $signed(spd_raw) >>> spd_shift;
  wire [15:0] speed16 = (!spd_shr[31] && spd_shr[30:15] != 16'h0000) ? 16'h7FFF :
                        ( spd_shr[31] && spd_shr[30:15] != 16'hFFFF) ? 16'h8000 : spd_shr[15:0];

  // ---- PWM ----
  wire [11:0] foc_da, foc_db, foc_dc;
  wire [11:0] act_a, act_b, act_c;
  wire        pwm_valley, pwm_adc_trig;
  wire [11:0] pwm_cnt;

  pwm3 u_pwm (
      .clk(clk), .rst(rst_pwm), .period(pwm_period), .deadtime(pwm_dead), .adc_adv(adc_adv),
      .out_en(out_en), .brake(c_brake), .pol_h_inv(pol_h), .pol_l_inv(pol_l),
      .duty_a(foc_da), .duty_b(foc_db), .duty_c(foc_dc),
      .act_a(act_a), .act_b(act_b), .act_c(act_c), .gate(gate),
      .valley(pwm_valley), .adc_trig(pwm_adc_trig), .cnt_o(pwm_cnt)
  );

  // ---- FOC engine ----
  wire [15:0] adc_a, adc_b;
  wire        adc_done, adc_drop;
  wire [15:0] f_ia, f_ib, f_id, f_iq, f_vd, f_vq, f_iqr, f_count;
  wire        f_done, f_oc, f_busy;

  // with the ADC on, the engine runs on every conversion; without it, once per
  // PWM period (voltage mode / open loop without current sensing)
  foc_engine u_foc (
      .clk(clk), .rst(rst_foc), .start(c_adc_en ? adc_done : pwm_adc_trig), .cur_valid(c_adc_en),
      .raw_a(adc_a), .raw_b(adc_b), .off_a(off_a), .off_b(off_b),
      .sign_a(adc_sign_a), .sign_b(adc_sign_b), .theta(theta),
      .cur_loop(c_cur_loop), .spd_loop(c_spd_loop), .int_reset(!out_en),
      .id_ref(id_ref), .iq_ref_ext(iq_ref), .vd_cmd(vd_cmd), .vq_cmd(vq_cmd),
      .kp(kp), .kp_sh(kp_sh), .ki(ki), .ki_sh(ki_sh), .vlim(vlim), .oc_lim(oc_lim),
      .speed(speed16), .spd_ref(spd_ref), .kps(kps), .kps_sh(kps_sh), .kis(kis), .kis_sh(kis_sh),
      .iq_lim(iq_lim), .spd_div(spd_div), .period(pwm_period), .dmin(dmin), .dmax(dmax),
      .ia(f_ia), .ib(f_ib), .id(f_id), .iq(f_iq), .vd(f_vd), .vq(f_vq), .iq_ref_int(f_iqr),
      .duty_a(foc_da), .duty_b(foc_db), .duty_c(foc_dc), .done(f_done),
      .oc_fault(f_oc), .count(f_count), .busy(f_busy)
  );

  // ---- bus master wires ----
  wire        l_req, l_we;
  wire [31:0] l_addr, l_wdata;
  wire [31:0] l_rdata;
  wire        l_ack;
  wire        c_req, c_we;
  wire [31:0] c_addr, c_wdata;
  wire [3:0]  c_wstrb;
  wire [31:0] c_rdata;
  wire        c_ack;

  // ---- SPI bus ----
  wire        fr0_req, fr1_req, fr0_ack, fr1_ack;
  wire [31:0] fr_data;
  wire [63:0] port_buf;
  wire        port_busy, port_owns;
  reg         port_go, port_w0, port_w1;
  reg  [3:0]  port_n;
  reg  [31:0] port_wdata;

  spi_bus u_spi (
      .clk(clk), .rst(rst_spi), .sck(sck), .cs_n(cs_n), .sd_o(sd_o), .sd_oe(sd_oe),
      .sd_i(sd_i), .sdob_i(sdob_i),
      .fl_quad(fl_quad), .fl_dummy(fl_dummy), .fl_div(fl_div), .fl_late(fl_late),
      .adc_nbits(adc_nbits), .adc_lsh(adc_lsh), .adc_div(adc_div), .adc_csdly(adc_csdly),
      .adc_cpol(adc_cpol), .adc_late(adc_late),
      .port_div(spi_div), .port_sync(c_spi_sync && c_adc_en),
      .fr0_req(fr0_req), .fr0_addr(l_addr[23:0]), .fr0_ack(fr0_ack),
      .fr1_req(fr1_req), .fr1_addr(c_addr[23:0]), .fr1_ack(fr1_ack), .fr_data(fr_data),
      .adc_trig(pwm_adc_trig && c_adc_en), .adc_a(adc_a), .adc_b(adc_b),
      .adc_done(adc_done), .adc_drop(adc_drop),
      .port_cs(spi_cs), .port_hold(spi_hold), .port_go(port_go), .port_n(port_n),
      .port_w0(port_w0), .port_w1(port_w1), .port_wdata(port_wdata),
      .port_buf(port_buf), .port_busy(port_busy), .port_owns(port_owns)
  );

  // ---- host link ----
  wire [15:0] link_div;
  wire        link_locked, wdt_expired;
  wire [7:0]  crc_errs, frames_ok;
  reg         link_div_set, link_unlock;
  reg  [15:0] link_div_val;

  host_link u_link (
      .clk(clk), .rst(rst_link), .rxd(rxd), .txd(txd),
      .div_set(link_div_set), .div_val(link_div_val), .unlock(link_unlock),
      .div_fx(link_div), .locked(link_locked),
      .m_req(l_req), .m_we(l_we), .m_addr(l_addr), .m_wdata(l_wdata),
      .m_rdata(l_rdata), .m_ack(l_ack), .id_word(ID_WORD),
      .wdt_en(c_wdt_en), .wdt_lim(wdt_lim), .wdt_expired(wdt_expired),
      .tel_en(tel_en), .tel_tick(c_adc_en ? f_done : pwm_valley), .tel_div(tel_div),
      .tel_n(tel_n), .tel_addr({24'h200000, tel_addr, 2'b00}),
      .crc_errs(crc_errs), .frames_ok(frames_ok)
  );

  // ---- CPU ----
  wire        cpu_halted, cpu_paused, cpu_fentry;
  wire [3:0]  cpu_cause;
  wire [31:0] cpu_pc;

  rv32e_core u_cpu (
      .clk(clk), .rst(rst_cpu || !cpu_run),
      .reset_pc(cpu_boot), .pause(cpu_pause), .step(cpu_step), .paused(cpu_paused),
      .fetch_entry(cpu_fentry),
      .m_req(c_req), .m_we(c_we), .m_addr(c_addr), .m_wdata(c_wdata), .m_wstrb(c_wstrb),
      .m_rdata(c_rdata), .m_ack(c_ack), .halted(cpu_halted), .cause(cpu_cause), .pc_o(cpu_pc)
  );

  // ================================================================ interconnect
  // flash region goes straight to the SPI bus (one requester port per master)
  wire l_flash = (l_addr[31:28] == 4'h0);
  wire c_flash = (c_addr[31:28] == 4'h0);
  assign fr0_req = l_req && l_flash && !l_we;
  assign fr1_req = c_req && c_flash && !c_we;

  // local bus (RAM + registers), single cycle access.  The grant is a register
  // (it moves to the other master whenever the granted one is idle), so the
  // address multiplexer is not on the request-decode path.
  reg         lk_ack, cp_ack, gnt_c;
  reg  [31:0] loc_rdata;
  wire        l_loc = l_req && !(l_flash && !l_we) && !lk_ack;
  wire        c_loc = c_req && !(c_flash && !c_we) && !cp_ack;
  wire        sel0 = !gnt_c && l_loc;
  wire        sel1 =  gnt_c && c_loc;
  wire        sel  = sel0 || sel1;
  wire [31:0] s_addr  = gnt_c ? c_addr  : l_addr;
  wire [31:0] s_wdata = gnt_c ? c_wdata : l_wdata;
  wire [3:0]  s_wstrb = gnt_c ? c_wstrb : 4'hF;
  wire        s_we    = gnt_c ? c_we    : l_we;
  wire        s_ram   = (s_addr[31:28] == 4'h1);
  wire        s_reg   = (s_addr[31:28] == 4'h2);
  wire        reg_wr  = sel && s_we && s_reg;
  wire [5:0]  ra      = s_addr[7:2];

  assign l_ack   = lk_ack || fr0_ack;
  assign l_rdata = fr0_ack ? fr_data : loc_rdata;
  assign c_ack   = cp_ack || fr1_ack;
  assign c_rdata = fr1_ack ? fr_data : loc_rdata;

  // ---- RAM ----
  reg  [31:0] ram [0:RAM_WORDS-1];
  wire [RAM_AW-1:0] ram_a = s_addr[RAM_AW+1:2];
  always @(posedge clk) begin
    if (sel && s_we && s_ram) begin
      if (s_wstrb[0]) ram[ram_a][7:0]   <= s_wdata[7:0];
      if (s_wstrb[1]) ram[ram_a][15:8]  <= s_wdata[15:8];
      if (s_wstrb[2]) ram[ram_a][23:16] <= s_wdata[23:16];
      if (s_wstrb[3]) ram[ram_a][31:24] <= s_wdata[31:24];
    end
  end

  // ---- register read mux ----
  reg [31:0] rr;
  always @(*) begin
    case (ra)
      6'h00: rr = ID_WORD;
      6'h01: rr = {21'd0, ctrl};
      6'h02: rr = {8'd0, nestop_pin, nfault_pin, f_busy, link_locked, port_busy, cpu_halted, h_moving,
                   h_dir, h_sector, h_code, out_en, fault_active, 2'b00, faults};
      6'h03: rr = {26'd0, fmask};
      6'h04: rr = {20'd0, pwm_period};
      6'h05: rr = {4'd0, adc_adv, 6'd0, pol_l, pol_h, pwm_dead};
      6'h06: rr = {4'd0, dmax, 4'd0, dmin};
      6'h07: rr = {4'd0, act_b, 4'd0, act_a};
      6'h08: rr = {20'd0, act_c};
      6'h09: rr = {adc_csdly, 7'd0, adc_sign_b, adc_sign_a, adc_late, adc_cpol, adc_div, adc_lsh, adc_nbits};
      6'h0B: rr = {off_b, off_a};
      6'h0C: rr = {adc_b, adc_a};
      6'h0D: rr = {iq_ref, id_ref};
      6'h0E: rr = {vq_cmd, vd_cmd};
      6'h0F: rr = {12'd0, kp_sh, kp};
      6'h10: rr = {12'd0, ki_sh, ki};
      6'h11: rr = {oc_lim, vlim};
      6'h12: rr = {8'd0, spd_div, spd_ref};
      6'h13: rr = {12'd0, kps_sh, kps};
      6'h14: rr = {12'd0, kis_sh, kis};
      6'h15: rr = {16'd0, iq_lim};
      6'h16: rr = {12'd0, spd_shift, ang_off};
      6'h17: rr = {{8{ol_step[23]}}, ol_step};
      6'h18: rr = {16'd0, forced_ang};
      6'h19: rr = {hall_filt, hall_map};
      6'h1A: rr = {f_ib, f_ia};
      6'h1B: rr = {f_iq, f_id};
      6'h1C: rr = {f_vq, f_vd};
      6'h1D: rr = {speed16, theta};
      6'h1E: rr = {8'd0, h_period};
      6'h1F: rr = {f_iqr, f_count};
      6'h20: rr = {22'd0, port_owns, port_busy, spi_auto, spi_div, spi_hold, spi_cs};
      6'h21: rr = port_buf[31:0];
      6'h22: rr = port_buf[63:32];
      6'h23: rr = {31'd0, port_busy};
      6'h24: rr = {23'd0, fl_late, fl_div, fl_dummy, fl_quad};
      6'h25: rr = {23'd0, cpu_bkhit, cpu_cause, cpu_halted, cpu_paused, cpu_pause, cpu_run};
      6'h26: rr = cpu_pc;
      6'h27: rr = mbox_h2c;
      6'h28: rr = mbox_c2h;
      6'h29: rr = time_cnt;
      6'h2A: rr = {15'd0, link_locked, link_div};
      6'h2B: rr = {16'd0, wdt_lim};
      6'h2C: rr = {8'd0, tel_div, 4'd0, tel_n, 7'd0, tel_en};
      6'h2D: rr = {24'h200000, tel_addr, 2'b00};
      6'h2E: rr = {16'd0, frames_ok, crc_errs};
      6'h2F: rr = {cpu_boot, 2'b00};
      6'h30: rr = {bkpt_addr, 1'b0, bkpt_en};
      default: rr = 32'd0;
    endcase
  end

  wire [31:0] local_rdata = s_ram ? ram[ram_a] : s_reg ? rr : 32'd0;

  always @(posedge clk) begin
    if (rst_bus) begin
      lk_ack <= 1'b0; cp_ack <= 1'b0; loc_rdata <= 32'd0; gnt_c <= 1'b0;
    end else begin
      lk_ack <= sel0;
      cp_ack <= sel1;
      if (!(gnt_c ? c_loc : l_loc)) gnt_c <= !gnt_c;
      if (sel) loc_rdata <= local_rdata;
    end
  end

  // ---- register writes + housekeeping ----
  wire [5:0] fault_now = {adc_drop, wdt_expired, (h_invalid || h_bad), f_oc, !nestop_pin, !nfault_pin};

  always @(posedge clk) begin
    if (rst_regs) begin
      ctrl <= 11'd0; faults <= 6'd0; fmask <= 6'b010111;
      pwm_period <= 12'd1250; pwm_dead <= 8'd25; pol_h <= 1'b0; pol_l <= 1'b0; adc_adv <= 12'd0;
      dmin <= 12'd0; dmax <= 12'd1200;
      adc_nbits <= 5'd16; adc_lsh <= 4'd0; adc_div <= 4'd1; adc_cpol <= 1'b0; adc_late <= 1'b0;
      adc_sign_a <= 1'b0; adc_sign_b <= 1'b0; adc_csdly <= 8'd0;
      off_a <= 16'h8000; off_b <= 16'h8000;
      id_ref <= 16'd0; iq_ref <= 16'd0; vd_cmd <= 16'd0; vq_cmd <= 16'd0;
      kp <= 16'd0; ki <= 16'd0; kps <= 16'd0; kis <= 16'd0;
      kp_sh <= 4'd0; ki_sh <= 4'd0; kps_sh <= 4'd0; kis_sh <= 4'd0;
      vlim <= 16'd18918; oc_lim <= 16'h7FFF; spd_ref <= 16'd0; iq_lim <= 16'd8192; spd_div <= 8'd9;
      ang_off <= 16'd0; spd_shift <= 4'd4; ol_step <= 24'd0; forced_ang <= 16'd0;
      hall_map <= {3'd0, 3'd4, 3'd0, 3'd5, 3'd2, 3'd3, 3'd1, 3'd0};  // code 7..0
      hall_filt <= 8'd50;
      spi_cs <= 2'd0; spi_hold <= 1'b0; spi_div <= 4'd1; spi_auto <= 1'b0;
      fl_quad <= 1'b0; fl_late <= 1'b0; fl_dummy <= 3'd4; fl_div <= 4'd1;
      cpu_run <= boot; cpu_pause <= 1'b0; cpu_step <= 1'b0; cpu_bkhit <= 1'b0;
      cpu_boot <= 30'd0; bkpt_addr <= 30'd0; bkpt_en <= 1'b0;
      mbox_h2c <= 32'd0; mbox_c2h <= 32'd0; time_cnt <= 32'd0;
      wdt_lim <= 16'd1000; tel_en <= 1'b0; tel_n <= 4'd0; tel_div <= 8'd0; tel_addr <= 6'd0;
      ol_acc <= 32'd0;
      port_go <= 1'b0; port_w0 <= 1'b0; port_w1 <= 1'b0; port_n <= 4'd0; port_wdata <= 32'd0;
      link_div_set <= 1'b0; link_unlock <= 1'b0; link_div_val <= 16'd0;
    end else begin
      time_cnt <= time_cnt + 32'd1;
      ol_acc   <= ol_acc + {{8{ol_step[23]}}, ol_step};
      port_go  <= 1'b0;
      port_w0  <= 1'b0;
      port_w1  <= 1'b0;
      link_div_set <= 1'b0;
      link_unlock  <= 1'b0;
      cpu_step     <= 1'b0;

      // hardware breakpoint: pause when the CPU reaches the address
      if (bkpt_en && cpu_fentry && cpu_pc[31:2] == bkpt_addr && cpu_run) begin
        cpu_pause <= 1'b1;
        cpu_bkhit <= 1'b1;
      end

      // faults latch; any active fault disarms the bridge
      faults <= faults | fault_now;
      if (fault_active) ctrl[0] <= 1'b0;

      if (reg_wr) begin
        case (ra)
          6'h01: ctrl <= s_wdata[10:0];
          6'h02: faults <= (faults & ~s_wdata[5:0]) | fault_now;
          6'h03: fmask <= s_wdata[5:0];
          6'h04: pwm_period <= s_wdata[11:0];
          6'h05: begin pwm_dead <= s_wdata[7:0]; pol_h <= s_wdata[8]; pol_l <= s_wdata[9];
                       adc_adv <= s_wdata[27:16]; end
          6'h06: begin dmin <= s_wdata[11:0]; dmax <= s_wdata[27:16]; end
          6'h09: begin adc_nbits <= s_wdata[4:0]; adc_lsh <= s_wdata[8:5]; adc_div <= s_wdata[12:9];
                       adc_cpol <= s_wdata[13]; adc_late <= s_wdata[14]; adc_sign_a <= s_wdata[15];
                       adc_sign_b <= s_wdata[16]; adc_csdly <= s_wdata[31:24]; end
          6'h0B: begin off_a <= s_wdata[15:0]; off_b <= s_wdata[31:16]; end
          6'h0D: begin id_ref <= s_wdata[15:0]; iq_ref <= s_wdata[31:16]; end
          6'h0E: begin vd_cmd <= s_wdata[15:0]; vq_cmd <= s_wdata[31:16]; end
          6'h0F: begin kp <= s_wdata[15:0]; kp_sh <= s_wdata[19:16]; end
          6'h10: begin ki <= s_wdata[15:0]; ki_sh <= s_wdata[19:16]; end
          6'h11: begin vlim <= s_wdata[15:0]; oc_lim <= s_wdata[31:16]; end
          6'h12: begin spd_ref <= s_wdata[15:0]; spd_div <= s_wdata[23:16]; end
          6'h13: begin kps <= s_wdata[15:0]; kps_sh <= s_wdata[19:16]; end
          6'h14: begin kis <= s_wdata[15:0]; kis_sh <= s_wdata[19:16]; end
          6'h15: iq_lim <= s_wdata[15:0];
          6'h16: begin ang_off <= s_wdata[15:0]; spd_shift <= s_wdata[19:16]; end
          6'h17: ol_step <= s_wdata[23:0];
          6'h18: forced_ang <= s_wdata[15:0];
          6'h19: begin hall_map <= s_wdata[23:0]; hall_filt <= s_wdata[31:24]; end
          6'h20: begin spi_cs <= s_wdata[1:0]; spi_hold <= s_wdata[2]; spi_div <= s_wdata[6:3];
                       spi_auto <= s_wdata[7]; end
          6'h21: begin port_w0 <= 1'b1; port_wdata <= s_wdata; end
          6'h22: begin port_w1 <= 1'b1; port_wdata <= s_wdata;
                       if (spi_auto) begin port_go <= 1'b1; port_n <= 4'd8; end end
          6'h23: begin port_go <= 1'b1; port_n <= s_wdata[3:0]; end
          6'h24: begin fl_quad <= s_wdata[0]; fl_dummy <= s_wdata[3:1]; fl_div <= s_wdata[7:4];
                       fl_late <= s_wdata[8]; end
          6'h25: begin cpu_run <= s_wdata[0]; cpu_pause <= s_wdata[1]; cpu_step <= s_wdata[2];
                       cpu_bkhit <= 1'b0; end
          6'h27: mbox_h2c <= s_wdata;
          6'h28: mbox_c2h <= s_wdata;
          6'h2A: begin link_div_set <= !s_wdata[16]; link_unlock <= s_wdata[16];
                       link_div_val <= s_wdata[15:0]; end
          6'h2B: wdt_lim <= s_wdata[15:0];
          6'h2C: begin tel_en <= s_wdata[0]; tel_n <= s_wdata[11:8]; tel_div <= s_wdata[23:16]; end
          6'h2D: tel_addr <= s_wdata[7:2];
          6'h2F: cpu_boot <= s_wdata[31:2];
          6'h30: begin bkpt_addr <= s_wdata[31:2]; bkpt_en <= s_wdata[0]; end
          default: ;
        endcase
      end
    end
  end

endmodule
