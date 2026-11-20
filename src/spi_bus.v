/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * spi_bus - one shared (Q)SPI bus, three chip selects, three clients:
 *
 *   CS0  flash   : execute-in-place reads (0x03 single or 0xEB quad I/O),
 *                  sequential words are streamed without re-sending the command
 *   CS1  ADC     : dual simultaneous-sampling SPI ADC (SDOA on SD1, SDOB on a
 *                  dedicated input), started by the PWM trigger.  HIGHEST priority:
 *                  any flash transfer is aborted immediately and retried later,
 *                  so the current sampling instant has a fixed latency.
 *   CS0..2 port  : byte transfers from an 8-byte buffer (flash programming,
 *                  gyro / IMU, anything SPI).  With `hold` the CS stays asserted
 *                  between transfers (needed for flash page programming).
 *                  With `port_sync` a transfer may only start within 31 clocks
 *                  after an ADC conversion, so it gets (almost) the whole PWM
 *                  period before the next sample and never delays it.
 *
 * SPI mode 0 (CPOL=0, sample on rising edge); the ADC may use CPOL=1.
 * SCK half period = (div+1) clocks.  `late` samples at the falling edge instead
 * (more round-trip delay margin at high SCK rates).
 * Pins follow the Tiny Tapeout QSPI Pmod: SD0..SD3, SD2=WP#, SD3=HOLD# driven
 * high in single-bit mode.
 */
`default_nettype none

module spi_bus (
    input  wire        clk,
    input  wire        rst,
    // pins
    output wire        sck,
    output reg  [2:0]  cs_n,
    output wire [3:0]  sd_o,
    output wire [3:0]  sd_oe,
    input  wire [3:0]  sd_i,
    input  wire        sdob_i,
    // flash configuration
    input  wire        fl_quad,
    input  wire [2:0]  fl_dummy,
    input  wire [3:0]  fl_div,
    input  wire        fl_late,
    // ADC configuration
    input  wire [4:0]  adc_nbits,
    input  wire [3:0]  adc_lsh,
    input  wire [3:0]  adc_div,
    input  wire [7:0]  adc_csdly,
    input  wire        adc_cpol,
    input  wire        adc_late,
    // port configuration
    input  wire [3:0]  port_div,
    input  wire        port_sync,
    // flash read requesters (0 = host link, gets an error if the port holds CS; 1 = CPU)
    input  wire        fr0_req,
    input  wire [23:0] fr0_addr,
    output reg         fr0_ack,
    input  wire        fr1_req,
    input  wire [23:0] fr1_addr,
    output reg         fr1_ack,
    output wire [31:0] fr_data,
    // ADC
    input  wire        adc_trig,
    output wire [15:0] adc_a,      // valid from adc_done for >= 6 clocks
    output wire [15:0] adc_b,
    output reg         adc_done,
    output reg         adc_drop,
    // port
    input  wire [1:0]  port_cs,
    input  wire        port_hold,
    input  wire        port_go,
    input  wire [3:0]  port_n,
    input  wire        port_w0,
    input  wire        port_w1,
    input  wire [31:0] port_wdata,
    output reg  [63:0] port_buf,
    output wire        port_busy,
    output wire        port_owns
);

  // ---------------------------------------------------------------- bit engine
  reg        e_busy, e_sck, e_quad, e_late, e_done;
  reg [5:0]  e_n;
  reg [3:0]  e_hc, e_div, e_oe;
  reg [31:0] e_out, e_in;
  reg [15:0] e_inb;
  reg        cpol_on;

  assign sck   = e_sck ^ cpol_on;
  assign sd_o  = e_quad ? e_out[31:28] : {2'b11, 1'b0, e_out[31]};
  assign sd_oe = e_quad ? e_oe : 4'b1101;

  // ---------------------------------------------------------------- control
  localparam [3:0] M_IDLE = 4'd0, M_GAP = 4'd1, M_ADC_PRE = 4'd2, M_ADC_DLY = 4'd3,
                   M_ADC_X = 4'd4, M_ADC_POST = 4'd5, M_PORT_X = 4'd6, M_PORT_HOLD = 4'd7,
                   M_FL_CMD = 4'd8, M_FL_ADDR = 4'd9, M_FL_DUM = 4'd10, M_FL_DATA = 4'd11,
                   M_FL_STREAM = 4'd12;

  reg [3:0]  m;
  reg [7:0]  cnt;
  reg        adc_pend, port_pend, port_cs_on;
  reg [4:0]  sync_win;    // clocks left in the post-conversion window
  reg [3:0]  port_left;
  reg [2:0]  port_i;
  reg [1:0]  port_cs_r;
  reg        fl_who;
  reg        fl_q;        // quad mode latched for the whole transaction / stream
  reg [23:0] fl_addr;
  reg        fr_err;
  wire [23:0] fl_next = fl_addr + 24'd4;

  assign port_busy = port_pend || (m == M_PORT_X);
  assign port_owns = port_cs_on;

  wire        fr_any  = fr0_req | fr1_req;
  wire [23:0] fr_a    = fr0_req ? fr0_addr : fr1_addr;
  wire        port_ok = port_pend && (!port_sync || sync_win != 5'd0);
  wire        fl_st   = (m == M_FL_CMD) || (m == M_FL_ADDR) || (m == M_FL_DUM) || (m == M_FL_DATA);
  wire [7:0]  port_tx = port_buf[port_i*8 +: 8];

  wire [31:0] fl_word = {e_in[7:0], e_in[15:8], e_in[23:16], e_in[31:24]};
  assign fr_data = fr_err ? 32'hFFFFFFFF : fl_word;
  assign adc_a   = e_in[15:0] << adc_lsh;
  assign adc_b   = e_inb << adc_lsh;

  always @(posedge clk) begin
    if (rst) begin
      e_busy <= 1'b0; e_sck <= 1'b0; e_quad <= 1'b0; e_late <= 1'b0; e_done <= 1'b0;
      e_n <= 6'd0; e_hc <= 4'd0; e_div <= 4'd0; e_oe <= 4'b1101;
      e_out <= 32'd0; e_in <= 32'd0; e_inb <= 16'd0; cpol_on <= 1'b0;
      m <= M_IDLE; cnt <= 8'd0; cs_n <= 3'b111;
      adc_pend <= 1'b0; port_pend <= 1'b0; port_cs_on <= 1'b0; sync_win <= 5'd0;
      port_left <= 4'd0; port_i <= 3'd0; port_cs_r <= 2'd0;
      fl_who <= 1'b0; fl_q <= 1'b0; fl_addr <= 24'd0; fr_err <= 1'b0;
      fr0_ack <= 1'b0; fr1_ack <= 1'b0;
      adc_done <= 1'b0; adc_drop <= 1'b0;
      port_buf <= 64'd0;
    end else begin
      fr0_ack  <= 1'b0;
      fr1_ack  <= 1'b0;
      adc_done <= 1'b0;
      adc_drop <= 1'b0;
      e_done   <= 1'b0;

      // ---- requests ----
      if (sync_win != 5'd0) sync_win <= sync_win - 5'd1;
      if (adc_trig) begin
        sync_win <= 5'd0;
        if (port_cs_on) adc_drop <= 1'b1;   // the port owns the bus: skip this sample
        else            adc_pend <= 1'b1;
      end
      if (port_go && port_n != 4'd0 && !port_busy) begin
        port_pend <= 1'b1;
        port_left <= (port_n > 4'd8) ? 4'd8 : port_n;
        port_i    <= 3'd0;
      end
      if (port_w0) port_buf[31:0]  <= port_wdata;
      if (port_w1) port_buf[63:32] <= port_wdata;

      // ---- bit engine ----
      if (e_busy) begin
        if (e_hc != 4'd0) begin
          e_hc <= e_hc - 4'd1;
        end else if (!e_sck) begin
          e_sck <= 1'b1;
          e_hc  <= e_div;
          if (!e_late) begin
            e_in  <= e_quad ? {e_in[27:0], sd_i} : {e_in[30:0], sd_i[1]};
            e_inb <= {e_inb[14:0], sdob_i};
          end
        end else begin
          e_sck <= 1'b0;
          e_hc  <= e_div;
          if (e_late) begin
            e_in  <= e_quad ? {e_in[27:0], sd_i} : {e_in[30:0], sd_i[1]};
            e_inb <= {e_inb[14:0], sdob_i};
          end
          e_out <= e_quad ? {e_out[27:0], 4'h0} : {e_out[30:0], 1'b0};
          e_n   <= e_n - 6'd1;
          if (e_n == 6'd1) begin
            e_busy <= 1'b0;
            e_done <= 1'b1;
          end
        end
      end

      // ---- bus state machine ----
      case (m)
        M_IDLE: begin
          e_quad <= 1'b0;
          if (adc_pend) begin
            cpol_on <= adc_cpol;
            cnt     <= 8'd2;
            m       <= M_ADC_PRE;
          end else if (port_ok) begin
            sync_win  <= 5'd0;
            port_cs_r <= port_cs;
            port_cs_on <= 1'b1;
            cs_n[port_cs] <= 1'b0;
            start_eng(6'd8, 1'b0, 4'b1101, {port_tx, 24'd0}, port_div, 1'b0);
            m <= M_PORT_X;
          end else if (fr_any) begin
            fl_who  <= !fr0_req;
            fl_q    <= fl_quad;
            fl_addr <= {fr_a[23:2], 2'b00};
            cs_n[0] <= 1'b0;
            if (fl_quad) start_eng(6'd8,  1'b0, 4'b1101, {8'hEB, 24'd0}, fl_div, fl_late);
            else         start_eng(6'd32, 1'b0, 4'b1101, {8'h03, fr_a[23:2], 2'b00}, fl_div, fl_late);
            m <= M_FL_CMD;
          end
        end

        M_GAP: begin
          cs_n   <= 3'b111;
          e_quad <= 1'b0;
          if (cnt == 8'd0) m <= M_IDLE;
          else             cnt <= cnt - 8'd1;
        end

        // ---------------- ADC ----------------
        M_ADC_PRE: begin
          if (cnt == 8'd0) begin
            cs_n[1] <= 1'b0;
            cnt     <= adc_csdly;
            m       <= M_ADC_DLY;
          end else cnt <= cnt - 8'd1;
        end

        M_ADC_DLY: begin
          if (cnt == 8'd0) begin
            adc_pend <= 1'b0;
            start_eng({1'b0, adc_nbits}, 1'b0, 4'b1101, 32'd0, adc_div, adc_late);
            m <= M_ADC_X;
          end else cnt <= cnt - 8'd1;
        end

        M_ADC_X: if (e_done) begin
          adc_done <= 1'b1;
          sync_win <= 5'd31;
          cs_n[1]  <= 1'b1;
          cnt      <= 8'd2;
          m        <= M_ADC_POST;
        end

        M_ADC_POST: begin
          if (cnt == 8'd0) begin
            cpol_on <= 1'b0;
            cnt     <= 8'd3;
            m       <= M_GAP;
          end else cnt <= cnt - 8'd1;
        end

        // ---------------- port ----------------
        M_PORT_X: begin
          // host flash reads fail fast only in manual-hold mode (CS may stay low
          // indefinitely); during a short burst they simply wait
          if (fr0_req && !fr0_ack && port_hold) begin fr0_ack <= 1'b1; fr_err <= 1'b1; end
          if (e_done) begin
            port_buf[port_i*8 +: 8] <= e_in[7:0];
            port_i    <= port_i + 3'd1;
            port_left <= port_left - 4'd1;
            if (port_left == 4'd1) begin
              port_pend <= 1'b0;
              if (port_hold) begin
                m <= M_PORT_HOLD;
              end else begin
                port_cs_on <= 1'b0;
                cs_n       <= 3'b111;
                cnt        <= 8'd3;
                m          <= M_GAP;
              end
            end else begin
              start_eng(6'd8, 1'b0, 4'b1101, {port_buf[(port_i+3'd1)*8 +: 8], 24'd0}, port_div, 1'b0);
            end
          end
        end

        M_PORT_HOLD: begin
          if (fr0_req && !fr0_ack) begin fr0_ack <= 1'b1; fr_err <= 1'b1; end
          if (port_pend) begin
            start_eng(6'd8, 1'b0, 4'b1101, {port_tx, 24'd0}, port_div, 1'b0);
            m <= M_PORT_X;
          end else if (!port_hold || port_cs != port_cs_r) begin
            port_cs_on <= 1'b0;
            cs_n       <= 3'b111;
            cnt        <= 8'd3;
            m          <= M_GAP;
          end
        end

        // ---------------- flash ----------------
        M_FL_CMD: if (e_done) begin
          if (fl_q) begin
            start_eng(6'd8, 1'b1, 4'b1111, {fl_addr, 8'hFF}, fl_div, fl_late);
            m <= M_FL_ADDR;
          end else begin
            start_eng(6'd32, 1'b0, 4'b1101, 32'd0, fl_div, fl_late);
            m <= M_FL_DATA;
          end
        end

        M_FL_ADDR: if (e_done) begin
          if (fl_dummy != 3'd0) begin
            start_eng({3'd0, fl_dummy}, 1'b1, 4'b0000, 32'd0, fl_div, fl_late);
            m <= M_FL_DUM;
          end else begin
            start_eng(6'd8, 1'b1, 4'b0000, 32'd0, fl_div, fl_late);
            m <= M_FL_DATA;
          end
        end

        M_FL_DUM: if (e_done) begin
          start_eng(6'd8, 1'b1, 4'b0000, 32'd0, fl_div, fl_late);
          m <= M_FL_DATA;
        end

        M_FL_DATA: if (e_done) begin
          fr_err <= 1'b0;
          if (fl_who) fr1_ack <= 1'b1;
          else        fr0_ack <= 1'b1;
          m       <= M_FL_STREAM;
        end

        M_FL_STREAM: begin
          // CS0 stays low; a sequential request continues without a new command
          if (adc_pend || port_ok) begin
            cs_n <= 3'b111;
            cnt  <= 8'd3;
            m    <= M_GAP;
          end else if (fr_any && !fr0_ack && !fr1_ack) begin
            if ({fr_a[23:2], 2'b00} == fl_next && fl_q == fl_quad) begin
              fl_who  <= !fr0_req;
              fl_addr <= fl_next;
              if (fl_q) start_eng(6'd8,  1'b1, 4'b0000, 32'd0, fl_div, fl_late);
              else         start_eng(6'd32, 1'b0, 4'b1101, 32'd0, fl_div, fl_late);
              m <= M_FL_DATA;
            end else begin
              cs_n <= 3'b111;
              cnt  <= 8'd3;
              m    <= M_GAP;
            end
          end
        end

        default: m <= M_IDLE;
      endcase

      // ---- ADC / port pre-empt any flash transfer ----
      if (fl_st && (adc_pend || port_ok)) begin
        e_busy <= 1'b0;
        e_sck  <= 1'b0;
        e_done <= 1'b0;
        cs_n   <= 3'b111;
        cnt    <= 8'd3;
        m      <= M_GAP;
      end
    end
  end

  task start_eng;
    input [5:0]  n;
    input        q;
    input [3:0]  oe;
    input [31:0] d;
    input [3:0]  dv;
    input        lt;
    begin
      e_busy <= 1'b1;
      e_sck  <= 1'b0;
      e_n    <= n;
      e_quad <= q;
      e_oe   <= oe;
      e_out  <= d;
      e_div  <= dv;
      e_hc   <= dv;
      e_late <= lt;
    end
  endtask

endmodule
