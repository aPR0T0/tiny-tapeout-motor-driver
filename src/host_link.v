/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * host_link - error-resistant framed protocol over the autobaud UART.
 *
 * Framing  : HDLC style.  0x7E delimits frames, 0x7D escapes (next byte ^ 0x20).
 *            A corrupted byte can never desynchronize the receiver for more
 *            than one frame.
 * Integrity: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over every byte,
 *            sent MSB first.
 * Request  : SEQ CMD ADDR[4](LE) LEN [DATA 4*LEN (LE words)] CRC[2]
 * Response : SEQ STATUS [DATA] CRC[2]
 *   CMD 0x00 PING       -> data = ID word
 *       0x01 READ       LEN words from ADDR, ADDR+4, ...
 *       0x02 WRITE      LEN (1..2) words to ADDR, ADDR+4, ...
 *       0x03 WRITE_FIFO LEN words to the same ADDR
 *       0x04 READ_FIFO  LEN words from the same ADDR
 *   STATUS 0x00 OK, 0x01 CRC error, 0x02 bad command, 0x03 bad length,
 *          0x80 telemetry frame (SEQ = telemetry counter, unsolicited)
 * Retries  : writes carry a sequence number; a repeated write with the same SEQ
 *            (because the response was lost) is acknowledged but not executed
 *            again, so every operation can be retried safely.
 * Watchdog : if enabled, no valid frame for `wdt_lim` * 1024 clocks raises
 *            `wdt_expired` (used as a motor fault -> safe stop).
 * Telemetry: every `tel_div`+1 FOC updates, TEL_N words starting at TEL_ADDR are
 *            streamed to the host without being asked.
 */
`default_nettype none

module host_link (
    input  wire        clk,
    input  wire        rst,
    input  wire        rxd,
    output wire        txd,
    // baud control
    input  wire        div_set,
    input  wire [15:0] div_val,
    input  wire        unlock,
    output wire [15:0] div_fx,
    output wire        locked,
    // bus master
    output reg         m_req,
    output reg         m_we,
    output reg  [31:0] m_addr,
    output wire [31:0] m_wdata,
    input  wire [31:0] m_rdata,
    input  wire        m_ack,
    // id
    input  wire [31:0] id_word,
    // watchdog
    input  wire        wdt_en,
    input  wire [15:0] wdt_lim,
    output reg         wdt_expired,
    // telemetry
    input  wire        tel_en,
    input  wire        tel_tick,
    input  wire [7:0]  tel_div,
    input  wire [3:0]  tel_n,
    input  wire [31:0] tel_addr,
    // statistics
    output reg  [7:0]  crc_errs,
    output reg  [7:0]  frames_ok
);

  // ---------------------------------------------------------------- UART
  wire [7:0] rx_data;
  wire       rx_valid, rx_ferr, rx_brk;
  reg        tx_start;
  reg  [7:0] tx_data;
  wire       tx_busy;

  uart_rx u_rx (
      .clk(clk), .rst(rst), .rxd(rxd), .div_set(div_set), .div_val(div_val),
      .unlock(unlock), .div_fx(div_fx), .locked(locked),
      .data(rx_data), .valid(rx_valid), .ferr(rx_ferr), .brk(rx_brk)
  );

  uart_tx u_tx (
      .clk(clk), .rst(rst), .div_fx(div_fx), .start(tx_start), .data(tx_data),
      .busy(tx_busy), .txd(txd)
  );

  function [15:0] crc16_byte;
    input [15:0] c;
    input [7:0]  b;
    integer k;
    reg [15:0] x;
    begin
      x = c ^ {b, 8'h00};
      for (k = 0; k < 8; k = k + 1)
        x = x[15] ? ({x[14:0], 1'b0} ^ 16'h1021) : {x[14:0], 1'b0};
      crc16_byte = x;
    end
  endfunction

  // ---------------------------------------------------------------- RX deframer
  reg        esc, hunt, bad;
  reg [4:0]  rn;          // bytes in current frame (saturates at 31)
  reg [15:0] rcrc;
  reg [7:0]  f_seq, f_cmd, f_len;
  reg [31:0] f_addr;
  reg [63:0] f_buf;       // up to 2 data words
  reg        cmd_pend;    // a valid frame waits for the executor
  reg        crc_bad;     // a long frame failed its CRC -> NAK

  wire [7:0] rb = esc ? (rx_data ^ 8'h20) : rx_data;

  // ---------------------------------------------------------------- executor
  localparam [3:0] E_IDLE = 4'd0, E_HDR0 = 4'd1, E_HDR1 = 4'd2, E_RD = 4'd3, E_RDB = 4'd4,
                   E_WR = 4'd5, E_CRC0 = 4'd6, E_CRC1 = 4'd7, E_END = 4'd8, E_DONE = 4'd9;
  reg [3:0]  es;
  reg [7:0]  r_seq, r_status;
  reg [8:0]  wleft;       // words left
  reg [1:0]  bi;          // byte index in word
  reg [31:0] word;
  reg        wi;          // write word index
  reg        is_tel, noinc, from_cmd;
  reg [15:0] tcrc;
  reg [7:0]  last_seq;
  reg        last_valid;
  reg [7:0]  tel_seq, tel_cnt;
  reg        tel_due;

  // executor -> stuffer handshake
  reg       es_emit;
  reg [7:0] es_byte;
  reg       es_raw;

  // frame length bookkeeping
  reg  [4:0] rn_at_frame;
  wire [4:0] wr_len_expect = 5'd9 + {f_len[2:0], 2'b00};
  assign m_wdata = wi ? f_buf[63:32] : f_buf[31:0];

  // TX byte stuffer: out_req with a raw byte, `stuff` handles the escape
  reg        ob_valid;   // a raw byte waits to be sent
  reg  [7:0] ob_byte;
  reg        ob_raw;     // send as-is (flags)
  reg        ob_esc2;    // second half of an escape pending
  wire       ob_free = !ob_valid && !ob_esc2;

  always @(posedge clk) begin
    if (rst) begin
      tx_start <= 1'b0; tx_data <= 8'd0;
      ob_valid <= 1'b0; ob_byte <= 8'd0; ob_raw <= 1'b0; ob_esc2 <= 1'b0;
    end else begin
      tx_start <= 1'b0;
      if (!tx_busy && !tx_start) begin
        if (ob_esc2) begin
          tx_data  <= ob_byte ^ 8'h20;
          tx_start <= 1'b1;
          ob_esc2  <= 1'b0;
          ob_valid <= 1'b0;
        end else if (ob_valid) begin
          if (!ob_raw && (ob_byte == 8'h7E || ob_byte == 8'h7D)) begin
            tx_data  <= 8'h7D;
            tx_start <= 1'b1;
            ob_esc2  <= 1'b1;
          end else begin
            tx_data  <= ob_byte;
            tx_start <= 1'b1;
            ob_valid <= 1'b0;
          end
        end
      end
      if (es_emit) begin
        ob_valid <= 1'b1;
        ob_byte  <= es_byte;
        ob_raw   <= es_raw;
      end
    end
  end

  wire is_rd  = (f_cmd == 8'h01) || (f_cmd == 8'h04);
  wire is_wr  = (f_cmd == 8'h02) || (f_cmd == 8'h03);
  wire is_png = (f_cmd == 8'h00);

  always @(posedge clk) begin
    if (rst) begin
      esc <= 1'b0; hunt <= 1'b1; bad <= 1'b0; rn <= 5'd0; rcrc <= 16'hFFFF;
      f_seq <= 8'd0; f_cmd <= 8'd0; f_len <= 8'd0; f_addr <= 32'd0; f_buf <= 64'd0;
      cmd_pend <= 1'b0; crc_bad <= 1'b0;
      es <= E_IDLE; r_seq <= 8'd0; r_status <= 8'd0; wleft <= 9'd0; bi <= 2'd0;
      word <= 32'd0; wi <= 1'b0; is_tel <= 1'b0; noinc <= 1'b0; from_cmd <= 1'b0;
      tcrc <= 16'hFFFF; last_seq <= 8'd0; last_valid <= 1'b0;
      tel_seq <= 8'd0; tel_cnt <= 8'd0; tel_due <= 1'b0;
      m_req <= 1'b0; m_we <= 1'b0; m_addr <= 32'd0;
      es_emit <= 1'b0; es_byte <= 8'd0; es_raw <= 1'b0;
      crc_errs <= 8'd0; frames_ok <= 8'd0;
    end else begin
      es_emit <= 1'b0;

      // ---------------- telemetry timing ----------------
      if (tel_tick && tel_en) begin
        if (tel_cnt >= tel_div) begin
          tel_cnt <= 8'd0;
          tel_due <= 1'b1;
        end else tel_cnt <= tel_cnt + 8'd1;
      end
      if (!tel_en) tel_due <= 1'b0;

      // ---------------- receive ----------------
      // While a command is pending / executing the receiver drops everything
      // and resynchronizes on the next flag (the protocol is stop-and-wait).
      if (rx_brk || rx_ferr) begin
        hunt <= 1'b1;
      end else if (rx_valid) begin
        if (cmd_pend) begin
          hunt <= 1'b1;
        end else if (rx_data == 8'h7E) begin
          if (!hunt && rn >= 5'd9) begin
            if (rcrc == 16'h0000 && !bad) cmd_pend <= 1'b1;
            else begin
              crc_bad  <= 1'b1;
              crc_errs <= crc_errs + 8'd1;
            end
          end
          hunt <= 1'b0;
          esc  <= 1'b0;
          bad  <= 1'b0;
          rn   <= 5'd0;
          rcrc <= 16'hFFFF;
        end else if (!hunt) begin
          if (rx_data == 8'h7D) begin
            esc <= 1'b1;
          end else begin
            esc  <= 1'b0;
            rcrc <= crc16_byte(rcrc, rb);
            if (rn != 5'd31) rn <= rn + 5'd1;
            else             bad <= 1'b1;
            case (rn)
              5'd0: f_seq  <= rb;
              5'd1: f_cmd  <= rb;
              5'd2: f_addr[7:0]   <= rb;
              5'd3: f_addr[15:8]  <= rb;
              5'd4: f_addr[23:16] <= rb;
              5'd5: f_addr[31:24] <= rb;
              5'd6: f_len  <= rb;
              default: if (rn < 5'd15) f_buf[(rn - 5'd7)*8 +: 8] <= rb;
            endcase
          end
        end
      end

      // ---------------- executor ----------------
      case (es)
        E_IDLE: begin
          if (crc_bad) begin
            crc_bad  <= 1'b0;
            r_seq    <= f_seq;
            r_status <= 8'h01;
            wleft    <= 9'd0;
            is_tel   <= 1'b0;
            from_cmd <= 1'b0;
            es       <= E_HDR0;
          end else if (cmd_pend) begin
            frames_ok <= frames_ok + 8'd1;
            r_seq     <= f_seq;
            is_tel    <= 1'b0;
            from_cmd  <= 1'b1;
            noinc     <= (f_cmd == 8'h03) || (f_cmd == 8'h04);
            m_addr    <= f_addr;
            wi        <= 1'b0;
            if ((is_png || is_rd) && rn_at_frame != 5'd9) begin
              r_status <= 8'h03; wleft <= 9'd0; es <= E_HDR0;
            end else if (is_png) begin
              r_status <= 8'h00; wleft <= 9'd1; word <= id_word; es <= E_HDR0;
            end else if (is_rd) begin
              if (f_len == 8'd0) begin
                r_status <= 8'h03; wleft <= 9'd0; es <= E_HDR0;
              end else begin
                r_status <= 8'h00; wleft <= {1'b0, f_len}; es <= E_HDR0;
              end
            end else if (is_wr) begin
              if (f_len == 8'd0 || f_len > 8'd2 || rn_at_frame != wr_len_expect) begin
                r_status <= 8'h03; wleft <= 9'd0; es <= E_HDR0;
              end else if (last_valid && f_seq == last_seq) begin
                r_status <= 8'h00; wleft <= 9'd0; es <= E_HDR0;   // duplicate: ack only
              end else begin
                r_status   <= 8'h00;
                wleft      <= {1'b0, f_len};
                last_seq   <= f_seq;
                last_valid <= 1'b1;
                es         <= E_WR;
              end
            end else begin
              r_status <= 8'h02; wleft <= 9'd0; es <= E_HDR0;
            end
          end else if (tel_due && tel_n != 4'd0) begin
            tel_due  <= 1'b0;
            tel_seq  <= tel_seq + 8'd1;
            r_seq    <= tel_seq;
            r_status <= 8'h80;
            is_tel   <= 1'b1;
            from_cmd <= 1'b0;
            noinc    <= 1'b0;
            m_addr   <= tel_addr;
            wleft    <= {5'd0, tel_n};
            es       <= E_HDR0;
          end
        end

        E_WR: begin
          if (!m_req) begin
            m_req   <= 1'b1;
            m_we    <= 1'b1;
          end else if (m_ack) begin
            m_req <= 1'b0;
            m_we  <= 1'b0;
            wi    <= 1'b1;
            if (!noinc) m_addr <= m_addr + 32'd4;
            wleft <= wleft - 9'd1;
            if (wleft == 9'd1) begin
              wleft <= 9'd0;
              es    <= E_HDR0;
            end
          end
        end

        E_HDR0: if (ob_free && !es_emit) begin    // opening flag
          es_emit <= 1'b1; es_byte <= 8'h7E; es_raw <= 1'b1;
          tcrc    <= 16'hFFFF;
          es      <= E_HDR1;
          bi      <= 2'd0;
        end

        E_HDR1: if (ob_free && !es_emit) begin
          es_emit <= 1'b1; es_raw <= 1'b0;
          if (bi == 2'd0) begin
            es_byte <= r_seq;
            tcrc    <= crc16_byte(tcrc, r_seq);
            bi      <= 2'd1;
          end else begin
            es_byte <= r_status;
            tcrc    <= crc16_byte(tcrc, r_status);
            bi      <= 2'd0;
            if (wleft == 9'd0 || r_status == 8'h01 || r_status == 8'h02 || r_status == 8'h03 ||
                (from_cmd && is_wr))
              es <= E_CRC0;
            else if (from_cmd && is_png)
              es <= E_RDB;
            else
              es <= E_RD;
          end
        end

        E_RD: begin
          if (!m_req) begin
            m_req <= 1'b1;
            m_we  <= 1'b0;
          end else if (m_ack) begin
            m_req <= 1'b0;
            word  <= m_rdata;
            if (!noinc) m_addr <= m_addr + 32'd4;
            bi    <= 2'd0;
            es    <= E_RDB;
          end
        end

        E_RDB: if (ob_free && !es_emit) begin
          es_emit <= 1'b1; es_raw <= 1'b0;
          es_byte <= word[bi*8 +: 8];
          tcrc    <= crc16_byte(tcrc, word[bi*8 +: 8]);
          bi      <= bi + 2'd1;
          if (bi == 2'd3) begin
            wleft <= wleft - 9'd1;
            es    <= (wleft == 9'd1) ? E_CRC0 : E_RD;
          end
        end

        E_CRC0: if (ob_free && !es_emit) begin
          es_emit <= 1'b1; es_raw <= 1'b0; es_byte <= tcrc[15:8];
          es      <= E_CRC1;
        end

        E_CRC1: if (ob_free && !es_emit) begin
          es_emit <= 1'b1; es_raw <= 1'b0; es_byte <= tcrc[7:0];
          es      <= E_END;
        end

        E_END: if (ob_free && !es_emit) begin
          es_emit <= 1'b1; es_raw <= 1'b1; es_byte <= 8'h7E;
          es      <= E_DONE;
        end

        default: begin  // E_DONE
          if (from_cmd) cmd_pend <= 1'b0;
          es <= E_IDLE;
        end
      endcase
    end
  end

  always @(posedge clk) begin
    if (rst) rn_at_frame <= 5'd0;
    else if (rx_valid && rx_data == 8'h7E && !hunt && !cmd_pend) rn_at_frame <= rn;
  end

  // ---------------- watchdog ----------------
  reg [9:0]  wdt_pre;
  reg [15:0] wdt_cnt;
  always @(posedge clk) begin
    if (rst || !wdt_en || (cmd_pend && es == E_IDLE)) begin
      wdt_pre     <= 10'd0;
      wdt_cnt     <= 16'd0;
      wdt_expired <= 1'b0;
    end else if (!wdt_expired) begin
      wdt_pre <= wdt_pre + 10'd1;
      if (wdt_pre == 10'h3FF) begin
        wdt_cnt <= wdt_cnt + 16'd1;
        if (wdt_cnt >= wdt_lim) wdt_expired <= 1'b1;
      end
    end
  end

endmodule
