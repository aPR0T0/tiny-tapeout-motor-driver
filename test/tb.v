`default_nettype none
`timescale 1ns / 1ps

/*
 * Testbench for tt_um_apr0t0_foc_mcu.
 *
 *   - QSPI flash model on CS0 (uio[0]) : READ 03h, quad I/O read EBh, RDSR,
 *     WREN, page program, 4K sector erase, JEDEC ID
 *   - dual simultaneous-sampling 16-bit ADC on CS1 (uio[6]); SDOA on SD1,
 *     SDOB on ui[5]; samples latched 1 ns after CS falls (values come from Python)
 *   - SPI register-file slave on CS2 (uio[7]) standing in for the gyro / IMU
 *   - gate on-time counters for the Python motor model
 *   - bus contention checker
 */
module tb ();

  initial begin
    if ($test$plusargs("dump")) begin
      $dumpfile("tb.fst");
      $dumpvars(0, tb);
    end
  end

  reg        clk;
  reg        rst_n;
  reg        ena;
  reg [2:0]  hall;
  reg        uart_rx;
  reg        nfault;
  reg        nestop;
  reg        boot;
  wire       adc_sdob;
  wire [7:0] ui_in = {boot, nestop, adc_sdob, nfault, uart_rx, hall};
  wire [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_apr0t0_foc_mcu user_project (
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif
      .ui_in  (ui_in),
      .uo_out (uo_out),
      .uio_in (uio_in),
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  // ---------------------------------------------------------------- pins
  wire uart_tx  = uo_out[4];
  wire drv_en   = uo_out[7];
  // gate signals in a fixed order for the tests: {CL, CH, BL, BH, AL, AH}
  wire [5:0] gates = {uo_out[6], uo_out[5], uo_out[3], uo_out[2], uo_out[1], uo_out[0]};
  wire fl_csn   = uio_oe[0] ? uio_out[0] : 1'b1;   // external pull-ups
  wire sck      = uio_oe[3] ? uio_out[3] : 1'b0;
  wire adc_csn  = uio_oe[6] ? uio_out[6] : 1'b1;
  wire aux_csn  = uio_oe[7] ? uio_out[7] : 1'b1;

  wire [3:0] chip_sd_o  = {uio_out[5], uio_out[4], uio_out[2], uio_out[1]};
  wire [3:0] chip_sd_oe = {uio_oe[5],  uio_oe[4],  uio_oe[2],  uio_oe[1]};

  wire [3:0] fl_o, fl_oe;
  wire       adc_sdoa, adc_sdoa_oe;
  wire       aux_miso, aux_miso_oe;

  // resolved bus (pull-ups when nobody drives)
  wire [3:0] sd;
  assign sd[0] = chip_sd_oe[0] ? chip_sd_o[0] : fl_oe[0] ? fl_o[0] : 1'b1;
  assign sd[1] = chip_sd_oe[1] ? chip_sd_o[1] : fl_oe[1] ? fl_o[1] :
                 adc_sdoa_oe ? adc_sdoa : aux_miso_oe ? aux_miso : 1'b1;
  assign sd[2] = chip_sd_oe[2] ? chip_sd_o[2] : fl_oe[2] ? fl_o[2] : 1'b1;
  assign sd[3] = chip_sd_oe[3] ? chip_sd_o[3] : fl_oe[3] ? fl_o[3] : 1'b1;

  assign uio_in = {aux_csn, adc_csn, sd[3], sd[2], sck, sd[1], sd[0], fl_csn};

  // contention monitor
  integer contention = 0;
  always @(posedge clk) begin
    if ((|(chip_sd_oe & fl_oe)) ||
        (chip_sd_oe[1] && (adc_sdoa_oe || aux_miso_oe)) ||
        (fl_oe[1] && (adc_sdoa_oe || aux_miso_oe)) ||
        (adc_sdoa_oe && aux_miso_oe)) begin
      contention = contention + 1;
      if (contention <= 5)
        $display("%t CONTENTION chip_oe=%b fl_oe=%b adc=%b aux=%b fl_csn=%b fl.cmd=%h fl.nclk=%0d",
                 $time, chip_sd_oe, fl_oe, adc_sdoa_oe, aux_miso_oe, fl_csn, flash.cmd, flash.nclk);
    end
  end

  // ---------------------------------------------------------------- models
  spiflash_model #(.MEMBYTES(65536)) flash (
      .csn(fl_csn), .sck(sck), .sd_i(sd), .sd_o(fl_o), .sd_oe(fl_oe)
  );

  reg [15:0] adc_val_a, adc_val_b;
  adc_model adc (
      .csn(adc_csn), .sck(sck), .val_a(adc_val_a), .val_b(adc_val_b),
      .sdoa(adc_sdoa), .sdoa_oe(adc_sdoa_oe), .sdob(adc_sdob)
  );

  spireg_model aux (
      .csn(aux_csn), .sck(sck), .mosi(sd[0]), .miso(aux_miso), .miso_oe(aux_miso_oe)
  );

  // ---------------------------------------------------------------- gate counters
  reg [31:0] cyc;
  reg [31:0] on_ah, on_al, on_bh, on_bl, on_ch, on_cl;
  reg [31:0] shoot_through;
  initial begin
    cyc = 0; on_ah = 0; on_al = 0; on_bh = 0; on_bl = 0; on_ch = 0; on_cl = 0;
    shoot_through = 0;
    adc_val_a = 16'h8000; adc_val_b = 16'h8000;
  end
  always @(posedge clk) begin
    cyc   <= cyc + 1;
    on_ah <= on_ah + (gates[0] === 1'b1);
    on_al <= on_al + (gates[1] === 1'b1);
    on_bh <= on_bh + (gates[2] === 1'b1);
    on_bl <= on_bl + (gates[3] === 1'b1);
    on_ch <= on_ch + (gates[4] === 1'b1);
    on_cl <= on_cl + (gates[5] === 1'b1);
    if (((gates[0] & gates[1]) | (gates[2] & gates[3]) | (gates[4] & gates[5])) === 1'b1)
      shoot_through <= shoot_through + 1;
  end

endmodule

// =============================================================================
// SPI NOR flash model (W25Q-like subset)
// =============================================================================
module spiflash_model #(
    parameter MEMBYTES = 65536
) (
    input  wire       csn,
    input  wire       sck,
    input  wire [3:0] sd_i,
    output reg  [3:0] sd_o,
    output reg  [3:0] sd_oe
);
  reg [7:0]  mem [0:MEMBYTES-1];
  reg [7:0]  cmd;
  reg [31:0] addr;
  integer    nclk;       // clocks since CS fell
  reg [7:0]  sr;         // status register: bit0 WIP, bit1 WEL
  reg [7:0]  dbyte;
  reg [7:0]  pp_buf [0:255];
  integer    pp_n;
  reg [23:0] pp_addr;
  integer    i;
  reg [7:0]  outb;
  integer    obit;
  integer    nreads;     // statistics
  integer    nerase, nprog;
  reg [1023:0] hexfile;

  initial begin
    for (i = 0; i < MEMBYTES; i = i + 1) mem[i] = 8'hFF;
    if ($value$plusargs("flash=%s", hexfile)) $readmemh(hexfile, mem);
    sr = 8'h00; sd_o = 4'h0; sd_oe = 4'h0; nreads = 0; nerase = 0; nprog = 0;
  end

  always @(negedge csn) begin
    nclk  = 0;
    cmd   = 8'h00;
    addr  = 0;
    pp_n  = 0;
    sd_oe = 4'h0;
  end

  always @(posedge csn) begin
    sd_oe = 4'h0;
    if (cmd == 8'h06) sr[1] = 1'b1;
    if (cmd == 8'h04) sr[1] = 1'b0;
    if (cmd == 8'h02 && sr[1] && !sr[0] && nclk >= 32 + 8) begin
      for (i = 0; i < pp_n && i < 256; i = i + 1)
        mem[{pp_addr[23:8], pp_addr[7:0] + i[7:0]} % MEMBYTES] =
          mem[{pp_addr[23:8], pp_addr[7:0] + i[7:0]} % MEMBYTES] & pp_buf[i];
      nprog = nprog + 1;
      sr = 8'h03; #(2000); sr = 8'h00;
    end
    if (cmd == 8'h20 && sr[1] && !sr[0] && nclk >= 32) begin
      for (i = 0; i < 4096; i = i + 1) mem[({addr[23:12], 12'h000} + i) % MEMBYTES] = 8'hFF;
      nerase = nerase + 1;
      sr = 8'h03; #(5000); sr = 8'h00;
    end
  end

  // inputs sampled on rising edges
  always @(posedge sck) begin
    if (!csn) begin
      if (nclk < 8) begin
        cmd = {cmd[6:0], sd_i[0]};
      end else if (cmd == 8'h03 || cmd == 8'h02 || cmd == 8'h20) begin
        if (nclk < 32) addr = {addr[30:0], sd_i[0]};
        else if (cmd == 8'h02) begin
          dbyte = {dbyte[6:0], sd_i[0]};
          if (((nclk - 32) % 8) == 7) begin
            if (pp_n == 0) pp_addr = addr[23:0];
            if (pp_n < 256) pp_buf[pp_n] = dbyte;
            pp_n = pp_n + 1;
          end
        end
      end else if (cmd == 8'hEB) begin
        if (nclk < 14) addr = {addr[27:0], sd_i};   // 6 address nibbles
      end
      nclk = nclk + 1;
    end
  end

  // outputs change on falling edges
  always @(negedge sck) begin
    if (!csn) begin
      if (cmd == 8'h03 && nclk >= 32) begin
        obit = (nclk - 32) % 8;
        if (obit == 0) begin
          outb = mem[addr % MEMBYTES];
          addr = addr + 1;
          nreads = nreads + 1;
        end
        sd_o  = {2'b00, outb[7 - obit], 1'b0};
        sd_oe = 4'b0010;
      end else if (cmd == 8'hEB && nclk >= 20) begin
        // 8 cmd + 6 addr + 2 mode + 4 dummy = 20 clocks, then nibbles
        if (((nclk - 20) % 2) == 0) begin
          outb = mem[addr % MEMBYTES];
          addr = addr + 1;
          nreads = nreads + 1;
          sd_o = outb[7:4];
        end else begin
          sd_o = outb[3:0];
        end
        sd_oe = 4'b1111;
      end else if (cmd == 8'h05 && nclk >= 8) begin
        obit  = (nclk - 8) % 8;
        sd_o  = {2'b00, sr[7 - obit], 1'b0};
        sd_oe = 4'b0010;
      end else if (cmd == 8'h9F && nclk >= 8) begin
        obit = (nclk - 8) % 8;
        case ((nclk - 8) / 8)
          0: outb = 8'hEF;
          1: outb = 8'h40;
          default: outb = 8'h18;
        endcase
        sd_o  = {2'b00, outb[7 - obit], 1'b0};
        sd_oe = 4'b0010;
      end else begin
        sd_oe = 4'h0;
      end
    end
  end
endmodule

// =============================================================================
// Dual simultaneous-sampling ADC (16 bit, MSB first, data changes on SCK fall)
// =============================================================================
module adc_model (
    input  wire        csn,
    input  wire        sck,
    input  wire [15:0] val_a,
    input  wire [15:0] val_b,
    output wire        sdoa,
    output wire        sdoa_oe,
    output wire        sdob
);
  reg [15:0] sa, sb;
  integer    nconv;
  initial begin sa = 0; sb = 0; nconv = 0; end
  always @(negedge csn) begin
    #1;
    sa = val_a;
    sb = val_b;
    nconv = nconv + 1;
  end
  always @(negedge sck) begin
    if (!csn) begin
      sa = {sa[14:0], 1'b0};
      sb = {sb[14:0], 1'b0};
    end
  end
  assign sdoa    = sa[15];
  assign sdoa_oe = !csn;
  assign sdob    = csn ? 1'b0 : sb[15];
endmodule

// =============================================================================
// SPI register-file slave (stand-in for a gyro / IMU): first byte = address
// with bit 7 = read, following bytes auto-increment.  WHO_AM_I (0x75) = 0x47.
// =============================================================================
module spireg_model (
    input  wire csn,
    input  wire sck,
    input  wire mosi,
    output wire miso,
    output wire miso_oe
);
  reg [7:0] regs [0:127];
  reg [7:0] sh, outb;
  reg [6:0] a;
  reg       rd;
  integer   n, i;
  initial begin
    for (i = 0; i < 128; i = i + 1) regs[i] = i[7:0] ^ 8'h5A;
    regs[7'h75] = 8'h47;
    outb = 8'h00;
  end
  always @(negedge csn) begin n = 0; outb = 8'h00; end
  always @(posedge sck) begin
    if (!csn) begin
      sh = {sh[6:0], mosi};
      n  = n + 1;
      if (n == 8) begin
        rd = sh[7];
        a  = sh[6:0];
      end else if (n > 8 && (n % 8) == 0) begin
        if (!rd) regs[a] = sh;
        a = a + 1;
      end
    end
  end
  always @(negedge sck) begin
    if (!csn) begin
      if (n >= 8 && (n % 8) == 0) outb = regs[a];
      else outb = {outb[6:0], 1'b0};
    end
  end
  assign miso    = outb[7];
  assign miso_oe = !csn;
endmodule
