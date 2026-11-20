/*
 * Copyright (c) 2026 aPR0T0
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny Tapeout wrapper: FOC BLDC motor controller + RV32E microcontroller.
 *
 *   ui[0..2] HALL_A/B/C      uo[0] PWM_AH   uio[0] FLASH_CS#  (QSPI Pmod CS0)
 *   ui[3]    UART_RX         uo[1] PWM_AL   uio[1] SD0 / MOSI
 *   ui[4]    nFAULT (driver) uo[2] PWM_BH   uio[2] SD1 / MISO / ADC_SDOA
 *   ui[5]    ADC_SDOB        uo[3] PWM_BL   uio[3] SCK
 *   ui[6]    nESTOP          uo[4] UART_TX  uio[4] SD2
 *   ui[7]    BOOT            uo[5] PWM_CH   uio[5] SD3
 *                            uo[6] PWM_CL   uio[6] ADC_CS#   (QSPI Pmod CS1)
 *                            uo[7] DRV_EN   uio[7] AUX_CS#   (QSPI Pmod CS2, gyro)
 *
 * Programming / control paths (all available at the same time):
 *   - UART on ui[3] / uo[4]: the pins the Tiny Tapeout demo board wires to its
 *     RP2350 hardware UART, so the board's USB port is a direct host link.
 *     The link is hardware: it works with a blank flash, a halted CPU or bad
 *     firmware, and can halt / single-step / breakpoint / restart the CPU.
 *   - BOOT (ui[7]) sampled at reset: 1 = run firmware, 0 = CPU held, host in charge.
 *   - While rst_n is low every uio pin is an input, so any external SPI programmer
 *     (the demo board RP2350, an FT232H, a CH341A) can write the flash directly.
 */
`default_nettype none

module tt_um_apr0t0_foc_mcu (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

  // reset synchronizer (async assert, sync release)
  reg [1:0] rst_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 2'b00;
    else        rst_sync <= {rst_sync[0], 1'b1};
  end
  wire rst = !rst_sync[1];

  wire [5:0] gate;
  wire       drv_en, txd, sck;
  wire [2:0] cs_n;
  wire [3:0] sd_o, sd_oe;

  foc_soc #(
      .RAM_WORDS(32),
      .RAM_AW(5)
  ) soc (
      .clk      (clk),
      .rst      (rst),
      .boot     (ui_in[7]),
      .hall_in  (ui_in[2:0]),
      .nfault_in(ui_in[4]),
      .nestop_in(ui_in[6]),
      .gate     (gate),
      .drv_en   (drv_en),
      .rxd      (ui_in[3]),
      .txd      (txd),
      .sck      (sck),
      .cs_n     (cs_n),
      .sd_o     (sd_o),
      .sd_oe    (sd_oe),
      .sd_i     ({uio_in[5], uio_in[4], uio_in[2], uio_in[1]}),
      .sdob_i   (ui_in[5])
  );

  // gate = {CL, CH, BL, BH, AL, AH}
  assign uo_out  = {drv_en, gate[5], gate[4], txd, gate[3:0]};
  assign uio_out = {cs_n[2], cs_n[1], sd_o[3], sd_o[2], sck, sd_o[1], sd_o[0], cs_n[0]};
  assign uio_oe  = rst ? 8'h00 : {1'b1, 1'b1, sd_oe[3], sd_oe[2], 1'b1, sd_oe[1], sd_oe[0], 1'b1};

  wire _unused = &{ena, uio_in[7], uio_in[6], uio_in[3], uio_in[0], 1'b0};

endmodule
