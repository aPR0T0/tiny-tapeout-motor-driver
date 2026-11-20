<!---
This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

A field-oriented-control (FOC) motor controller for 3-phase BLDC / PMSM motors, plus a small RISC-V
microcontroller, in one Tiny Tapeout project. The chip produces the six gate signals for an
external 3-phase bridge (24–48 V in the reference design). It reads two phase currents from an SPI ADC
and the rotor position from three hall sensors, and closes the current and speed loops in hardware.

Blocks:

- **Gate-drive logic (`pwm3`)**: center-aligned 3-phase PWM (f = f_clk / 2P, 12-bit). Duties are
  double-buffered and latched at the counter valley. It inserts programmable dead time and has a
  shoot-through interlock on the registered outputs, output polarity per side (for inverted gate-driver
  inputs), brake mode, and a min/max duty clamp that keeps bootstrap capacitors charged. Outputs go off
  within 5 clocks (100 ns at 50 MHz, including the input synchronizer) of a fault pin going low.
- **Hall estimator (`hall_sensor`)**: synchronizer and glitch filter, a programmable code→sector map,
  direction detection, 24-bit edge-period measurement, a sequential divider for speed, and per-clock
  angle interpolation between edges (clamped to one sector).
- **FOC engine (`foc_engine`)**: offsets → Clarke → Park (CORDIC) → speed PI → d/q current PI with
  anti-windup → inverse Park (CORDIC) → inverse Clarke with min/max zero-sequence injection
  (equivalent to SVPWM). It uses one shared 16×16 sequential multiplier and one 16-iteration CORDIC, and
  takes about 330 clocks per update (6.6 µs at 50 MHz). Overcurrent detection runs on every sample.
- **Shared SPI bus (`spi_bus`)**: one QSPI bus with three chip selects:
  - **CS0** is the program flash: execute-in-place with 03h single or EBh quad I/O reads; sequential
    words are streamed.
  - **CS1** is a dual simultaneous-sampling current ADC, triggered by the PWM at the low-side
    conduction centre. It always pre-empts flash traffic, so the sampling instant is deterministic.
  - **CS2** is an auxiliary SPI device (gyro / IMU). It takes 8-byte atomic bursts that can be
    synchronized to finish between ADC samples.
- **Host link (`host_link`)**: a UART with automatic baud detection and 1/16-clock fractional timing,
  up to about f_clk/4 (12 Mbaud at 50 MHz). Frames are HDLC-style (0x7E flags, 0x7D escapes) with
  CRC-16/CCITT. Corrupted frames get a NAK, retried writes are suppressed by sequence number, a
  watchdog stops the motor if the host goes silent, and register telemetry can be streamed at the PWM
  rate.
- **CPU (`rv32e_core`)**: a multi-cycle RV32E. It executes from the QSPI flash, has 128 bytes of
  on-chip RAM and reaches every peripheral register. Running it is optional; the host can do
  everything over the link. ECALL, EBREAK and illegal instructions halt the core with a cause code.

All registers are described in `docs/register_map.md`.

## How to test

You need a 50 MHz clock and the Tiny Tapeout QSPI Pmod (flash on CS0). The host link is on ui[3] (RX)
and uo[4] (TX), which the demo board wires to its RP2350 UART1. Run `host/tt_demoboard_bridge.py` on
the board and use its USB port, or connect any 3.3 V USB-UART to those two pins.

1. Leave `BOOT` (ui[7]) low. Release reset.
2. Send `0x55` five or more times at any rate between 9600 baud and 12 Mbaud. The chip averages the
   bit time over four characters and locks.
3. Run `host/focctl.py -p /dev/ttyUSB0 -b 3000000 ping`. It must print ID `0x464f4301`.
4. `focctl.py flash-id` should show the flash JEDEC ID (`ef4018` for the W25Q128 on the QSPI Pmod).
5. Build the firmware (`make -C firmware`), then
   `focctl.py flash-write firmware/build/firmware.bin --verify` and `focctl.py cpu run`.
   `focctl.py read MBOX_C2H` must return the self-test signature from
   `firmware/build/selftest_expected.txt`.
6. Without a motor: write `PWM_PERIOD`, `VCMD` and `FORCED_ANGLE`, then set CTRL = ARM | ANGLE_MODE=3.
   The six PWM outputs show complementary, dead-time-separated center-aligned PWM. Pulling
   nFAULT (ui[4]) or nESTOP (ui[6]) low switches them off immediately.

With a power stage, ADC and hall sensors connected, `focctl.py setup --config motor.json` loads PWM,
ADC and PI settings. CTRL = ARM | DRV_EN | ADC_EN | CUR_LOOP then runs the current loop in hall mode.

## External hardware

- An isolated 3-phase power stage. The six PWM outputs go through a default-low digital isolator to
  a 3-phase gate driver with 6 inputs, enable and nFAULT (e.g. TI DRV8353RH). Alternatively use
  3 isolated dual-channel gate drivers. Add six 100 V N-channel MOSFETs and low-side shunts with
  current-sense amplifiers. Fault, hall and ADC signals return through isolator channels, so no bus
  voltage can reach the chip.
- A dual simultaneous-sampling 16-bit SPI ADC with separate SDOA / SDOB outputs, on CS1.
- Three hall sensors with pull-ups to 3.3 V and RC filters.
- The Tiny Tapeout QSPI Pmod or any W25Q-compatible flash on CS0.
- Optional: an SPI IMU / gyroscope (e.g. ICM-42688-P) on CS2.
- Host: the demo board's USB (RP2350 UART1), or a USB-UART that does high rates (FT232H: 12 Mbaud).

See `docs/hardware.md` in the repository for the reference design and wiring.
