# Verification

## Test environment

`test/` is a cocotb testbench around the Tiny Tapeout top level. It drives and observes the
chip only through its pins, so the same suite runs against the gate-level netlist (`GATES=yes`).

| Model | File | Purpose |
|-------|------|---------|
| QSPI NOR flash | `tb.v: spiflash_model` | 03h read, EBh quad I/O read, RDSR, WREN, page program, 4K erase, JEDEC ID; loads the firmware image |
| Dual current ADC | `tb.v: adc_model` | simultaneous sampling, SDOA on SD1, SDOB on ui[5]; values come from the motor model |
| IMU stand-in | `tb.v: spireg_model` | SPI register file, WHO_AM_I (0x75) = 0x47 like the ICM-42688-P |
| Bus checker | `tb.v` | counts any clock where two devices drive the same SD line |
| Gate counters | `tb.v` | per-switch on-time and shoot-through counters |
| Host link | `harness.py: Link` | asynchronous bit-banged UART (independent of the chip clock) + `host/foc_protocol.py` |
| Motor | `harness.py: Motor` | surface PMSM in alpha/beta, R, L, flux, pole pairs, J, viscous load; phase voltages from the measured gate on-times including dead-time diode conduction; hall signals from the rotor angle |

## Running

```bash
source env.sh
make -C firmware          # firmware image + expected ISA self-test signature (host build)
cd test && make           # RTL
make GATES=yes            # gate level (needs test/gate_level_netlist.v from the GDS run and PDK_ROOT)
python -m pytest ../host  # host protocol unit tests
```

## Results (RTL, Icarus Verilog 13, cocotb 2.1)

`TESTS=11 PASS=11 FAIL=0` (on the GCP VM the 11 tests run in parallel in about 90 s).

| Test | Result | Key numbers |
|------|--------|-------------|
| `test_link` | PASS | autobaud at 5 Mbaud → divisor 160/16 clk/bit; corrupted frame NAKed and not executed; duplicate write suppressed; 7 telemetry frames, sequential numbering |
| `test_link_12mbaud` | PASS | 12 Mbaud locked at 4.125 clk/bit (50 MHz); 3 × 16-word burst reads; break → re-lock at 2 Mbaud; also passes with the clock at 10 / 23 / 31.25 / 40 MHz (baud scaled) |
| `test_pwm_deadtime_faults` | PASS | duties 127/72/72 vs float 127.5/72.5/72.5; on-time exact to the clock; every edge ≥ dead time; 0 shoot-through clocks; outputs off ≤ 6 clocks after nFAULT / nESTOP |
| `test_hall_estimator` | PASS | period 2000 clocks (exact), speed 22370 vs 22370.3; interpolated angle stays inside the sector; reverse direction |
| `test_foc_math` | PASS | id/iq error ≤ 5 LSB of 32768 at 4 angles; SVPWM duties within 1 count of the float model |
| `test_closed_loop_motor` | PASS | 24 V, 4 pole pairs: iq 4755 vs ref 4915 while accelerating (plant 2.89 A / 0.13 A d-axis), speed loop holds 1500 rpm within 3 %, overcurrent trip disarms the bridge |
| `test_flash` | PASS | JEDEC EF4018, erase + program over the link, single and quad XIP read-back identical, 0 contention clocks |
| `test_imu_port` | PASS | WHO_AM_I 0x47, register write/read burst, 25 bursts (2 and 8 bytes) at random points of the PWM period with ADC sampling running, no dropped samples |
| `test_watchdog` | PASS | host silence → bridge off, WDT fault latched |
| `test_cpu_firmware` | PASS | ISA self-test signature 0x0F770638 = host build; quad XIP enabled by firmware; mailbox echo; speed ramp; EBREAK halt (cause 2); restart while the ADC pre-empts flash fetches at 50 kHz (373 conversions, signature still correct) |
| `test_cpu_debug` | PASS | program uploaded to RAM over the link and booted from `CPU_BOOT` = RAM; pause stops it; 3 single steps = exactly one loop iteration; hardware breakpoint stops before the store, resume runs one more iteration to the breakpoint |

Host-side unit tests (`host/test_host.py`): 5 passed (CRC-16 reference value 0x29B1, stuffing round-trip,
corruption detection and resync, request layout, gain encoding).

## Bugs found by the testbench and fixed

1. The hall period counter was one clock short (speed 0.05 % high).
2. While held in reset, the CPU kept requesting instruction fetches. A flash read could then straddle a
   FLASH_CFG change and switch to quad mid-transaction, causing bus contention.
3. Flash mode is now latched per transaction and the stream is broken when the mode changes.
4. The line-break detector needed 5.2 ms of break. Now 2.6 ms.
5. At 12 Mbaud (about 4 clocks per bit) a single-character autobaud measurement has up to 3 % error
   from edge quantization, and the 3-sample majority vote reaches into neighbouring bits. Autobaud now
   averages four characters, and majority voting is only used at 8 or more clocks per bit.
   `test_link_12mbaud` passes at 10, 23, 31.25, 40 and 50 MHz clocks (different phase relations).
6. `SPI_SYNC` only guaranteed that a port transfer started *after* a conversion. A request that
   arrived late in the PWM period could still overlap the next current sample (found by
   `test_imu_port`). Transfers now have to start within a 31-clock window after a conversion.
   The test fires 25 bursts at pseudo-random points of the period.
7. Timing (slow corner): the CPU data address, the FOC integrator update and the bus grant are now
   registered, which removed the longest combinational paths.
8. Post-route STA showed the synchronous reset (one register driving about 4000 flops through a chain of
   delay buffers) as the slow-corner critical path. Each block now gets its own registered reset copy,
   which recovered about 3–4 ns at ss_100C_1v60.

## Gate level (final routed netlist)

`GATES=yes`, SKY130 functional cell models with 1 ns unit delays, test clock 10 MHz (all timing in the
tests scales with `CLK_HZ`). The tests ran in parallel on the VM. `test_closed_loop_motor` is RTL only
because of its run time.

| Test | Result | Wall time (s) |
|------|--------|--------------:|
| `test_link` | PASS | 91 |
| `test_link_12mbaud` | PASS | 217 |
| `test_pwm_deadtime_faults` | PASS | 60 |
| `test_hall_estimator` | PASS | 111 |
| `test_foc_math` | PASS | 101 |
| `test_flash` | PASS | 215 |
| `test_imu_port` | PASS | 506 |
| `test_watchdog` | PASS | 258 |
| `test_cpu_firmware` | PASS | 1197 |
| `test_cpu_debug` | PASS | 97 |
