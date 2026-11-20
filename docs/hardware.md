# External hardware: isolated 24–48 V power stage

## What is on the chip and what has to be on the board

The Tiny Tapeout die is a 3.3 V digital chip. Its pads are 3.3 V CMOS, and no SKY130 device survives
48 V (the highest-voltage devices are rated for about 10–20 V). A galvanic isolation barrier can't be
built in a Tiny Tapeout tile either, because isolators need thick-oxide capacitors, transformers or
optocouplers. So the work is split:

| Part of a gate driver | Where | What it does |
|-----------------------|-------|--------------|
| 6 gate-drive channels (AH AL BH BL CH CL): timing, dead time, shoot-through interlock, min-off-time for bootstrap refresh, polarity, brake, fault shutdown in ≤ 100 ns, enable | **on chip** (`pwm3`) | decides when each MOSFET may switch, and makes sure the two switches of a leg are never on together |
| Isolation barrier (keeps bus, gate and switching voltages away from the chip and the host PC) | **board**: digital isolators or isolated gate drivers | 3 kV to 5 kV class barrier, all bus-referenced circuits on the far side |
| Level shifting to 10–12 V gate drive, bootstrap for the high side, amps of gate current | **board**: gate driver ICs (6 channels = 3 half-bridge drivers) | turns the chip's 3.3 V logic decisions into gate charge |
| MOSFETs, shunts, bus capacitors, TVS | **board** | the 24–48 V power path |

## Isolated reference architecture

Everything referenced to the motor bus (`PGND`, 24–48 V, gates, phase nodes, shunts, hall sensors on
the motor) is on the **power side**. The chip, the flash, the IMU and the host USB are on the **logic
side** (`GND`). Only isolator channels cross the gap. No copper and no ground connection crosses it.

```
   LOGIC SIDE (3.3 V, GND)            ║ ISOLATION ║           POWER SIDE (PGND, 24..48 V)
                                       ║  BARRIER  ║
  tt chip                              ║           ║
  uo[0] PWM_AH  ──────┐                ║           ║   ┌──> INHA ┐
  uo[1] PWM_AL  ──────┤  6-channel     ║           ║   ├──> INLA │  3-phase gate driver      6 × 100 V
  uo[2] PWM_BH  ──────┤  digital       ║  ISO7760  ║   ├──> INHB │  (DRV8353RH: bootstrap,   N-MOSFETs
  uo[3] PWM_BL  ──────┤  isolator      ║  6 fwd    ║   ├──> INLB │   12 V gate drive, VDS   ──> motor
  uo[5] PWM_CH  ──────┤                ║           ║   ├──> INHC │   protection, CSAs)
  uo[6] PWM_CL  ──────┘                ║           ║   └──> INLC ┘
                                       ║           ║
  uo[7] DRV_EN  ─────── fwd ─┐         ║           ║   ┌──> ENABLE
  uio[6] ADC_CS# ────── fwd ─┤ ISO7763 ║ 3 fwd     ║   ├──> CS#   ┐ dual simultaneous-sampling
  uio[3] SCK    ─────── fwd ─┤         ║ 3 rev     ║   ├──> SCLK  │ SPI ADC (SOA, SOB from the
  uio[2] SD1    <────── rev ─┤         ║           ║   ├─── SDOA  │ driver's current-sense amps)
  ui[5]  ADC_SDOB <──── rev ─┤         ║           ║   ├─── SDOB  ┘
  ui[4]  nFAULT <────── rev ─┘         ║           ║   └─── nFAULT
                                       ║           ║
  ui[0..2] HALL_A/B/C <─ rev ─ ISO7730 (3 rev)     ║   <─── hall sensors (5 V, on the motor)
                                       ║           ║
  3.3 V logic supply (USB / LDO)       ║           ║   driver's buck: 3.3 V / 5 V for isolator side 2,
                                       ║           ║   ADC and halls (no power crosses the barrier)
```

Why the halls and the ADC are on the power side: hall sensors sit inside the motor and share its frame
and cable, and the shunts sit in the bridge's ground path. Putting both behind the barrier means no
signal from the motor reaches the chip without isolation.

What the chip settings need for this:

- Isolator delay (about 10–15 ns per direction) adds about 30 ns of round-trip delay on the ADC SPI.
  Set `ADC_CFG` late sampling (bit 14) and SCK div ≥ 3 (6.25 MHz at 50 MHz). The chip then samples
  SDOA/SDOB at the falling edge, giving ≥ 80 ns of margin.
- Use isolators whose outputs **default low** when their input side is unpowered. TI marks these with
  an F suffix, for example ISO7760F; check the datasheet. The gates then stay off if the logic side
  loses power. Add 10 kΩ pull-downs on the driver inputs on the power side.
- nFAULT on the power side is open drain: pull it up to the isolator's side-2 supply.

### Alternative: isolated gate drivers instead of digital isolators

3 dual-channel isolated half-bridge gate drivers (for example the TI UCC21520 / UCC21530 family, Silicon
Labs Si827x, ADI ADuM4221) take the six PWM signals directly on their isolated input side, so no
separate PWM isolator is needed. Each one gives you two isolated output channels: high side via
bootstrap, low side. Points to check:

- enable/disable pin polarity (`DRV_EN` is active high; invert with a gate, or use `PWM_CFG` polarity
  bits for active-low inputs)
- their own dead-time pin: set it to the minimum and let the chip's dead time govern
- they have no fault output, so add desaturation or shunt-comparator protection and feed it to nFAULT
  through an isolator channel
- you still need ADC and hall isolation as above (or isolated current amplifiers such as AMC1301 with
  the ADC on the logic side)

### Non-isolated option (24–48 V SELV, bench use)

For a bench setup below 60 V DC you can drop the barrier. A DRV8353 then connects directly. Its
internal level shifters keep the gate and bootstrap voltages off the logic pins. Use a single star
ground at the shunts, 100 Ω series resistors plus 10 kΩ pull-downs on all six PWM lines, and TVS/ESD
clamps on the hall and nFAULT inputs. Never let a 5 V hall output reach a chip input directly. This
option offers no protection against a failed MOSFET pulling the gate driver's logic side up, so use the
isolated version for anything beyond the bench.

## Bill of materials (starting point; check each datasheet)

| Function | Suggested part | Notes |
|----------|----------------|-------|
| PWM isolator (6 forward) | TI ISO7760F (default-low variant) | 6 channels, reinforced isolation |
| Control isolator (3 fwd / 3 rev) | TI ISO7763 | DRV_EN, ADC CS#, SCK forward; nFAULT, SDOA, SDOB back |
| Hall isolator (3 rev) | TI ISO7730 (mounted logic side = output side) | HALL A/B/C back |
| 3-phase gate driver | TI DRV8353RH (9–75 V, 100 V abs max, 6x PWM mode, 3 CSAs, VDS overcurrent, nFAULT, integrated buck) | Set MODE for 6x PWM. The chip supplies the dead time, so set the driver's own to its minimum. The buck powers the power-side logic |
| Alternative driver | 3× isolated dual-channel drivers (UCC21520 class) | see above |
| MOSFETs | 100 V N-channel, a few mΩ (e.g. TI CSD19532Q5B, Infineon BSC040N10NS5) | 100 V gives margin above 48 V plus regeneration spikes |
| Shunts | 1–2 mΩ, 2 W, Kelvin connected | Full scale `ifs` = (Vref/2) / (gain × R_shunt) |
| Current ADC | dual simultaneous-sampling 16-bit SPI ADC with two SDO pins (e.g. ADI AD7380 family) | Power side. Set `ADC_CFG` (frame length, CS→SCK delay, CPOL, alignment, late sample) |
| Flash | Winbond W25Q32/64/128 (or the Tiny Tapeout QSPI Pmod) | Logic side, CS0. Pull-up on CS# so it stays deselected during reset |
| Gyro / IMU (later) | TDK ICM-42688-P or Bosch BMI270 in SPI mode | Logic side, CS2, MISO on SD1. Use `CTRL.SPI_SYNC` |
| Host link | Tiny Tapeout demo board USB (RP2350 UART1 on ui[3]/uo[4]), or FT232H (12 Mbaud) | Logic side, so the host PC is isolated from the bus too |
| Bus protection | TVS (e.g. SMBJ58A for 48 V), ≥ 470 µF low-ESR bulk + ceramics at each half bridge | Brake chopper if the motor can regenerate hard |

Creepage and clearance across the barrier: follow the isolators' datasheet (several mm on the PCB, with a
slot under the barrier for reinforced isolation). Keep the barrier free of copper on every layer.

## Settings that depend on the power stage

- **Dead time**: `PWM_CFG[7:0]` in clocks (25 = 500 ns at 50 MHz). Use at least the MOSFET turn-off
  time plus the driver's and isolator's propagation skew (isolator channel-to-channel skew is a few ns).
- **Polarity**: `PWM_CFG[8]` / `[9]` invert the high / low side signals for drivers with active-low
  inputs. While the chip is in reset its outputs are low and `DRV_EN` is low, so the driver stays
  disabled.
- **Bootstrap**: keep `DUTY_LIM` max below `PWM_PERIOD` (for example 400 ns of off-time every period)
  so the high-side bootstrap capacitors recharge.
- **Current sampling**: the ADC trigger fires at the PWM counter peak, the centre of the low-side
  on-time. `PWM_CFG[27:16]` moves it earlier to compensate ADC acquisition plus isolator delay.
- **Faults**: wire the driver nFAULT (through the isolator) to ui[4] and an emergency-stop switch to
  ui[6] (both active low). Either one switches all six outputs off within 5 clocks (100 ns), clears ARM
  and pulls DRV_EN low.

## Programming and control paths

| Path | Pins | Works when |
|------|------|------------|
| Host link (UART, CRC-framed) | ui[3] RX, uo[4] TX: the Tiny Tapeout demo board's RP2350 UART1 (GPIO20 → ui[3], uo[4] → GPIO37) | always; it's hardware, so it works with a blank flash, a halted CPU or broken firmware |
| Boot strap | ui[7] BOOT, sampled at reset | 1 = run firmware from `CPU_BOOT` (default flash 0), 0 = CPU held, host in charge |
| Direct flash programming | uio[0..5] (QSPI Pmod pinout) | while rst_n is low every uio pin is an input, so the demo board RP2350, an FT232H or a CH341A can program the flash |
| In-system flash programming | host link → SPI port | any time; `focctl.py flash-write` halts the CPU, erases, programs and verifies |
| Run without flash | host link → RAM, `CPU_BOOT` = 0x1000_0000 | `focctl.py ram-run prog.bin` |
| Select a firmware image | `CPU_BOOT` = flash offset | A/B images, `focctl.py boot-addr 0x10000` |
| Debug | `CPU_CTRL` pause / step, `CPU_BKPT`, `CPU_PC`, register / RAM access | live, through the host link |

## Bring-up on the Tiny Tapeout demo board

1. Plug the QSPI Pmod into the uio header (its flash is CS0). On the custom board, the ADC goes on CS1
   and the IMU on CS2. Don't fit the Pmod PSRAMs on those chip selects at the same time.
2. Clock the project at 50 MHz from the RP2350, set `BOOT` (ui[7]) low and release reset.
3. Run `host/tt_demoboard_bridge.py` on the demo board (MicroPython). The board's USB port then carries
   the host link (RP2350 UART1: TX GPIO20 → ui[3], RX GPIO37 ← uo[4]). A separate USB-UART on ui[3] /
   uo[4] works too.
4. `host/focctl.py ping`, `flash-id`, then follow `docs/info.md` → How to test.
5. Before connecting the power stage: check dead time and polarity with a logic analyzer in forced-angle
   voltage mode (`ANGLE_MODE` = 3), then pull nFAULT / nESTOP low and check that all outputs go low.
6. Power-stage bring-up at low bus voltage (12 V, current-limited supply). Calibrate the ADC offsets with
   the bridge disarmed. Find the hall map and angle offset with the forced-angle sweep: rotate
   `FORCED_ANGLE` slowly with a small `VCMD.vd` and record the hall code at each angle.
7. Raise the current-loop bandwidth (`focctl.py gains`), then enable the speed loop.
