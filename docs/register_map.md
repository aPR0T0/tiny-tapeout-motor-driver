# Register map

Generated from `host/foc_regs.py` - do not edit by hand.

Peripheral registers live at `0x20000000` (CPU and host see the same map).
All registers are 32 bit and word aligned. The host link can also read the
flash (`0x00000000`, execute-in-place window) and the 128-byte RAM (`0x10000000`).

| Offset | Name | Access | Description |
|-------:|------|--------|-------------|
| `0x00` | `ID` | RO | identification 0x464F4301 |
| `0x04` | `CTRL` | RW | control bits, see CTRL_* |
| `0x08` | `STATUS` | R/W1C | [5:0] latched faults (write 1 to clear), live status above |
| `0x0C` | `FAULT_MASK` | RW | [5:0] faults that stop the bridge (reset 0x17) |
| `0x10` | `PWM_PERIOD` | RW | [11:0] P: f_pwm = f_clk / (2 P) (reset 1250 = 20 kHz @ 50 MHz) |
| `0x14` | `PWM_CFG` | RW | [7:0] dead time clocks, [8] high-side invert, [9] low-side invert, [27:16] ADC trigger advance |
| `0x18` | `DUTY_LIM` | RW | [11:0] min duty, [27:16] max duty (bootstrap refresh) |
| `0x1C` | `DUTY_AB` | RO | active duty A [11:0] / B [27:16] |
| `0x20` | `DUTY_C` | RO | active duty C [11:0] |
| `0x24` | `ADC_CFG` | RW | [4:0] SCK cycles, [8:5] left shift, [12:9] SCK div, [13] CPOL, [14] late sample, [15] invert A, [16] invert B, [31:24] CS-to-SCK delay |
| `0x2C` | `ADC_OFFS` | RW | [15:0] offset A, [31:16] offset B |
| `0x30` | `ADC_RAW` | RO | [15:0] raw A, [31:16] raw B (only valid right after a conversion) |
| `0x34` | `IREF` | RW | [15:0] id_ref, [31:16] iq_ref (Q15 of ADC full scale) |
| `0x38` | `VCMD` | RW | [15:0] vd, [31:16] vq in voltage mode (Q15 of Vbus) |
| `0x3C` | `CUR_KP` | RW | [15:0] Kp, [19:16] shift: Kp_eff = Kp / 2^shift |
| `0x40` | `CUR_KI` | RW | [15:0] Ki, [19:16] shift: Ki per update = Ki / 2^(shift+16) |
| `0x44` | `LIMITS` | RW | [15:0] voltage limit (18918 = Vbus/sqrt3), [31:16] overcurrent limit |
| `0x48` | `SPD_CFG` | RW | [15:0] speed reference, [23:16] speed loop divider |
| `0x4C` | `SPD_KP` | RW | [15:0] Kp, [19:16] shift |
| `0x50` | `SPD_KI` | RW | [15:0] Ki, [19:16] shift |
| `0x54` | `IQ_LIM` | RW | [15:0] iq limit of the speed loop |
| `0x58` | `ANGLE_CFG` | RW | [15:0] angle offset (hall modes), [19:16] speed shift |
| `0x5C` | `OL_STEP` | RW | [23:0] signed open-loop angle step per clock (8.16) |
| `0x60` | `FORCED_ANGLE` | RW | [15:0] angle in forced mode |
| `0x64` | `HALL_CFG` | RW | [23:0] sector of each hall code (3 bits x 8), [31:24] glitch filter clocks |
| `0x68` | `I_AB` | RO | [15:0] ia, [31:16] ib |
| `0x6C` | `I_DQ` | RO | [15:0] id, [31:16] iq |
| `0x70` | `V_DQ` | RO | [15:0] vd, [31:16] vq |
| `0x74` | `ANG_SPD` | RO | [15:0] electrical angle, [31:16] speed |
| `0x78` | `HALL_PERIOD` | RO | [23:0] clocks between the last two hall edges |
| `0x7C` | `FOC_STAT` | RO | [15:0] FOC update counter, [31:16] iq_ref from speed loop |
| `0x80` | `SPI_CTRL` | RW | [1:0] CS (0 flash, 1 ADC, 2 aux), [2] hold CS, [6:3] SCK div, [7] auto: writing SPI_BUF1 sends 8 bytes; R: [8] busy, [9] owns bus |
| `0x84` | `SPI_BUF0` | RW | transfer buffer bytes 0..3 (TX before, RX after) |
| `0x88` | `SPI_BUF1` | RW | transfer buffer bytes 4..7 |
| `0x8C` | `SPI_XFER` | RW | W: [3:0] number of bytes (1..8) to transfer; R: [0] busy |
| `0x90` | `FLASH_CFG` | RW | [0] quad I/O read, [3:1] dummy clocks, [7:4] SCK div, [8] late sample |
| `0x94` | `CPU_CTRL` | RW | W: [0] run (0 = reset), [1] pause, [2] step one instruction; R: [0] run, [1] pause, [2] paused, [3] halted (trap), [7:4] trap cause (1 ecall 2 ebreak 3 illegal 4 misaligned), [8] breakpoint hit |
| `0x98` | `CPU_PC` | RO | program counter |
| `0x9C` | `MBOX_H2C` | RW | mailbox host -> CPU |
| `0xA0` | `MBOX_C2H` | RW | mailbox CPU -> host |
| `0xA4` | `TIME` | RO | free running clock counter |
| `0xA8` | `LINK_CFG` | RW | W: [15:0] baud divisor Q12.4 (locks) or [16]=1 re-run autobaud; R: [15:0] divisor, [16] locked |
| `0xAC` | `LINK_WDT` | RW | [15:0] host watchdog timeout in units of 1024 clocks |
| `0xB0` | `TEL_CFG` | RW | [0] telemetry on, [11:8] words, [23:16] divider (FOC updates) |
| `0xB4` | `TEL_ADDR` | RW | [7:2] first register streamed as telemetry (peripheral space) |
| `0xB8` | `LINK_STAT` | RO | [7:0] CRC errors, [15:8] good frames |
| `0xBC` | `CPU_BOOT` | RW | [31:2] boot address used when the CPU leaves reset (flash image, or RAM) |
| `0xC0` | `CPU_BKPT` | RW | [31:2] breakpoint address, [0] enable: the CPU pauses before that instruction |

## CTRL bits

| Bit | Name | Meaning |
|----:|------|---------|
| 0 | ARM | gate outputs active (cleared automatically by any enabled fault) |
| 1 | DRV_EN | drive the gate-driver enable pin (forced low while a fault is active) |
| 2 | ADC_EN | sample currents every PWM period; the FOC update runs on each sample |
| 4 | CUR_LOOP | current PI loops (else vd / vq come from `VCMD`) |
| 5 | SPD_LOOP | speed PI loop produces iq_ref |
| 7:6 | ANGLE_MODE | 0 hall + interpolation, 1 hall sector centre, 2 open loop (`OL_STEP`), 3 forced (`FORCED_ANGLE`) |
| 8 | BRAKE | all low-side switches on |
| 9 | WDT_EN | host-link watchdog |
| 10 | SPI_SYNC | SPI port transfers may only start within 31 clocks after an ADC conversion (keep bursts shorter than the PWM period) |

## Fault bits (STATUS[5:0], FAULT_MASK[5:0])

| Bit | Fault | Source |
|----:|-------|--------|
| 0 | NFAULT | gate driver nFAULT pin low |
| 1 | ESTOP | nESTOP pin low |
| 2 | OVERCUR | |ia|, |ib| or |ia+ib| above `LIMITS[31:16]` |
| 3 | HALL | invalid hall code (000 / 111) or a skipped sector |
| 4 | WDT | no valid host frame within `LINK_WDT` |
| 5 | ADC_DROP | a current sample was skipped because the SPI port held the bus |

STATUS also shows: [8] fault active, [9] outputs enabled, [12:10] hall code, [15:13] sector,
[16] direction, [17] moving, [18] CPU halted, [19] SPI port busy, [20] link locked,
[21] FOC busy, [22] nFAULT pin, [23] nESTOP pin.
