"""Register map of tt_um_apr0t0_foc_mcu (single source of truth).

Run `python host/foc_regs.py` to regenerate firmware/foc_regs.h.
All peripheral registers are 32-bit, word aligned, at PERIPH_BASE + offset.
"""

FLASH_BASE = 0x0000_0000
RAM_BASE = 0x1000_0000
RAM_BYTES = 128
PERIPH_BASE = 0x2000_0000

ID_VALUE = 0x464F_4301  # "FOC" v1

# name: (offset, access, description)
REGS = {
    "ID":           (0x00, "RO", "identification 0x464F4301"),
    "CTRL":         (0x04, "RW", "control bits, see CTRL_*"),
    "STATUS":       (0x08, "R/W1C", "[5:0] latched faults (write 1 to clear), live status above"),
    "FAULT_MASK":   (0x0C, "RW", "[5:0] faults that stop the bridge (reset 0x17)"),
    "PWM_PERIOD":   (0x10, "RW", "[11:0] P: f_pwm = f_clk / (2 P) (reset 1250 = 20 kHz @ 50 MHz)"),
    "PWM_CFG":      (0x14, "RW", "[7:0] dead time clocks, [8] high-side invert, [9] low-side invert, [27:16] ADC trigger advance"),
    "DUTY_LIM":     (0x18, "RW", "[11:0] min duty, [27:16] max duty (bootstrap refresh)"),
    "DUTY_AB":      (0x1C, "RO", "active duty A [11:0] / B [27:16]"),
    "DUTY_C":       (0x20, "RO", "active duty C [11:0]"),
    "ADC_CFG":      (0x24, "RW", "[4:0] SCK cycles, [8:5] left shift, [12:9] SCK div, [13] CPOL, [14] late sample, [15] invert A, [16] invert B, [31:24] CS-to-SCK delay"),
    "ADC_OFFS":     (0x2C, "RW", "[15:0] offset A, [31:16] offset B"),
    "ADC_RAW":      (0x30, "RO", "[15:0] raw A, [31:16] raw B (only valid right after a conversion)"),
    "IREF":         (0x34, "RW", "[15:0] id_ref, [31:16] iq_ref (Q15 of ADC full scale)"),
    "VCMD":         (0x38, "RW", "[15:0] vd, [31:16] vq in voltage mode (Q15 of Vbus)"),
    "CUR_KP":       (0x3C, "RW", "[15:0] Kp, [19:16] shift: Kp_eff = Kp / 2^shift"),
    "CUR_KI":       (0x40, "RW", "[15:0] Ki, [19:16] shift: Ki per update = Ki / 2^(shift+16)"),
    "LIMITS":       (0x44, "RW", "[15:0] voltage limit (18918 = Vbus/sqrt3), [31:16] overcurrent limit"),
    "SPD_CFG":      (0x48, "RW", "[15:0] speed reference, [23:16] speed loop divider"),
    "SPD_KP":       (0x4C, "RW", "[15:0] Kp, [19:16] shift"),
    "SPD_KI":       (0x50, "RW", "[15:0] Ki, [19:16] shift"),
    "IQ_LIM":       (0x54, "RW", "[15:0] iq limit of the speed loop"),
    "ANGLE_CFG":    (0x58, "RW", "[15:0] angle offset (hall modes), [19:16] speed shift"),
    "OL_STEP":      (0x5C, "RW", "[23:0] signed open-loop angle step per clock (8.16)"),
    "FORCED_ANGLE": (0x60, "RW", "[15:0] angle in forced mode"),
    "HALL_CFG":     (0x64, "RW", "[23:0] sector of each hall code (3 bits x 8), [31:24] glitch filter clocks"),
    "I_AB":         (0x68, "RO", "[15:0] ia, [31:16] ib"),
    "I_DQ":         (0x6C, "RO", "[15:0] id, [31:16] iq"),
    "V_DQ":         (0x70, "RO", "[15:0] vd, [31:16] vq"),
    "ANG_SPD":      (0x74, "RO", "[15:0] electrical angle, [31:16] speed"),
    "HALL_PERIOD":  (0x78, "RO", "[23:0] clocks between the last two hall edges"),
    "FOC_STAT":     (0x7C, "RO", "[15:0] FOC update counter, [31:16] iq_ref from speed loop"),
    "SPI_CTRL":     (0x80, "RW", "[1:0] CS (0 flash, 1 ADC, 2 aux), [2] hold CS, [6:3] SCK div, [7] auto: writing SPI_BUF1 sends 8 bytes; R: [8] busy, [9] owns bus"),
    "SPI_BUF0":     (0x84, "RW", "transfer buffer bytes 0..3 (TX before, RX after)"),
    "SPI_BUF1":     (0x88, "RW", "transfer buffer bytes 4..7"),
    "SPI_XFER":     (0x8C, "RW", "W: [3:0] number of bytes (1..8) to transfer; R: [0] busy"),
    "FLASH_CFG":    (0x90, "RW", "[0] quad I/O read, [3:1] dummy clocks, [7:4] SCK div, [8] late sample"),
    "CPU_CTRL":     (0x94, "RW", "W: [0] run (0 = reset), [1] pause, [2] step one instruction; R: [0] run, [1] pause, [2] paused, [3] halted (trap), [7:4] trap cause (1 ecall 2 ebreak 3 illegal 4 misaligned), [8] breakpoint hit"),
    "CPU_PC":       (0x98, "RO", "program counter"),
    "MBOX_H2C":     (0x9C, "RW", "mailbox host -> CPU"),
    "MBOX_C2H":     (0xA0, "RW", "mailbox CPU -> host"),
    "TIME":         (0xA4, "RO", "free running clock counter"),
    "LINK_CFG":     (0xA8, "RW", "W: [15:0] baud divisor Q12.4 (locks) or [16]=1 re-run autobaud; R: [15:0] divisor, [16] locked"),
    "LINK_WDT":     (0xAC, "RW", "[15:0] host watchdog timeout in units of 1024 clocks"),
    "TEL_CFG":      (0xB0, "RW", "[0] telemetry on, [11:8] words, [23:16] divider (FOC updates)"),
    "TEL_ADDR":     (0xB4, "RW", "[7:2] first register streamed as telemetry (peripheral space)"),
    "LINK_STAT":    (0xB8, "RO", "[7:0] CRC errors, [15:8] good frames"),
    "CPU_BOOT":     (0xBC, "RW", "[31:2] boot address used when the CPU leaves reset (flash image, or RAM)"),
    "CPU_BKPT":     (0xC0, "RW", "[31:2] breakpoint address, [0] enable: the CPU pauses before that instruction"),
}

for _n, (_o, _a, _d) in REGS.items():
    globals()[_n] = PERIPH_BASE + _o

CPU_RUN = 1 << 0
CPU_PAUSE = 1 << 1
CPU_STEP = 1 << 2
CPU_PAUSED = 1 << 2
CPU_HALTED = 1 << 3
CPU_BKPT_HIT = 1 << 8

CTRL_ARM = 1 << 0
CTRL_DRV_EN = 1 << 1
CTRL_ADC_EN = 1 << 2
CTRL_CUR_LOOP = 1 << 4
CTRL_SPD_LOOP = 1 << 5
CTRL_AMODE_SHIFT = 6
AMODE_HALL = 0
AMODE_HALL_CENTER = 1
AMODE_OPEN_LOOP = 2
AMODE_FORCED = 3
CTRL_BRAKE = 1 << 8
CTRL_WDT_EN = 1 << 9
CTRL_SPI_SYNC = 1 << 10

FAULT_NFAULT = 1 << 0
FAULT_ESTOP = 1 << 1
FAULT_OVERCUR = 1 << 2
FAULT_HALL = 1 << 3
FAULT_WDT = 1 << 4
FAULT_ADC_DROP = 1 << 5

STATUS_FAULT_ACTIVE = 1 << 8
STATUS_OUT_EN = 1 << 9

VLIM_LINEAR = 18918  # Vbus / sqrt(3) in Q15 of Vbus


def s16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def pack16(lo, hi):
    return (lo & 0xFFFF) | ((hi & 0xFFFF) << 16)


def gen_c_header(path):
    lines = [
        "/* Generated by host/foc_regs.py - do not edit. */",
        "#ifndef FOC_REGS_H",
        "#define FOC_REGS_H",
        "#include <stdint.h>",
        "",
        f"#define FLASH_BASE  0x{FLASH_BASE:08X}u",
        f"#define RAM_BASE    0x{RAM_BASE:08X}u",
        f"#define PERIPH_BASE 0x{PERIPH_BASE:08X}u",
        "#define REG(off) (*(volatile uint32_t *)(PERIPH_BASE + (off)))",
        "",
    ]
    for n, (o, a, d) in REGS.items():
        lines.append(f"#define {n:<13s} REG(0x{o:02X})  /* {a}: {d} */")
    lines.append("")
    for n, v in globals().items():
        if n.startswith(("CTRL_", "AMODE_", "FAULT_", "STATUS_", "CPU_")) and isinstance(v, int) and n not in ("FAULT_MASK", "CPU_CTRL", "CPU_PC", "CPU_BOOT", "CPU_BKPT"):
            lines.append(f"#define {n:<20s} {v}u")
    lines += ["", "#endif", ""]
    with open(path, "w") as f:
        f.write("\n".join(lines))


def gen_markdown(path):
    lines = [
        "# Register map",
        "",
        "Generated from `host/foc_regs.py` - do not edit by hand.",
        "",
        f"Peripheral registers live at `{PERIPH_BASE:#010x}` (CPU and host see the same map).",
        "All registers are 32 bit and word aligned. The host link can also read the",
        f"flash (`{FLASH_BASE:#010x}`, execute-in-place window) and the {RAM_BYTES}-byte RAM (`{RAM_BASE:#010x}`).",
        "",
        "| Offset | Name | Access | Description |",
        "|-------:|------|--------|-------------|",
    ]
    for n, (o, a, d) in REGS.items():
        lines.append(f"| `0x{o:02X}` | `{n}` | {a} | {d} |")
    lines += [
        "",
        "## CTRL bits",
        "",
        "| Bit | Name | Meaning |",
        "|----:|------|---------|",
        "| 0 | ARM | gate outputs active (cleared automatically by any enabled fault) |",
        "| 1 | DRV_EN | drive the gate-driver enable pin (forced low while a fault is active) |",
        "| 2 | ADC_EN | sample currents every PWM period; the FOC update runs on each sample |",
        "| 4 | CUR_LOOP | current PI loops (else vd / vq come from `VCMD`) |",
        "| 5 | SPD_LOOP | speed PI loop produces iq_ref |",
        "| 7:6 | ANGLE_MODE | 0 hall + interpolation, 1 hall sector centre, 2 open loop (`OL_STEP`), 3 forced (`FORCED_ANGLE`) |",
        "| 8 | BRAKE | all low-side switches on |",
        "| 9 | WDT_EN | host-link watchdog |",
        "| 10 | SPI_SYNC | SPI port transfers may only start within 31 clocks after an ADC conversion (keep bursts shorter than the PWM period) |",
        "",
        "## Fault bits (STATUS[5:0], FAULT_MASK[5:0])",
        "",
        "| Bit | Fault | Source |",
        "|----:|-------|--------|",
        "| 0 | NFAULT | gate driver nFAULT pin low |",
        "| 1 | ESTOP | nESTOP pin low |",
        "| 2 | OVERCUR | |ia|, |ib| or |ia+ib| above `LIMITS[31:16]` |",
        "| 3 | HALL | invalid hall code (000 / 111) or a skipped sector |",
        "| 4 | WDT | no valid host frame within `LINK_WDT` |",
        "| 5 | ADC_DROP | a current sample was skipped because the SPI port held the bus |",
        "",
        "STATUS also shows: [8] fault active, [9] outputs enabled, [12:10] hall code, [15:13] sector,",
        "[16] direction, [17] moving, [18] CPU halted, [19] SPI port busy, [20] link locked,",
        "[21] FOC busy, [22] nFAULT pin, [23] nESTOP pin.",
        "",
    ]
    with open(path, "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    import os

    here = os.path.dirname(__file__)
    gen_c_header(os.path.join(here, "..", "firmware", "foc_regs.h"))
    gen_markdown(os.path.join(here, "..", "docs", "register_map.md"))
