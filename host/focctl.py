#!/usr/bin/env python3
"""focctl - host PC tool for the tt_um_apr0t0_foc_mcu motor controller.

Talks to the chip over a USB-UART (FT232H / FT2232H / CP2102N / RP2040 bridge)
using the CRC-protected framed protocol.  Every request is retried on CRC errors
or timeouts; writes are de-duplicated by the chip, so retries are always safe.

Examples
  focctl.py -p /dev/ttyUSB0 -b 12000000 ping
  focctl.py -p /dev/ttyUSB0 dump
  focctl.py -p /dev/ttyUSB0 write CTRL 0
  focctl.py -p /dev/ttyUSB0 flash-write firmware/build/firmware.bin --verify
  focctl.py -p /dev/ttyUSB0 cpu run
  focctl.py -p /dev/ttyUSB0 cpu pause ; focctl.py cpu step 3 ; focctl.py bkpt 0x1a8 ; focctl.py cpu resume
  focctl.py -p /dev/ttyUSB0 boot-addr 0x10000 ; focctl.py cpu run        (second firmware image)
  focctl.py -p /dev/ttyACM0 ram-run test.bin                             (no flash needed)
  focctl.py -p /dev/ttyUSB0 setup --config motor.json
  focctl.py -p /dev/ttyUSB0 telemetry --regs I_DQ V_DQ ANG_SPD --div 9 --seconds 2 --csv log.csv
  focctl.py gains --R 0.5 --L 0.5e-3 --vbus 24 --ifs 20 --bw 1000 --fpwm 20000
"""

import argparse
import json
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foc_protocol as P  # noqa: E402
import foc_regs as R  # noqa: E402


class FocLink:
    def __init__(self, port, baud=3_000_000, timeout=0.05, retries=5, verbose=False):
        import serial  # pyserial

        self.ser = serial.Serial(port, baud, timeout=0)
        self.timeout = timeout
        self.retries = retries
        self.verbose = verbose
        self.seq = int(time.time()) & 0xFF
        self.deframer = P.Deframer()
        self.telemetry = []
        self.stats = {"retries": 0, "crc_naks": 0}

    # ---- low level ----
    def sync(self):
        """Break (re-arms autobaud), then 0x55 bytes to measure the baud rate."""
        self.ser.send_break(0.01)
        time.sleep(0.005)
        self.ser.write(b"\x55" * 8)
        self.ser.flush()
        time.sleep(0.002)
        self.ser.reset_input_buffer()

    def _poll(self, seq, deadline):
        while time.monotonic() < deadline:
            data = self.ser.read(4096)
            if not data:
                time.sleep(0.0002)
                continue
            for payload, ok in self.deframer.feed(data):
                if not ok or len(payload) < 2:
                    continue
                s, st, words = P.parse_response(payload)
                if st == P.ST_TELEMETRY:
                    self.telemetry.append((time.monotonic(), s, words))
                elif s == seq:
                    return st, words
        return None

    def xact(self, cmd, addr=0, n=0, words=()):
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFF
        frame = P.request(seq, cmd, addr, n, words)
        resp_len = 8 + 4 * n
        budget = self.timeout + 20 * (len(frame) + resp_len) / self.ser.baudrate
        for attempt in range(self.retries):
            self.ser.write(frame)
            r = self._poll(seq, time.monotonic() + budget)
            if r is None or r[0] == P.ST_CRC:
                self.stats["retries"] += 1
                if r is not None:
                    self.stats["crc_naks"] += 1
                if attempt == 2:
                    self.sync()
                continue
            st, w = r
            if st != P.ST_OK:
                raise P.LinkError(f"status {st:#x} for cmd {cmd} at {addr:#x}")
            return w
        raise P.LinkError(f"no response (cmd {cmd} addr {addr:#x})")

    # ---- register access ----
    def ping(self):
        return self.xact(P.CMD_PING)[0]

    def read(self, addr, n=1):
        out = []
        while n:
            k = min(n, P.MAX_READ_WORDS)
            out += self.xact(P.CMD_READ, addr, k)
            addr += 4 * k
            n -= k
        return out

    def rd(self, addr):
        return self.read(addr, 1)[0]

    def write(self, addr, words):
        words = list(words)
        while words:
            k = words[: P.MAX_WRITE_WORDS]
            self.xact(P.CMD_WRITE, addr, len(k), k)
            addr += 4 * len(k)
            words = words[len(k):]

    def wr(self, addr, v):
        self.write(addr, [v & 0xFFFFFFFF])

    # ---- SPI port / flash ----
    def spi(self, cs, tx, hold=False, div=1):
        tx = bytes(tx)
        assert 1 <= len(tx) <= 8
        b = tx + bytes(8 - len(tx))
        self.wr(R.SPI_CTRL, cs | (int(hold) << 2) | (div << 3))
        self.write(R.SPI_BUF0, [int.from_bytes(b[:4], "little"), int.from_bytes(b[4:], "little")])
        self.wr(R.SPI_XFER, len(tx))
        while self.rd(R.SPI_XFER) & 1:
            pass
        w = self.read(R.SPI_BUF0, 2)
        return (struct.pack("<II", *w))[: len(tx)]

    def flash_busy_wait(self, tmo=10.0):
        t_end = time.monotonic() + tmo
        while self.spi(0, [0x05, 0])[1] & 1:
            if time.monotonic() > t_end:
                raise P.LinkError("flash busy timeout")

    def flash_id(self):
        return self.spi(0, [0x9F, 0, 0, 0])[1:]

    def flash_erase_4k(self, addr):
        self.spi(0, [0x06])
        self.spi(0, [0x20, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF])
        self.flash_busy_wait()

    def flash_program_page(self, addr, data):
        assert len(data) <= 256 and (addr & 0xFF) + len(data) <= 256
        self.spi(0, [0x06])
        self.spi(0, [0x02, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF], hold=True)
        self.wr(R.SPI_CTRL, 0 | (1 << 2) | (1 << 3) | (1 << 7))  # hold + auto-send on BUF1 write
        data = bytes(data) + bytes((-len(data)) % 8)
        for k in range(0, len(data), 8):
            self.write(R.SPI_BUF0, list(struct.unpack("<II", data[k:k + 8])))
        self.wr(R.SPI_CTRL, 0)  # release CS -> the flash programs the page
        self.flash_busy_wait()

    def flash_read(self, addr, n):
        words = self.read(R.FLASH_BASE + (addr & ~3), (n + 3) // 4)
        return b"".join(struct.pack("<I", w) for w in words)[:n]

    def flash_write(self, addr, data, verify=True, progress=True):
        cpu = self.rd(R.CPU_CTRL)
        self.wr(R.CPU_CTRL, 0)  # the CPU must not execute from flash meanwhile
        try:
            end = addr + len(data)
            for sec in range(addr & ~0xFFF, end, 0x1000):
                self.flash_erase_4k(sec)
            pos = addr
            while pos < end:
                n = min(256 - (pos & 0xFF), end - pos)
                chunk = data[pos - addr: pos - addr + n]
                if chunk.strip(b"\xff"):
                    # pad to 8 bytes with 0xFF, which leaves the flash unchanged
                    self.flash_program_page(pos, chunk + b"\xff" * ((-len(chunk)) % 8))
                pos += n
                if progress:
                    print(f"\r  programmed {pos - addr}/{len(data)} bytes", end="", flush=True)
            if progress:
                print()
            if verify:
                got = self.flash_read(addr, len(data))
                if got != data:
                    bad = next(i for i in range(len(data)) if got[i] != data[i])
                    raise P.LinkError(f"verify failed at {addr + bad:#x}")
        finally:
            self.wr(R.CPU_CTRL, cpu & R.CPU_RUN)


# ----------------------------------------------------------------------------- tuning helpers
def current_pi_gains(R_, L, vbus, ifs, bw_hz, f_update):
    """PI gains in chip units for a current loop bandwidth bw_hz.

    Chip units: current Q15 of ifs amps, voltage Q15 of vbus volts.
    Kp_eff = wc*L*ifs/vbus ; Ki per update = wc*R*ifs/vbus / f_update.
    """
    wc = 2 * math.pi * bw_hz
    kp_eff = wc * L * ifs / vbus
    ki_eff = wc * R_ * ifs / vbus / f_update
    kp_sh = max(0, min(15, int(math.floor(math.log2(32767 / kp_eff))) if kp_eff > 0 else 0))
    kp = round(kp_eff * 2 ** kp_sh)
    ki_sh = 0
    ki = ki_eff * 65536
    while ki < 16384 and ki_sh < 15:
        ki *= 2
        ki_sh += 1
    return (min(kp, 65535), kp_sh), (min(round(ki), 65535), ki_sh)


def _encode(kp_eff, ki_eff):
    kp_sh = max(0, min(15, int(math.floor(math.log2(32767 / kp_eff))) if kp_eff > 0 else 0))
    kp = round(kp_eff * 2 ** kp_sh)
    ki_sh, ki = 0, ki_eff * 65536
    while ki < 16384 and ki_sh < 15:
        ki *= 2
        ki_sh += 1
    return (min(kp, 65535), kp_sh), (min(round(ki), 65535), ki_sh)


def speed_pi_gains(J, Kt, pp, ifs, bw_hz, f_update, clk_hz=50_000_000, speed_shift=4):
    """Speed PI gains in chip units (speed = hall step >> speed_shift, current Q15 of ifs).

    f_update = FOC rate / (speed divider + 1).  Zero of the PI at bw/2.
    """
    ws = 2 * math.pi * bw_hz
    su = pp * 2 ** 32 / (2 * math.pi * clk_hz * 2 ** speed_shift)  # speed units per rad/s
    tq = Kt * ifs / 32768                                         # Nm per iq unit
    kp_eff = J * ws / (tq * su)
    ki_eff = kp_eff * ws / 2 / f_update
    return _encode(kp_eff, ki_eff)


def pack_gain(g):
    return g[0] | (g[1] << 16)


def setup_from_config(link, cfg):
    """Load a motor configuration (dict, see docs/motor_config_example.json)."""
    clk = cfg.get("clk_hz", 50_000_000)
    fpwm = cfg.get("pwm_hz", 20_000)
    period = round(clk / (2 * fpwm))
    dead = round(cfg.get("dead_time_ns", 500) * clk / 1e9)
    link.wr(R.CTRL, 0)
    link.wr(R.PWM_PERIOD, period)
    link.wr(R.PWM_CFG, dead | (cfg.get("adc_advance", 0) << 16))
    margin = round(cfg.get("min_off_ns", 400) * clk / 1e9)
    link.wr(R.DUTY_LIM, margin | ((period - margin) << 16))
    adc = cfg.get("adc", {})
    link.wr(R.ADC_CFG, adc.get("bits", 16) | (adc.get("lshift", 0) << 5) | (adc.get("div", 1) << 9)
            | (adc.get("cpol", 0) << 13) | (adc.get("late", 0) << 14) | (adc.get("invert", 1) << 15)
            | (adc.get("invert", 1) << 16) | (adc.get("cs_delay", 0) << 24))
    m = cfg["motor"]
    kp, ki = current_pi_gains(m["R"], m["L"], cfg["vbus"], cfg["ifs"], cfg.get("current_bw_hz", 1000), fpwm)
    link.wr(R.CUR_KP, pack_gain(kp))
    link.wr(R.CUR_KI, pack_gain(ki))
    oc = min(32767, round(cfg.get("overcurrent_a", 0.9 * cfg["ifs"]) / cfg["ifs"] * 32768))
    link.wr(R.LIMITS, R.pack16(R.VLIM_LINEAR, oc))
    if "hall_map" in cfg:
        hm = 0
        for code, sector in cfg["hall_map"].items():
            hm |= (sector & 7) << (3 * int(code))
        link.wr(R.HALL_CFG, hm | (cfg.get("hall_filter", 50) << 24))
    link.wr(R.ANGLE_CFG, cfg.get("angle_offset", 0) | (cfg.get("speed_shift", 4) << 16))
    return kp, ki


# ----------------------------------------------------------------------------- CLI
def print_cpu(link):
    v = link.rd(R.CPU_CTRL)
    print(f"run={v & 1} pause={(v >> 1) & 1} paused={(v >> 2) & 1} halted={(v >> 3) & 1} "
          f"cause={(v >> 4) & 0xF} bkpt_hit={(v >> 8) & 1} pc={link.rd(R.CPU_PC):#010x} "
          f"boot={link.rd(R.CPU_BOOT):#010x}")


def reg_addr(name):
    name = name.upper()
    if name in R.REGS:
        return R.PERIPH_BASE + R.REGS[name][0]
    return int(name, 0)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default=os.environ.get("FOC_PORT", "/dev/ttyUSB0"))
    ap.add_argument("-b", "--baud", type=int, default=int(os.environ.get("FOC_BAUD", 3_000_000)))
    ap.add_argument("-v", "--verbose", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")
    sub.add_parser("dump")
    s = sub.add_parser("read"); s.add_argument("reg"); s.add_argument("n", nargs="?", type=int, default=1)
    s = sub.add_parser("write"); s.add_argument("reg"); s.add_argument("value", type=lambda x: int(x, 0))
    sub.add_parser("flash-id")
    s = sub.add_parser("flash-write"); s.add_argument("file"); s.add_argument("--addr", type=lambda x: int(x, 0), default=0)
    s.add_argument("--verify", action="store_true")
    s = sub.add_parser("flash-read"); s.add_argument("file"); s.add_argument("length", type=lambda x: int(x, 0))
    s.add_argument("--addr", type=lambda x: int(x, 0), default=0)
    s = sub.add_parser("cpu", help="run / halt / pause / resume / step / status")
    s.add_argument("action", choices=["run", "halt", "pause", "resume", "step", "status"])
    s.add_argument("n", nargs="?", type=int, default=1, help="number of steps")
    s = sub.add_parser("boot-addr", help="set the CPU boot address (flash image or RAM)")
    s.add_argument("addr", type=lambda x: int(x, 0))
    s = sub.add_parser("bkpt", help="set a hardware breakpoint (address) or 'off'")
    s.add_argument("addr")
    s = sub.add_parser("ram-run", help="upload a raw binary into on-chip RAM and boot it from there")
    s.add_argument("file")
    s = sub.add_parser("setup"); s.add_argument("--config", required=True)
    s = sub.add_parser("telemetry"); s.add_argument("--regs", nargs="+", default=["I_DQ", "V_DQ", "ANG_SPD"])
    s.add_argument("--div", type=int, default=9); s.add_argument("--seconds", type=float, default=1.0)
    s.add_argument("--csv")
    s = sub.add_parser("gains")
    for k in ("R", "L", "vbus", "ifs", "bw", "fpwm"):
        s.add_argument(f"--{k}", type=float, required=True)
    a = ap.parse_args(argv)

    if a.cmd == "gains":
        kp, ki = current_pi_gains(a.R, a.L, a.vbus, a.ifs, a.bw, a.fpwm)
        print(f"CUR_KP = {pack_gain(kp):#010x}  (Kp={kp[0]}, shift={kp[1]})")
        print(f"CUR_KI = {pack_gain(ki):#010x}  (Ki={ki[0]}, shift={ki[1]})")
        return 0

    link = FocLink(a.port, a.baud, verbose=a.verbose)
    link.sync()
    ident = link.ping()
    if ident != R.ID_VALUE:
        print(f"unexpected ID {ident:#x}", file=sys.stderr)
        return 1

    if a.cmd == "ping":
        cfg = link.rd(R.LINK_CFG)
        print(f"ID {ident:#010x}, baud divisor {(cfg & 0xFFFF) / 16:.3f} clk/bit")
    elif a.cmd == "dump":
        names = list(R.REGS)
        words = link.read(R.PERIPH_BASE, max(o for o, _, _ in R.REGS.values()) // 4 + 1)
        for n in names:
            off = R.REGS[n][0]
            print(f"{n:<13s} {off:#04x}  {words[off // 4]:#010x}   {R.REGS[n][2]}")
    elif a.cmd == "read":
        for i, w in enumerate(link.read(reg_addr(a.reg), a.n)):
            print(f"{reg_addr(a.reg) + 4 * i:#010x}: {w:#010x}")
    elif a.cmd == "write":
        link.wr(reg_addr(a.reg), a.value)
    elif a.cmd == "flash-id":
        print("JEDEC ID", link.flash_id().hex())
    elif a.cmd == "flash-write":
        data = open(a.file, "rb").read()
        t0 = time.time()
        link.flash_write(a.addr, data, verify=a.verify)
        print(f"wrote {len(data)} bytes in {time.time() - t0:.1f} s ({link.stats['retries']} retries)")
    elif a.cmd == "flash-read":
        open(a.file, "wb").write(link.flash_read(a.addr, a.length))
    elif a.cmd == "cpu":
        if a.action == "run":            # restart from the boot address
            link.wr(R.CPU_CTRL, 0)
            link.wr(R.CPU_CTRL, R.CPU_RUN)
        elif a.action == "halt":         # hold in reset
            link.wr(R.CPU_CTRL, 0)
        elif a.action == "pause":
            link.wr(R.CPU_CTRL, R.CPU_RUN | R.CPU_PAUSE)
        elif a.action == "resume":
            link.wr(R.CPU_CTRL, R.CPU_RUN)
        elif a.action == "step":
            for _ in range(a.n):
                link.wr(R.CPU_CTRL, R.CPU_RUN | R.CPU_PAUSE | R.CPU_STEP)
        print_cpu(link)
    elif a.cmd == "boot-addr":
        link.wr(R.CPU_BOOT, a.addr)
        print(f"boot address {a.addr:#010x} (takes effect at the next 'cpu run')")
    elif a.cmd == "bkpt":
        if a.addr == "off":
            link.wr(R.CPU_BKPT, 0)
        else:
            link.wr(R.CPU_BKPT, (int(a.addr, 0) & ~3) | 1)
        print_cpu(link)
    elif a.cmd == "ram-run":
        data = open(a.file, "rb").read()
        assert len(data) <= R.RAM_BYTES, f"program larger than {R.RAM_BYTES} bytes of RAM"
        data += bytes((-len(data)) % 4)
        link.wr(R.CPU_CTRL, 0)
        link.write(R.RAM_BASE, list(struct.unpack(f"<{len(data)//4}I", data)))
        link.wr(R.CPU_BOOT, R.RAM_BASE)
        link.wr(R.CPU_CTRL, R.CPU_RUN)
        print_cpu(link)
    elif a.cmd == "setup":
        kp, ki = setup_from_config(link, json.load(open(a.config)))
        print(f"loaded; Kp={kp} Ki={ki}")
    elif a.cmd == "telemetry":
        addrs = [reg_addr(r) for r in a.regs]
        base = min(addrs)
        n = (max(addrs) - base) // 4 + 1
        assert n <= 15, "telemetry window is at most 15 consecutive registers"
        link.wr(R.TEL_ADDR, base)
        link.wr(R.TEL_CFG, 1 | (n << 8) | (a.div << 16))
        t_end = time.monotonic() + a.seconds
        while time.monotonic() < t_end:
            link._poll(-1, time.monotonic() + 0.05)
        link.wr(R.TEL_CFG, 0)
        rows = link.telemetry
        print(f"{len(rows)} telemetry frames ({len(rows) / a.seconds:.0f}/s)")
        if a.csv:
            with open(a.csv, "w") as f:
                f.write("t,seq," + ",".join(a.regs) + "\n")
                for t, seq, words in rows:
                    f.write(f"{t:.6f},{seq}," + ",".join(f"{words[(ad - base) // 4]:#010x}" for ad in addrs) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
