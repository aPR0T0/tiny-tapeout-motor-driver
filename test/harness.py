"""cocotb harness: clock/reset, host link over a bit-banged UART, motor plant.

The protocol code is the same module the host PC tool uses (host/foc_protocol.py).
"""

import math
import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Event, FallingEdge, Timer, with_timeout

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "host"))
import foc_protocol as P  # noqa: E402
import foc_regs as R  # noqa: E402

# Gate-level simulation uses unit-delay cell models (1 ns per gate), so it runs
# at a slower clock; all test timings and baud rates scale with CLK_HZ.
GL = os.environ.get("GATES") == "yes"
CLK_HZ = int(float(os.environ.get("CLK_HZ", 10e6 if GL else 50e6)))
CLK_PS = round(1e12 / CLK_HZ)
F = 50e6 / CLK_HZ  # time scale factor relative to the nominal 50 MHz


def us(t):
    """A duration given for 50 MHz, scaled to the simulation clock (in us)."""
    return round(t * F, 6)   # whole picoseconds


def baud(b):
    """A baud rate given for 50 MHz, scaled to the simulation clock."""
    return round(b / F)


async def start(dut, boot=0, hall=0b101):
    """Start the clock, apply reset, return when the design is running."""
    cocotb.start_soon(Clock(dut.clk, CLK_PS, unit="ps").start())
    dut.ena.value = 1
    dut.hall.value = hall
    dut.uart_rx.value = 1
    dut.nfault.value = 1
    dut.nestop.value = 1
    dut.boot.value = boot
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)


# ----------------------------------------------------------------------------- link
class Link:
    """Host side of the link, talking to the DUT through its UART pins."""

    def __init__(self, dut, baud=5_000_000, tel_cb=None):
        self.dut = dut
        self.set_baud(baud)
        self.deframer = P.Deframer()
        self.responses = []
        self.telemetry = []
        self.bad_frames = 0
        self.seq = 1
        self.ev = Event()
        self.tel_cb = tel_cb
        self.corrupt_next = None  # (byte index, xor mask) for fault injection
        self.tx_bytes = 0
        cocotb.start_soon(self._rx_monitor())

    def set_baud(self, baud):
        self.baud = baud
        self.bit_ps = round(1e12 / baud)

    async def send(self, data):
        dut = self.dut
        for k, b in enumerate(data):
            if self.corrupt_next and self.corrupt_next[0] == k:
                b ^= self.corrupt_next[1]
            for bit in [0] + [(b >> i) & 1 for i in range(8)] + [1]:
                dut.uart_rx.value = bit
                await Timer(self.bit_ps, unit="ps")
        self.corrupt_next = None
        self.tx_bytes += len(data)

    async def send_break(self, us=6000):
        self.dut.uart_rx.value = 0
        await Timer(round(us * 1e6), unit="ps")
        self.dut.uart_rx.value = 1
        await Timer(20 * self.bit_ps, unit="ps")

    async def _rx_monitor(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.uart_tx)
            await Timer(self.bit_ps * 3 // 2, unit="ps")
            v = 0
            for i in range(8):
                v |= int(dut.uart_tx.value) << i
                await Timer(self.bit_ps, unit="ps")
            if int(dut.uart_tx.value) != 1:
                dut._log.warning("UART framing error from DUT")
            for payload, ok in self.deframer.feed(bytes([v])):
                if not ok or len(payload) < 2:
                    self.bad_frames += 1
                    continue
                seq, status, words = P.parse_response(payload)
                if status == P.ST_TELEMETRY:
                    self.telemetry.append((seq, words))
                    if self.tel_cb:
                        self.tel_cb(seq, words)
                else:
                    self.responses.append((seq, status, words))
                    self.ev.set()

    async def autobaud(self):
        await self.send(b"\x55" * 6)
        await Timer(20 * self.bit_ps, unit="ps")

    async def _wait_resp(self, seq, timeout_us):
        t_end = cocotb.utils.get_sim_time("ps") + round(timeout_us * 1e6)
        while True:
            for r in self.responses:
                if r[0] == seq:
                    self.responses.remove(r)
                    return r
            left = t_end - cocotb.utils.get_sim_time("ps")
            if left <= 0:
                return None
            self.ev.clear()
            try:
                await with_timeout(self.ev.wait(), int(left), "ps")
            except cocotb.triggers.SimTimeoutError:
                return None

    async def xact(self, cmd, addr=0, n=0, words=(), retries=4, seq=None):
        frame_bytes = len(P.request(0, cmd, addr, n, words))
        resp_bytes = 8 + 4 * (n if cmd in (P.CMD_READ, P.CMD_READ_FIFO) else 1)
        timeout_us = 50 + (frame_bytes + 2 * resp_bytes) * 12 * self.bit_ps / 1e6
        if seq is None:
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFF
        for _ in range(retries):
            self.responses.clear()
            await self.send(P.request(seq, cmd, addr, n, words))
            r = await self._wait_resp(seq, timeout_us)
            if r is None or r[1] == P.ST_CRC:
                continue
            return r
        raise P.LinkError(f"no response to cmd {cmd} addr {addr:#x}")

    async def ping(self):
        _, st, w = await self.xact(P.CMD_PING)
        assert st == P.ST_OK
        return w[0]

    async def read(self, addr, n=1):
        out = []
        while n > 0:
            k = min(n, P.MAX_READ_WORDS)
            _, st, w = await self.xact(P.CMD_READ, addr, k)
            assert st == P.ST_OK, f"read status {st}"
            out += w
            addr += 4 * k
            n -= k
        return out

    async def rd(self, addr):
        return (await self.read(addr, 1))[0]

    async def write(self, addr, words):
        words = list(words)
        while words:
            k = words[: P.MAX_WRITE_WORDS]
            _, st, _ = await self.xact(P.CMD_WRITE, addr, len(k), k)
            assert st == P.ST_OK, f"write status {st}"
            addr += 4 * len(k)
            words = words[len(k):]

    async def wr(self, addr, value):
        await self.write(addr, [value])


# ----------------------------------------------------------------------------- motor
class Motor:
    """Surface PMSM / sinusoidal BLDC in the alpha-beta frame + 3-phase bridge.

    Phase voltages come from the measured gate on-times of each PWM period
    (dead time conducts through the diode chosen by the current sign).
    """

    def __init__(self, R_=0.5, L=0.5e-3, pp=4, lam=0.01, J=2e-5, B=1e-5,
                 vbus=24.0, ifs=20.0, load=0.0, noise=0.0, seed=1):
        self.R, self.L, self.pp, self.lam, self.J, self.B = R_, L, pp, lam, J, B
        self.vbus, self.ifs, self.load, self.noise = vbus, ifs, load, noise
        self.ia_ = 0.0  # alpha
        self.ib_ = 0.0  # beta
        self.theta = 0.0  # electrical angle of the rotor flux, rad
        self.wm = 0.0  # mechanical speed rad/s
        self.t = 0.0
        self.rng = random.Random(seed)

    @property
    def we(self):
        return self.wm * self.pp

    def phase_currents(self):
        a = self.ia_
        b = -0.5 * self.ia_ + math.sqrt(3) / 2 * self.ib_
        c = -0.5 * self.ia_ - math.sqrt(3) / 2 * self.ib_
        return a, b, c

    def iq(self):
        return -self.ia_ * math.sin(self.theta) + self.ib_ * math.cos(self.theta)

    def id(self):
        return self.ia_ * math.cos(self.theta) + self.ib_ * math.sin(self.theta)

    def torque(self):
        return 1.5 * self.pp * self.lam * self.iq()

    def step(self, va, vb, vc, dt, nsub=16):
        valpha = (2.0 / 3.0) * (va - 0.5 * (vb + vc))
        vbeta = (vb - vc) / math.sqrt(3)
        h = dt / nsub
        for _ in range(nsub):
            we = self.we
            ea = -self.lam * we * math.sin(self.theta)
            eb = self.lam * we * math.cos(self.theta)
            self.ia_ += h * (valpha - self.R * self.ia_ - ea) / self.L
            self.ib_ += h * (vbeta - self.R * self.ib_ - eb) / self.L
            te = self.torque()
            self.wm += h * (te - self.B * self.wm - self.load * (1 if self.wm > 0 else -1 if self.wm < 0 else 0)) / self.J
            self.theta = (self.theta + h * self.we) % (2 * math.pi)
        self.t += dt

    def hall_code(self, theta=None):
        th = math.degrees(self.theta if theta is None else theta) % 360
        a = 1 if th < 180 else 0
        b = 1 if 120 <= th < 300 else 0
        c = 1 if (th >= 240 or th < 60) else 0
        return a | (b << 1) | (c << 2)

    def adc_code(self, i):
        i += self.rng.gauss(0, self.noise) if self.noise else 0.0
        v = 32768 - round(i / self.ifs * 32768)  # inverting current sense
        return max(0, min(65535, v))

    def q15_current(self, amps):
        return round(amps / self.ifs * 32768)


async def plant(dut, motor, log_every=0, trace=None):
    """Advance the motor once per ADC conversion using the measured gate times."""
    names = ["on_ah", "on_al", "on_bh", "on_bl", "on_ch", "on_cl"]
    last = None
    n = 0
    while True:
        await FallingEdge(dut.adc_csn)
        cyc = int(dut.cyc.value)
        cnt = [int(getattr(dut, x).value) for x in names]
        if last is not None:
            dc = cyc - last[0]
            if dc > 0:
                d = [cnt[k] - last[1][k] for k in range(6)]
                cur = motor.phase_currents()
                v = []
                all_off = True
                for ph in range(3):
                    h, lo = d[2 * ph], d[2 * ph + 1]
                    if h or lo:
                        all_off = False
                    dead = dc - h - lo
                    frac = (h + (dead if cur[ph] < 0 else 0)) / dc
                    v.append(frac * motor.vbus)
                if all_off and abs(motor.ia_) + abs(motor.ib_) < 0.05:
                    motor.ia_ = motor.ib_ = 0.0
                    motor.step(0, 0, 0, dc * CLK_PS * 1e-12)
                    motor.ia_ = motor.ib_ = 0.0
                else:
                    motor.step(v[0], v[1], v[2], dc * CLK_PS * 1e-12)
        last = (cyc, cnt)
        ia, ib, _ = motor.phase_currents()
        dut.adc_val_a.value = motor.adc_code(ia)
        dut.adc_val_b.value = motor.adc_code(ib)
        dut.hall.value = motor.hall_code()
        n += 1
        if trace is not None:
            trace.append((motor.t, motor.id(), motor.iq(), motor.wm, motor.theta))
        if log_every and n % log_every == 0:
            dut._log.info(f"t={motor.t*1e3:7.2f} ms  iq={motor.iq():6.2f} A  id={motor.id():6.2f} A  "
                          f"rpm={motor.wm*60/(2*math.pi):8.1f}")
