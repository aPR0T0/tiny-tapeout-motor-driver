# SPDX-FileCopyrightText: © 2026 aPR0T0
# SPDX-License-Identifier: Apache-2.0
"""Verification of tt_um_apr0t0_foc_mcu.

Everything is driven through the chip pins only (UART host link, hall inputs,
SPI flash / ADC / IMU models), so the same tests run on the gate-level netlist.
"""

import math
import os
import random

import cocotb
from cocotb.triggers import ClockCycles, RisingEdge, Timer

from harness import CLK_HZ, GL, Link, Motor, P, R, baud, plant, start, us

# ----------------------------------------------------------------------------- helpers


def s16(v):
    return R.s16(v)


async def link_up(dut, rate=5_000_000, **kw):
    link = Link(dut, baud=baud(rate), **kw)
    await link.autobaud()
    ident = await link.ping()
    assert ident == R.ID_VALUE, f"bad ID {ident:#x}"
    return link


def ctrl(*bits, amode=0):
    v = 0
    for b in bits:
        v |= b
    return v | (amode << R.CTRL_AMODE_SHIFT)


def svpwm_ref(vd, vq, theta, period, dmin, dmax):
    """Floating point reference of the FOC output stage."""
    th = theta / 65536 * 2 * math.pi
    va_ = vd * math.cos(th) - vq * math.sin(th)
    vb_ = vd * math.sin(th) + vq * math.cos(th)
    a = va_
    b = -0.5 * va_ + math.sqrt(3) / 2 * vb_
    c = -0.5 * va_ - math.sqrt(3) / 2 * vb_
    mid = (max(a, b, c) + min(a, b, c)) / 2
    out = []
    for v in (a - mid, b - mid, c - mid):
        d = period / 2 + v * period / 32768
        out.append(min(max(d, dmin), min(dmax, period)))
    return out


# ============================================================================= link
@cocotb.test()
async def test_link(dut):
    """Autobaud, ping, read/write, CRC error NAK, duplicate suppression, bad length, telemetry."""
    await start(dut)
    link = await link_up(dut, rate=5_000_000)

    # locked divisor ~ 50e6/5e6 = 10 clocks -> Q12.4 = 160
    cfg = await link.rd(R.LINK_CFG)
    assert cfg & (1 << 16), "link not locked"
    assert abs((cfg & 0xFFFF) - 160) <= 2, f"divisor {cfg & 0xFFFF}"

    # register read/write (two words per frame)
    await link.write(R.IREF, [R.pack16(-1234, 2345), R.pack16(100, -200)])
    w = await link.read(R.IREF, 2)
    assert s16(w[0]) == -1234 and s16(w[0] >> 16) == 2345
    assert s16(w[1]) == 100 and s16(w[1] >> 16) == -200

    # escaping: values containing 0x7E / 0x7D bytes
    await link.wr(R.MBOX_H2C, 0x7E7D7E7D)
    assert await link.rd(R.MBOX_H2C) == 0x7E7D7E7D

    # CRC error -> NAK with status 1 (no retry here)
    seq = 0x42
    link.corrupt_next = (5, 0x10)
    await link.send(P.request(seq, P.CMD_WRITE, R.MBOX_H2C, 1, [0x11111111]))
    r = await link._wait_resp(seq, us(200))
    assert r is not None and r[1] == P.ST_CRC, f"expected NAK, got {r}"
    assert await link.rd(R.MBOX_H2C) == 0x7E7D7E7D, "corrupted frame was executed"
    stat = await link.rd(R.LINK_STAT)
    assert stat & 0xFF == 1, f"crc error count {stat & 0xFF}"

    # duplicate write (same SEQ) must not execute twice
    seq = 0x55
    _, st, _ = await link.xact(P.CMD_WRITE, R.MBOX_H2C, 1, [0xAAAA0001], seq=seq)
    assert st == P.ST_OK
    await link.wr(R.MBOX_H2C, 0xBBBB0002)        # different seq
    _, st, _ = await link.xact(P.CMD_WRITE, R.MBOX_H2C, 1, [0xAAAA0001], seq=0x56)
    assert st == P.ST_OK
    _, st, _ = await link.xact(P.CMD_WRITE, R.MBOX_H2C, 1, [0xCCCC0003], seq=0x56)  # retry
    assert st == P.ST_OK
    assert await link.rd(R.MBOX_H2C) == 0xAAAA0001, "duplicate write executed"

    # bad length / bad command
    _, st, _ = await link.xact(P.CMD_WRITE, R.MBOX_H2C, 3, [1, 2, 3])
    assert st == P.ST_BAD_LEN
    _, st, _ = await link.xact(0x33, R.MBOX_H2C, 1)
    assert st == P.ST_BAD_CMD

    # FIFO read of the same address
    _, st, w = await link.xact(P.CMD_READ_FIFO, R.ID, 3)
    assert st == P.ST_OK and w == [R.ID_VALUE] * 3

    # telemetry: stream ID + CTRL every 3 PWM periods (ADC off -> PWM based)
    await link.wr(R.PWM_PERIOD, 200)
    await link.wr(R.TEL_ADDR, R.ID)
    await link.wr(R.TEL_CFG, 1 | (2 << 8) | (2 << 16))
    await Timer(us(150), unit="us")
    await link.wr(R.TEL_CFG, 0)
    await Timer(us(20), unit="us")
    tel = list(link.telemetry)
    dut._log.info(f"telemetry frames: {len(tel)}")
    assert len(tel) >= 5
    for k, (seq, words) in enumerate(tel):
        assert words[0] == R.ID_VALUE
        if k:
            assert seq == (tel[k - 1][0] + 1) & 0xFF
    # the link still works after telemetry and a reads interleaved with it
    assert await link.ping() == R.ID_VALUE
    assert link.bad_frames == 0


@cocotb.test()
async def test_link_12mbaud(dut):
    """12 Mbaud (about 4.2 clocks per bit at 50 MHz), burst read, break + re-autobaud."""
    await start(dut)
    link = await link_up(dut, rate=12_000_000)
    cfg = await link.rd(R.LINK_CFG)
    dut._log.info(f"12 Mbaud divisor Q12.4 = {cfg & 0xFFFF} ({(cfg & 0xFFFF)/16:.3f} clk/bit)")
    for _ in range(3):
        w = await link.read(R.ID, 16)
        assert w[0] == R.ID_VALUE
    # break -> unlock -> autobaud at a different rate
    await link.send_break(us(3000))
    link.set_baud(baud(2_000_000))
    await link.autobaud()
    assert await link.ping() == R.ID_VALUE
    cfg = await link.rd(R.LINK_CFG)
    assert abs((cfg & 0xFFFF) - 400) <= 3


# ============================================================================= PWM
@cocotb.test()
async def test_pwm_deadtime_faults(dut):
    """Center aligned PWM, dead time, no shoot-through, fault shutdown, brake."""
    await start(dut)
    link = await link_up(dut)
    period, dead = 200, 12
    await link.wr(R.PWM_PERIOD, period)
    await link.wr(R.PWM_CFG, dead)
    await link.wr(R.DUTY_LIM, 0 | (period << 16))
    # voltage mode, forced angle 0, vd only -> va = vd, vb = vc = -vd/2
    vd = 6000
    await link.wr(R.VCMD, R.pack16(vd, 0))
    await link.wr(R.FORCED_ANGLE, 0)
    await link.wr(R.CTRL, ctrl(R.CTRL_ARM, R.CTRL_DRV_EN, amode=R.AMODE_FORCED))
    await Timer(us(30), unit="us")
    w = await link.read(R.DUTY_AB, 2)
    da, db, dc = w[0] & 0xFFF, (w[0] >> 16) & 0xFFF, w[1] & 0xFFF
    ref = svpwm_ref(vd, 0, 0, period, 0, period)
    dut._log.info(f"duties {da} {db} {dc}  ref {[round(x, 1) for x in ref]}")
    for got, exp in zip((da, db, dc), ref):
        assert abs(got - exp) <= 2

    # measure pins over two PWM periods
    st0 = int(dut.shoot_through.value)
    hist = []
    for _ in range(4 * period):
        await RisingEdge(dut.clk)
        hist.append(int(dut.gates.value))
        assert int(dut.drv_en.value) == 1, "DRV_EN low"
    assert int(dut.shoot_through.value) == st0
    for ph in range(3):
        hi = [(h >> (2 * ph)) & 1 for h in hist]
        lo = [(h >> (2 * ph + 1)) & 1 for h in hist]
        on_h = sum(hi)
        # per period: cnt < d for 2d-1 clocks, minus one dead time at turn-on
        exp = 2 * (2 * [da, db, dc][ph] - 1 - dead)
        assert abs(on_h - exp) <= 2, f"phase {ph} high time {on_h} vs {exp}"
        # dead time: every transition has >= dead clocks with both off
        k = 0
        while k < len(hist) - 1:
            if (hi[k] and not hi[k + 1]) or (lo[k] and not lo[k + 1]):
                gap = 0
                j = k + 1
                while j < len(hist) and not hi[j] and not lo[j]:
                    gap += 1
                    j += 1
                if j < len(hist):
                    assert gap >= dead, f"phase {ph}: dead time {gap} < {dead}"
                k = j
            else:
                k += 1

    # gate-driver fault: outputs off within a few clocks, ARM cleared, DRV_EN low
    dut.nfault.value = 0
    await ClockCycles(dut.clk, 6)
    assert int(dut.gates.value) == 0 and int(dut.drv_en.value) == 0, f"outputs not off: {int(dut.uo_out.value):#x}"
    dut.nfault.value = 1
    st = await link.rd(R.STATUS)
    assert st & R.FAULT_NFAULT and st & R.STATUS_FAULT_ACTIVE
    assert (await link.rd(R.CTRL)) & R.CTRL_ARM == 0
    await link.wr(R.STATUS, 0x3F)
    assert (await link.rd(R.STATUS)) & 0x3F == 0

    # emergency stop input
    await link.wr(R.CTRL, ctrl(R.CTRL_ARM, R.CTRL_DRV_EN, amode=R.AMODE_FORCED))
    await Timer(us(5), unit="us")
    dut.nestop.value = 0
    await ClockCycles(dut.clk, 6)
    assert int(dut.gates.value) == 0 and int(dut.drv_en.value) == 0
    dut.nestop.value = 1
    await link.wr(R.STATUS, 0x3F)

    # brake: all low sides on
    await link.wr(R.CTRL, ctrl(R.CTRL_ARM, R.CTRL_BRAKE, amode=R.AMODE_FORCED))
    await Timer(us(5), unit="us")
    assert int(dut.gates.value) == 0b101010
    await link.wr(R.CTRL, 0)
    await ClockCycles(dut.clk, 4)
    assert int(dut.gates.value) == 0


# ============================================================================= hall
@cocotb.test()
async def test_hall_estimator(dut):
    """Hall sector decoding, direction, speed and angle interpolation."""
    await start(dut)
    link = await link_up(dut)
    motor = Motor()
    sector_us = us(40.0)                   # 60 electrical degrees every 40 us (at 50 MHz)
    # forward rotation
    th = 1.0  # deg
    readings = []
    for k in range(14):
        dut.hall.value = motor.hall_code(math.radians(th + 60 * k))
        await Timer(sector_us, unit="us")
    w = await link.read(R.ANG_SPD, 1)
    per = (await link.rd(R.HALL_PERIOD)) & 0xFFFFFF
    exp_per = sector_us * 1e-6 * CLK_HZ
    assert abs(per - exp_per) <= 4, f"period {per} vs {exp_per}"
    spd = s16(w[0] >> 16)
    exp_step = 10923 * 65536 / exp_per
    exp_spd = exp_step / 16  # default speed shift 4
    dut._log.info(f"speed {spd} expected {exp_spd:.1f}")
    assert abs(spd - exp_spd) <= 2
    st = await link.rd(R.STATUS)
    assert (st >> 16) & 1 == 1, "direction should be forward"
    assert (st >> 17) & 1 == 1, "should be moving"
    # interpolation: sample the angle inside one sector
    dut.hall.value = motor.hall_code(math.radians(th + 60 * 14))
    t_edge = cocotb.utils.get_sim_time("us")
    angs = []
    for _ in range(3):
        await Timer(us(8), unit="us")
        a = (await link.rd(R.ANG_SPD)) & 0xFFFF
        angs.append((cocotb.utils.get_sim_time("us") - t_edge, a))
    base = round(((14 % 6) * 65536) / 6) % 65536
    for t, a in angs:
        d = (a - base) % 65536
        assert 0 < d <= 10923, f"angle {a} not inside the sector starting at {base}"
    assert angs[0][1] != angs[-1][1] and ((angs[-1][1] - angs[0][1]) % 65536) < 10923

    # reverse rotation
    for k in range(12):
        dut.hall.value = motor.hall_code(math.radians(-1.0 - 60 * k))
        await Timer(sector_us, unit="us")
    st = await link.rd(R.STATUS)
    assert (st >> 16) & 1 == 0, "direction should be reverse"
    spd = s16((await link.rd(R.ANG_SPD)) >> 16)
    assert spd < 0 and abs(spd + exp_spd) <= 2
    # invalid code -> hall fault latched (not masked by default)
    dut.hall.value = 0
    await Timer(us(5), unit="us")
    st = await link.rd(R.STATUS)
    assert st & R.FAULT_HALL
    assert (st & R.STATUS_FAULT_ACTIVE) == 0


# ============================================================================= FOC math
@cocotb.test()
async def test_foc_math(dut):
    """Clarke/Park of measured currents and the SVPWM output vs a float model."""
    await start(dut)
    link = await link_up(dut)
    period = 500
    await link.wr(R.PWM_PERIOD, period)
    await link.wr(R.DUTY_LIM, 0 | (period << 16))
    await link.wr(R.ADC_CFG, 16 | (1 << 9) | (1 << 15) | (1 << 16))   # 16 clocks, div 1, inverted
    errs = []
    for theta_deg, amp, phi in [(0, 8000, 90), (37, 12000, 10), (200, 5000, -120), (300, 15000, 180)]:
        theta = round(theta_deg / 360 * 65536) % 65536
        # currents of amplitude amp at angle (theta + phi) -> id = amp cos phi, iq = amp sin phi
        ang = math.radians(theta_deg + phi)
        ia = amp * math.cos(ang)
        ib = amp * math.cos(ang - 2 * math.pi / 3)
        dut.adc_val_a.value = 32768 - round(ia)
        dut.adc_val_b.value = 32768 - round(ib)
        vd, vq = 3000, -7000
        await link.wr(R.FORCED_ANGLE, theta)
        await link.wr(R.VCMD, R.pack16(vd, vq))
        await link.wr(R.CTRL, ctrl(R.CTRL_ADC_EN, amode=R.AMODE_FORCED))
        await Timer(us(40), unit="us")
        w = await link.read(R.I_AB, 2)
        gid, giq = s16(w[1]), s16(w[1] >> 16)
        eid, eiq = amp * math.cos(math.radians(phi)), amp * math.sin(math.radians(phi))
        dut._log.info(f"theta {theta_deg:3d}: id {gid:6d} ({eid:8.1f})  iq {giq:6d} ({eiq:8.1f})")
        errs += [abs(gid - eid), abs(giq - eiq)]
        w = await link.read(R.DUTY_AB, 2)
        duties = [w[0] & 0xFFF, (w[0] >> 16) & 0xFFF, w[1] & 0xFFF]
        ref = svpwm_ref(vd, vq, theta, period, 0, period)
        dut._log.info(f"           duties {duties}  ref {[round(x, 1) for x in ref]}")
        for g, e in zip(duties, ref):
            assert abs(g - e) <= 1.5
    assert max(errs) < 12, f"dq error {max(errs)}"


# ============================================================================= closed loop
async def motor_setup(link, period=500, kp=(10723, 12), ki=(3431, 0)):
    await link.wr(R.PWM_PERIOD, period)
    await link.wr(R.PWM_CFG, 20)
    await link.wr(R.DUTY_LIM, 10 | ((period - 10) << 16))
    await link.wr(R.ADC_CFG, 16 | (1 << 9) | (1 << 15) | (1 << 16))
    await link.wr(R.ADC_OFFS, R.pack16(0x8000, 0x8000))
    await link.wr(R.CUR_KP, kp[0] | (kp[1] << 16))
    await link.wr(R.CUR_KI, ki[0] | (ki[1] << 16))
    await link.wr(R.LIMITS, R.pack16(R.VLIM_LINEAR, 30000))


@cocotb.test(skip=GL)
async def test_closed_loop_motor(dut):
    """Current loop + hall commutation spin a simulated 24 V BLDC; speed loop; overcurrent trip."""
    await start(dut)
    link = await link_up(dut)
    # 24 V, 4 pole pairs, Kt = 0.06 Nm/A, viscous load -> ~2900 rpm at 3 A
    motor = Motor(vbus=24.0, ifs=20.0, J=1e-5, B=6e-4)
    trace = []
    cocotb.start_soon(plant(dut, motor, log_every=250, trace=trace))
    await motor_setup(link)
    iq_a = 3.0
    iq_ref = motor.q15_current(iq_a)
    await link.wr(R.IREF, R.pack16(0, iq_ref))
    await link.wr(R.CTRL, ctrl(R.CTRL_ARM, R.CTRL_DRV_EN, R.CTRL_ADC_EN, R.CTRL_CUR_LOOP,
                               amode=R.AMODE_HALL))
    await Timer(us(15 * 1000), unit="us")
    w = await link.read(R.I_DQ, 1)
    gid, giq = s16(w[0]), s16(w[0] >> 16)
    rpm = motor.wm * 60 / 2 / math.pi
    # plant currents averaged over the last 2 ms
    tail = [x for x in trace if x[0] > motor.t - 2e-3]
    pid = sum(x[1] for x in tail) / len(tail)
    piq = sum(x[2] for x in tail) / len(tail)
    dut._log.info(f"current loop: chip id={gid} iq={giq} (ref {iq_ref}); plant id={pid:.2f} A "
                  f"iq={piq:.2f} A; {rpm:.0f} rpm")
    assert abs(giq - iq_ref) < 0.05 * iq_ref, "q current loop not tracking"
    assert abs(gid) < 0.05 * iq_ref, "d current loop not tracking"
    assert abs(piq - iq_a) < 0.3, "plant q current wrong"
    assert abs(pid) < 0.5, "plant d current too large (commutation angle error)"
    assert rpm > 1000, "motor did not accelerate"

    # speed loop: hold a target speed (speed units = hall step >> 4)
    rpm_target = 1500
    we = rpm_target / 60 * 2 * math.pi * motor.pp
    step = we / (2 * math.pi) * 65536 / CLK_HZ * 65536
    spd_ref = round(step / 16)
    # ~50 Hz speed loop: Kp = J*ws/(Kt_unit*speed_unit) ~ 25, Ki/update = Kp*ws/2/10 kHz ~ 0.39
    await link.wr(R.SPD_KP, 25600 | (10 << 16))
    await link.wr(R.SPD_KI, 25850 | (0 << 16))
    await link.wr(R.IQ_LIM, motor.q15_current(6.0))
    await link.wr(R.SPD_CFG, spd_ref | (4 << 16))
    await link.wr(R.CTRL, ctrl(R.CTRL_ARM, R.CTRL_DRV_EN, R.CTRL_ADC_EN, R.CTRL_CUR_LOOP,
                               R.CTRL_SPD_LOOP, amode=R.AMODE_HALL))
    await Timer(us(30 * 1000), unit="us")
    rpm = motor.wm * 60 / (2 * math.pi)
    w = await link.read(R.ANG_SPD, 1)
    spd = s16(w[0] >> 16)
    iqr = s16((await link.rd(R.FOC_STAT)) >> 16)
    dut._log.info(f"speed loop: plant {rpm:.0f} rpm (target {rpm_target}), chip speed {spd} "
                  f"(ref {spd_ref}), iq_ref from speed PI {iqr}")
    assert abs(rpm - rpm_target) < 0.03 * rpm_target
    st = await link.rd(R.STATUS)
    assert st & 0x3F & ~R.FAULT_HALL == 0, f"faults {st & 0x3F:#x}"
    assert int(dut.shoot_through.value) == 0
    # overcurrent protection: lower the limit below the running current
    await link.wr(R.LIMITS, R.pack16(R.VLIM_LINEAR, 200))
    await Timer(us(100), unit="us")
    st = await link.rd(R.STATUS)
    assert st & R.FAULT_OVERCUR and (await link.rd(R.CTRL)) & R.CTRL_ARM == 0
    assert int(dut.gates.value) == 0


# ============================================================================= flash
async def spi_xfer(link, cs, tx, hold=False):
    """Send up to 8 bytes through the SPI port, return the received bytes."""
    data = bytes(tx) + bytes(8 - len(tx))
    await link.wr(R.SPI_CTRL, cs | (int(hold) << 2) | (1 << 3))
    await link.write(R.SPI_BUF0, [int.from_bytes(data[0:4], "little"), int.from_bytes(data[4:8], "little")])
    await link.wr(R.SPI_XFER, len(tx))
    while (await link.rd(R.SPI_XFER)) & 1:
        pass
    w = await link.read(R.SPI_BUF0, 2)
    return (w[0].to_bytes(4, "little") + w[1].to_bytes(4, "little"))[: len(tx)]


async def flash_wait(link):
    while True:
        r = await spi_xfer(link, 0, [0x05, 0])
        if not r[1] & 1:
            return


async def flash_program(link, addr, data):
    for page in range(0, len(data), 256):
        await spi_xfer(link, 0, [0x06])
        a = addr + page
        await spi_xfer(link, 0, [0x02, (a >> 16) & 0xFF, (a >> 8) & 0xFF, a & 0xFF], hold=True)
        chunk = data[page:page + 256]
        await link.wr(R.SPI_CTRL, 0 | (1 << 2) | (1 << 3) | (1 << 7))   # hold + auto
        for k in range(0, len(chunk), 8):
            b = chunk[k:k + 8] + bytes(8 - len(chunk[k:k + 8]))
            await link.write(R.SPI_BUF0, [int.from_bytes(b[0:4], "little"), int.from_bytes(b[4:8], "little")])
        await link.wr(R.SPI_CTRL, 0)  # release CS -> program
        await flash_wait(link)


@cocotb.test()
async def test_flash(dut):
    """JEDEC ID, XIP reads (single and quad), sector erase + page program over the link."""
    await start(dut)
    link = await link_up(dut)
    jid = await spi_xfer(link, 0, [0x9F, 0, 0, 0])
    assert jid[1:] == bytes([0xEF, 0x40, 0x18]), jid.hex()
    # erase sector 0x8000 and program 64 bytes
    await spi_xfer(link, 0, [0x06])
    await spi_xfer(link, 0, [0x20, 0x00, 0x80, 0x00])
    await flash_wait(link)
    w = await link.read(0x8000, 4)
    assert w == [0xFFFFFFFF] * 4
    data = bytes((i * 37 + 11) & 0xFF for i in range(64))
    await flash_program(link, 0x8000, data)
    got = b"".join(x.to_bytes(4, "little") for x in await link.read(0x8000, 16))
    assert got == data, "single-bit XIP read back mismatch"
    # switch to quad I/O reads (EBh, 4 dummy clocks, SCK = clk/2)
    await link.wr(R.FLASH_CFG, 1 | (4 << 1) | (0 << 4))
    got = b"".join(x.to_bytes(4, "little") for x in await link.read(0x8000, 16))
    assert got == data, "quad XIP read back mismatch"
    # while the port holds CS, host XIP reads return an error pattern instead of hanging
    await link.wr(R.SPI_CTRL, 0 | (1 << 2))
    await link.wr(R.SPI_BUF0, 0x05)
    await link.wr(R.SPI_XFER, 1)
    assert await link.rd(0x8000) == 0xFFFFFFFF
    await link.wr(R.SPI_CTRL, 0)
    assert await link.rd(0x8000) == int.from_bytes(data[0:4], "little")
    assert int(dut.contention.value) == 0


@cocotb.test()
async def test_imu_port(dut):
    """Aux chip select (future gyro): burst register read synchronized to the ADC."""
    await start(dut)
    link = await link_up(dut)
    r = await spi_xfer(link, 2, [0x80 | 0x75, 0])
    assert r[1] == 0x47, f"WHO_AM_I {r.hex()}"
    # write two registers, read them back in one 8-byte burst
    await spi_xfer(link, 2, [0x10, 0xDE, 0xAD])
    r = await spi_xfer(link, 2, [0x80 | 0x0F, 0, 0, 0, 0, 0, 0, 0])
    assert r[2:4] == bytes([0xDE, 0xAD]) and r[1] == (0x0F ^ 0x5A)
    # with the ADC running and SPI_SYNC set, bursts never disturb sampling
    await link.wr(R.PWM_PERIOD, 300)
    await link.wr(R.ADC_CFG, 16 | (1 << 9))
    await link.wr(R.CTRL, ctrl(R.CTRL_ADC_EN, R.CTRL_SPI_SYNC, amode=R.AMODE_FORCED))
    n0 = int(dut.adc.nconv.value)
    rng = random.Random(7)
    for k in range(25):
        # requests land at pseudo-random points of the PWM period
        await Timer(round(rng.uniform(0, 600) * 1e12 / CLK_HZ), unit="ps")
        n = 2 if k % 2 else 8
        r = await spi_xfer(link, 2, [0x80 | 0x75] + [0] * (n - 1))
        assert r[1] == 0x47
    st = await link.rd(R.STATUS)
    assert not st & R.FAULT_ADC_DROP
    assert int(dut.adc.nconv.value) > n0
    assert int(dut.contention.value) == 0


# ============================================================================= watchdog
@cocotb.test()
async def test_watchdog(dut):
    """Host link watchdog stops the bridge when the host goes silent."""
    await start(dut)
    link = await link_up(dut)
    await link.wr(R.PWM_PERIOD, 200)
    await link.wr(R.LINK_WDT, 50)          # 50 * 1024 clocks = ~1 ms
    await link.wr(R.FAULT_MASK, R.FAULT_NFAULT | R.FAULT_ESTOP | R.FAULT_OVERCUR | R.FAULT_WDT)
    await link.wr(R.CTRL, ctrl(R.CTRL_ARM, R.CTRL_DRV_EN, R.CTRL_WDT_EN, amode=R.AMODE_FORCED))
    for _ in range(4):                     # keep alive
        await Timer(us(400), unit="us")
        assert (await link.rd(R.CTRL)) & R.CTRL_ARM
    await Timer(us(1500), unit="us")           # silence
    assert int(dut.gates.value) == 0 and int(dut.drv_en.value) == 0, "bridge still active after watchdog timeout"
    st = await link.rd(R.STATUS)
    assert st & R.FAULT_WDT


# ============================================================================= CPU
FW_DIR = os.path.join(os.path.dirname(__file__), "..", "firmware", "build")


async def wait_reg(link, addr, pred, timeout_us, poll_us=None):
    poll_us = us(50) if poll_us is None else poll_us
    t_end = cocotb.utils.get_sim_time("us") + timeout_us
    while True:
        v = await link.rd(addr)
        if pred(v):
            return v
        assert cocotb.utils.get_sim_time("us") < t_end, f"timeout waiting on {addr:#x} (last {v:#x})"
        await Timer(poll_us, unit="us")


@cocotb.test()
async def test_cpu_firmware(dut):
    """RV32E boots from QSPI flash (XIP), passes the ISA self test, runs the mailbox app."""
    exp_path = os.path.join(FW_DIR, "selftest_expected.txt")
    with open(exp_path) as f:
        expected = int(f.read(), 16)
    await start(dut, boot=1)
    link = await link_up(dut)
    v = await wait_reg(link, R.MBOX_C2H, lambda x: x != 0, us(60000), poll_us=us(500))
    cpu = await link.rd(R.CPU_CTRL)
    pc = await link.rd(R.CPU_PC)
    dut._log.info(f"self test signature {v:#010x} (expected {expected:#010x}), cpu_ctrl {cpu:#x}, pc {pc:#x}, "
                  f"t = {cocotb.utils.get_sim_time('us'):.0f} us, flash reads {int(dut.flash.nreads.value)}")
    assert v == expected, "ISA self test failed"
    assert cpu & 1 and not cpu & R.CPU_HALTED, "CPU not running"
    assert (await link.rd(R.FLASH_CFG)) & 1, "firmware did not enable quad I/O"
    assert (await link.rd(R.PWM_PERIOD)) == 1250, "firmware did not load motor defaults"
    # mailbox echo
    await link.wr(R.MBOX_H2C, (2 << 24) | 0x1233)
    await wait_reg(link, R.MBOX_H2C, lambda x: x == 0, us(3000))
    assert await link.rd(R.MBOX_C2H) == 0xEC001234
    # speed ramp, 1 unit per 2048 clocks
    await link.wr(R.MBOX_H2C, (1 << 24) | 12)
    await wait_reg(link, R.SPD_CFG, lambda x: (x & 0xFFFF) == 12, us(3000))
    assert (await link.rd(R.SPD_CFG)) >> 16 & 0xFF == 9
    # EBREAK halts the core with cause 2; the host can restart it
    await link.wr(R.MBOX_H2C, 3 << 24)
    cpu = await wait_reg(link, R.CPU_CTRL, lambda x: x & R.CPU_HALTED, us(3000))
    assert (cpu >> 4) & 0xF == 2, f"halt cause {cpu:#x}"
    # restart with current sampling at 50 kHz: every ADC conversion pre-empts the
    # CPU's flash fetches, which must restart cleanly
    await link.wr(R.MBOX_C2H, 0)
    await link.wr(R.MBOX_H2C, 0)
    await link.wr(R.CPU_CTRL, 0)
    await link.wr(R.PWM_PERIOD, 500)
    await link.wr(R.ADC_CFG, 16 | (1 << 9) | (1 << 15) | (1 << 16))
    # SPI_SYNC: the firmware's flash bursts must wait for the post-conversion window,
    # otherwise a burst can overlap a sample (the chip then skips it and flags ADC_DROP)
    await link.wr(R.CTRL, ctrl(R.CTRL_ADC_EN, R.CTRL_SPI_SYNC, amode=R.AMODE_FORCED))
    n0 = int(dut.adc.nconv.value)
    t0 = cocotb.utils.get_sim_time("us")
    await link.wr(R.CPU_CTRL, 1)
    v = await wait_reg(link, R.MBOX_C2H, lambda x: x != 0, us(80000), poll_us=us(500))
    dt = cocotb.utils.get_sim_time("us") - t0
    nconv = int(dut.adc.nconv.value) - n0
    dut._log.info(f"with ADC pre-emption: signature {v:#010x}, {nconv} conversions in {dt:.0f} us")
    assert v == expected, "self test failed while the ADC pre-empts flash fetches"
    # 50 kHz until the firmware loads its 20 kHz defaults right after the signature;
    # the last polling interval (<= 500 us) may run at the slower rate
    assert nconv >= int((dt - us(500)) / us(20)) - 3, "ADC conversions missing"
    st = await link.rd(R.STATUS)
    assert not st & R.FAULT_ADC_DROP
    assert int(dut.contention.value) == 0


# ============================================================================= CPU debug
RAM_PROG = [0x20000537, 0x00000593, 0x00158593, 0x0AB52023, 0xFF9FF06F]  # see docs: count into MBOX_C2H
RAM_LOOP_SW = R.RAM_BASE + 0x0C   # address of the store in the loop


@cocotb.test()
async def test_cpu_debug(dut):
    """Boot address in RAM (no flash needed), pause, single step, hardware breakpoint."""
    await start(dut, boot=0)
    link = await link_up(dut)
    # upload a program into RAM over the link and boot it from there
    await link.write(R.RAM_BASE, RAM_PROG)
    assert await link.read(R.RAM_BASE, len(RAM_PROG)) == RAM_PROG
    await link.wr(R.CPU_BOOT, R.RAM_BASE)
    await link.wr(R.CPU_CTRL, R.CPU_RUN)
    v1 = await wait_reg(link, R.MBOX_C2H, lambda x: x > 10, us(500), poll_us=us(5))
    # pause: the counter stops and the core reports "paused"
    await link.wr(R.CPU_CTRL, R.CPU_RUN | R.CPU_PAUSE)
    st = await link.rd(R.CPU_CTRL)
    assert st & R.CPU_PAUSED, f"not paused {st:#x}"
    c0 = await link.rd(R.MBOX_C2H)
    await Timer(us(20), unit="us")
    assert await link.rd(R.MBOX_C2H) == c0, "CPU kept running while paused"
    # single step: the loop is 3 instructions -> 3 steps advance the counter by one
    for _ in range(3):
        await link.wr(R.CPU_CTRL, R.CPU_RUN | R.CPU_PAUSE | R.CPU_STEP)
    await Timer(us(2), unit="us")
    assert await link.rd(R.MBOX_C2H) == c0 + 1, "3 single steps should be one loop iteration"
    # breakpoint on the store: resume, the core stops there with the flag set
    await link.wr(R.CPU_BKPT, RAM_LOOP_SW | 1)
    await link.wr(R.CPU_CTRL, R.CPU_RUN)
    st = await wait_reg(link, R.CPU_CTRL, lambda x: x & R.CPU_PAUSED, us(200), poll_us=us(2))
    assert st & R.CPU_BKPT_HIT, f"breakpoint flag missing {st:#x}"
    assert await link.rd(R.CPU_PC) == RAM_LOOP_SW
    c1 = await link.rd(R.MBOX_C2H)
    # resuming from the breakpoint executes the store and stops there again one loop later
    await link.wr(R.CPU_CTRL, R.CPU_RUN)
    await wait_reg(link, R.CPU_CTRL, lambda x: x & R.CPU_PAUSED, us(200), poll_us=us(2))
    assert await link.rd(R.MBOX_C2H) == c1 + 1
    # remove the breakpoint, free run
    await link.wr(R.CPU_BKPT, 0)
    await link.wr(R.CPU_CTRL, R.CPU_RUN)
    await Timer(us(20), unit="us")
    assert await link.rd(R.MBOX_C2H) > c1 + 5
    dut._log.info(f"RAM boot ok (counter {v1}), pause / step / breakpoint ok")
