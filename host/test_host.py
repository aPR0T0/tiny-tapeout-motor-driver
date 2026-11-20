"""Unit tests for the host-side protocol and tuning helpers (pytest)."""
import random

import foc_protocol as P
import focctl


def test_crc_reference_vector():
    assert P.crc16(b"123456789") == 0x29B1          # CRC-16/CCITT-FALSE check value


def test_frame_roundtrip_with_escapes():
    rng = random.Random(1)
    d = P.Deframer()
    for _ in range(200):
        payload = bytes(rng.choice([0x7E, 0x7D, 0x00, 0xFF, rng.randrange(256)]) for _ in range(rng.randrange(2, 40)))
        frames = d.feed(P.frame(payload))
        assert frames == [(payload, True)]


def test_corruption_detected_and_resync():
    d = P.Deframer()
    good = P.frame(b"\x01\x00hello")
    bad = bytearray(good)
    bad[4] ^= 0x01
    out = d.feed(bytes(bad) + good)
    assert (b"\x01\x00hello", True) in out
    assert any(not ok for _, ok in out)


def test_request_layout():
    f = P.request(7, P.CMD_WRITE, 0x20000034, 1, [0x11223344])
    d = P.Deframer()
    (payload, ok), = d.feed(f)
    assert ok and payload == bytes([7, 2, 0x34, 0, 0, 0x20, 1, 0x44, 0x33, 0x22, 0x11])


def test_gain_encoding():
    (kp, kps), (ki, kis) = focctl.current_pi_gains(0.5, 0.5e-3, 24, 20, 1000, 50000)
    assert abs(kp / 2 ** kps - 2.618) < 0.01
    assert abs(ki / 2 ** (kis + 16) - 0.05236) < 1e-4
    (kp, kps), (ki, kis) = focctl.speed_pi_gains(1e-5, 0.06, 4, 20, 50, 10000)
    assert abs(kp / 2 ** kps - 25.1) < 0.2
