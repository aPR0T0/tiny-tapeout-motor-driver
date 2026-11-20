"""Host link protocol of tt_um_apr0t0_foc_mcu (transport independent).

Frame on the wire : 0x7E | stuffed(payload + CRC16) | 0x7E
Stuffing          : 0x7E -> 7D 5E, 0x7D -> 7D 5D
CRC               : CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), big endian
Request payload   : SEQ CMD ADDR(4, LE) LEN [DATA (4*LEN, LE words)]
Response payload  : SEQ STATUS [DATA]
"""

import struct

FLAG = 0x7E
ESC = 0x7D

CMD_PING = 0x00
CMD_READ = 0x01
CMD_WRITE = 0x02
CMD_WRITE_FIFO = 0x03
CMD_READ_FIFO = 0x04

ST_OK = 0x00
ST_CRC = 0x01
ST_BAD_CMD = 0x02
ST_BAD_LEN = 0x03
ST_TELEMETRY = 0x80

MAX_WRITE_WORDS = 2
MAX_READ_WORDS = 255


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def stuff(data):
    out = bytearray()
    for b in data:
        if b in (FLAG, ESC):
            out += bytes([ESC, b ^ 0x20])
        else:
            out.append(b)
    return bytes(out)


def frame(payload):
    body = bytes(payload) + struct.pack(">H", crc16(payload))
    return bytes([FLAG]) + stuff(body) + bytes([FLAG])


def request(seq, cmd, addr=0, n=0, words=()):
    payload = struct.pack("<BBIB", seq & 0xFF, cmd, addr & 0xFFFFFFFF, n & 0xFF)
    for w in words:
        payload += struct.pack("<I", w & 0xFFFFFFFF)
    return frame(payload)


class Deframer:
    """Feed raw bytes, get (payload, crc_ok) tuples for every complete frame."""

    def __init__(self):
        self.buf = None
        self.esc = False

    def feed(self, data):
        out = []
        for b in data:
            if b == FLAG:
                if self.buf is not None and len(self.buf) >= 2:
                    ok = crc16(self.buf) == 0
                    out.append((bytes(self.buf[:-2]), ok))
                self.buf = bytearray()
                self.esc = False
            elif self.buf is None:
                continue
            elif b == ESC:
                self.esc = True
            else:
                self.buf.append(b ^ 0x20 if self.esc else b)
                self.esc = False
        return out


def parse_response(payload):
    """-> (seq, status, [words])"""
    seq, status = payload[0], payload[1]
    data = payload[2:]
    words = [struct.unpack_from("<I", data, i)[0] for i in range(0, len(data) - len(data) % 4, 4)]
    return seq, status, words


class LinkError(Exception):
    pass
