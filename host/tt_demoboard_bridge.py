"""USB <-> host-link bridge for the Tiny Tapeout demo board (MicroPython, RP2350).

Copy to the demo board (e.g. `mpremote cp host/tt_demoboard_bridge.py :`) and run
`mpremote run host/tt_demoboard_bridge.py` or import it from main.py.  It

  1. selects tt_um_apr0t0_foc_mcu on the shuttle and clocks it at CLOCK_HZ,
  2. drives BOOT (ui[7]), nFAULT (ui[4]) and nESTOP (ui[6]) inactive-high levels,
  3. resets the chip (which also re-arms its autobaud),
  4. forwards raw bytes between the USB serial port and the chip's UART:
       RP2350 UART1 TX = GPIO20 -> ui[3] (chip RX)
       RP2350 UART1 RX = GPIO37 <- uo[4] (chip TX)
     so `host/focctl.py -p /dev/ttyACM0 -b <BAUD> ...` works through the board's USB.

While the chip is held in reset all uio pins are inputs, so the same board can
also program the QSPI Pmod flash directly (Tiny Tapeout flasher).

Not tested on hardware yet; written against tt-micropython-firmware's DemoBoard API.
"""

import sys
import select
import time

import machine
import micropython
from ttboard.demoboard import DemoBoard, RPMode

PROJECT = "tt_um_apr0t0_foc_mcu"
CLOCK_HZ = 50_000_000
BAUD = 2_000_000          # chip autobauds; keep focctl -b equal to this
BOOT = 0                  # 1 = run firmware from flash after reset
UART_TX_GPIO = 20         # -> ui[3]
UART_RX_GPIO = 37         # <- uo[4]


def setup():
    tt = DemoBoard.get()
    tt.mode = RPMode.ASIC_RP_CONTROL
    getattr(tt.shuttle, PROJECT).enable()
    tt.ui_in.value = (BOOT << 7) | (1 << 6) | (1 << 4) | (1 << 3)   # BOOT, nESTOP, nFAULT, idle RX
    tt.clock_project_PWM(CLOCK_HZ)
    tt.reset_project(True)
    time.sleep_ms(5)
    tt.reset_project(False)
    time.sleep_ms(5)
    # take GPIO20 / GPIO37 over from the SDK for the hardware UART
    uart = machine.UART(1, baudrate=BAUD, bits=8, parity=None, stop=1,
                        tx=machine.Pin(UART_TX_GPIO), rx=machine.Pin(UART_RX_GPIO),
                        txbuf=4096, rxbuf=16384)
    return tt, uart


def bridge(uart):
    micropython.kbd_intr(-1)            # binary data may contain 0x03
    usb_in = sys.stdin.buffer
    usb_out = sys.stdout.buffer
    poll = select.poll()
    poll.register(usb_in, select.POLLIN)
    buf = bytearray(256)
    while True:
        # host -> chip
        n = 0
        while n < len(buf) and poll.poll(0):
            buf[n] = usb_in.read(1)[0]
            n += 1
        if n:
            uart.write(memoryview(buf)[:n])
        # chip -> host
        k = uart.any()
        if k:
            usb_out.write(uart.read(k))


def main():
    _, uart = setup()
    bridge(uart)


main()
