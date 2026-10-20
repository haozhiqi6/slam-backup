#!/usr/bin/env python3
"""从串口里把 STM32 打印的文本自检报告抠出来（忽略 39 字节的二进制帧）。

用法: python3 dump_serial_text.py <端口> <波特率> <秒数>
"""

from __future__ import annotations

import sys
import time

import serial


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 460800
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

    ser = serial.Serial(port, baud, timeout=0.1)
    buf = bytearray()
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            buf += ser.read(8192)
    finally:
        ser.close()

    chars = []
    for byte in buf:
        if byte in (10, 13) or 32 <= byte < 127:
            chars.append(chr(byte))
        else:
            chars.append("\n")
    lines = [line.strip() for line in "".join(chars).splitlines()]
    lines = [line for line in lines if len(line) >= 4]

    print("=== 串口 %s @ %d，采 %.0f 秒，共 %d 字节 ===" % (port, baud, duration, len(buf)))
    for line in lines[:60]:
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
