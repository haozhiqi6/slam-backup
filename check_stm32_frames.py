#!/usr/bin/env python3
"""快速检查 STM32 有没有在往串口发 39 字节帧（不依赖 ROS）。

用法: python3 check_stm32_frames.py [端口] [波特率] [秒数]
"""

from __future__ import annotations

import struct
import sys
import time

import serial

HEAD = b"\xAA\x55"
FRAME_LEN = 39


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 460800
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:  # noqa: BLE001
        print("打不开 %s：%s" % (port, exc))
        return 1

    buf = bytearray()
    good = bad = 0
    first_t_us = None
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            buf += ser.read(4096)
            while True:
                i = buf.find(HEAD)
                if i < 0 or len(buf) - i < FRAME_LEN:
                    if i > 0:
                        del buf[:i]
                    elif len(buf) > 4096:
                        del buf[:-FRAME_LEN]
                    break
                frame = bytes(buf[i : i + FRAME_LEN])
                del buf[: i + FRAME_LEN]
                chk = 0
                for byte in frame[2:38]:
                    chk ^= byte
                if chk == frame[38]:
                    good += 1
                    if first_t_us is None:
                        first_t_us = struct.unpack_from("<I", frame, 4)[0]
                else:
                    bad += 1
    finally:
        ser.close()

    print("串口 %s @ %d" % (port, baud))
    print("采集 %.1f 秒" % duration)
    print("好帧 %d  坏帧 %d  速率约 %.1f Hz" % (good, bad, good / duration))
    if first_t_us is not None:
        print("第一帧的 STM32 微秒时间戳: %d  (= %.3f 秒)" % (first_t_us, first_t_us / 1e6))
    return 0


if __name__ == "__main__":
    sys.exit(main())
