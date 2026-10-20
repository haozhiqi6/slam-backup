#!/usr/bin/env python3
"""手动给 STM32 推 UTC（T<秒>,<毫秒> 换行），验证 PPS 相位锁定能不能生效。

用法: python3 push_utc.py <端口> <秒数> [每字节间隔ms]

为什么要"每字节间隔"：STM32 的 USART 接收只有一个字节的缓冲，而固件是
在主循环里轮询读的。主循环里有读 IMU 的 5ms 级等待，如果一次性把 15 个字节
打过去（@460800 只用 326us），绝大多数字节会因为溢出而丢掉。
所以慢发（默认 10ms/字节）反而更可靠。
"""

from __future__ import annotations

import sys
import time

import serial


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    gap_ms = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0

    ser = serial.Serial(port, 460800, timeout=0.1)
    print("打开 %s @ 460800，推 %.0f 秒，每字节间隔 %.0f ms" % (port, duration, gap_ms))
    t0 = time.time()
    count = 0
    try:
        while time.time() - t0 < duration:
            now = time.time()
            msg = b"T%d,%d\n" % (int(now), int((now % 1) * 1000))
            for byte in msg:
                ser.write(bytes([byte]))
                ser.flush()
                time.sleep(gap_ms / 1000.0)
            count += 1
            if count <= 5 or count % 5 == 0:
                print("  发: %s" % msg.decode().strip())
            time.sleep(max(0.0, 1.0 - (now % 1.0) - len(msg) * gap_ms / 1000.0))
    finally:
        ser.close()
    print("共发 %d 条" % count)
    return 0


if __name__ == "__main__":
    sys.exit(main())
