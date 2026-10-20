#!/usr/bin/env python3
"""假 GPS：每秒往一个串口写一条 $GPRMC，给 Livox 驱动做时间同步用。

配合 socat 造的虚拟串口（驱动读一头、这个脚本写另一头）：

    # 终端 A：造一对虚拟串口
    socat -d -d pty,raw,echo=0,link=/tmp/gps_drv pty,raw,echo=0,link=/tmp/gps_wr

    # 终端 B：跑这个脚本，往 /tmp/gps_wr 写
    python3 fake_gps.py --port /tmp/gps_wr --baud 9600

    # 然后驱动配置里把 timesync 指向 /tmp/gps_drv（见 README 第九节）

时间取主机的 UTC（和 STM32 拿到的那个 UTC 是同一个）。
"""

from __future__ import annotations

import argparse
import time

import serial

TALKER = "$GPRMC"
# 用一个真实位置（北京附近），免得解析器嫌 "0 度" 非法
LAT = "3958.0000,N"
LON = "11623.0000,E"


def build(utc: float) -> bytes:
    tm = time.gmtime(utc)
    body = "%s,%02d%02d%02d.00,A,%s,%s,0.0,0.0,%02d%02d%02d,,,A" % (
        TALKER,
        tm.tm_hour,
        tm.tm_min,
        tm.tm_sec,
        LAT,
        LON,
        tm.tm_mday,
        tm.tm_mon,
        tm.tm_year % 100,
    )
    chk = 0
    for ch in body[1:]:
        chk ^= ord(ch)
    return ("%s*%02X\r\n" % (body, chk)).encode()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="写哪一头，例如 /tmp/gps_wr")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--once", action="store_true", help="只发一条，用于测试")
    args = ap.parse_args()

    sp = serial.Serial(args.port, args.baud, timeout=0.2)
    print(f"假 GPS 已打开 {args.port} @ {args.baud}")

    try:
        while True:
            utc = time.time()
            line = build(utc)
            sp.write(line)
            print(line.decode().strip())
            if args.once:
                break
            # 对齐到下一个整秒再发（跟真 GPS 一样，正好在秒边界之后）
            time.sleep(max(0.0, 1.0 - (utc % 1.0)))
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        sp.close()


if __name__ == "__main__":
    main()
