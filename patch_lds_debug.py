#!/usr/bin/env python3
"""给 livox_ros2_driver 的 lds.cpp 加一行调试打印：把雷达报的 timestamp_type 打出来。

0=未同步  1=PPS  3=PPS+PTP  4=PPS+GPS
只在类型变化时打印，不会刷屏。
"""

from __future__ import annotations

import sys

PATH = (
    "/home/orangepi/dl/livox_ws/src/livox_ros2_driver/"
    "livox_ros2_driver/livox_ros2_driver/lds.cpp"
)

NEEDLE = "  timestamp = RawLdsStampToNs(cur_timestamp, eth_packet->timestamp_type);"

DEBUG_BLOCK = (
    "  {\n"
    "    static uint8_t last_tt = 0xFF;\n"
    "    if (eth_packet->timestamp_type != last_tt) {\n"
    '      printf("[DBG] timestamp_type -> %d '
    '(0=no_sync 1=pps 3=pps+ptp 4=pps+gps)\\n",\n'
    "             eth_packet->timestamp_type);\n"
    "      fflush(stdout);\n"
    "      last_tt = eth_packet->timestamp_type;\n"
    "    }\n"
    "  }\n"
)


def main() -> int:
    with open(PATH, encoding="utf-8", errors="replace") as handle:
        source = handle.read()

    if "[DBG] timestamp_type" in source:
        print("已经打过补丁了，跳过")
        return 0

    if NEEDLE not in source:
        print("没找到插入点，源码可能变了")
        return 1

    source = source.replace(NEEDLE, DEBUG_BLOCK + NEEDLE, 1)
    with open(PATH, "w", encoding="utf-8") as handle:
        handle.write(source)

    print("已插入调试打印")
    return 0


if __name__ == "__main__":
    sys.exit(main())
