#!/bin/bash
# 重启 fake_gps，这次用 python3 -u 让它不缓冲，日志里能看到它到底在不在写
# （单独放脚本里跑，避免 pkill 的模式匹配到调用它的那条命令行）

pkill -f "[f]ake_gps.py"
sleep 1
nohup python3 -u /home/orangepi/fake_gps.py --port /tmp/gps_wr --baud 9600 \
  > /home/orangepi/dl/fake_gps.log 2>&1 &
sleep 4

echo "--- fake_gps 日志 ---"
tail -4 /home/orangepi/dl/fake_gps.log
echo "--- 进程 ---"
pgrep -af "[f]ake_gps.py" > /dev/null && echo "  在跑" || echo "  没在跑"
echo "--- 驱动打开的串口 ---"
DRV=$(pgrep -f "[l]ivox_ros2_driver_node" | head -1)
ls -l /proc/"$DRV"/fd 2>/dev/null | grep pts || echo "  (驱动没开 pts)"
