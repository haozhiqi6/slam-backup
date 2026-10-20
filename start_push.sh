#!/bin/bash
# 每秒给 STM32 推一条 UTC，持续 3 分钟，供 Keil 里观察 g_rx_cnt / g_utc_valid

pkill -f "[e]pson_imu_node"
pkill -f "[p]ush_utc"
sleep 1

P=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -z "$P" ]; then
  echo "没有串口"
  exit 1
fi
echo "串口 $P"

nohup python3 -u /home/orangepi/push_utc.py "$P" 1800 \
  > /home/orangepi/dl/push.log 2>&1 &
sleep 5
echo "已开始每秒推 UTC（持续 30 分钟）"
tail -5 /home/orangepi/dl/push.log
