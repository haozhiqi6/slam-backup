#!/bin/bash
# 停 IMU 节点 -> 先看一次雷达偏差 -> 手动推 UTC 20 秒 -> 再看一次

pkill -f "[e]pson_imu_node"
sleep 1

P=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -z "$P" ]; then
  echo "没有串口"
  exit 1
fi

source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash

echo "=== 推 UTC 之前的雷达偏差 ==="
echo -n "主机 "
date +%s.%N
timeout 5 ros2 topic echo /livox/lidar --once 2>&1 | sed -n "3,4p"

echo
echo "=== 手动推 UTC 20 秒 ==="
python3 /home/orangepi/push_utc.py "$P" 20

echo
echo "=== 推完之后的雷达偏差 ==="
echo -n "主机 "
date +%s.%N
timeout 5 ros2 topic echo /livox/lidar --once 2>&1 | sed -n "3,4p"
