#!/bin/bash
# 校准实验：给 STM32 发的 UTC 加一个人为偏移，看雷达偏差跟不跟着走
# 用法: bash calib_test.sh <shift秒>

SHIFT=${1:-0}

source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash

pkill -f "[e]pson_imu_node"
sleep 1

P=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -z "$P" ]; then
  echo "没有串口"
  exit 1
fi

nohup ros2 run sensor_sync epson_imu_node --ros-args \
  -p port:="$P" -p tx_gap:=0.0 -p push_interval:=2.0 -p utc_shift:="$SHIFT" \
  > /home/orangepi/dl/imu_run.log 2>&1 &
sleep 25

echo "=== utc_shift = $SHIFT 秒 ==="
timeout 30 ros2 run sensor_sync check_align --ros-args -p duration:=10.0 2>&1 \
  | grep -E "lidar|imu|偏差"
