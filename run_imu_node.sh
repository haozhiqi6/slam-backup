#!/bin/bash
# 起 IMU 节点（单独放脚本里跑，避免 pkill 的模式匹配到调用它的那条命令行）

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/orangepi/dl/livox_ws/install/setup.bash

pkill -f "[e]pson_imu_node"
sleep 1

# 串口名字会变（ttyUSB0 / ttyUSB1 ...），自动找第一个
PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -z "$PORT" ]; then
  echo "找不到 /dev/ttyUSB*，CH340 没插好"
  exit 1
fi
echo "用串口: $PORT"

nohup ros2 run sensor_sync epson_imu_node --ros-args \
  -p port:="$PORT" -p tx_gap:=0.0 -p push_interval:=2.0 -p utc_shift:=-0.69 \
  > /home/orangepi/dl/imu_run.log 2>&1 &
sleep 12

echo "=== IMU 节点日志 ==="
tail -12 /home/orangepi/dl/imu_run.log

echo "=== 话题 ==="
ros2 topic list | sort

echo "=== /livox/imu 频率 ==="
timeout 6 ros2 topic hz /livox/imu 2>&1 | head -4
