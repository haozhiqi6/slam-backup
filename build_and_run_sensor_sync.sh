#!/bin/bash
# 部署 + 编译 sensor_sync，然后起 IMU 节点

WS=/home/orangepi/dl/livox_ws
LOG=/home/orangepi/dl/sensor_sync_build.log

echo "=== 解包到工作空间 ==="
cd "$WS/src" || exit 1
rm -rf sensor_sync
tar -xzf /home/orangepi/sensor_sync.tgz || { echo "解包失败"; exit 1; }
ls -d sensor_sync

echo "=== 编译 ==="
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd "$WS" || exit 1
colcon build --packages-select sensor_sync > "$LOG" 2>&1
if [ $? -ne 0 ]; then
  echo "编译失败，最后 25 行："
  tail -25 "$LOG"
  exit 1
fi
echo "  编译完成"

echo "=== 起 IMU 节点 ==="
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
nohup ros2 run sensor_sync epson_imu_node --ros-args -p port:=/dev/ttyUSB0 \
  > /home/orangepi/dl/imu_run.log 2>&1 &
sleep 12

echo "=== IMU 节点日志 ==="
tail -12 /home/orangepi/dl/imu_run.log

echo "=== 话题 ==="
ros2 topic list | sort

echo "=== /livox/imu 频率 ==="
timeout 6 ros2 topic hz /livox/imu 2>&1 | head -4
