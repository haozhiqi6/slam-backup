#!/bin/bash
# 带调试打印重编 livox_ros2_driver 并重启
# （放脚本里跑，避免 pkill 的模式匹配到调用它的命令行把自己杀掉）

echo "=== 停掉旧驱动 ==="
pkill -f "[l]ivox_ros2_driver_node"
pkill -f "ros2 launch livox_ros2_driver"
sleep 1

echo "=== 重编 livox_ros2_driver ==="
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
colcon build --packages-select livox_ros2_driver \
  --cmake-args -DCMAKE_BUILD_TYPE=Release > /home/orangepi/dl/rebuild.log 2>&1
if [ $? -ne 0 ]; then
  echo "重编失败，最后 20 行："
  tail -20 /home/orangepi/dl/rebuild.log
  exit 1
fi
echo "  重编完成"

echo "=== 重启驱动 ==="
# shellcheck disable=SC1091
source /home/orangepi/dl/livox_ws/install/setup.bash
nohup ros2 launch livox_ros2_driver livox_lidar_msg_launch.py \
  > /home/orangepi/dl/livox_run.log 2>&1 &
sleep 15

echo "=== 驱动日志里的关键行 ==="
grep -E "DBG|start sample|status_code|disconnect|timestamp_type" /home/orangepi/dl/livox_run.log | tail -12

echo "=== 雷达时间戳 ==="
echo -n "主机: "
date +%s.%N
timeout 6 ros2 topic echo /livox/lidar --once 2>&1 | head -6
