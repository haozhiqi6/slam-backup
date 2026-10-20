#!/bin/bash
# 在香橙派上把 livox_ros_driver2 编进一个 ROS2 工作空间
# 日志: /home/orangepi/dl/driver2_build.log

WS=/home/orangepi/dl/ws
SRC=/home/orangepi/dl/livox_ros_driver2
LOG=/home/orangepi/dl/driver2_build.log

echo "=== 开始 $(date) ===" > "$LOG"

mkdir -p "$WS/src"
rm -rf "$WS/src/livox_ros_driver2"
cp -r "$SRC" "$WS/src/livox_ros_driver2"

cd "$WS/src/livox_ros_driver2" || { echo NO_SRC >> "$LOG"; exit 1; }

# 这个包同时带 ROS1/ROS2 两套清单，ROS2 用 package_ROS2.xml + launch_ROS2
cp -f package_ROS2.xml package.xml
rm -rf launch
cp -r launch_ROS2 launch

echo "--- package.xml / launch 已就位 ---" >> "$LOG"

echo "--- PCL 检查 ---" >> "$LOG"
ls /usr/include/pcl-* -d >> "$LOG" 2>&1 || echo "(没有 PCL)" >> "$LOG"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WS" || exit 1
colcon build \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_BUILD_TYPE=Release \
  >> "$LOG" 2>&1 || { echo "COLCON_FAIL" >> "$LOG"; exit 1; }

echo "BUILD_OK $(date)" >> "$LOG"
