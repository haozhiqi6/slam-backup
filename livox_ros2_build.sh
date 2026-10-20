#!/bin/bash
# 在香橙派上编译 livox_ros2_driver（ROS2 Humble + Livox-SDK v1）
# 日志: /home/orangepi/dl/livox_build.log

WS=/home/orangepi/dl/livox_ws
SRC=/home/orangepi/dl/livox_ros2_driver
LOG=/home/orangepi/dl/livox_build.log

echo "=== 开始 $(date) ===" > "$LOG"

mkdir -p "$WS/src"
rm -rf "$WS/src/livox_ros2_driver"
cp -r "$SRC" "$WS/src/livox_ros2_driver" || { echo "COPY_FAIL" >> "$LOG"; exit 1; }

# 清掉压缩包里带过来的旧构建产物 —— 那些 CMake 缓存里写的是别人的绝对路径
rm -rf "$WS"/src/livox_ros2_driver/livox_ros2_driver/Livox-SDK/build
echo "--- 已清理旧构建缓存 ---" >> "$LOG"

echo "--- 系统里有 livox_sdk 吗（vendor 找不到就会去 GitHub 拉，板子上不通）---" >> "$LOG"
ls /usr/local/include/livox_sdk.h /usr/local/lib/liblivox_sdk_static.a >> "$LOG" 2>&1

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WS" || exit 1
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release >> "$LOG" 2>&1 \
  || { echo "COLCON_FAIL" >> "$LOG"; exit 1; }

echo "BUILD_OK $(date)" >> "$LOG"
