#!/bin/bash
D=/home/orangepi/dl/livox_ws/src/FASTLIVO2_ROS2-master
cp /tmp/voxel_map.h.new2 $D/include/voxel_map.h
echo "头文件已更新"
LOG=/home/orangepi/dl/rebuild7.log
{
echo "=== 编译 fast_livo（第二次尝试）开始 $(date +%H:%M:%S) ==="
} > "$LOG" 2>&1
source /opt/ros/humble/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
MAKEFLAGS="-j1" colcon build --packages-select fast_livo --parallel-workers 1 >> "$LOG" 2>&1
rc=$?
{
echo "=== 编译结束 rc=$rc  $(date +%H:%M:%S) ==="
tail -10 "$LOG"
} >> "$LOG" 2>&1