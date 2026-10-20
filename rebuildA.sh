#!/bin/bash
D=/home/orangepi/dl/livox_ws/src/FASTLIVO2_ROS2-master
cp /tmp/g3.cpp  $D/src/LIVMapper.cpp
cp /tmp/g3.h    $D/include/LIVMapper.h
echo "源码已更新"
LOG=/home/orangepi/dl/rebuildA.log
{ echo "=== 编译（姿态突变保护）开始 $(date +%H:%M:%S) ==="; } > "$LOG" 2>&1
source /opt/ros/humble/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
MAKEFLAGS="-j1" colcon build --packages-select fast_livo --parallel-workers 1 >> "$LOG" 2>&1
rc=$?
{ echo "=== 编译结束 rc=$rc  $(date +%H:%M:%S) ==="; tail -8 "$LOG"; } >> "$LOG" 2>&1