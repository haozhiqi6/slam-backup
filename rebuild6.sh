#!/bin/bash
D=/home/orangepi/dl/livox_ws/src/FASTLIVO2_ROS2-master
cp /tmp/voxel_map.cpp.new  $D/src/voxel_map.cpp
cp /tmp/voxel_map.h.new    $D/include/voxel_map.h
cp /tmp/LIVMapper.cpp.new  $D/src/LIVMapper.cpp
cp /tmp/LIVMapper.h.new    $D/include/LIVMapper.h
echo "源码已更新: $(grep -c publishMap $D/src/LIVMapper.cpp) 处调用"
LOG=/home/orangepi/dl/rebuild6.log
{
echo "=== 编译 fast_livo（新增累积地图发布）开始 $(date +%H:%M:%S) ==="
free -h | head -3
} > "$LOG" 2>&1
source /opt/ros/humble/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
MAKEFLAGS="-j1" colcon build --packages-select fast_livo --parallel-workers 1 >> "$LOG" 2>&1
rc=$?
{
echo "=== 编译结束 rc=$rc  $(date +%H:%M:%S) ==="
tail -12 "$LOG"
} >> "$LOG" 2>&1