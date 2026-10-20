#!/bin/bash
D=/home/orangepi/dl/livox_ws/src/FASTLIVO2_ROS2-master
cp /tmp/vm2.cpp  $D/src/voxel_map.cpp
cp /tmp/vm2.h    $D/include/voxel_map.h
cp /tmp/lm2.cpp  $D/src/LIVMapper.cpp
cp /tmp/lm2.h    $D/include/LIVMapper.h
echo "源码已更新, stride 引用: $(grep -c pub_map_stride $D/src/LIVMapper.cpp)"
LOG=/home/orangepi/dl/rebuild9.log
{ echo "=== 编译（地图发布加抽样）开始 $(date +%H:%M:%S) ==="; } > "$LOG" 2>&1
source /opt/ros/humble/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
MAKEFLAGS="-j1" colcon build --packages-select fast_livo --parallel-workers 1 >> "$LOG" 2>&1
rc=$?
{ echo "=== 编译结束 rc=$rc  $(date +%H:%M:%S) ==="; tail -10 "$LOG"; } >> "$LOG" 2>&1