#!/bin/bash
LOG=/home/orangepi/dl/rebuild_fastlivo.log
{
echo "=== 停 FAST-LIVO2 / RViz 释放内存 ==="
pkill -f "[r]viz2"
pkill -f "[f]astlivo_mapping"
pkill -f "ros2 launch fast_[l]ivo"
sleep 3
free -h | head -2
echo "=== 开始编译 $(date +%H:%M:%S) ==="
} > "$LOG" 2>&1
source /opt/ros/humble/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
MAKEFLAGS="-j1" colcon build --packages-select fast_livo --parallel-workers 1 >> "$LOG" 2>&1
rc=$?
{
echo "=== 编译结束 rc=$rc  $(date +%H:%M:%S) ==="
tail -8 "$LOG"
} >> "$LOG" 2>&1