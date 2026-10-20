#!/bin/bash
LOG=/home/orangepi/dl/rebuild3.log
{
echo "=== 编译 fast_livo（含安全阀）开始 $(date +%H:%M:%S) ==="
free -h | head -3
} > "$LOG" 2>&1
source /opt/ros/humble/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
MAKEFLAGS="-j1" colcon build --packages-select fast_livo --parallel-workers 1 >> "$LOG" 2>&1
rc=$?
{
echo "=== 编译结束 rc=$rc  $(date +%H:%M:%S) ==="
tail -14 "$LOG"
} >> "$LOG" 2>&1