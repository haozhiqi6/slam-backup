#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
echo '只监测、不自动停止：时间异常、断流、飘飞均只报警。'
echo '无自动结束时限；按 Ctrl+C 才请求结束本轮建图和录包。'
exec /usr/bin/python3 -u /home/orangepi/capture_monitor_only.py --seconds "${1:-0}"
