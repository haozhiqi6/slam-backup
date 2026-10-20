#!/bin/bash
# 重启 FAST-LIVO2（不带 rviz），日志写 /home/orangepi/dl/fastlivo_run.log

pkill -f "[f]astlivo_mapping"
pkill -f "ros2 launch fast_[l]ivo"
sleep 2

source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash

cd /home/orangepi/dl/livox_ws || exit 1
nohup ros2 launch fast_livo mapping_avia.launch.py use_rviz:=False \
  > /home/orangepi/dl/fastlivo_run.log 2>&1 &
sleep 30

echo "--- FAST-LIVO2 日志 ---"
tail -30 /home/orangepi/dl/fastlivo_run.log
