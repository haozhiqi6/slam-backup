#!/bin/bash
# 关掉所有 RViz，重开一个最简的（只看点云 + 轨迹）
# 顺便重启 FAST-LIVO2，让算法状态干净

echo "=== 1. 关掉所有 RViz ==="
pkill -f "[r]viz2"
sleep 3
pgrep -c -f "[r]viz2" 2>/dev/null || echo "  已全关"

echo "=== 2. 重启 FAST-LIVO2 ==="
pkill -f "[f]astlivo_mapping"
pkill -f "ros2 launch fast_[l]ivo"
sleep 2

source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
nohup ros2 launch fast_livo mapping_avia.launch.py use_rviz:=False \
  > /home/orangepi/dl/fastlivo_run.log 2>&1 &
sleep 20

echo "=== 3. 只开一个 RViz（最简配置）==="
nohup env DISPLAY=:0 XAUTHORITY=/home/orangepi/.Xauthority \
  rviz2 -d /home/orangepi/cloud_only.rviz \
  > /home/orangepi/dl/rviz2.log 2>&1 &
sleep 18

echo "--- rviz2 进程 ---"
pgrep -af "[r]viz2"
echo "--- 内存 ---"
free -h | head -2
echo "--- /cloud_registered ---"
timeout 8 ros2 topic hz /cloud_registered 2>&1 | head -3
echo "--- 有效特征 ---"
grep -E "effective feature num" /home/orangepi/dl/fastlivo_run.log | tail -2
