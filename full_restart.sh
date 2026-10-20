#!/bin/bash
# 整套干净重启：传感器 → FAST-LIVO2 → 一个 RViz → 自检

LOG=/home/orangepi/dl

echo "=== 0. 全停 ==="
pkill -f "[r]viz2"
pkill -f "[f]astlivo_mapping"
pkill -f "ros2 launch fast_[l]ivo"
sleep 3
echo "  已停"

echo "=== 1. 起传感器（网口→socat→假GPS→雷达→IMU）==="
bash /home/orangepi/start_all.sh > "$LOG/start_all.log" 2>&1
grep -E "timestamp_type|已打开" "$LOG/start_all.log" | tail -3

echo "=== 2. 起 FAST-LIVO2 ==="
source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash
cd /home/orangepi/dl/livox_ws || exit 1
nohup ros2 launch fast_livo mapping_avia.launch.py use_rviz:=False \
  > "$LOG/fastlivo_run.log" 2>&1 &
sleep 20

echo "=== 3. 起一个 RViz（只显示点云+轨迹）==="
nohup env DISPLAY=:0 XAUTHORITY=/home/orangepi/.Xauthority \
  rviz2 -d /home/orangepi/cloud_only.rviz > "$LOG/rviz2.log" 2>&1 &
sleep 15

echo "=== 4. 自检 ==="
echo "--- 雷达距离 ---"
python3 /home/orangepi/check_lidar_range.py 2>&1 | grep -E "中位|距离 >=   2.0"
echo "--- LIO 有效特征 ---"
grep -E "effective feature num" "$LOG/fastlivo_run.log" | tail -2
echo "--- /cloud_registered ---"
timeout 6 ros2 topic hz /cloud_registered 2>&1 | head -2
echo "--- 内存 ---"
free -h | head -2
