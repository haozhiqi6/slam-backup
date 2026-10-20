#!/bin/bash
# 在香橙派上把雷达授时（timesync）打开：
#   1. 配置里 enable_timesync: true, device_name: /tmp/gps_drv
#   2. socat 造一对虚拟串口
#   3. fake_gps 每秒往 /tmp/gps_wr 写一条 $GPRMC
#   4. 重启雷达驱动
#
# 日志：/home/orangepi/dl/fake_gps.log / socat.log / livox_run.log

CFG_INST=/home/orangepi/dl/livox_ws/install/livox_ros2_driver/share/livox_ros2_driver/config/livox_lidar_config.json
CFG_SRC=/home/orangepi/dl/livox_ws/src/livox_ros2_driver/livox_ros2_driver/config/livox_lidar_config.json

echo "=== 1. 停掉旧的 ==="
pkill -f "[l]ivox_ros2_driver_node"
pkill -f "ros2 launch livox_ros2_driver"
pkill -f "[s]ocat -d -d pty"
pkill -f "[f]ake_gps.py"
sleep 1
echo "  已停"

echo "=== 2. 改配置（install 和 src 两份都改，防止重编后被打回）==="
for f in "$CFG_INST" "$CFG_SRC"; do
  sed -i 's/"enable_timesync": false/"enable_timesync": true/' "$f"
  sed -i 's#"device_name":[[:space:]]*"[^"]*"#"device_name": "/tmp/gps_drv"#' "$f"
done
grep -A6 timesync_config "$CFG_INST"

echo "=== 3. socat 造虚拟串口 ==="
nohup socat -d -d pty,raw,echo=0,link=/tmp/gps_drv pty,raw,echo=0,link=/tmp/gps_wr \
  > /home/orangepi/dl/socat.log 2>&1 &
sleep 2
ls -l /tmp/gps_drv /tmp/gps_wr 2>&1

echo "=== 4. fake_gps 每秒写一条 GPRMC ==="
nohup python3 /home/orangepi/fake_gps.py --port /tmp/gps_wr --baud 9600 \
  > /home/orangepi/dl/fake_gps.log 2>&1 &
sleep 3
tail -3 /home/orangepi/dl/fake_gps.log

echo "=== 5. 起雷达驱动 ==="
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/orangepi/dl/livox_ws/install/setup.bash
nohup ros2 launch livox_ros2_driver livox_lidar_msg_launch.py \
  > /home/orangepi/dl/livox_run.log 2>&1 &
sleep 15
echo "--- 驱动日志 ---"
tail -25 /home/orangepi/dl/livox_run.log
