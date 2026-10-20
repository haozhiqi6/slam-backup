#!/bin/bash
# 香橙派一键启动：网口 -> socat -> fake_gps -> 雷达驱动 -> IMU 节点
# 日志都在 /home/orangepi/dl/ 下

LOG_DIR=/home/orangepi/dl

echo "=== 1. 雷达网口 enP4p65s0 ==="
echo orangepi | sudo -S -p "" ip link set enP4p65s0 up
echo orangepi | sudo -S -p "" ip addr add 192.168.10.50/24 dev enP4p65s0 2>/dev/null
sleep 2
ip -br a show enP4p65s0
ping -c 2 -W 2 192.168.10.18 2>&1 | tail -2

echo "=== 2. 清掉旧进程 ==="
pkill -f "[l]ivox_ros2_driver_node"
pkill -f "ros2 launch livox_ros2_driver"
pkill -f "[s]ocat -d -d pty"
pkill -f "[f]ake_gps.py"
pkill -f "[e]pson_imu_node"
sleep 1
echo "  已清"

echo "=== 3. socat 造虚拟串口 ==="
nohup socat -d -d pty,raw,echo=0,link=/tmp/gps_drv pty,raw,echo=0,link=/tmp/gps_wr \
  > "$LOG_DIR/socat.log" 2>&1 &
sleep 2
ls -l /tmp/gps_drv /tmp/gps_wr 2>&1

echo "=== 4. fake_gps 每秒写 GPRMC ==="
nohup python3 -u /home/orangepi/fake_gps.py --port /tmp/gps_wr --baud 9600 \
  > "$LOG_DIR/fake_gps.log" 2>&1 &
sleep 2
tail -2 "$LOG_DIR/fake_gps.log"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/orangepi/dl/livox_ws/install/setup.bash

echo "=== 5. 雷达驱动 ==="
nohup ros2 launch livox_ros2_driver livox_lidar_msg_launch.py \
  > "$LOG_DIR/livox_run.log" 2>&1 &
sleep 12
grep -E "timestamp_type|working state|disconnect" "$LOG_DIR/livox_run.log" | tail -4

echo "=== 6. IMU 节点 ==="
IMU_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
if [ -n "$IMU_PORT" ]; then
  nohup ros2 run sensor_sync epson_imu_node --ros-args \
    -p port:="$IMU_PORT" -p tx_gap:=0.0 -p push_interval:=2.0 -p utc_shift:=-0.69 \
    > "$LOG_DIR/imu_run.log" 2>&1 &
  sleep 10
  tail -5 "$LOG_DIR/imu_run.log"
else
  echo "  没找到 /dev/ttyUSB*，IMU 节点没起——先检查 CH340 的 USB"
fi

echo "=== 7. 话题 ==="
ros2 topic list | sort

echo "=== /livox/lidar 频率 ==="
timeout 5 ros2 topic hz /livox/lidar 2>&1 | head -3
echo "=== /livox/imu 频率 ==="
timeout 5 ros2 topic hz /livox/imu 2>&1 | head -3
