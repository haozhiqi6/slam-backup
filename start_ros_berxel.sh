#!/bin/bash
# Berxel iHawk100Q ROS2 一键启动 (Orange Pi 5 Pro)
#   1) berxel_camera_ros2 节点: 发布 /depth/depth_raw /color/color_raw /berxel_cloudpoint
#   2) foxglove_bridge: 浏览器看图用, WebSocket 端口 8765
# 用法: ./start_ros_berxel.sh [stop]
#   不带参数 = 启动(后台, 日志 /tmp/berxel_ros.log 与 /tmp/foxglove_bridge.log)
#   stop      = 全部停止
SDK=~/berxel_sdk/BerxelSDK-Linux-2.0.140_arm64_7.5.0
LOG=/tmp/berxel_ros.log
FLOG=/tmp/foxglove_bridge.log

if [ "$1" = "stop" ]; then
    pkill -f "berxel_camera_iHawk100.launch" 2>/dev/null
    pkill -f berxel_camera_ros2 2>/dev/null
    pkill -f foxglove_bridge 2>/dev/null
    echo "已停止"
    exit 0
fi

source /opt/ros/humble/setup.bash
source ~/berxel_ws/install/setup.bash
export BERXEL_SDK_LIBRARY="$SDK"
export LD_LIBRARY_PATH="$SDK/libs:$LD_LIBRARY_PATH"

if pgrep -f berxel_camera_ros2 > /dev/null; then
    echo "节点已在运行, 先 ./start_ros_berxel.sh stop"
    exit 1
fi
echo "==> 启动 Berxel ROS2 节点 (日志: $LOG)"
nohup ros2 launch berxel_camera_ros2 berxel_camera_iHawk100.launch.py > "$LOG" 2>&1 &
sleep 5
timeout 3 ros2 topic list 2>/dev/null | grep -q depth && echo "==> 深度话题已发布, 运行中" || echo "==> 启动中, 查看 $LOG"

if pgrep -f foxglove_bridge > /dev/null; then
    echo "==> foxglove bridge 已在运行"
else
    echo "==> 启动 foxglove bridge (端口 8765)"
    nohup ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765 > "$FLOG" 2>&1 &
    sleep 4
    ss -tln | grep -q 8765 && echo "==> bridge 就绪: ws://192.168.43.7:8765" || echo "==> bridge 异常, 查看 $FLOG"
fi
