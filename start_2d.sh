#!/bin/bash
# ============================================================
# 在 FAST-LIVO2 已经在跑的前提下，启动 2D 建图链路：
#   3D 点云 → /scan → slam_toolbox → 2D 栅格图（带回环）
#
# 用法：   bash ~/start_2d.sh
# 前置：   start_all.sh 和 run_fastlivo.sh 都已经跑起来
# 停止：   Ctrl+C（会停掉本脚本启动的两个节点）
# 存图：   ros2 run nav2_map_server map_saver_cli -f ~/maps/myroom
# ============================================================
set -o pipefail

source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash

LOG=/home/orangepi/dl
PARAMS=/home/orangepi/slam2d/mapper_params_online_async.yaml

echo "=== 1. 检查 FAST-LIVO2 是否在跑 ==="
if ! timeout 5 ros2 topic list 2>/dev/null | grep -q "^/cloud_registered$"; then
    echo "  没看到 /cloud_registered —— 先把传感器和 FAST-LIVO2 起起来"
    echo "  应该是： bash ~/start_all.sh   然后   bash ~/run_fastlivo.sh"
    exit 1
fi
echo "  OK"

echo "=== 2. 起 点云→激光 转换 + tf ==="
pkill -f "[p]c2scan_tf.py" 2>/dev/null
sleep 1
setsid nohup python3 /home/orangepi/pc2scan_tf.py > "$LOG/pc2scan.log" 2>&1 < /dev/null &
sleep 4

if timeout 6 ros2 topic list 2>/dev/null | grep -q "^/scan$"; then
    echo "  /scan 已出现"
else
    echo "  没有 /scan，看日志： tail -20 $LOG/pc2scan.log"
fi

echo "=== 3. 起 slam_toolbox（2D 建图 + 回环）==="
pkill -f "[s]lam_toolbox" 2>/dev/null
sleep 2
setsid nohup ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$PARAMS" > "$LOG/slam_toolbox.log" 2>&1 < /dev/null &
sleep 8

echo "=== 4. 自检 ==="
echo "--- /scan 频率（期望 ~10 Hz）---"
timeout 5 ros2 topic hz /scan 2>&1 | head -3
echo "--- /map 频率（有回环优化时会更新）---"
timeout 5 ros2 topic hz /map 2>&1 | head -2
echo
echo "现在可以拿着雷达走动。地图在 RViz/Foxglove 的 /map 里看。"
echo "存图： ros2 run nav2_map_server map_saver_cli -f ~/maps/myroom"
echo "日志： $LOG/pc2scan.log   $LOG/slam_toolbox.log"
