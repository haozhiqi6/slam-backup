#!/bin/bash
# 网页版 SLAM 可视化（浏览器打开 http://<板子IP>:8000）
source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash
exec python3 /home/orangepi/web_view.py
