#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash
exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml