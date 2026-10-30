#!/usr/bin/env bash
# New external launcher: no edits to the original mapper or YAML files.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/orangepi/dl/livox_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
task_tool_dir=$(cd -- "$(dirname -- "$0")" && pwd)
exec /usr/bin/python3 -u "$task_tool_dir/run_task.py" "$@"
