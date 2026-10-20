"""拉起 IMU 节点（雷达驱动要单独 launch）。

用法：
    ros2 launch sensor_sync sensor_sync.launch.py port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("topic", default_value="/livox/imu"),
        DeclareLaunchArgument("frame_id", default_value="livox_frame"),
        DeclareLaunchArgument("push_interval", default_value="10.0"),
        Node(
            package="sensor_sync",
            executable="epson_imu_node",
            name="epson_imu_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "port": LaunchConfiguration("port"),
                "topic": LaunchConfiguration("topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "push_interval": LaunchConfiguration("push_interval"),
            }],
        ),
    ])
