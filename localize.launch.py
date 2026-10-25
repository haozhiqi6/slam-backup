#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
加载已有 2D 栅格图，用 AMCL 做重定位。

前置：FAST-LIVO2 + pc2scan_tf.py 已经在跑（提供 /scan 和 odom->base_link）。

用法：
    ros2 launch ~/localize.launch.py map:=$HOME/maps/myroom.yaml
    或者
    ros2 launch ~/localize.launch.py map:=$HOME/mymap.yaml

启动后会：
    1. map_server 读入 .pgm/.yaml，发布 /map
    2. amcl 订阅 /scan 和 /map，做粒子滤波定位
    3. 发布 map->odom 的 tf（导航需要的那一环）
    4. 发布 /amcl_pose（当前估计位姿）

看定位效果：
    ros2 topic echo /amcl_pose --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = os.path.expanduser("~/maps/myroom.yaml")

    map_arg = DeclareLaunchArgument(
        "map", default_value=default_map, description="2D 栅格图的 yaml 路径"
    )
    map_yaml = LaunchConfiguration("map")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_yaml,
            "use_sim_time": False,
            "topic_name": "map",
            "frame_id": "map",
        }],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "alpha1": 0.2, "alpha2": 0.2, "alpha3": 0.2, "alpha4": 0.2, "alpha5": 0.2,
            "base_frame_id": "base_link",
            "global_frame_id": "map",
            "odom_frame_id": "odom",
            "scan_topic": "scan",
            "map_topic": "map",
            "laser_min_range": 0.3,
            "laser_max_range": 12.0,
            "laser_model_type": "likelihood_field",
            "max_beams": 60,
            "min_particles": 500,
            "max_particles": 2000,
            "pf_err": 0.05,
            "pf_z": 0.99,
            "resample_interval": 1,
            "update_min_d": 0.15,
            "update_min_a": 0.15,
            "transform_tolerance": 1.0,
            "tf_broadcast": True,
            "robot_model_type": "nav2_amcl::DifferentialMotionModel",
            "set_initial_pose": True,
            "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "always_reset_initial_pose": False,
        }],
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "bond_timeout": 0.0,
            "node_names": ["map_server", "amcl"],
        }],
    )

    return LaunchDescription([map_arg, map_server, amcl, lifecycle])
