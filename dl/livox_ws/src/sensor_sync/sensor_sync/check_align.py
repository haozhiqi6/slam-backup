#!/usr/bin/env python3
"""量两路时间戳的对齐程度（ROS 2 版，对应 Windows 上的 check_align.py）。

分别订阅雷达和 IMU，各自算「消息时间戳 - 主机当前时间」，
跑 30 秒后统计平均值和抖动。两个平均值之差就是两路之间的偏差。

用法：
    ros2 run sensor_sync check_align
    ros2 run sensor_sync check_align --ros-args -p duration:=60.0
"""

from __future__ import annotations

import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

# 雷达消息类型：这份 ROS2 驱动的包名是 livox_interfaces（不是 livox_ros_driver2）
try:
    from livox_interfaces.msg import CustomMsg as LidarMsg
    _LIDAR_KIND = "livox_interfaces/CustomMsg"
except Exception:  # noqa: BLE001
    from sensor_msgs.msg import PointCloud2 as LidarMsg  # type: ignore
    _LIDAR_KIND = "sensor_msgs/PointCloud2"


def _stat(name: str, values: deque):
    data = list(values)
    if not data:
        print("%-6s 没收到数据" % name)
        return None
    mean = statistics.mean(data)
    try:
        sd = statistics.pstdev(data)
    except Exception:  # noqa: BLE001
        sd = 0.0
    spread = max(abs(x - mean) for x in data) * 1000.0
    print(
        "%-6s 样本 %5d  平均 %+8.4f s  抖动(1sigma) %6.1f ms  最大偏差 %6.1f ms"
        % (name, len(data), mean, sd * 1000.0, spread)
    )
    return mean


class CheckAlign(Node):

    def __init__(self) -> None:
        super().__init__("check_align")
        self.declare_parameter("duration", 30.0)
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("imu_topic", "/livox/imu")

        self.duration = float(self.get_parameter("duration").value)
        lidar_topic = self.get_parameter("lidar_topic").value
        imu_topic = self.get_parameter("imu_topic").value

        self.d_lidar: deque = deque(maxlen=200000)
        self.d_imu: deque = deque(maxlen=200000)

        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(LidarMsg, lidar_topic, self._on_lidar, qos)
        self.create_subscription(Imu, imu_topic, self._on_imu, qos)

        self.get_logger().info(
            "雷达 %s（%s） / IMU %s，采样 %.0f 秒"
            % (lidar_topic, _LIDAR_KIND, imu_topic, self.duration)
        )
        self.t0 = time.time()

    def _on_lidar(self, msg) -> None:
        self.d_lidar.append(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - time.time()
        )

    def _on_imu(self, msg) -> None:
        self.d_imu.append(
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - time.time()
        )

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CheckAlign()
    try:
        # 用 spin_once 循环而不是 rclpy.spin + 计时器里调 rclpy.shutdown()：
        # 后者会在回调里关闭 executor，直接卡死
        while rclpy.ok() and time.time() - node.t0 < node.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        print("-" * 72)
        ml = _stat("lidar", node.d_lidar)
        mi = _stat("imu", node.d_imu)
        if ml is not None and mi is not None:
            print("-> 两路时间戳的平均偏差 = %+.4f 秒" % (ml - mi))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
