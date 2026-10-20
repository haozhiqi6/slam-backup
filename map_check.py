import time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import struct

got = {}
def cb(m):
    got["m"] = m

rclpy.init()
n = Node("map_check")
q = QoSProfile(depth=5); q.reliability = ReliabilityPolicy.RELIABLE; q.durability = DurabilityPolicy.VOLATILE
n.create_subscription(PointCloud2, "/Laser_map", cb, q)
t0 = time.time()
while time.time()-t0 < 12 and "m" not in got:
    rclpy.spin_once(n, timeout_sec=0.2)
if "m" not in got:
    print("12 秒内没收到 /Laser_map")
else:
    m = got["m"]
    print("收到！点云尺寸: 宽 %d × 高 %d = %d 个点" % (m.width, m.height, m.width*m.height))
    print("frame_id: %s   point_step: %d   is_dense: %s" % (m.header.frame_id, m.point_step, m.is_dense))
    # 读前 5 个点的 xyz
    step = m.point_step
    print("前 5 个点的坐标:")
    for i in range(min(5, m.width)):
        x, y, z = struct.unpack_from("<fff", bytes(m.data), i*step)
        print("  %.3f  %.3f  %.3f" % (x, y, z))
rclpy.shutdown()