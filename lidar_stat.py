import time, rclpy
from rclpy.node import Node
from livox_interfaces.msg import CustomMsg

latest = {"m": None}
def main():
    rclpy.init()
    n = Node("lidar_stat")
    n.create_subscription(CustomMsg, "/livox/lidar", lambda m: latest.update(m=m), 10)
    t0 = time.time()
    while time.time() - t0 < 6 and latest["m"] is None:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = latest["m"]
    if m is None:
        print("没收到"); return
    pts = list(m.points)
    tot = len(pts)
    zero = 0; rs = []
    for p in pts:
        r = (p.x*p.x + p.y*p.y + p.z*p.z) ** 0.5
        if r < 0.05:
            zero += 1
        else:
            rs.append(r)
    rs.sort()
    print("总点数 %d   无效点(坐标全0) %d (%.1f%%)   有效回波 %d" % (tot, zero, 100.0*zero/tot, len(rs)))
    if rs:
        k = len(rs)
        print("有效回波的距离分布:")
        print("  最近 %.2f m" % rs[0])
        print("  中位 %.2f m" % rs[k//2])
        print("  最远 %.2f m" % rs[-1])
        for th in (0.8, 1.0, 2.0, 5.0, 10.0):
            c = sum(1 for r in rs if r >= th)
            print("  >= %4.1f m: %5d 个 (%.1f%%)" % (th, c, 100.0*c/k))
    n.destroy_node(); rclpy.shutdown()
main()