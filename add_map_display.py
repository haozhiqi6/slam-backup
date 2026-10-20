import re, shutil
p = "/home/orangepi/cloud_only.rviz"
s = open(p, encoding="utf-8").read()
shutil.copy(p, p + ".bak3")
if "/Laser_map" in s:
    print("已经有 /Laser_map 了，跳过")
else:
    blk = """    - Class: rviz_default_plugins/PointCloud2
      Name: map
      Enabled: true
      Topic:
        Value: /Laser_map
        Depth: 2
        Durability Policy: Volatile
        Reliability Policy: Reliable
        History Policy: Keep Last
      Style: Points
      Size (Pixels): 2
      Size (m): 0.01
      Color Transformer: FlatColor
      Color: 80; 255; 120
      Position Transformer: XYZ
      Alpha: 1
      Autocompute Intensity Bounds: true
      Use rainbow: false
      Use Fixed Frame: true
      Selectable: true
      Decay Time: 0
"""
    # 插在 "Displays:" 之后（也就是所有显示层的最前面）
    s = s.replace("  Displays:\n", "  Displays:\n" + blk, 1)
    open(p, "w", encoding="utf-8").write(s)
    print("已插入 /Laser_map 显示层")
print("--- 校验 ---")
for i, line in enumerate(open(p, encoding="utf-8"), 1):
    if "/Laser_map" in line or "Name: map" in line:
        print(i, line.rstrip())