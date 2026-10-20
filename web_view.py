#!/usr/bin/env python3
"""极简网页版 SLAM 可视化：浏览器里看点云累积地图 + 轨迹。

起法:  bash ~/run_webview.sh
看:    浏览器打开 http://<板子IP>:8000
"""
import json
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

VOX = 0.15        # 地图体素边长（米），用来去重，免得点太多
MAXPTS = 25000    # 最多显示多少点
MAXPATH = 2000    # 轨迹最多保留多少个点（用抽样的方式压缩）


class Store:
    """订阅回调往这里塞数据，HTTP 线程从这里取。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.vox = set()
        self.pts = []      # 累积地图，扁平存 [x,y,z,x,y,z,...]
        self.path = []     # 轨迹

    def add_cloud(self, msg):
        try:
            step = msg.point_step or 16
            data = bytes(msg.data)
            n = msg.width * msg.height
        except Exception:
            return
        new = []
        with self.lock:
            for i in range(n):
                base = i * step
                if base + 12 > len(data):
                    break
                x, y, z = struct.unpack_from("<fff", data, base)
                if not (-1e4 < x < 1e4 and -1e4 < y < 1e4 and -1e4 < z < 1e4):
                    continue
                key = (int(x / VOX), int(y / VOX), int(z / VOX))
                if key in self.vox:
                    continue
                self.vox.add(key)
                new.append((round(x, 2), round(y, 2), round(z, 2)))
            self.pts.extend(new)
            if len(self.pts) > MAXPTS:
                # 太满了：隔一个丢一个，重新建索引
                self.pts = self.pts[::2]
                self.vox = set(
                    (int(p[0] / VOX), int(p[1] / VOX), int(p[2] / VOX)) for p in self.pts
                )

    def add_path(self, msg):
        try:
            raw = [
                [round(p.pose.position.x, 2), round(p.pose.position.y, 2),
                 round(p.pose.position.z, 2)]
                for p in msg.poses
            ]
            # 轨迹是每帧一个点攒出来的，跑久了会有几万个。
            # 这里抽样到 MAXPATH 个，最后一个点一定保留（画到当前位置）。
            n = len(raw)
            if n > MAXPATH:
                step = max(1, n // MAXPATH)
                self.path = raw[::step]
                if self.path[-1] != raw[-1]:
                    self.path.append(raw[-1])
            else:
                self.path = raw
        except Exception:
            pass

    def snapshot(self):
        with self.lock:
            flat = []
            for p in self.pts:
                flat.extend(p)
            return {"pts": flat, "path": self.path}


store = Store()

HTML = """<!doctype html><html><head><meta charset="utf-8">
<title>SLAM 点云</title><style>
html,body{margin:0;height:100%;background:#111;color:#eee;font:14px/1.5 sans-serif;overflow:hidden}
canvas{display:block}
#hud{position:fixed;left:12px;top:12px;background:#000a;padding:8px 12px;border-radius:6px}
b{color:#6f8}</style></head><body>
<canvas id="c"></canvas>
<div id="hud">拖动=旋转 · 滚轮=缩放 · 点数 <b id="n">0</b> · 轨迹 <b id="m">0</b> 点</div>
<script>
const c=document.getElementById('c'),ctx=c.getContext('2d');
let yaw=-0.6,pitch=0.55,zoom=1,pts=[],path=[];
function resize(){c.width=innerWidth;c.height=innerHeight}
addEventListener('resize',resize);resize();
let drag=false,lx=0,ly=0;
c.addEventListener('mousedown',e=>{drag=true;lx=e.clientX;ly=e.clientY});
addEventListener('mouseup',()=>drag=false);
addEventListener('mousemove',e=>{if(!drag)return;yaw+=(e.clientX-lx)*0.008;
  pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-ly)*0.008));lx=e.clientX;ly=e.clientY});
c.addEventListener('wheel',e=>{zoom*=e.deltaY>0?0.9:1.1;e.preventDefault()},{passive:false});
function proj(x,y,z){
  const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
  const X=x*cy-y*sy, Y=(x*sy+y*cy)*cp-z*sp;
  const s=260*zoom;
  return [c.width/2+X*s, c.height/2+Y*s];
}
function draw(){
  ctx.fillStyle='#111';ctx.fillRect(0,0,c.width,c.height);
  for(let i=0;i<pts.length;i+=3){
    const q=proj(pts[i],pts[i+1],pts[i+2]);
    const t=Math.max(0,Math.min(1,(pts[i+2]+1)/3));
    ctx.fillStyle='rgb('+Math.round(70+180*t)+','+Math.round(205-130*t)+',255)';
    ctx.fillRect(q[0],q[1],1.6,1.6);
  }
  if(path.length>1){
    ctx.strokeStyle='#f33';ctx.lineWidth=2;ctx.beginPath();
    for(let i=0;i<path.length;i++){const q=proj(path[i][0],path[i][1],path[i][2]);
      i?ctx.lineTo(q[0],q[1]):ctx.moveTo(q[0],q[1]);}
    ctx.stroke();
  }
  requestAnimationFrame(draw);
}
async function poll(){
  try{const d=await (await fetch('/data')).json();
    pts=d.pts;path=d.path;
    document.getElementById('n').textContent=pts.length/3;
    document.getElementById('m').textContent=path.length;}
  catch(e){}
  setTimeout(poll,1500);
}
draw();poll();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith("/data"):
            body = json.dumps(store.snapshot(), separators=(",", ":")).encode()
            ctype = "application/json"
        else:
            body = HTML.encode()
            ctype = "text/html; charset=utf-8"
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception:
            pass

    def log_message(self, *args):
        pass


def main():
    rclpy.init()
    node = Node("web_view")
    qos = QoSProfile(depth=10)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.VOLATILE
    node.create_subscription(PointCloud2, "/cloud_registered", store.add_cloud, qos)
    node.create_subscription(Path, "/path", store.add_path, qos)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    server = ThreadingHTTPServer(("0.0.0.0", 8000), Handler)
    print("网页可视化已启动: http://0.0.0.0:8000", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
