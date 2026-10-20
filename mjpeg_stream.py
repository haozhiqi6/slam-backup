#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Berxel 相机 -> 浏览器 MJPEG 视频流
# 订阅 /color/color_raw 与 /depth/depth_raw, 浏览器 http://<IP>:8000/ 直接看
import io, time, threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

import numpy as np
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

PORT, FPS_CAP, JPEG_Q = 8000, 15, 62
COLOR, DEPTH = "/color/color_raw", "/depth/depth_raw"

frames, stamp = {}, {}
lock = threading.Lock()

class Relay(Node):
    def __init__(self):
        super().__init__("mjpeg_relay")
        self._ema = None
        self.create_subscription(Image, COLOR, self.cb_color, 5)
        self.create_subscription(Image, DEPTH, self.cb_depth, 5)

    def cb_color(self, m):
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        with lock:
            frames[COLOR] = a
            self.cnt_c = getattr(self, chr(99)+chr(110)+chr(116)+chr(95)+chr(99), 0)+1
            if self.cnt_c % 30 == 0: print("color 收到", self.cnt_c, "帧", flush=True)
            stamp[COLOR] = time.time()

    def cb_depth(self, m):
        u16 = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.uint16)
        p1, p99 = int(np.percentile(u16, 1)), int(np.percentile(u16, 99))
        if p99 > p1:
            if self._ema is None:
                self._ema = (float(p1), float(p99))
            else:
                self._ema = (self._ema[0]*0.7 + p1*0.3, self._ema[1]*0.7 + p99*0.3)
        lo, hi = self._ema if self._ema is not None else (0.0, 65535.0)
        g = np.clip((u16.astype(np.float32) - lo) * 255.0 / max(hi - lo, 1), 0, 255).astype(np.uint8)
        with lock:
            frames[DEPTH] = g
            stamp[DEPTH] = time.time()

def to_jpeg(topic):
    with lock:
        a = frames.get(topic)
    if a is None:
        return None
    im = PILImage.fromarray(a, "RGB" if a.ndim == 3 else "L")
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=JPEG_Q)
    return buf.getvalue()

TOPIC_MAP = {"/color": COLOR, "/depth": DEPTH}

def stream(handler, path):
    topic = TOPIC_MAP.get(path)
    if topic is None:
        return
    handler.send_response(200)
    handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=berxel")
    handler.send_header("Cache-Control", "no-cache")
    handler.end_headers()
    sent, last_send = 0, 0.0
    try:
        while True:
            with lock:
                t, f = stamp.get(topic, 0), frames.get(topic)
            if f is not None and t != sent and time.time() - last_send >= 1.0/FPS_CAP:
                jpg = to_jpeg(topic)
                if jpg:
                    handler.wfile.write(b"--berxel\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                        str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    handler.wfile.flush()
                    sent, last_send = t, time.time()
            time.sleep(0.015)
    except Exception:
        pass  # 浏览器断开连接

HTML = """<!DOCTYPE html><html><head><meta charset="utf-8"><title>Berxel 相机</title>
<style>body{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:16px}
img{max-width:49vw;border:1px solid #555;background:#000} h1{font-size:15px}
.w{display:flex;gap:6px;flex-wrap:wrap} .c{text-align:center}</style></head>
<body><h1>Berxel iHawk100Q -- color / depth</h1><div class="w">
<div class="c"><img src="/color"><br>Color <span style="color:#888">/color/color_raw</span></div>
<div class="c"><img src="/depth"><br>Depth <span style="color:#888">/depth/depth_raw</span></div>
</div></body></html>"""

class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ("/color", "/depth"):
            stream(self, self.path)
        else:
            self.send_response(404)
            self.end_headers()

def main():
    rclpy.init()
    node = Relay()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    print("MJPEG 服务: http://0.0.0.0:%d/  彩色 /color, 深度 /depth" % PORT)
    ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever()

if __name__ == "__main__":
    main()
