#!/usr/bin/env python3
"""server.py — Web 网页地面站后端（单文件，仅标准库）

与机载链路：
  UDP:14555 遥测 JSON（gcs_bridge 20Hz）→ 浏览器 20Hz 轮询 /api/tele
  UDP:14556 MJPEG JPEG 流              → /video 多段流（<img> 直接显示）
  POST /api/mission {waypoints}        → UDP 转发机载 14556
  POST /api/params  {params:{k:v}}     → UDP 转发机载
  GET  /api/logs                       → 列出机载日志（/var/log/uav 挂载）
多端接入：Qt 地面站、手机 APP（同 JSON/RTSP 协议）、本 Web 端并行互不影响。
运行：python3 server.py --http 8000 --tele 14555 --video 14556 --drone 192.168.10.1
"""
import argparse
import json
import socket
import pathlib
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = pathlib.Path(__file__).parent
lock = threading.Lock()
latest_tele = None
video_frame = None
drone_addr = ("192.168.10.1", 14556)


def tele_loop(port):
    """遥测 UDP 接收线程"""
    global latest_tele
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    while True:
        try:
            pkt, _ = sock.recvfrom(4096)
            with lock:
                latest_tele = pkt.decode("utf-8", "replace")
        except OSError:
            pass


def video_loop(port):
    """视频 UDP JPEG 接收线程"""
    global video_frame
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    buf = b""
    while True:
        try:
            data, _ = sock.recvfrom(65536)
        except OSError:
            continue
        buf += data
        if buf.endswith(b"\xff\xd9"):
            with lock:
                video_frame = buf
            buf = b""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._file(ROOT / "index.html", "text/html; charset=utf-8")
        elif self.path == "/video":
            self._mjpeg()
        elif self.path == "/api/tele":
            with lock:
                raw = latest_tele
            self._json({"raw": raw})
        elif self.path == "/api/logs":
            logdir = pathlib.Path("/var/log/uav")
            files = sorted(f.name for f in logdir.glob("*")) if logdir.exists() else []
            self._json({"logs": files})
        else:
            self.send_error(404)

    def do_POST(self):
        global drone_addr
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return self.send_error(400)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.path == "/api/mission":
            sock.sendto(json.dumps({"mission": body.get("waypoints", [])}).encode(), drone_addr)
            self._json({"ok": True, "n": len(body.get("waypoints", []))})
        elif self.path == "/api/params":
            sock.sendto(json.dumps({"params": body.get("params", {})}).encode(), drone_addr)
            self._json({"ok": True})
        else:
            self.send_error(404)

    def _json(self, obj):
        data = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _file(self, path, ctype):
        try:
            data = path.read_bytes()
        except OSError:
            return self.send_error(404)
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with lock:
                    frame = video_frame
                if frame:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame + b"\r\n")
                self.wfile.flush()
        except (ConnectionAbortedError, BrokenPipeError, TimeoutError):
            pass


def main():
    global drone_addr
    ap = argparse.ArgumentParser()
    ap.add_argument("--http", type=int, default=8000)
    ap.add_argument("--tele", type=int, default=14555)
    ap.add_argument("--video", type=int, default=14556)
    ap.add_argument("--drone", default="192.168.10.1")
    a = ap.parse_args()
    drone_addr = (a.drone, 14556)

    threading.Thread(target=tele_loop, args=(a.tele,), daemon=True).start()
    threading.Thread(target=video_loop, args=(a.video,), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", a.http), Handler)
    print(f"Web 地面站: http://0.0.0.0:{a.http}  （手机/平板浏览器同地址即可接入）")
    srv.serve_forever()


if __name__ == "__main__":
    main()
