#!/usr/bin/env python3
"""gcs_qt.py — Qt 地面站骨架：实时视频 + 遥测面板 + 航迹图 + 任务编辑
与机载链路：遥测 JSON@20Hz UDP（gcs_bridge）+ 视频 MJPEG UDP（osd_overlay→图传转发）。
任务/参数指令走 UDP 回传端口（Orin 端 mavlink_router 转发至飞控）。
运行：python gcs_qt.py --bind 0.0.0.0
"""
import json
import socket
import sys
import threading
import cv2
import numpy as np
from PyQt5 import QtCore, QtWidgets, QtGui
import pyqtgraph as pg


class TelemetryThread(QtCore.QThread):
    telemetry = QtCore.pyqtSignal(dict)

    def __init__(self, port=14555):
        super().__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(1.0)
        self.running = True

    def run(self):
        while self.running:
            try:
                pkt, _ = self.sock.recvfrom(4096)
                self.telemetry.emit(json.loads(pkt))
            except socket.timeout:
                continue


class VideoWidget(QtWidgets.QLabel):
    """MJPEG@UDP 视频窗（H.265 正式链路走硬件解码，地面站侧 demo 用 MJPEG 等价）"""
    def __init__(self, port=14556):
        super().__init__()
        self.setMinimumSize(640, 360)
        self.cap_thread = threading.Thread(target=self._loop, daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.frame = None
        self.cap_thread.start()

    def _loop(self):
        buf = b""
        while True:
            try:
                data, _ = self.sock.recvfrom(65536)
                buf += data
                if buf.endswith(b"\xff\xd9"):                    # JPEG EOI
                    arr = np.frombuffer(buf, dtype=np.uint8)
                    self.frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    buf = b""
            except socket.timeout:
                buf = b""

    def paintEvent(self, _):
        if self.frame is None:
            return
        rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        qimg = QtGui.QImage(rgb.data, w, h, w * 3, QtGui.QImage.Format_RGB888)
        self.setPixmap(QtGui.QPixmap.fromImage(qimg).scaled(
            self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("无人机地面站 v1.0 — PX4 + ROS2 Humble")
        self.traj = {"x": [], "y": []}
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central)

        left = QtWidgets.QVBoxLayout(); h.addLayout(left, 7)
        self.video = VideoWidget(); left.addWidget(self.video, 7)
        self.plot = pg.PlotWidget(title="飞行轨迹 (NED 北东)"); left.addWidget(self.plot, 3)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot(pen=pg.mkPen("g", width=2))

        right = QtWidgets.QVBoxLayout(); h.addLayout(right, 3)
        self.labels = {}
        for key, title in [("mode", "模式"), ("alt", "高度"), ("spd", "速度"),
                           ("bat", "电量"), ("ai", "AI 链路")]:
            box = QtWidgets.QGroupBox(title)
            self.labels[key] = QtWidgets.QLabel("--")
            v = QtWidgets.QVBoxLayout(box); v.addWidget(self.labels[key])
            right.addWidget(box)

        # 任务编辑：航点表 + 下发
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["序号", "北向m", "东向m"])
        btn_add = QtWidgets.QPushButton("加航点")
        btn_add.clicked.connect(self.add_row)
        btn_send = QtWidgets.QPushButton("下发任务")
        btn_send.clicked.connect(self.send_mission)
        right.addWidget(self.table); right.addWidget(btn_add); right.addWidget(btn_send)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.tele = TelemetryThread()
        self.tele.telemetry.connect(self.on_telemetry)
        self.tele.start()
        self.timer = QtCore.QTimer(); self.timer.timeout.connect(self.repaint_video)
        self.timer.start(33)

    def add_row(self):
        self.table.insertRow(self.table.rowCount())

    def send_mission(self):
        wps = []
        for r in range(self.table.rowCount()):
            try:
                wps.append([float(self.table.item(r, 1).text()),
                            float(self.table.item(r, 2).text())])
            except (AttributeError, ValueError):
                continue
        self.cmd_sock.sendto(json.dumps({"mission": wps}).encode(),
                             ("192.168.10.1", 14556))       # 机载指令端口

    def on_telemetry(self, pkt):
        vfr, bat, st = pkt.get("vfr", {}), pkt.get("bat", {}), pkt.get("state", {})
        ai = pkt.get("ai", {})
        self.labels["mode"].setText(str(st.get("mode", "--")))
        self.labels["alt"].setText(f"{vfr.get('alt', 0):.1f} m")
        self.labels["spd"].setText(f"{vfr.get('spd', 0):.1f} m/s")
        self.labels["bat"].setText(f"{(bat.get('pct') or 0)*100:.0f}%")
        worst = max([v.get("st", 0) for v in ai.values()] or [0])
        self.labels["ai"].setText("正常" if worst == 0 and ai else ("降级" if worst == 1 else "熔断"))
        o = pkt.get("odom", {})
        if "x" in o:
            self.traj["x"].append(o["x"]); self.traj["y"].append(o["y"])
            if len(self.traj["x"]) > 3000: self.traj["x"].pop(0); self.traj["y"].pop(0)
            self.curve.setData(self.traj["y"], self.traj["x"])   # 北向上

    def repaint_video(self):
        self.video.update()

    def closeEvent(self, e):
        self.tele.running = False; self.tele.wait(); e.accept()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())
