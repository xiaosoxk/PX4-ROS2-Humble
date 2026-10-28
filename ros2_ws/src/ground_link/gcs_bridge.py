#!/usr/bin/env python3
"""gcs_bridge.py — 地面站链路：把遥测聚合成 JSON@20Hz UDP 广播 + 接收任务指令
链路复用：OFDM 图传下行带宽里划出 32kbps 给遥测(JSON 二进制精简编码可再省)；
上行 8kbps 任务/参数指令。与图传视频流在同一条射频链路上做频内复用。
"""
import json
import socket
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, VFR_HUD
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from ai_msgs.msg import AiHealth, Detection


class GcsBridge(Node):
    def __init__(self):
        super().__init__("gcs_bridge")
        self.declare_parameter("gcs_ips", ["192.168.10.10"])  # 多端接入: Qt/手机APP/Web 并行
        self.declare_parameter("gcs_port", 14555)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.state, self.vfr, self.bat, self.odom, self.ai = {}, {}, {}, {}, {}
        for t, cb in [("/mavros/state", State, self.s_state),
                      ("/mavros/vfr_hud", VFR_HUD, self.s_vfr),
                      ("/mavros/battery", BatteryState, self.s_bat),
                      ("/mavros/local_position/odom", Odometry, self.s_odom),
                      ("/ai/health", AiHealth, self.s_ai)]:
            self.create_subscription(t[1], t[0], cb, 10)
        self.create_timer(0.05, self.push)                  # 20Hz
        self.det_sub = self.create_subscription(Detection, "/ai/detection", self.s_det, 10)

    def s_state(self, m): self.state = {"mode": m.mode, "armed": m.armed}
    def s_vfr(self, m): self.vfr = {"alt": m.altitude, "spd": m.groundspeed, "hdg": m.heading}
    def s_bat(self, m): self.bat = {"pct": m.percentage, "v": m.voltage}
    def s_odom(self, m):
        p = m.pose.pose.position
        self.odom = {"x": p.x, "y": p.y, "z": p.z}
    def s_ai(self, m): self.ai[m.node_name] = {"ms": m.latency_ms, "st": m.state}
    def s_det(self, m):
        self.det = {"cls": m.cls, "conf": round(m.conf, 2), "brg": round(m.bearing_deg, 1)}

    def push(self):
        pkt = {"t": self.get_clock().now().nanoseconds, "state": self.state,
               "vfr": self.vfr, "bat": self.bat, "odom": self.odom, "ai": self.ai,
               "det": getattr(self, "det", None)}
        data = json.dumps(pkt).encode()
        for ip in self.get_parameter("gcs_ips").value:        # 多端广播
            try:
                self.sock.sendto(data, (ip, self.get_parameter("gcs_port").value))
            except OSError:
                pass


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GcsBridge())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
