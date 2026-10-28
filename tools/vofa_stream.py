#!/usr/bin/env python3
"""vofa_stream.py — Vofa+ justfloat 调试流输出
把任意 ROS2 话题数值打成 Vofa+ justfloat 帧（[f32..., 0x00,0x00,0x80,0x7f] 尾标），
UDP/串口均可。Vofa+ 端选 justfloat 即可看实时波形。
用法：python vofa_stream.py --udp 127.0.0.1:1347 --topic /ai/health
"""
import argparse
import struct
import socket
import serial
import rclpy
from rclpy.node import Node
from ai_msgs.msg import AiHealth

class VofaStream(Node):
    def __init__(self, udp, topic):
        super().__init__("vofa_stream")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        addr = udp.split(":")
        self.dst = (addr[0], int(addr[1]))
        self.ser = None
        self.create_subscription(AiHealth, topic, self.on_h, 20)

    def on_h(self, h):
        payload = struct.pack("<fff", h.latency_ms, float(h.state), 0.0) + b"\x00\x00\x80\x7f"
        self.sock.sendto(payload, self.dst)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--udp", default="127.0.0.1:1347")
    ap.add_argument("--topic", default="/ai/health")
    a = ap.parse_args()
    rclpy.init()
    rclpy.spin(VofaStream(a.udp, a.topic))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
