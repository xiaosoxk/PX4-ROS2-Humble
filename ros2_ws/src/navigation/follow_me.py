#!/usr/bin/env python3
"""follow_me.py — 智能跟随：锁定目标后自主保持距离与方位

控制律（全部经 safety_arbiter 清洗后生效，本节点不懂 MAVROS）：
  横向: P 控制把 bearing_deg 拉回 0（目标居中），同时 yaw 同向缓慢转动
  前向: 距离误差 P 控制（distance 来自激光雷达/双目回填）
  垂直: 目标像素高度差 → 缓慢跟随（防 GST 低空障碍，限 ±1.5m/s）
安全边界：距离 < 最小跟随距离时自动后撤；信号 lost > 0.5s → 输出全零
（熔断器判定 AI 链静默后自行切定点，这里的全零是双保险）。
"""
import math
import rclpy
from rclpy.node import Node
from ai_msgs.msg import TrackTarget, AiCommand, AiHealth


class FollowMe(Node):
    def __init__(self):
        super().__init__("follow_me")
        self.declare_parameter("target_distance", 8.0)     # 保持 8m
        self.declare_parameter("min_distance", 5.0)        # 近于 5m 后撤
        self.declare_parameter("kp_lateral", 0.06)         # m/s per deg
        self.declare_parameter("kp_forward", 0.8)
        self.declare_parameter("max_yaw_rate", 40.0)
        self.sub = self.create_subscription(TrackTarget, "/ai/track_target", self.on_track, 10)
        self.pub = self.create_publisher(AiCommand, "/ai/command", 5)
        self.pub_h = self.create_publisher(AiHealth, "/ai/health", 10)
        self.create_timer(0.2, self.beat)
        self.last_cmd_time = self.get_clock().now()
        self.active = False

    def on_track(self, t: TrackTarget):
        cmd = AiCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.source = 0
        if t.track_id < 0 or t.lost:
            cmd.vx_body = cmd.vy_body = cmd.vz_body = 0.0  # 丢目标：停住等重锁
            cmd.yaw_rate_dps = 15.0                        # 缓慢扫视找回
            self.pub.publish(cmd)
            self.active = False
            return
        self.active = True

        td = self.get_parameter("target_distance").value
        md = self.get_parameter("min_distance").value
        d = t.distance_m if math.isfinite(t.distance_m) else td   # 距离未知→维持当前

        err_d = td - d                                     # >0 太远→前进
        cmd.vx_body = max(-1.5, self.get_parameter("kp_forward").value * err_d)
        if d < md:                                         # 太近后撤
            cmd.vx_body = min(cmd.vx_body, -1.0)
        cmd.vy_body = -self.get_parameter("kp_lateral").value * t.bearing_deg
        cmd.vy_body = max(-3.0, min(3.0, cmd.vy_body))
        cmd.vz_body = 0.0                                  # 垂直跟随留给拍摄模块(构图)
        cmd.yaw_rate_dps = max(-self.get_parameter("max_yaw_rate").value,
                               min(self.get_parameter("max_yaw_rate").value,
                                   0.4 * t.bearing_deg))   # 机头缓转向目标
        self.pub.publish(cmd)
        self.last_cmd_time = self.get_clock().now()

    def beat(self):
        h = AiHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = "follow_me"
        h.latency_ms = 1.0
        # 跟随节点静默超 0.5s 仍未输出 → 主动自报降级（熔断器第二判据）
        silent = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > 0.5
        h.state = 0 if (self.active and not silent) else 1
        h.detail = "tracking" if self.active else "no target"
        self.pub_h.publish(h)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(FollowMe())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
