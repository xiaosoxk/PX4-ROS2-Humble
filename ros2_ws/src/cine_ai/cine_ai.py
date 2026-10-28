#!/usr/bin/env python3
"""cine_ai.py — 智能拍摄 AI：自动构图 / 环绕运镜 / 大师镜头轨迹

大师镜头 = 原语序列状态机（对标 DJI MasterShots 的机载简化版）：
  冲天 PULL_UP → 渐远 PULL_BACK → 环绕 ORBIT → 俯拍 SPOT_HOVER
每个原语输出 AiCommand（经熔断器），原语结束自动切下一个；
构图模块持续把主体放到三分线上（云台/速度双通道微调）。
"""
import math
import rclpy
from rclpy.node import Node
from ai_msgs.msg import TrackTarget, AiCommand, AiHealth
from std_msgs.msg import Empty, Float32MultiArray


class CineAi(Node):
    SEQUENCE = ["PULL_UP", "PULL_BACK", "ORBIT", "SPOT_HOVER"]
    DURATION = {"PULL_UP": 5.0, "PULL_BACK": 6.0, "ORBIT": 10.0, "SPOT_HOVER": 4.0}

    def __init__(self):
        super().__init__("cine_ai")
        self.declare_parameter("orbit_radius", 10.0)
        self.declare_parameter("orbit_speed_dps", 20.0)
        self.declare_parameter("timelapse_interval_s", 5.0)
        self.declare_parameter("compose_enable", True)     # 自动构图: 主体上三分线
        self.running = False
        self.timelapse = False
        self.last_capture = 0.0
        self.idx = 0
        self.t0 = self.get_clock().now()
        self.orbit_dir = 1.0
        self.target = None
        self.sub_t = self.create_subscription(TrackTarget, "/ai/track_target", self.on_track, 10)
        self.pub = self.create_publisher(AiCommand, "/ai/command", 5)
        self.pub_h = self.create_publisher(AiHealth, "/ai/health", 10)
        # 延时摄影快门触发 + 构图偏移(给 gimbal_ctrl 叠加)
        self.pub_cap = self.create_publisher(Empty, "/ai/capture_trigger", 10)
        self.pub_cmp = self.create_publisher(Float32MultiArray, "/ai/compose_offset", 10)
        self.create_subscription(Empty, "/ai/timelapse_start", lambda m: self.start_timelapse(), 10)
        self.create_subscription(Empty, "/ai/timelapse_stop", lambda m: self.stop_timelapse(), 10)
        self.create_timer(0.2, self.tick)

    def start(self):
        """由地面站/任务触发（ROS2 service 亦可，这里用话题触发保持解耦）"""
        self.running, self.idx = True, 0
        self.t0 = self.get_clock().now()
        self.get_logger().info("MasterShot sequence started")

    def start_timelapse(self):
        self.timelapse, self.last_capture = True, 0.0
        self.get_logger().info("timelapse mode: hover + interval capture")

    def stop_timelapse(self):
        self.timelapse = False

    def on_track(self, t: TrackTarget):
        self.target = t

    def compose(self):
        """自动构图: 主体从画面中心偏到上三分线交点(水平 1/3, 垂直 1/3)
        云台水平 FOV 90°/垂直 60° → 偏移 = ±FOV/6; 主体在左半画面则往右偏"""
        if not self.get_parameter("compose_enable").value or self.target is None:
            self.pub_cmp.publish(Float32MultiArray(data=[0.0, 0.0]))
            return
        fov_h, fov_v = 90.0, 60.0
        yaw_off = -fov_h / 6.0 if self.target.bearing_deg > 0 else fov_h / 6.0
        pitch_off = fov_v / 12.0                            # 主体略放低, 天空留白
        self.pub_cmp.publish(Float32MultiArray(data=[yaw_off, pitch_off]))

    def tick(self):
        h = AiHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = "cine_ai"
        h.latency_ms = 1.0
        h.state = 0
        self.pub_h.publish(h)

        # ---- 延时摄影模式：定点悬停 + 间隔快门（与大师镜头互斥） ----
        if self.timelapse:
            self.pub.publish(self.zero_cmd())               # 位置保持
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self.last_capture >= self.get_parameter("timelapse_interval_s").value:
                self.last_capture = now
                self.pub_cap.publish(Empty())               # 相机节点订阅此话题拍帧
            return

        if not self.running:
            self.compose()                                  # 非运镜时也持续构图
            return
        self.compose()
        elapsed = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        phase = self.SEQUENCE[self.idx]
        if elapsed > self.DURATION[phase]:
            self.idx += 1
            self.t0 = self.get_clock().now()
            if self.idx >= len(self.SEQUENCE):                 # 一轮完成 → 悬停收尾
                self.running = False
                self.pub.publish(self.zero_cmd())
                self.get_logger().info("MasterShot done, hovering")
                return
            phase, elapsed = self.SEQUENCE[self.idx], 0.0

        cmd = self.zero_cmd()
        cmd.source = 2
        if phase == "PULL_UP":                                 # 冲天：上升+微退+俯视
            cmd.vz_body = 1.2
            cmd.vx_body = -0.5
        elif phase == "PULL_BACK":                             # 渐远：直线后退
            cmd.vx_body = -1.8
        elif phase == "ORBIT":                                 # 环绕：切向速度+机头指向目标
            w = math.radians(self.get_parameter("orbit_speed_dps").value)
            R = self.get_parameter("orbit_radius").value
            cmd.vy_body = self.orbit_dir * w * R               # 切向
            cmd.vx_body = 0.2                                  # 微外推平衡
            cmd.yaw_rate_dps = self.orbit_dir * math.degrees(w)
        elif phase == "SPOT_HOVER":                            # 高位悬停俯拍
            cmd.vx_body = cmd.vy_body = cmd.vz_body = 0.0
        self.pub.publish(cmd)

    @staticmethod
    def zero_cmd():
        c = AiCommand()
        c.source = 2
        return c


def main(args=None):
    from std_msgs.msg import Empty
    rclpy.init(args=args)
    node = CineAi()
    # 地面站触发：收到一次 /ai/master_shot_start 就开跑（示例用话题，宜换 service）
    node.create_subscription(Empty, "/ai/master_shot_start", lambda m: node.start(), 10)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
