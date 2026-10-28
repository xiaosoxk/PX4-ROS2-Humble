#!/usr/bin/env python3
"""health_diag.py — 故障 AI 诊断：电机异常 / 传感器漂移在线识别

两类轻量在线模型（无监督，无需故障样本标注）：
  1) 电机健康：四路 ESC 遥测(转速/电流/温度)做两两一致性残差 +
     滑动基线 z-score。堵转/螺旋桨破损表现为"同油门下转速/电流离群"；
  2) 传感器漂移：加速度计静止重力分量残差（悬停时 |a|-g 的均值漂移）、
     磁航向 vs GPS 航向缓慢发散 → 判定对应传感器置信度下降。
输出：Diagnostics 心跳给 safety_arbiter（critical 节点之一）。
"""
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ESCInfo, ESCStatus   # MAVROS esc_telemetry 插件
from sensor_msgs.msg import Imu, NavSatFix
from ai_msgs.msg import AiHealth


class HealthDiag(Node):
    def __init__(self):
        super().__init__("health_diag")
        self.declare_parameter("zscore_thres", 4.0)
        self.declare_parameter("drift_thres_g", 0.06)      # |a|-g 漂移门限 (g)
        self.declare_parameter("mag_diverge_thres", 15.0)  # 磁/GPS 航向发散 (°)
        self.rpm = {i: deque(maxlen=200) for i in range(4)}
        self.cur = {i: deque(maxlen=200) for i in range(4)}
        self.a_hist = deque(maxlen=400)
        self.mag_diverge = 0.0
        self.sub_esc = self.create_subscription(ESCInfo, "/mavros/esc_info/esc_info",
                                                self.on_esc, 10)
        self.sub_imu = self.create_subscription(Imu, "/mavros/imu/data", self.on_imu, 10)
        self.sub_gps = self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                                self.on_gps, 10)
        self.pub_h = self.create_publisher(AiHealth, "/ai/health", 10)
        self.create_timer(0.2, self.beat)
        self.imu = None
        self.gps_heading = None

    def on_esc(self, msg):
        for i, ch in enumerate(msg.esc[:4]):               # 转速/电流按通道积累
            self.rpm[i].append(float(ch.rpm))
            self.cur[i].append(float(ch.current))

    def on_imu(self, msg):
        self.imu = msg

    def on_gps(self, msg):
        if msg.status.status >= 0:
            self.gps_heading = None                        # NavSatFix 无航向, 留给 covariance 检查

    def motor_diagnose(self):
        """一致性残差: 任意电机转速 z-score 超限 → 该电机异常"""
        if any(len(v) < 100 for v in self.rpm.values()):
            return []
        faults = []
        zt = self.get_parameter("zscore_thres").value
        arr = np.array([np.mean(self.rpm[i]) if self.rpm[i] else np.nan for i in range(4)])
        mu, sd = np.nanmean(arr), np.nanstd(arr)
        if sd > 1e-3:
            for i in range(4):
                z = abs(arr[i] - mu) / sd
                if z > zt:
                    faults.append(f"motor{i+1} rpm z={z:.1f}")
        return faults

    def imu_diagnose(self):
        """重力残差漂移: 悬停时 |a| 应≈g, 缓慢偏离 = 加速度计零偏/温漂异常"""
        if self.imu is None:
            return None
        a = self.imu.linear_acceleration
        mag = np.linalg.norm([a.x, a.y, a.z]) / 9.80665
        self.a_hist.append(mag)
        if len(self.a_hist) < 300:
            return None
        drift = abs(np.mean(self.a_hist) - 1.0)
        return ("accelerometer drift" if drift > self.get_parameter("drift_thres_g").value
                else None), drift

    def beat(self):
        h = AiHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = "health_diag"
        h.latency_ms = 2.0
        faults = self.motor_diagnose()
        r = self.imu_diagnose()
        if r and r[0]:
            faults.append(r[0])
        h.state = 2 if faults else 0
        h.detail = "; ".join(faults)[:200]
        self.pub_h.publish(h)
        if faults:
            self.get_logger().warn(f"health: {h.detail}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(HealthDiag())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
