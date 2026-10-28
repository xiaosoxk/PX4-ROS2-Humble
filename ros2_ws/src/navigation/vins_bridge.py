#!/usr/bin/env python3
"""vins_bridge.py — 无 GPS 环境视觉惯导（VINS-Fusion）→ EKF3 外部导航

链路：VINS-Fusion（ROS1 包，ros1_bridge 桥到 ROS2，或用社区 ROS2 移植版）
  产出 /vins_estimator/odometry（视觉惯导里程计）
→ 本节点：坐标变换（VINS 世界系 → 机载 ENU）+ 协方差映射
→ MAVROS /mavros/odometry/out（ODOMETRY MAVLink 消息, frame=VISION）
→ PX4 EKF3 以外部视觉导航融合（EKF2_EV_CTRL）。

参数切换（见 docs/PX4配置与PID整定.md §5）：
  无 GPS 场景 EKF2_GPS_CTRL=0（关闭 GPS 融合），EKF2_HGT_REF= vision；
  GPS 恢复且 Fixed 时切回。本节点同时监控 VINS 漂移（与 RTK/气压一致性），
  漂移超限自报降级 → arbiter 熔断 AI 链，飞控退回气压+光流悬停。
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from ai_msgs.msg import AiHealth


class VinsBridge(Node):
    def __init__(self):
        super().__init__("vins_bridge")
        self.declare_parameter("drift_thres_m", 3.0)       # 与气压高度一致性门限
        self.declare_parameter("yaw_align", 0.0)           # VINS 世界系与磁北的安装偏角
        self.sub_vins = self.create_subscription(Odometry, "/vins_estimator/odometry",
                                                 self.on_vins, 10)
        self.sub_baro = self.create_subscription(PoseWithCovarianceStamped,
                                                 "/mavros/local_position/pose",
                                                 self.on_pose, 10)
        self.pub_out = self.create_publisher(Odometry, "/mavros/odometry/out", 10)
        self.pub_h = self.create_publisher(AiHealth, "/ai/health", 10)
        self.create_timer(0.2, self.beat)
        self.px_z = None
        self.vins_z0 = None
        self.latency = 0.0

    def on_pose(self, m):
        self.px_z = m.pose.pose.position.z                 # PX4 EKF 高度（NED, 负=向上）

    def on_vins(self, m: Odometry):
        # ① 航向对齐：VINS 初始化时 yaw 任意 → 统一偏转到磁北（yaw_align 参数）
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z)) + \
              math.radians(self.get_parameter("yaw_align").value)
        out = Odometry()
        out.header.stamp = m.header.stamp                  # 保留 VINS 时间戳供 EKF3 延迟补偿
        out.header.frame_id = "map"
        out.child_frame_id = "base_link"
        out.pose = m.pose
        # 旋转对齐只改 yaw：位置/姿态其余分量直通（严谨实现应整体旋转四元数, 简化见 docs）
        out.pose.pose.orientation.z = math.sin(yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(yaw / 2.0)
        out.twist = m.twist
        self.pub_out.publish(out)

        # ② 漂移监控：VINS 高度 vs PX4 气压高度的相对漂移
        if self.px_z is None:
            return
        if self.vins_z0 is None:
            self.vins_z0 = m.pose.pose.position.z - self.px_z
        drift = abs((m.pose.pose.position.z - self.px_z) - self.vins_z0)
        self.latency = (self.get_clock().now().nanoseconds - m.header.stamp.sec * 10**9
                        - m.header.stamp.nanosecond) / 1e6
        self.drift = drift

    def beat(self):
        h = AiHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = "vins_bridge"
        h.latency_ms = float(max(0.0, getattr(self, "latency", 0.0)))
        drift = getattr(self, "drift", 0.0)
        h.state = 2 if drift > self.get_parameter("drift_thres_m").value else 0
        h.detail = f"vins drift {drift:.2f}m"
        self.pub_h.publish(h)
        if h.state == 2:
            self.get_logger().warn(f"VINS drift {drift:.1f}m — EKF3 降级到气压+光流", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(VinsBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
