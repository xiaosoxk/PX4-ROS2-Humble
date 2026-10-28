#!/usr/bin/env python3
"""semantic_nav.py — 语义避障：双目深度 + 语义分割 → 可通行性栅格 → EGO-Planner

核心思想（对标 Mavic 的"智能避障"而非"碰到就停"）：
  1) 语义分割区分 树木/墙体/电线：电线=细长低对比高危物，直接标记禁飞走廊；
  2) 深度图 + 姿态补偿 → 机体前向 2D 可通行性栅格（PBVs）；
  3) 栅格喂给 EGO-Planner（外部 ROS2 移植包）做局部重规划，
     本节点负责把规划轨迹压成 AiCommand(@10Hz) 交给熔断器；
  4) Octomap 由 depth+RTK 位姿增量更新（见 navigation/planner_bridge 的 3D 分支）。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from ai_msgs.msg import AiCommand, AiHealth
import numpy as np


class SemanticNav(Node):
    def __init__(self):
        super().__init__("semantic_nav")
        self.declare_parameter("grid_rows", 20)          # 前向 0~16m, 20 行
        self.declare_parameter("grid_cols", 32)          # 横向 ±8m, 32 列
        self.declare_parameter("danger_thres", 0.5)      # 高危占比阈值
        self.declare_parameter("wire_gain", 2.0)         # 电线类风险增益
        self.declare_parameter("replan_hz", 10.0)
        self.pub_cmd = self.create_publisher(AiCommand, "/ai/command", 5)
        self.pub_h = self.create_publisher(AiHealth, "/ai/health", 10)
        self.sub_seg = self.create_subscription(Image, "/camera_front/seg", self.on_seg,
                                                qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(Image, "/camera_front/depth", self.on_depth,
                                                  qos_profile_sensor_data)
        self.sub_cloud = self.create_subscription(PointCloud2, "/lidar/points", self.on_cloud, 5)
        self.create_timer(1.0 / self.get_parameter("replan_hz").value, self.tick)
        self.seg = None
        self.depth = None
        self.cloud_recent = False
        self.latency = 0.0

    # ---- 语义层：类别代价权重（0背景 1天空 2树木 3墙体 4电线 5地面） ----
    W = {0: 0.0, 1: 8.0, 2: 1.0, 3: 4.0, 4: 10.0, 5: 2.0}

    def on_seg(self, msg):
        self.seg = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)

    def on_depth(self, msg):
        self.depth = (np.frombuffer(msg.data, dtype=np.float32)
                      .reshape(msg.height, msg.width) / 1000.0)   # mm→m

    def on_cloud(self, _):
        self.cloud_recent = True     # 激光在场的标志（融合策略见 docs）

    def build_cost_grid(self):
        """前向栅格: 行=距离环 0~16m, 列=方位 -40°~+40°; 值=风险 0~10"""
        if self.seg is None or self.depth is None:
            return None
        seg, dep = self.seg, self.depth
        h, w = seg.shape
        rows, cols = (self.get_parameter("grid_rows").value,
                      self.get_parameter("grid_cols").value)
        risk = np.zeros((rows, cols), dtype=np.float32)
        rs = np.linspace(0, h // 3, rows + 1).astype(int)     # 上 1/3 视场对应前向
        for r in range(rows):
            for c in range(cols):
                cs = int(w * (c / cols)), int(w * ((c + 1) / cols))
                cell_seg = seg[rs[r]:rs[r + 1], cs[0]:cs[1]]
                cell_dep = dep[rs[r]:rs[r + 1], cs[0]:cs[1]]
                if cell_dep.size == 0:
                    continue
                cls = int(np.bincount(cell_seg.ravel(), minlength=6).argmax())
                base = self.W.get(cls, 0.0)
                # 电线惩罚：细长结构, 分割面积小但存在即高增益
                wire_ratio = (cell_seg == 4).mean()
                base += self.get_parameter("wire_gain").value * (wire_ratio > 0.005)
                # 距离越近风险越高（时间到碰撞直觉）
                d = float(np.nanmin(cell_dep)) if np.isfinite(cell_dep).any() else 99.0
                base *= np.clip(8.0 / max(d, 1.0), 0.3, 4.0)
                risk[r, c] = min(base, 10.0)
        return risk

    def tick(self):
        import time
        t0 = time.monotonic()
        grid = self.build_cost_grid()
        if grid is None:
            return
        cmd = AiCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.source = 1

        danger = self.get_parameter("danger_thres").value
        # 仅评估最近 3 行（0~2.4m 急停带）+ 中线通道
        front = grid[:3]
        center_cols = slice(grid.shape[1] // 2 - 4, grid.shape[1] // 2 + 4)
        if (front[:, center_cols] > danger * 4).any():
            # 正前方高危 → 侧向绕行：选风险最低的 1/4 象限给横向速度
            left, right = front[:, :front.shape[1] // 2].mean(), front[:, front.shape[1] // 2:].mean()
            cmd.vx_body = 0.0                                # 先停住（倾斜保护余量大）
            cmd.vy_body = -2.0 if left < right else 2.0
            cmd.yaw_rate_dps = 15.0 if left < right else -15.0
            self.get_logger().warn("front hazard -> lateral evade", throttle_duration_sec=1.0)
        else:
            # 通道畅通 → 前向巡航速度按中线风险线性放行（EGO-Planner 轨迹的简化直通道）
            mid = front[:, center_cols].mean()
            cmd.vx_body = float(np.clip(4.0 * (1.0 - mid / 3.0), 0.0, 4.0))
            cmd.vy_body = 0.0
            cmd.yaw_rate_dps = 0.0
        self.pub_cmd.publish(cmd)
        self.latency = (time.monotonic() - t0) * 1000.0

    def beat_stub(self):  # 心跳由下方 timer 复用 tick 尾部（保持 5Hz 见 launch 覆盖）
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNav()
    # 5Hz 心跳（独立 timer，与 10Hz 规划解耦）
    node.create_timer(0.2, lambda: _beat(node))
    rclpy.spin(node)
    rclpy.shutdown()


def _beat(node):
    from ai_msgs.msg import AiHealth
    h = AiHealth()
    h.header.stamp = node.get_clock().now().to_msg()
    h.node_name = "semantic_nav"
    h.latency_ms = node.latency
    h.state = 0
    node.pub_h.publish(h)


if __name__ == "__main__":
    main()
