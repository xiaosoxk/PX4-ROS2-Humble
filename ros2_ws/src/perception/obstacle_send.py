#!/usr/bin/env python3
"""obstacle_send.py — 五向感知 → PX4 碰撞预防（障碍物制动）

PX4 自带 Collision Prevention：消费 OBSTACLE_DISTANCE MAVLink 消息，
参数 CP_DIST（最小安全距离）/CP_GO_NO_DATA（无数据区是否视为障碍）生效——
在位置模式下由飞控位置环直接制动，属于**飞控域兜底**，与本工程 AI 绕行
（semantic_nav）互补：AI 失效时碰撞预防仍然工作。

本节点把五向双目/激光雷达的距离-角度分布压成 MAVROS ObstacleDistance
（前向 0~40° 扇区映射，其余方向按组上报），10Hz 发布到
/mavros/obstacle/send。
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Image
from mavros_msgs.msg import ObstacleDistance


class ObstacleSend(Node):
    N_SECTOR = 36                       # 每 10° 一扇区, 覆盖 -180°~180°
    MAX_CM = 2000                       # 上报上限 20m（OBSTACLE_DISTANCE 单位 cm）

    def __init__(self):
        super().__init__("obstacle_send")
        self.declare_parameter("cp_dist_m", 2.5)         # 与飞控 CP_DIST 保持一致
        # 五向距离源：前向激光（优先）+ 五向双目最近距离表
        self.sub_scan = self.create_subscription(LaserScan, "/lidar/scan", self.on_scan, 5)
        self.sub_front = self.create_subscription(Image, "/camera_front/depth", self.on_depth, 2)
        self.sub_down = self.create_subscription(Image, "/camera_down/depth", self.on_depth, 2)
        self.pub = self.create_publisher(ObstacleDistance, "/mavros/obstacle/send", 10)
        self.create_timer(0.1, self.publish_10hz)        # 10Hz
        self.front_bins = np.full(self.N_SECTOR, self.MAX_CM, dtype=np.float32)
        self.down_min_cm = self.MAX_CM

    def on_scan(self, scan: LaserScan):
        """激光 → 扇区最近距离（cm）"""
        if scan.range_max <= scan.range_min:
            return
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            ang = scan.angle_min + i * scan.angle_increment    # rad, 0=前
            deg = math.degrees(ang)
            k = int(((deg + 180.0) % 360.0) // 10.0) % self.N_SECTOR
            cm = min(self.MAX_CM, int(r * 100.0))
            if cm < self.front_bins[k]:
                self.front_bins[k] = cm

    def on_depth(self, msg: Image):
        """双目深度 → 前向扇区补充（激光视野外/低反射率障碍）"""
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width) / 1000.0
        valid = d[np.isfinite(d) & (d > 0.2) & (d < 20.0)]
        if valid.size:
            cm = int(np.percentile(valid, 5) * 100)            # 5 分位 = 最近障碍
            k0 = int(((0 + 180.0) % 360.0) // 10.0)            # 该相机方位的扇区
            for k in (k0 - 1, k0, k0 + 1):
                self.front_bins[k % self.N_SECTOR] = min(self.front_bins[k % self.N_SECTOR], cm)

    def publish_10hz(self):
        m = ObstacleDistance()
        m.header.stamp = self.get_clock().now().to_msg()
        m.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        m.frame_id = "body"                                     # 机体坐标系
        m.min_distance = 30                                     # 0.3m
        m.max_distance = self.MAX_CM
        m.angle_increment = math.radians(10.0)
        m.angle_offset = math.radians(-180.0)                   # 第0扇区 = -180°
        distances = np.clip(self.front_bins, 0, self.MAX_CM).astype(np.uint16)
        m.distances = list(distances)
        m.increment_f = 0.0
        self.pub.publish(m)
        # 扇区衰减：无新数据的方向逐渐回到 MAX，防止旧数据永久制动
        self.front_bins = np.minimum(self.front_bins + 25, self.MAX_CM)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ObstacleSend())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
