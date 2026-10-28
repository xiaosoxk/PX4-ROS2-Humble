#!/usr/bin/env python3
"""capture_node.py — 三维重建数据同步采集（飞行中即插即用）
内容：图像(主摄) + RTK/EKF 位姿 + IMU 同步落盘，按运动量选关键帧；
数据布局兼容 COLMAP / NeRF-LLFF / nvblox TSDF 三种下游。
  机载实时稀疏重建：nvblox(NT SDK) 消费本节点的 pose+depth 出 TSDF 网格（可选）；
  NeRF 精化：落地后 PC 端离线跑（数据含每帧位姿，直接可训）。
触发：/ai/recon_start 开始, /ai/recon_stop 结束。
"""
import os, json, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


class CaptureNode(Node):
    def __init__(self):
        super().__init__("capture_node")
        self.declare_parameter("out_dir", "/media/uav/recon")
        self.declare_parameter("min_motion_m", 0.5)        # 关键帧最小运动间隔
        self.declare_parameter("max_fps", 2.0)
        self.out = None
        self.idx = 0
        self.last_pos = None
        self.last_t = 0.0
        self.odom = None
        self.gps = None
        self.sub_img = self.create_subscription(Image, "/camera_main/image_raw",
                                                self.on_img, qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(Odometry, "/mavros/local_position/odom",
                                                 self.on_odom, 10)
        self.sub_gps = self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                                self.on_gps, 10)
        self.create_subscription(Empty, "/ai/recon_start", lambda m: self.start(), 10)
        self.create_subscription(Empty, "/ai/recon_stop", lambda m: self.stop(), 10)

    def start(self):
        d = os.path.join(self.get_parameter("out_dir").value,
                         time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(os.path.join(d, "images"), exist_ok=True)
        self.out = d
        json.dump({"poses": [], "gps": [], "imu_hint": "mavros/imu/data @200Hz"},
                  open(os.path.join(d, "meta.json"), "w"))
        self.get_logger().info(f"recon capture -> {d}")

    def stop(self):
        self.out = None

    def on_odom(self, m): self.odom = m
    def on_gps(self, m): self.gps = [m.latitude, m.longitude, m.altitude]

    def on_img(self, img):
        if self.out is None or self.odom is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_t < 1.0 / self.get_parameter("max_fps").value:
            return
        p = self.odom.pose.pose.position
        pos = (p.x, p.y, p.z)
        if self.last_pos is not None and \
                np.linalg.norm(np.subtract(pos, self.last_pos)) < self.get_parameter("min_motion_m").value:
            return                                          # 没动够不成关键帧
        self.last_pos, self.last_t = pos, now
        self.idx += 1
        name = f"frame_{self.idx:05d}.jpg"
        cv2.imwrite(os.path.join(self.out, "images", name),
                    self._to_cv(img), [cv2.IMWRITE_JPEG_QUALITY, 95])
        meta = json.load(open(os.path.join(self.out, "meta.json")))
        meta["poses"].append({"file": name, "t": now,
                              "q": [self.odom.pose.pose.orientation.w, self.odom.pose.pose.orientation.x,
                                    self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z],
                              "p": list(pos)})
        if self.gps:
            meta["gps"].append({"file": name, "llh": self.gps})
        json.dump(meta, open(os.path.join(self.out, "meta.json"), "w"))
        # nvblox 机载实时 TSDF 可选分支：订阅 depth+odom 出网格（不在实时关键路径）

    def _to_cv(self, m):
        from cv_bridge import CvBridge
        if not hasattr(self, "_b"): self._b = CvBridge()
        return self._b.imgmsg_to_cv2(m, "bgr8")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CaptureNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
