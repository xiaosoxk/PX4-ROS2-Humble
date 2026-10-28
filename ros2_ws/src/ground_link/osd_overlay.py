#!/usr/bin/env python3
"""osd_overlay.py — 视频流 OSD 叠加：位置/高度/速度/电量/信号/AI识别框
输入：图传前源视频(来自相机节点) + mavros 遥测 + /ai/detection
输出：/gcs/video_mjpeg (叠加后 JPEG 序列, 由图传模块硬编码 H.265 前的 OSD 层)
注意：OSD 绘制在 Orin 上用 OpenCV；帧率预算 30fps, 绘制 <3ms。
"""
import cv2, json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, BatteryState
from mavros_msgs.msg import State, VFR_HUD
from ai_msgs.msg import Detection


class OsdOverlay(Node):
    def __init__(self):
        super().__init__("osd_overlay")
        self.pub = self.create_publisher(Image, "/gcs/video_osd", 10)
        self.sub_img = self.create_subscription(Image, "/camera_front/color/image_raw",
                                                self.on_img, qos_profile_sensor_data)
        self.sub_det = self.create_subscription(Detection, "/ai/detection", self.on_det, 10)
        self.sub_state = self.create_subscription(State, "/mavros/state", self.on_state, 10)
        self.sub_vfr = self.create_subscription(VFR_HUD, "/mavros/vfr_hud", self.on_vfr, 10)
        self.sub_bat = self.create_subscription(BatteryState, "/mavros/battery", self.on_bat, 10)
        self.dets, self.state, self.vfr, self.bat = [], None, None, None

    def on_det(self, m): self.dets = m
    def on_state(self, m): self.state = m
    def on_vfr(self, m): self.vfr = m
    def on_bat(self, m): self.bat = m

    def on_img(self, img):
        cv = cv2.cvtColor(self.bridge_img(img), cv2.COLOR_RGB2BGR)
        # AI 识别框（青色）+ 类别名
        for d in [self.dets] if isinstance(self.dets, Detection) else []:
            p1 = (int(d.x_min), int(d.y_min)); p2 = (int(d.x_max), int(d.y_max))
            cv2.rectangle(cv, p1, p2, (255, 255, 0), 2)
            cv2.putText(cv, f"cls{d.cls} {d.conf:.2f}", (p1[0], max(20, p1[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        # 遥测 OSD 左上两列
        if self.vfr:
            lines = [f"ALT {self.vfr.altitude:6.1f}m  SPD {self.vfr.groundspeed:5.1f}m/s",
                     f"HDG {self.vfr.heading:5.1f}deg  THR {self.vfr.throttle:3.0f}%"]
            if self.bat:
                lines[0] += f"  BAT {self.bat.percentage*100:.0f}%"
            if self.state:
                lines[0] += f"  {self.state.mode}{'[AI]' if self.state.mode=='offboard' else ''}"
            for i, s in enumerate(lines):
                cv2.putText(cv, s, (16, 34 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(cv, s, (16, 34 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 255), 2, cv2.LINE_AA)
        ok, jpg = cv2.imencode(".jpg", cv, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            out = self.bridge_jpg(jpg)
            self.pub.publish(out)

    def bridge_img(self, m):
        from cv_bridge import CvBridge
        if not hasattr(self, "_b"): self._b = CvBridge()
        return self._b.imgmsg_to_cv2(m, "rgb")

    def bridge_jpg(self, arr):
        from cv_bridge import CvBridge
        return self._b.cv2_to_imgmsg(arr, "bgr8")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OsdOverlay())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
