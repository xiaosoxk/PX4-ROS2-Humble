#!/usr/bin/env python3
"""tracker.py — 轻量目标跟踪节点：检测 → 稳定 ID → TrackTarget

无重量级依赖的 IoU+质心关联跟踪（毕设体量够用，论文里可升级 ByteTrack，
接口不变）。云台对准、智能跟随都消费这里的 TrackTarget。
"""
import rclpy
from rclpy.node import Node
from ai_msgs.msg import Detection, TrackTarget, AiHealth
import numpy as np


class Track:
    __slots__ = ("id", "cls", "box", "bearing", "miss", "conf")

    def __init__(self, tid, cls, det):
        self.id, self.cls = tid, cls
        self.box = (det.x_min, det.y_min, det.x_max, det.y_max)
        self.bearing = det.bearing_deg
        self.conf = det.conf
        self.miss = 0


class TrackerNode(Node):
    def __init__(self):
        super().__init__("tracker")
        self.declare_parameter("iou_thres", 0.3)
        self.declare_parameter("max_miss", 8)          # 丢 8 帧(~0.4s)判死
        self.declare_parameter("lock_cls", -1)         # -1=自动锁最大目标, 否则指定类别
        self.tracks, self.next_id = [], 1
        self.locked_id = -1
        self.pub = self.create_publisher(TrackTarget, "/ai/track_target", 10)
        self.pub_h = self.create_publisher(AiHealth, "/ai/health", 10)
        self.sub = self.create_subscription(Detection, "/ai/detection", self.on_det, 10)
        self.create_timer(0.2, self.beat)

    def iou(self, a, b):
        x1, y1 = max(a[0], b[0]), max(a[1], b[1])
        x2, y2 = min(a[2], b[2]), min(a[3], b[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
        return inter / ua if ua > 0 else 0.0

    def on_det(self, det: Detection):
        # 1) 关联：与现有轨迹做 IoU 匹配
        best_t, best_i = None, self.get_parameter("iou_thres").value
        for t in self.tracks:
            v = self.iou(t.box, (det.x_min, det.y_min, det.x_max, det.y_max))
            if v > best_i:
                best_t, best_i = t, v
        if best_t:
            best_t.box = (det.x_min, det.y_min, det.x_max, det.y_max)
            best_t.bearing, best_t.conf, best_t.miss = det.bearing_deg, det.conf, 0
        else:
            self.tracks.append(Track(self.next_id, det.cls, det))
            self.next_id += 1

        # 2) 锁定策略：自动模式取"面积最大"的轨迹作为主目标
        lock_cls = self.get_parameter("lock_cls").value
        cands = [t for t in self.tracks if lock_cls < 0 or t.cls == lock_cls]
        if cands:
            main = max(cands, key=lambda t: (t.box[2] - t.box[0]) * (t.box[3] - t.box[1]))
            self.locked_id = main.id

        # 3) 发布锁定目标的 TrackTarget（miss 期间用最后一次状态，标记 lost 渐进）
        out = TrackTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        for t in self.tracks:
            if t.id == self.locked_id:
                out.track_id, out.cls = t.id, t.cls
                out.bearing_deg, out.conf = t.bearing, t.conf
                out.distance_m = float("nan")       # 由 semantic_nav 用深度图回填
                out.lost = t.miss > 4
                self.pub.publish(out)
                return
        out.track_id, out.lost = -1, True           # 无锁定目标
        self.pub.publish(out)

    def beat(self):
        # 老化：丢帧计数 +1，超限删除
        th = self.get_parameter("max_miss").value
        self.tracks = [t for t in self.tracks if (t := t)] and [
            t for t in self.tracks if not self._age(t, th)]
        h = AiHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = "tracker"
        h.latency_ms = 1.0                          # 纯内存运算，亚毫秒
        h.state = 0
        self.pub_h.publish(h)

    @staticmethod
    def _age(t, th):
        t.miss += 1
        return t.miss > th


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TrackerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
