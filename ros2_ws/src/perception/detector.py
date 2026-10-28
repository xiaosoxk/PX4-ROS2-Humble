#!/usr/bin/env python3
"""detector.py — YOLOv8s INT8 目标检测节点（Orin NX NPU/GPU 硬件加速）

管线：图像 → YOLOv8s(TensorRT INT8) → 感兴趣类别过滤 → Detection + 心跳
安全约定：本节点不碰任何控制话题，只发布感知结果与 AiHealth（5Hz 心跳），
健康状态由 safety_arbiter 审计；推理延迟超阈值会被熔断降级。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ai_msgs.msg import Detection, AiHealth
from cv_bridge import CvBridge
import numpy as np
import time

# COCO 类别 → 项目类别映射：0行人(粗粒度含骑行者) 1车辆 2船只 3动物
CLASS_MAP = {
    0: 0, 1: 0,                          # person / bicycle -> 行人
    2: 1, 3: 1, 5: 1, 7: 1,              # car / motorcycle / bus / truck -> 车辆
    8: 2,                                # boat -> 船只
    14: 3, 15: 3, 16: 3, 17: 3, 18: 3, 19: 3, 20: 3, 23: 3,   # 鸟/猫/狗/马/羊/牛/熊/长颈鹿
}
CLS_NAME = ["行人", "车辆", "船只", "动物"]


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.declare_parameter("engine", "/opt/models/yolov8s_int8.engine")
        self.declare_parameter("conf_thres", 0.45)
        self.declare_parameter("input_hz", 30.0)
        eng = self.get_parameter("engine").value
        conf = self.get_parameter("conf_thres").value

        # TensorRT 引擎加载（由 ai_training/export_int8.py 生成）
        import tensorrt as trt
        import pycuda.driver as cuda
        import pycuda.autoinit
        self.trt, self.cuda = trt, cuda
        runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
        with open(eng, "rb") as f:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self._alloc_io()
        self.get_logger().info(f"TRT engine loaded: {eng}")

        self.bridge = CvBridge()
        self.pub_det = self.create_publisher(Detection, "/ai/detection", 10)
        self.pub_health = self.create_publisher(AiHealth, "/ai/health", 10)
        self.sub_img = self.create_subscription(
            Image, "/camera_front/color/image_raw", self.on_image, qos_profile_sensor_data)
        # 节流订阅：30Hz 输入只取 ~20Hz 送推理，留出 GPU 余量给编码器
        self.min_dt = 1.0 / self.get_parameter("input_hz").value
        self._last = 0.0
        self._conf = conf
        self.health_timer = self.create_timer(0.2, self.beat)   # 5Hz 心跳
        self.latency = 0.0

    def _alloc_io(self):
        import pycuda.driver as cuda
        self.host_in = np.zeros(tuple(self.engine.get_binding_shape(0)), dtype=np.float32)
        self.host_out = np.zeros(tuple(self.engine.get_binding_shape(1)), dtype=np.float32)
        self.dev_in = cuda.mem_alloc(self.host_in.nbytes)
        self.dev_out = cuda.mem_alloc(self.host_out.nbytes)

    def infer(self, blob: np.ndarray) -> np.ndarray:
        import pycuda.driver as cuda
        np.copyto(self.host_in, blob)
        cuda.memcpy_htod_async(self.dev_in, self.host_in, self.stream)
        self.context.execute_async_v2(
            bindings=[int(self.dev_in), int(self.dev_out)], stream_handle=self.stream.handle)
        cuda.memcpy_dtoh_async(self.host_out, self.dev_out, self.stream)
        self.stream.synchronize()
        return self.host_out

    def on_image(self, msg: Image):
        t0 = time.monotonic()
        if t0 - self._last < self.min_dt:
            return
        self._last = t0
        cv = self.bridge.imgmsg_to_cv2(msg, "rgb")
        blob = self.preprocess(cv)                       # letterbox 640x640, /255
        out = self.infer(blob)                           # [1, 84, 8400] YOLOv8 格式
        self.latency = (time.monotonic() - t0) * 1000.0

        for box, cls, conf in self.postprocess(out, msg.width, msg.height):
            if cls not in CLASS_MAP or conf < self._conf:
                continue
            m = Detection()
            m.header.stamp = msg.header.stamp            # 沿图像时间戳，链路延迟可审计
            m.cls = CLASS_MAP[cls]
            m.conf = float(conf)
            m.x_min, m.y_min, m.x_max, m.y_max = map(float, box)
            # 方位角：像素偏移 → 视场角比例（水平 FOV 90°，粗标定即可跟随用）
            cx = (box[0] + box[2]) / 2.0 / msg.width - 0.5
            m.bearing_deg = float(-cx * 90.0)
            self.pub_det.publish(m)

    def beat(self):
        h = AiHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = self.get_name()
        h.latency_ms = float(self.latency)
        h.state = 2 if self.latency > 120.0 else 0       # 推理超时自报，熔断器据此降级
        h.detail = f"{h.latency_ms:.1f}ms"
        self.pub_health.publish(h)

    # ---- YOLOv8 前后处理（精简实现，详见 ai_training/README） ----
    def preprocess(self, img):
        import cv2
        h, w = img.shape[:2]
        r = 640 / max(h, w)
        img = cv2.resize(img, (int(w * r), int(h * r)), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((640, 640, 3), dtype=np.float32)
        canvas[:img.shape[0], :img.shape[1]] = img
        return (canvas.transpose(2, 0, 1)[None] / 255.0).astype(np.float32)

    def postprocess(self, out, w, h):
        pred = out.reshape(84, -1).T                     # [8400, 84]
        boxes = pred[:, :4]
        scores = pred[:, 4:]
        cls_ids = scores.argmax(1)
        confs = scores.max(1)
        keep = confs > 0.25
        boxes, cls_ids, confs = boxes[keep], cls_ids[keep], confs[keep]
        # NMS
        import cv2
        idx = cv2.dnn.NMSBoxes(boxes.tolist(), confs.tolist(), 0.25, 0.45)
        r = 640 / max(w, h)
        for i in np.array(idx).flatten():
            x, y, bw, bh = boxes[i]
            yield (max(0, (x - bw / 2) / r), max(0, (y - bh / 2) / r),
                   min(w, (x + bw / 2) / r), min(h, (y + bh / 2) / r)), int(cls_ids[i]), float(confs[i])


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DetectorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
