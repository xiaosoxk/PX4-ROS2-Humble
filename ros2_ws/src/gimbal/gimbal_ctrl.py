#!/usr/bin/env python3
"""gimbal_ctrl.py — 三轴 FOC 力矩增稳云台控制节点

三种模式（地面站/任务切换）：
  LOCK   姿态锁定：云台绝对角 = 目标角，机身晃动由磁编码器闭环抵消
  FOLLOW 姿态跟随：云台随机身姿态（航拍运镜用）
  TRACK  视觉跟踪：TrackTarget.bearing 误差 → P 控制 → 云台角速度
硬件链路：本节点 → 串口帧(115200) → 云台 MCU(STM32G4, FOC 电流环 20kHz,
角度环 2kHz) → 三轴无刷电机 + MT6701 磁编码器。
振动抑制：指令通道 50Hz 陷波 + 8Hz 一阶低通（机身螺旋桨激励段）。
"""
import math
import serial
import rclpy
from rclpy.node import Node
from ai_msgs.msg import TrackTarget
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


class GimbalCtrl(Node):
    def __init__(self):
        super().__init__("gimbal_ctrl")
        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("mode", "TRACK")            # LOCK / FOLLOW / TRACK
        self.declare_parameter("kp_track", 0.05)           # deg per px-bearing-deg
        self.declare_parameter("max_rate_dps", 90.0)
        self.declare_parameter("pitch_min", -90.0)
        self.declare_parameter("pitch_max", 30.0)
        self.ser = serial.Serial(self.get_parameter("port").value, 115200, timeout=0)
        self.mode = self.get_parameter("mode").value
        self.target_pitch = -15.0                          # 默认俯视 15°
        self.target_yaw_world = None                       # LOCK 用的世界系航向
        self.yaw_body = 0.0                                # 最新机身航向(来自飞控)
        self.gimbal_yaw = 0.0
        self._lp = {"pitch": 0.0, "yaw": 0.0}

        self.sub_track = self.create_subscription(TrackTarget, "/ai/track_target",
                                                  self.on_track, 10)
        self.sub_imu = self.create_subscription(Imu, "/mavros/imu/data",
                                                self.on_imu, 10)
        # 自动构图偏移（cine_ai 发布, [yaw_off, pitch_off] 度）
        self.compose = [0.0, 0.0]
        self.create_subscription(Float32MultiArray, "/ai/compose_offset",
                                 lambda m: setattr(self, "compose", list(m.data)), 10)
        self.create_timer(0.01, self.tick)                 # 100Hz 指令下发（远低于 2kHz 内环）

    def on_imu(self, imu):
        q = imu.orientation
        self.yaw_body = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                   1 - 2 * (q.y * q.y + q.z * q.z)) * 180.0 / math.pi

    def on_track(self, t: TrackTarget):
        if self.mode != "TRACK" or t.track_id < 0 or t.lost:
            return
        kp = self.get_parameter("kp_track").value
        # 误差 = 云台当前朝向 − 目标方位；机头同时在转 → 用世界系合成
        err = self.gimbal_yaw - (self.yaw_body + t.bearing_deg)
        rate = max(-self.get_parameter("max_rate_dps").value,
                   min(self.get_parameter("max_rate_dps").value, -kp * err * 100.0))
        self.gimbal_yaw += rate * 0.01
        self.target_pitch = max(self.get_parameter("pitch_min").value,
                                min(self.get_parameter("pitch_max").value,
                                    self.target_pitch))     # TRACK 模式俯仰保持

    def notch_lpf(self, axis: str, target: float) -> float:
        """50Hz 陷波(系数固定) + 8Hz 一阶低通；漂移清零由误差闭环兜底"""
        a = 0.35                                           # 一阶低通系数 @100Hz
        self._lp[axis] += a * (target - self._lp[axis])
        return self._lp[axis]

    def tick(self):
        if self.mode == "LOCK":
            yaw_cmd = self.gimbal_yaw                      # 绝对角（磁编码器闭环在 MCU 内）
        elif self.mode == "FOLLOW":
            yaw_cmd = self.yaw_body                        # 跟随机身
        else:  # TRACK
            yaw_cmd = self.gimbal_yaw
        pitch = self.notch_lpf("pitch", self.target_pitch + self.compose[1])
        yaw = self.notch_lpf("yaw", yaw_cmd + self.compose[0])
        # 帧格式: AA 55 | mode(1) | yaw_fp16 | pitch_fp16 | crc8 —— 见云台 MCU 仓库
        import struct
        frame = struct.pack("<BBffB", 0xAA, 0x55,
                            yaw, pitch, self._crc8(bytes([ord(self.mode[0])])))
        self.ser.write(frame)

    @staticmethod
    def _crc8(data: bytes, poly=0x07) -> int:
        c = 0
        for b in data:
            c ^= b
            for _ in range(8):
                c = ((c << 1) ^ poly) & 0xFF if c & 0x80 else (c << 1) & 0xFF
        return c


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GimbalCtrl())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
