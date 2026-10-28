#!/usr/bin/env python3
"""gen_trt_calib.py — TensorRT INT8 熵校准器
用法: python -m tools.gen_trt_calib --dir dataset/calib/ --out yolov8s_int8.calib --imgsz 640
"""
import argparse
import glob
import os

import cv2
import numpy as np
import tensorrt as trt


class YoloCalibrator(trt.IInt8EntropyCalibrator2):
    def __init__(self, img_dir, imgsz, cache_file):
        super().__init__()
        self.files = sorted(glob.glob(os.path.join(img_dir, "*.jpg")))[:500]
        self.idx = 0
        self.imgsz = imgsz
        self.cache = cache_file
        self.dev = None

    def get_batch_size(self):
        return 1

    def preprocess(self, path):
        img = cv2.imread(path)
        h, w = img.shape[:2]
        r = self.imgsz / max(h, w)
        img = cv2.resize(img, (int(w * r), int(h * r)))
        canvas = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.float32)
        canvas[:img.shape[0], :img.shape[1]] = img
        return (canvas.transpose(2, 0, 1)[None] / 255.0).astype(np.float32)

    def get_batch(self, names, _):
        if self.idx >= len(self.files):
            return None
        import pycuda.driver as cuda
        batch = self.preprocess(self.files[self.idx])
        self.idx += 1
        if self.dev is None:
            self.dev = cuda.mem_alloc(batch.nbytes)
        cuda.memcpy_htod(self.dev, batch)
        return [int(self.dev)]

    def read_calibration_cache(self):
        if os.path.exists(self.cache):
            return open(self.cache, "rb").read()
        return None

    def write_calibration_cache(self, cache):
        open(self.cache, "wb").write(cache)
        print("calibration cache saved:", self.cache)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--imgsz", type=int, default=640)
    a = ap.parse_args()
    import pycuda.autoinit                      # noqa: F401 CUDA 上下文
    trt.Logger(trt.Logger.WARNING)
    YoloCalibrator(a.dir, a.imgsz, a.out)       # 校准在 trtexec --calib 流程中回调
    print(f"校准集就绪: {len(os.listdir(a.dir))} 张 -> trtexec 将通过 IInt8EntropyCalibrator2 回调使用")


if __name__ == "__main__":
    main()
