#!/usr/bin/env python3
"""export_int8.py — best.pt → ONNX → TensorRT INT8（Orin NX 部署格式）
INT8 校准集要求：500 张覆盖全部类别/光照/高度的无标注图（见 dataset/README.md §4）。
流程：① ultralytics 导出 ONNX ② trtexec INT8 校准（脚本生成校准缓存）
预期性能（Orin NX 16GB, MAXN）：640x640 INT8 ≈ 12~18ms/帧（batch=1）。
"""
import subprocess
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="runs/drone/yolov8s/weights/best.pt")
    ap.add_argument("--calib_dir", default="dataset/calib/")   # 500 张校准图
    ap.add_argument("--imgsz", type=int, default=640)
    args = ap.parse_args()

    # ① pt → ONNX（opset 13, 静态 batch1, 与 detector.py 的绑定顺序一致）
    from ultralytics import YOLO
    model = YOLO(args.weights)
    onnx = model.export(format="onnx", imgsz=args.imgsz, opset=13,
                        dynamic=False, simplify=True)
    print("ONNX:", onnx)

    # ② 生成 INT8 校准缓存文件（YOLOv8 preprocessor 输出范围 [0,1]）
    calib_cache = subprocess.run(
        ["python", "-m", "tools.gen_trt_calib", "--dir", args.calib_dir,
         "--out", "yolov8s_int8.calib", "--imgsz", str(args.imgsz)],
        check=True, cwd="..")
    # ③ trtexec 构建引擎（--best=INT8+FP16 混合）
    eng = "yolov8s_int8.engine"
    subprocess.run(["trtexec", f"--onnx={onnx}", f"--saveEngine={eng}",
                    "--best", "--calib=yolov8s_int8.calib",
                    "--memPoolSize=workspace:1024"], check=True)
    print(f"完成: {eng} → 拷贝到 Orin /opt/models/，detector.py 直接加载")

if __name__ == "__main__":
    main()
