#!/usr/bin/env python3
"""train_yolov8.py — YOLOv8s 目标检测训练（行人/车辆/船只/动物）
数据集见 dataset/README.md。训练平台不限（PC/云 GPU），训练完导出后部署 Orin。
用法：
  python train_yolov8.py --data dataset/drone.yaml --epochs 120 --imgsz 640
"""
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="dataset/drone.yaml")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--weights", default="yolov8s.pt")     # s 档：精度/速度平衡, 11.2M 参数
    args = ap.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, optimizer="AdamW", lr0=1e-3,
                mosaic=1.0, close_mosaic=15,               # 强增广：小目标+遮挡鲁棒
                degrees=10.0, flipud=0.0, fliplr=0.5,
                project="runs/drone", name="yolov8s")
    metrics = model.val()
    print("mAP50 =", metrics.box.map50)
    print("训练完成: runs/drone/yolov8s/weights/best.pt → 交给 export_int8.py")

if __name__ == "__main__":
    main()
