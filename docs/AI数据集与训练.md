# AI 数据集与训练（速览，完整规范见 ai_training/dataset/README.md）

## 类别与验收
4 类：行人/车辆/船只/动物。验收：mAP50 ≥ 0.85，行人召回 ≥ 0.9（跟随安全底线），
INT8 相对 FP16 掉点 ≤ 2 mAP。

## 流水线
1. 采集：三高度×三光照×两视角，目标 ≥6000 帧 + VisDrone 公开集；
2. 标注：YOLO 格式，**按飞行架次划分** 8:1:1（防泄漏）；
3. 训练：`python ai_training/train_yolov8.py`（YOLOv8s, 强增广, AdamW）；
4. 导出：`python ai_training/export_int8.py`（ONNX→trtexec INT8，
   500 张校准图见 §4，Orin NX 预期 12~18ms/帧）；
5. 部署：`/opt/models/yolov8s_int8.engine`，detector.py 加载即用。

## 故障诊断模型（health_diag）
无需训练集：一致性残差 + z-score 在线无监督；阈值见节点参数。
论文可扩展：收集已知故障注入数据后升级为轻量分类器。
