# 航拍无人机全栈工程框架 — PX4 + ROS2 Humble（对标 Mavic 级能力）

STM32H743 飞控（PX4，硬实时域）+ Jetson Orin NX 机载计算机（ROS2 Humble，AI 非实时域）的双计算机构架，**严格隔离飞控时序与 AI 进程**：AI 全链路崩溃也不影响飞行安全。覆盖目标检测、智能跟随、语义避障、动态规划、大师镜头、故障诊断的完整机载 AI 栈，配套熔断器、地面站与全部工程文档。

> ⚠️ 这是**工程框架**（对标 Mavic 的能力架构与代码骨架），不是玩具小四轴：双 IMU 冗余、EKF3、RTK、五向感知、FOC 云台、OFDM 图传、BMS 全部按航拍机标准设计。实机制作前必读 `docs/制作指南.md` 的安全声明。

## 双计算机架构（核心设计）

```
┌─────────── 硬实时域：STM32H743 + PX4 (NuttX) ───────────┐   ┌─── 非实时域：Jetson Orin NX (ROS2) ───┐
│ ICM42688×2 @4kHz → EKF3(多源融合) → 姿态/位置控制器      │   │ YOLOv8s INT8 检测(20Hz)  ByteTrack 跟踪 │
│ 电机混控 DShot → ESC                                    │◀──│ 语义避障+EGO-Planner(10Hz)  智能跟随     │
│ 双IMU投票/气压计/磁力计/RTK-GNSS                         │MAV│ 智能拍摄状态机  故障AI诊断  Octomap      │
│ commander failsafe：失控/低电/围栏/堵转                  │ROS│ H.265 NVENC 编码 → OFDM 图传            │
│ ★ ai_gateway：AI指令网关(流超时/倾角/围栏/电量裁决)       │→  │ ★ safety_arbiter：AI控制权熔断器        │
└─────────────────────────────────────────────────────────┘   └────────────────────────────────────────┘
        隔离保证：AI 全栈崩溃 → 网关 500ms 流超时 → 飞控内建 failsafe 定点/返航，飞控时序零受扰
```

**AI 不是指令源，是"申请者"**：任何 AI 输出都要过两道门——Orin 上的 `safety_arbiter`（心跳/延迟审计+指令清洗）和飞控里的 `ai_gateway`（流超时+倾角+围栏+电量裁决），任何一道说"不"就是定点悬停保护。

## 仓库结构

| 目录 | 内容 |
|---|---|
| `firmware/px4_modules/ai_gateway/` | PX4 外部模块（C，飞控侧守护，R1~R5 裁决规则） |
| `ros2_ws/src/safety_arbiter/` | AI 控制权熔断器（C++，心跳审计+指令清洗+POSCTL 回退） |
| `ros2_ws/src/perception/` | YOLOv8s INT8 检测 / IoU 跟踪 / 语义避障栅格 / obstacle_send（PX4 碰撞预防数据源） |
| `ros2_ws/src/navigation/` | 智能跟随 / planner_bridge（全局航点+Octomap+EGO-Planner） / vins_bridge（无GPS视觉惯导） |
| `ros2_ws/src/gimbal/` | 三轴 FOC 云台：LOCK/FOLLOW/TRACK 三模式+振动抑制 |
| `ros2_ws/src/cine_ai/` | 大师镜头状态机（冲天/渐远/环绕/俯拍） |
| `ros2_ws/src/health/` | 电机/传感器故障在线诊断 |
| `ros2_ws/src/ground_link/` | OSD 叠加 + 遥测聚合 UDP 桥 |
| `ros2_ws/src/bringup/` | 总启动文件 + 全局参数 |
| `ground_station/` | Qt 地面站 + **Web 网页地面站**（视频/识别框/三维地图/任务/参数/日志）+ 多端接入（手机APP 同协议） |
| `ai_training/` | YOLOv8 训练 + ONNX→TensorRT INT8 导出 + 数据集规范 |
| `tools/` | 延迟基准 / MAVLink 回环模拟器 / INT8 校准器 |
| `scripts/` | 编译 / PX4 SITL 仿真 / Orin 部署 |
| `docs/` | 8 份工程文档 + 项目手册（见下） |

## 文档索引

| 文档 | 内容 |
|---|---|
| [docs/制作指南.md](docs/制作指南.md) | 从购物到首飞：BOM 采购/组装/固件/仿真/首飞检查单 |
| [docs/任务分区与实时性设计.md](docs/任务分区与实时性设计.md) | ★ 硬实时/非实时五道隔离机制、延迟预算表 |
| [docs/BOM清单.md](docs/BOM清单.md) | 全部器件与价格区间 |
| [docs/硬件架构与原理图说明.md](docs/硬件架构与原理图说明.md) | 双电源域、通信拓扑、PCB 分区 |
| [docs/PX4配置与PID整定.md](docs/PX4配置与PID整定.md) | EKF3 冗余参数、PID 整定流程 |
| [docs/AI数据集与训练.md](docs/AI数据集与训练.md) | 数据采集/标注/训练/INT8 校准/验收指标 |
| [docs/调试流程与可视化.md](docs/调试流程与可视化.md) | PlotJuggler / Vofa+ / 延迟基准 |
| [docs/故障排查手册.md](docs/故障排查手册.md) | 症状→根因→解法 |
| [docs/航拍无人机-全栈工程手册.docx](docs/航拍无人机-全栈工程手册.docx) | 完整工程手册（24页：分区架构/安全体系/AI栈/答辩15问，PDF 预览版同目录） |

## 快速开始

```bash
# 1) 仿真先行（不需要任何硬件）
./scripts/sim.sh                     # PX4 SITL + Gazebo + 全 AI 栈
ros2 run tools latency_bench         # 看延迟预算是否达标

# 2) 机载部署（Orin NX）
./scripts/deploy_orin.sh orin@192.168.10.1

# 3) 飞控固件（H743）
make px4_fmu-v5_default EXTERNAL_MODULES_LOCATION=$PWD/firmware/px4_modules/ai_gateway
```

## 量化指标（设计目标）

| 指标 | 目标 | 实测 |
|---|---|---|
| 姿态内环频率 | ≥ 800 Hz（H743 域，与 AI 完全无关） | 【待测】 |
| 检测延迟 p95 | ≤ 35 ms @640 INT8 | 【待测】 |
| 端到端 AI 延迟 p95 | ≤ 120 ms（检测+跟踪+规划+下发） | 【待测】 |
| AI 崩溃恢复 | ≤ 500 ms 自动切定点 | 【待测】 |
| 跟随距离误差 | ±1.5 m @8 m 跟随 | 【待测】 |
| RTK 定位 | 水平 ≤ 5 cm | 【待测】 |

## 多端接入与 Web 地面站

```bash
python3 ground_station/web_gcs/server.py --http 8000 --tele 14555 --video 14556
# 浏览器（手机/平板同地址）: 视频 + 识别框 + 三维地图(Octomap/轨迹/航点) + 任务编辑 + 参数调参 + 日志列表
```
Qt 地面站、手机 APP、Web 端共用 gcs_bridge 的 20Hz 遥测 JSON 广播（gcs_ips 多端并行），
APP 侧另可用 MAVLink 直连（mavlink_router 分流）。H.265 编码走 NVENC 硬件单元，
OSD 叠加 <3ms，自动曝光由 IMX415 ISP 3A 完成，EIS 预留参数（默认关，云台机械增稳为主）。

## 安全声明

此为本科毕设级全栈框架。实机测试务必：先 SITL 全流程 → 带桨台架 → 系绳首飞 → 开阔地单独飞行；永远先飞 `POSCTL` 基线再开 `OFFBOARD`；AI 功能的首次开启都应在熔断器 `critical_nodes` 只挂一个节点的情况下进行（见制作指南 §6）。

## License

MIT。用于毕业设计请理解每个模块（答辩会问），引用的开源组件见 docs/项目手册.md 附录 B。
