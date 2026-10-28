# -*- coding: utf-8 -*-
"""无人机毕设插图：双机架构 / 端到端延迟预算 / 熔断状态机"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import os

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
DARK = "#0E2230"; TEAL = "#1B6B7A"; CYAN = "#2FB8CF"; GRAY = "#90989F"
LIGHT = "#EAF3F5"; WARM = "#C49B72"; RED = "#B5495B"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)))
os.makedirs(OUT, exist_ok=True)

def box(ax, x, y, w, h, text, fc, ec, tc, fs=9.5):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.004,rounding_size=0.012",
                                fc=fc, ec=ec, lw=1.4))
    ax.text(x + w/2, y + h/2, text, ha="center", va="center", fontsize=fs,
            color=tc, fontweight="bold", linespacing=1.45)

def arrow(ax, x1, y1, x2, y2, color, lw=1.8, ls="-"):
    ax.add_patch(FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="-|>", mutation_scale=14,
                                 color=color, lw=lw, linestyle=ls, shrinkA=2, shrinkB=2))

# ---------------- 图1 双计算机架构 ----------------
fig, ax = plt.subplots(figsize=(10.2, 5.6), dpi=200)
ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
ax.text(0.02, 0.95, "硬实时域：STM32H743 + PX4（NuttX）", fontsize=12, color=TEAL, fontweight="bold")
ax.text(0.56, 0.95, "非实时域：Jetson Orin NX（ROS2 Humble）", fontsize=12, color=WARM, fontweight="bold")

box(ax, 0.02, 0.74, 0.22, 0.15, "双 IMU 采样 @4kHz\nEKF3 多源融合\n(气压/磁/RTK/光流)", DARK, TEAL, "w")
box(ax, 0.02, 0.52, 0.22, 0.15, "姿态内环 @800Hz\n位置环 @50Hz\nDShot600 混控", DARK, TEAL, "w")
box(ax, 0.02, 0.30, 0.22, 0.15, "commander failsafe\n失控/低电/围栏/堵转", DARK, TEAL, "w")
box(ax, 0.02, 0.08, 0.22, 0.15, "★ ai_gateway\nAI 指令网关 R1~R5\n(流超时/倾角/围栏/电量)", "#FDF3E7", RED, "#7A4A20")

box(ax, 0.30, 0.52, 0.19, 0.37, "MAVROS\nUART 921600\n(备份: ETH)\n\n≤30Hz setpoint\n50Hz 遥测", "#F4F7F8", GRAY, DARK, 9)
box(ax, 0.56, 0.74, 0.20, 0.15, "YOLOv8s INT8 检测\n@20Hz ≤35ms\n(TensorRT)", LIGHT, WARM, DARK)
box(ax, 0.56, 0.52, 0.20, 0.15, "语义避障+EGO-Planner\n@10Hz + Octomap", LIGHT, WARM, DARK)
box(ax, 0.56, 0.30, 0.20, 0.15, "智能跟随 / 大师镜头\n云台 TRACK 对准", LIGHT, WARM, DARK)
box(ax, 0.56, 0.08, 0.20, 0.15, "★ safety_arbiter\n心跳审计+指令清洗\n异常→切 POSCTL", "#FDF3E7", RED, "#7A4A20")
box(ax, 0.80, 0.30, 0.18, 0.59, "H.265 NVENC 60fps\nOFDM 图传(复用遥测)\n\n五向双目+激光雷达\nRTK 三模\n\nQt/Web 地面站", "#F4F7F8", GRAY, DARK, 9)

arrow(ax, 0.13, 0.74, 0.13, 0.67, TEAL); arrow(ax, 0.13, 0.52, 0.13, 0.45, TEAL)
arrow(ax, 0.24, 0.155, 0.30, 0.30, RED, ls="--")            # gateway ←→ MAVROS
arrow(ax, 0.30, 0.595, 0.24, 0.595, GRAY)
arrow(ax, 0.49, 0.30, 0.56, 0.155, RED, ls="--")            # arbiter → MAVROS
arrow(ax, 0.66, 0.74, 0.66, 0.67, WARM)
arrow(ax, 0.66, 0.52, 0.66, 0.45, WARM)
arrow(ax, 0.76, 0.155, 0.80, 0.30, GRAY)
ax.text(0.5, 0.015, "隔离底线：AI 全栈崩溃 → 网关 500ms 流超时 → 飞控内建 failsafe 定点/返航；飞控时序零受扰",
        fontsize=9.5, color=RED, ha="center", fontweight="bold")
plt.tight_layout(); plt.savefig(f"{OUT}/fig1_1.png", bbox_inches="tight", facecolor="white"); plt.close()

# ---------------- 图2 端到端延迟预算 ----------------
items = [("YOLOv8s INT8 检测", 20, TEAL), ("跟踪/关联", 3, CYAN),
         ("语义避障+规划", 35, WARM), ("arbiter 清洗+下发", 6, DARK),
         ("MAVLink 链路", 15, GRAY), ("飞控消费+内环生效", 20, "#7A9AA5")]
fig, ax = plt.subplots(figsize=(9.0, 3.6), dpi=200)
left = 0
for name, w, c in items:
    ax.barh(0, w, left=left, height=0.45, color=c, ec="white", lw=1.2)
    ax.text(left + w/2, -0.42, f"{name}\n≤{w}ms", ha="center", fontsize=8.5, color=DARK)
    left += w
ax.axhline(0, color="white", lw=0)
ax.axvline(99, color=RED, lw=1.6, ls="--")
ax.text(99.5, 0.28, "p50 ≈ 99ms\np95 预算 120ms", color=RED, fontsize=9.5)
ax.set_xlim(0, 120); ax.set_ylim(-0.7, 0.6); ax.set_yticks([])
ax.set_xlabel("端到端延迟 (ms)：图像时间戳 → setpoint 进飞控")
ax.set_title("AI 链端到端延迟预算（任一环超标即触发降级审计）", fontsize=12, color=DARK, fontweight="bold")
for s in ["top", "right", "left"]: ax.spines[s].set_visible(False)
plt.tight_layout(); plt.savefig(f"{OUT}/fig2_1.png", bbox_inches="tight", facecolor="white"); plt.close()

# ---------------- 图6 熔断状态机 ----------------
fig, ax = plt.subplots(figsize=(9.6, 4.4), dpi=200)
ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
box(ax, 0.04, 0.35, 0.24, 0.28, "NORMAL\nAI 全链授权\narbiter 转发清洗指令", DARK, TEAL, "w", 10)
box(ax, 0.66, 0.35, 0.30, 0.28, "FUSED → 定点悬停保护\n停发 setpoint + 请求 POSCTL\n(飞控 ai_gateway 500ms 兜底)", "#FDF3E7", RED, "#7A4A20", 10)
arrow(ax, 0.28, 0.49, 0.66, 0.49, RED, lw=2.2)
ax.text(0.47, 0.53, "触发（任一）：\n· 心跳丢失 >0.5s\n· 延迟 p95 >180ms\n· 节点自报故障\n· 电量 <20%", fontsize=9.5,
        color=RED, ha="center", va="bottom")
arrow(ax, 0.66, 0.38, 0.28, 0.38, TEAL, lw=1.8, ls="--")
ax.text(0.47, 0.31, "恢复：AI 链健康 + 飞控 connected + 电量 OK\n+ 操作员确认（防振荡重连）", fontsize=9, color=TEAL, ha="center")
box(ax, 0.30, 0.02, 0.40, 0.16, "飞控内建 failsafe（与上面完全独立）:\nOFFBOARD 失联 COM_OF_LOSS_T=1.0s → HOLD\n— 物理分区兜底，Orin 死机也生效", "#F4F7F8", GRAY, DARK, 9)
plt.tight_layout(); plt.savefig(f"{OUT}/fig6_1.png", bbox_inches="tight", facecolor="white"); plt.close()
print("figs:", sorted(os.listdir(OUT)))
