# ai_gateway — 飞控侧 AI 网关（硬实时域守护）

接入方式（二选一）：
1. **作为 PX4 外部模块**（推荐，不动 PX4 源码树）：
   `make px4_fmu-v5_default EXTERNAL_MODULES_LOCATION=<本目录路径>`
2. 放入 `PX4-Autopilot/src/modules/ai_gateway/`，在 `boards/px4/fmu-v5/default.px4board` 加
   `CONFIG_MODULES_AI_GATEWAY=y` 后正常编译。

启动：`rc.txt`（SD 卡 `/etc/rc.txt`）加一行 `ai_gateway start`。

裁决规则（R1~R5）见 ai_gateway.c 文件头注释；全部参数与《任务分区与实时性设计》文档一致。
线程属性：SCHED_DEFAULT，优先级 = DEFAULT-10 —— 高于通信任务、低于姿态内环，
保证 AI 网关无论怎么跑都抢不到飞控时序。
