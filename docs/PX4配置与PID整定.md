# PX4 配置与 PID 整定

## 1. 传感器冗余配置（EKF3）
| 参数 | 值 | 说明 |
|---|---|---|
| EKF2_MULTI_IMU | 2 | 双 IMU 融合 |
| SENS_IMU_MODE | 0 | EKF3 内部投票接管 IMU 选择 |
| CAL_ACC0_ID / CAL_GYRO0_ID | 按提示 | 双 IMU 分别校准（热补偿开启 TC_A0/TC_G0） |
| EKF2_MAG_TYPE | 5(MagHdg) | 磁航向融合模式, 电动干扰下更稳 |
| EKF2_GPS_CTRL | 7 | 3D+双天线航向(有条件) |
| EKF2_BARO_DELAY | 默认 | MS5611 滤波延迟匹配 |
| GPS_1_CONFIG | TELEM1 | F9P 协议 ubx, 波特 460800 |
| SER_TEL2_BAUD | 921600 | Orin 链路 |

## 2. 失控保护矩阵（commander + ai_gateway 双层）
| 场景 | 触发 | 动作 |
|---|---|---|
| RC 失联 | COM_RC_LOSS_T | 自动返航(RTL) |
| offboard 流断 | COM_OF_LOSS_T=1.0s + ai_gateway R1=0.5s | 定点悬停(POSCTL) |
| 低电量 40% | BAT_CRIT_THR | 建议返航 |
| 低电量 25% | BAT_EMERGEN_THR | 强制返航 |
| 围栏越界 | GF_* + ai_gateway R3 | 定点/钳制 |
| 电机堵转 | ESC 遥测+health_diag | 降级告警, 提示降落 |

## 3. PID 整定流程（顺序不可颠倒）
1. **角速率环**（MC_RATE_*）：AirMode 下小幅度阶跃打杆。
   - 先 P：增大至出现 5~10Hz 振荡，回退 30%；
   - 再 D：增大至电机噪声刺耳，回退 25%；
   - 后 FF（MC_RATE_R_FF）：跟随后滞，目标是阶跃响应无平台期；
   - I：仅消稳态误差，取小值 0.2~0.5；
2. **姿态环**（MC_ROLL/PITCH/YOG_*）：先 P 后 D，比例角速率环小 5~10 倍；
3. **位置环**（MPC_XY_VEL_* / MPC_Z_VEL_*）：默认值通常够，修 P gain 直到
   悬停位置波动 <±0.15m；加速度限制 MPC_ACC_HOR_MAX 决定跟手程度；
4. **验证**：PlotJuggler 看 rate 环 setpoint vs actual 曲线重合度；
   快速前推杆 30° 回中，姿态超调 <5%。

## 4. 整定参数记录表（答辩/复现用）
| 参数 | 初值 | 最终值 | 备注 |
|---|---|---|---|
| MC_RATE_R_P | 0.08 | 【填】 | |
| MC_RATE_R_D | 0.003 | 【填】 | |
| MC_ROLL_P | 6.5 | 【填】 | |
| MPC_XY_VEL_P_ACC | 1.8 | 【填】 | |
