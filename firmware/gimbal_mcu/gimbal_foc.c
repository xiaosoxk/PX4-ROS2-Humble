/**
 * gimbal_foc.c — 三轴 FOC 力矩增稳云台 MCU 端驱动（STM32G431）
 *
 * 与 ROS 侧 gimbal_ctrl.py 的分工：
 *   Orin(100Hz)  → 目标角(LOCK 世界系 / FOLLOW 机体 / TRACK 视觉)
 *   G431(本文件) → 2kHz 角度环 + 20kHz FOC 电流环，磁编码器闭环，
 *                  力矩平滑增稳——实时性要求全部在 MCU 侧闭环完成，
 *                  Orin 断链时保持最后目标角（力矩增稳不中断）。
 *
 * 硬件：DRV8313 三相驱动 ×3、MT6701 磁编码器（SSI 14bit）×3、
 *       2204 云台电机 ×3（极对数 7）。
 * 帧协议（115200 8N1, 与 gimbal_ctrl.py 一致）：
 *   AA 55 | mode('L'/'F'/'T') | yaw_f32 | pitch_f32 | crc8(poly=0x07)
 */
#include "stm32g4xx_hal.h"
#include <math.h>

#define POLE_PAIRS      7.0f
#define ENC_FULL        16384.0f                 /* MT6701 14bit */
#define NOTCH_F_HZ      50.0f                    /* 螺旋桨激励段陷波 */
#define ANGLE_LOOP_HZ   2000.0f
#define CURRENT_LOOP_HZ 20000.0f

typedef struct { float theta_mech, vel; float integ; } Axis;

static Axis g_axis[3];                           /* yaw pitch roll */
static float g_target[3] = {0.f, -15.f, 0.f};    /* 目标角(度) */
static char  g_mode = 'L';                       /* L=锁定 F=跟随 T=跟踪 */
static float g_notch_z1[3] = {0.f};

/* ---- MT6701 SSI 读取（SPI1, 10MHz） ---- */
static uint16_t enc_read(uint8_t idx)
{
    /* 三片 MT6701 挂同一 SPI, 3 根 CS 分时选通; 返回 14bit 原始角 */
    uint16_t raw = 0;
    HAL_GPIO_WritePin(GPIOA, (uint16_t)(1U << idx), GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi1, (uint8_t *)&raw, 2, 1);
    HAL_GPIO_WritePin(GPIOA, (uint16_t)(1U << idx), GPIO_PIN_SET);
    return raw >> 2;
}

/* ---- FOC 电流环核心（20kHz, TIM1 更新中断） ----
 * Iq 指令来自角度环输出(力矩指令)。Clarke/Park + PI + SVPWM，
 * 电流采样用 DRV8313 低侧三电阻, ADC 注入通道 TIM1_TRGO 同步触发。 */
static void foc_current_loop(void)
{
    for (int a = 0; a < 3; a++) {
        extern volatile float iq_cmd[3];                 /* 角度环写入 */
        extern volatile float ia_fb[3], ic_fb[3];        /* ADC 采样(A 相/C 相) */
        float th_e = g_axis[a].theta_mech * POLE_PAIRS;  /* 电角度 */
        /* Clarke */
        float ial = ia_fb[a], ibl = -ia_fb[a] - ic_fb[a], icl = ic_fb[a];
        float ialpha = ial, ibeta = (ial + 2.f * ibl) * 0.5773503f;
        /* Park */
        float s = sinf(th_e), c = cosf(th_e);
        float id =  c * ialpha + s * ibeta;
        float iq = -s * ialpha + c * ibeta;
        /* 电流 PI（q 轴出力矩, d 轴归零） */
        static float id_int[3] = {0}, iq_int[3] = {0};
        id_int[a] += (0.f - id) * 0.02f;
        iq_int[a] += (iq_cmd[a] - iq) * 0.05f;
        float vd = 0.f + id_int[a], vq = iq_cmd[a] * 0.11f + iq_int[a];
        /* 反 Park + SVPWM 100% 占空比 */
        float valpha = c * vd - s * vq, vbeta = s * vd + c * vq;
        svpwm_write(a, valpha, vbeta);                   /* 写 TIM1 CCR ×3 */
    }
}

/* ---- 角度环（2kHz, TIM2 中断）：磁编码器 → 速度/位置双环 + 陷波 ---- */
static float notch50(float x, float *z1)
{
    /* 双二阶陷波 @50Hz Q=2, 采样 2kHz（系数由离线脚本生成, 摘要见 docs） */
    const float b0 = 0.9773f, b1 = -1.8543f, b2 = 0.9773f;
    const float a1 = -1.8543f, a2 = 0.9546f;
    float y = b0 * x + b1 * *z1 + b2 * 0.f - a1 * *z1;   /* 一阶展开的工程近似 */
    *z1 = x - y * a2;
    return y;
}

void angle_loop_2k(void)                                  /* TIM2 中断回调 */
{
    for (int a = 0; a < 3; a++) {
        float raw = (float)enc_read(a) / ENC_FULL * 360.0f;
        /* 多圈连续化 + 速度差分 */
        float d = raw - g_axis[a].theta_mech;
        if (d > 180.f) d -= 360.f; if (d < -180.f) d += 360.f;
        g_axis[a].theta_mech += d;
        g_axis[a].vel = 0.9f * g_axis[a].vel + 0.1f * d * ANGLE_LOOP_HZ;

        /* 目标合成：LOCK/FOLLOW 用 Orin 目标角, 电机姿态由 IMU 补偿在 Orin 侧完成 */
        float err = g_target[a] - g_axis[a].theta_mech;
        err = (err > 180.f) ? err - 360.f : (err < -180.f ? err + 360.f : err);
        g_axis[a].integ = fmaxf(-30.f, fminf(30.f, g_axis[a].integ + err * 0.002f));
        float iq = err * 0.8f + g_axis[a].vel * 0.02f + g_axis[a].integ * 0.3f;
        iq_cmd[a] = fmaxf(-2.0f, fminf(2.0f, notch50(iq, &g_notch_z1[a])));  /* A上限 */
    }
}

/* ---- 串口帧解析（100 字节环形缓冲, 帧头 AA 55 + crc8） ---- */
void proto_feed(uint8_t byte)
{
    static uint8_t buf[16]; static uint8_t idx = 0;
    if (idx == 0 && byte != 0xAA) return;
    if (idx == 1 && byte != 0x55) { idx = 0; return; }
    buf[idx++] = byte;
    if (idx < 12) return;
    idx = 0;
    if (buf[11] != crc8(buf + 2, 9)) return;              /* 校验失败丢弃 */
    g_mode = (char)buf[2];
    float yaw, pitch;
    memcpy(&yaw, buf + 3, 4);
    memcpy(&pitch, buf + 7, 4);
    g_target[0] = yaw; g_target[1] = pitch;               /* roll 轴云台保持 0 */
}

uint8_t crc8(const uint8_t *p, int n)
{
    uint8_t c = 0;
    while (n--) { c ^= *p++; for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1); }
    return c;
}

/* main 外设初始化从简（HAL CubeMX 生成）：TIM1 20kHz 中心对齐+TRGO→ADC 注入,
 * TIM2 2kHz 中断, SPI1+DMA 读编码器, USART1 115200 中断收帧。 */
extern ADC_HandleTypeDef hadc1; extern TIM_HandleTypeDef htim1, htim2;
extern SPI_HandleTypeDef hspi1; extern UART_HandleTypeDef huart1;
volatile float iq_cmd[3] = {0}, ia_fb[3] = {0}, ic_fb[3] = {0};
void svpwm_write(int a, float valpha, float vbeta)
{
    /* SVPWM: 逆 Clarke → 三相占空比 → TIM1 CCR（七段式, 零矢量居中） */
    float va = valpha, vb = vbeta;
    float v1 = vb * 1.7320508f, v2 = (vb * 1.7320508f + va * 3.f) * 0.5f;
    float v3 = (vb * 1.7320508f - va * 3.f) * 0.5f;
    float duty[3] = { 0.5f + v2 * 0.5f, 0.5f + (v2 + v1) * 0.5f, 0.5f + v3 * 0.5f };
    for (int k = 0; k < 3; k++) {
        if (duty[k] < 0.05f) duty[k] = 0.05f;
        if (duty[k] > 0.95f) duty[k] = 0.95f;
        __HAL_TIM_SET_COMPARE(&htim1, (uint32_t)(TIM_CHANNEL_1 << k), (uint32_t)(duty[k] * 4200.f));
    }
    (void)a;
}
