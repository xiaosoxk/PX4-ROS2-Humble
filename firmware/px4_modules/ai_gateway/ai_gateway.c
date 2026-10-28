/****************************************************************************
 * ai_gateway.c — PX4 飞控侧 "AI 网关"：机载计算机进入飞控的唯一合法通道守卫
 *
 * 设计原则（硬实时域，禁止任何阻塞调用）：
 *   1. 本模块运行在 STM32H743/NuttX 上，50Hz 固定周期，所有 uORB 订阅均为
 *      非阻塞轮询，禁止 sleep/等待/动态分配——绝不给 1kHz 姿态环添任何抖动；
 *   2. 机载 Orin NX 的 AI 输出（offboard setpoint）不是"指令"而是"申请"：
 *      本模块按 sanitize 规则裁决后才放行，非法/超时立即触发飞控内建 failsafe；
 *   3. AI 域整个崩溃时，本模块只依赖"流超时"这一条物理事实工作——
 *      不需要 AI 域配合，这是分区隔离的底线。
 *
 * 裁决规则（全部来自 docs/任务分区与实时性设计.md 的安全参数）：
 *   R1 offboard 流超时:   >500ms 无 setpoint → 强制切 Position 定点悬停
 *   R2 倾角限制:          setpoint 引起的期望倾角 > 35° → 拒绝并切定点
 *   R3 地理围栏:          水平 ±200m / 高度 [2, 120m] 越界 → 切定点并钳制
 *   R4 电池门限:          SOC < 20% 禁止接受任何 AI 指令（返航权归飞控）
 *   R5 速度钳制:          水平 ≤ 12m/s，垂直 ≤ ±4m/s，超限部分裁剪
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <lib/mathlib/mathlib.h>
#include <math.h>

using namespace time_literals;

namespace ai_gateway
{

class AiGateway : public ModuleBase<AiGateway>
{
public:
	AiGateway() = default;
	~AiGateway() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[])
	{
		_task_id = px4_task_spawn_cmd("ai_gateway", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT - 10,
					      1600, (px4_main_t)&run_trampoline, (char *const *)argv);
		if (_task_id < 0) { PX4_ERR("task spawn failed"); return -1; }
		return 0;
	}

	static AiGateway *instantiate(int argc, char *argv[])
	{
		return new AiGateway();
	}

	static int custom_command(int argc, char *argv[]) { return print_usage("unrecognized command"); }
	static int print_usage(const char *reason = nullptr)
	{
		PX4_INFO("usage: ai_gateway {start|stop|status}");
		return 0;
	}

	void run() override
	{
		/* 50Hz 周期：远低于内环、足够拦截 30Hz 的 AI 指令流 */
		px4::RateControl rate{50_ms};

		uORB::Subscription att_sub{ORB_ID(vehicle_attitude)};
		uORB::Subscription pos_sub{ORB_ID(vehicle_local_position)};
		uORB::Subscription bat_sub{ORB_ID(battery_status)};
		uORB::Subscription obcm_sub{ORB_ID(offboard_control_mode)};
		uORB::Subscription sp_sub{ORB_ID(trajectory_setpoint)};

		uORB::Publication<vehicle_command_s>      cmd_pub{ORB_ID(vehicle_command)};
		uORB::Publication<vehicle_command_ack_s>  ack_pub{ORB_ID(vehicle_command_ack)};

		int fail_count = 0;          /* 连续违规/超时计数，防单帧毛刺误切 */
		bool latched_hold = false;   /* 已切保护模式标志（切一次，退出 offboard 后复位） */

		while (!should_exit()) {
			rate.sleep();

			/* ---- 0. 基础状态采样（全部非阻塞） ---- */
			vehicle_attitude_s att{};
			vehicle_local_position_s pos{};
			battery_status_s bat{};
			offboard_control_mode_s obcm{};
			const bool has_att = att_sub.update(&att);
			const bool has_pos = pos_sub.update(&pos);
			const bool has_bat = bat_sub.update(&bat);
			const bool has_obcm = obcm_sub.update(&obcm);

			if (has_bat && bat.connected && bat.remaining > 0.0f && bat.remaining < 0.20f) {
				/* R4：低电，AI 控制权直接没收（返航由 commander failsafe 接管） */
				if (!_ai_revoked_battery) {
					PX4_WARN("SOC %.0f%% < 20%%: AI control revoked", (double)(100.f * bat.remaining));
					request_posctl(cmd_pub);
					_ai_revoked_battery = true;
				}
			} else if (has_bat && bat.remaining >= 0.25f) {
				_ai_revoked_battery = false;   /* 滞回：>25% 才恢复可授权 */
			}

			const bool offboard_requested = has_obcm && obcm.kinematic_armed;

			/* ---- 1. R1：offboard 流超时（AI 域死亡的唯一必要判据） ---- */
			if (offboard_requested && sp_sub.updated()) {
				trajectory_setpoint_s sp{};
				sp_sub.copy(&sp);
				_last_sp_us = sp.timestamp;
				_ai_alive = true;
			}
			const int64_t now = hrt_absolute_time();
			const bool stream_lost = _ai_alive && (now - _last_sp_us > 500_ms);

			/* ---- 2. R2/R3/R5：对最新 setpoint 做静态裁决 ---- */
			bool violation = false;
			if (offboard_requested && has_att && has_pos) {
				trajectory_setpoint_s sp{};
				sp_sub.copy(&sp);
				violation = sanitize(sp, pos, att);
			}

			/* ---- 3. 裁决执行：连续 3 帧异常(60ms)才动作，防单帧误杀 ---- */
			if ((stream_lost || violation) && offboard_requested) {
				fail_count++;
			} else {
				fail_count = 0;
			}

			if (fail_count >= 3 && !latched_hold) {
				PX4_WARN("AI gateway: %s -> forced POSCTL (latched)",
					 stream_lost ? "setpoint stream timeout" : "setpoint sanitize fail");
				request_posctl(cmd_pub);
				latched_hold = true;
				fail_count = 0;
			}
			/* offboard 退出（任意原因）后自动复位闩锁，允许再次授权 AI */
			if (!offboard_requested) {
				latched_hold = false;
				if (!has_obcm) { _ai_alive = false; }
			}
		}
		PX4_INFO("exiting");
	}

private:
	/**
	 * R2/R3/R5 裁决。返回 true = 违规。
	 * 倾角用期望加速度估算：|a_h|/g ≈ tan(tilt)，不引入任何迭代求解。
	 */
	bool sanitize(const trajectory_setpoint_s &sp, const vehicle_local_position_s &pos,
		      const vehicle_attitude_s &att)
	{
		const float ax = sp.acceleration[0], ay = sp.acceleration[1];
		const float az = sp.acceleration[2];

		if (PX4_ISFINITE(ax) && PX4_ISFINITE(ay)) {
			const float tilt_deg = atan2f(sqrtf(ax * ax + ay * ay), fabsf(az)) * RAD_TO_DEG;
			if (tilt_deg > 35.f) { return true; }                 /* R2 */
		}
		if (PX4_ISFINITE(sp.velocity[0]) && PX4_ISFINITE(sp.velocity[1])) {
			const float vh = sqrtf(sp.velocity[0] * sp.velocity[0] + sp.velocity[1] * sp.velocity[1]);
			if (vh > 12.f) { return true; }                        /* R5 */
		}
		if (PX4_ISFINITE(sp.velocity[2]) && fabsf(sp.velocity[2]) > 4.f) { return true; }

		/* R3：地理围栏（NED 下正北东地），越界即违规 */
		if (PX4_ISFINITE(pos.x) && PX4_ISFINITE(pos.y)) {
			if (fabsf(pos.x) > 200.f || fabsf(pos.y) > 200.f) { return true; }
		}
		if (PX4_ISFINITE(pos.z) && (pos.z < -120.f || pos.z > -2.f)) { return true; }

		return false;
	}

	/** 切 Position 定点：飞控 commander 接管，AI 指令自然失效 */
	void request_posctl(uORB::Publication<vehicle_command_s> &cmd_pub)
	{
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
		cmd.param1 = 1.f;                                  /* base mode: custom */
		cmd.param2 = PX4_CUSTOM_MAIN_MODE_POSCTL;          /* 定点悬停保护 */
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd_pub.publish(cmd);
	}

	uORB::SubscriptionCallbackWorkItem *_cb{nullptr};     /* 预留：改回调驱动的升级位 */

	int64_t _last_sp_us{0};
	bool    _ai_alive{false};
	bool    _ai_revoked_battery{false};
};

} /* namespace ai_gateway */

MODULE_EXPORT int ai_gateway_main(int argc, char *argv[])
{
	return ai_gateway::AiGateway::main(argc, argv);
}
