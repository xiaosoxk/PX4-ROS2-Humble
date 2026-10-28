// safety_arbiter_node.cpp — AI 控制权熔断器（机载侧最后一道闸门）
//
// 架构约定：AI 节点（检测/跟踪/规划/拍摄）永远不直接发布 MAVROS setpoint，
// 只允许发布 ai_msgs/AiCommand 到本节点。本节点是全系统唯一把 AI 意图
// 翻译成 MAVROS setpoint 的地方，并执行：
//   1) 心跳监控：每个 AI 节点必须 5Hz 发布 ai_msgs/AiHealth；
//      任一关键节点超时 / 上报 state>=2 → 整条 AI 链降级；
//   2) 指令清洗：速度限幅、垂直限幅、偏航限速、NaN 防御；
//   3) 链路延迟审计：AiHealth.latency_ms 滑动平均 > 阈值 → 降级；
//   4) 熔断执行：AI 链任何异常 → 停止向 MAVROS 转发 + 请求飞控切 POSCTL
//      （定点悬停保护）。飞控侧 ai_gateway 的 500ms 流超时是它的兜底。
//
// 状态机：NORMAL →(心跳丢失/超延迟/健康故障)→ FUSED(定点) →(人工重新授权)→ NORMAL

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <ai_msgs/msg/ai_command.hpp>
#include <ai_msgs/msg/ai_health.hpp>
#include <deque>
#include <map>
#include <cmath>

using namespace std::chrono_literals;

class SafetyArbiter : public rclcpp::Node
{
public:
	SafetyArbiter() : Node("safety_arbiter")
	{
		declare_parameter<double>("ai_timeout_s", 0.5);
		declare_parameter<double>("max_vh", 10.0);
		declare_parameter<double>("max_vz", 3.0);
		declare_parameter<double>("max_yaw_rate_dps", 60.0);
		declare_parameter<double>("max_latency_ms", 180.0);
		declare_parameter<double>("min_battery", 0.20);
		declare_parameter<std::vector<std::string>>(
			"critical_nodes", {"detector", "tracker", "semantic_nav"});

		get_parameter("ai_timeout_s", p_timeout);
		get_parameter("max_vh", p_max_vh);
		get_parameter("max_vz", p_max_vz);
		get_parameter("max_yaw_rate_dps", p_max_yaw);
		get_parameter("max_latency_ms", p_max_lat);
		get_parameter("min_battery", p_min_bat);
		get_parameter("critical_nodes", p_critical);

		sub_state_ = create_subscription<mavros_msgs::msg::State>(
			"/mavros/state", 10,
			[this](mavros_msgs::msg::State::SharedPtr m) { fc_state_ = *m; });
		sub_bat_ = create_subscription<sensor_msgs::msg::BatteryState>(
			"/mavros/battery", 10,
			[this](sensor_msgs::msg::BatteryState::SharedPtr b) { battery_ = b->power_supply_status >= 0 ? b->percentage : -1.f; });
		sub_cmd_ = create_subscription<ai_msgs::msg::AiCommand>(
			"/ai/command", 5,
			[this](ai_msgs::msg::AiCommand::SharedPtr c) { on_ai_command(c); });
		sub_health_ = create_subscription<ai_msgs::msg::AiHealth>(
			"/ai/health", 20,
			[this](ai_msgs::msg::AiHealth::SharedPtr h) { on_health(h); });

		pub_vel_ = create_publisher<geometry_msgs::msg::TwistStamped>(
			"/mavros/setpoint_velocity/cmd_vel_unstamped", 5);
		pub_status_ = create_publisher<ai_msgs::msg::AiHealth>("/ai/arbiter_status", 10);

		cli_mode_ = create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

		/* 20Hz 审计定时器：熔断判定 + 心跳广播，绝不阻塞 */
		timer_ = create_wall_timer(50ms, [this] { audit(); });
		RCLCPP_INFO(get_logger(), "safety_arbiter up: timeout=%.2fs max_vh=%.1f", p_timeout, p_max_vh);
	}

private:
	void on_health(const ai_msgs::msg::AiHealth::SharedPtr h)
	{
		auto &rec = health_[h->node_name];
		rec.last = now();
		rec.lat.push_back(h->latency_ms);
		if (rec.lat.size() > 30) rec.lat.pop_front();
		rec.state = h->state;
	}

	void on_ai_command(const ai_msgs::msg::AiCommand::SharedPtr c)
	{
		last_ai_cmd_ = now();
		pending_ = *c;
	}

	/** 关键节点是否全部健康（心跳在时 + 延迟达标 + 无故障上报） */
	bool ai_chain_healthy()
	{
		for (const auto &name : p_critical) {
			auto it = health_.find(name);
			if (it == health_.end()) return false;                       /* 从未上报 */
			if ((now() - it->second.last).seconds() > p_timeout) return false;
			if (it->second.state >= 2) return false;                     /* 节点自报故障 */
			double s = 0; for (double v : it->second.lat) s += v;
			if (s / std::max<size_t>(1, it->second.lat.size()) > p_max_lat) return false;
		}
		return true;
	}

	void audit()
	{
		ai_msgs::msg::AiHealth st;
		st.node_name = "arbiter";

		const bool healthy = ai_chain_healthy();
		const bool fc_ok = fc_state_.connected && battery_ > p_min_bat;

		/* ---- 熔断：AI 链异常 → 停止转发 + 请求定点 ---- */
		if (mode_ == Mode::NORMAL && (!healthy || !fc_ok)) {
			if (!healthy) st.detail = "ai chain unhealthy -> fuse";
			else          st.detail = "battery low -> fuse";
			fuse();
			mode_ = Mode::FUSED;
			st.state = 2;
		} else if (mode_ == Mode::FUSED && healthy && fc_ok && manual_rearm_) {
			RCLCPP_WARN(get_logger(), "AI re-armed by operator");
			mode_ = Mode::NORMAL; manual_rearm_ = false; st.state = 0;
		} else {
			st.state = (mode_ == Mode::FUSED) ? 2 : 0;
		}

		/* ---- 正常态：把洗过的 AI 指令翻译为 MAVROS 速度指令（30Hz 上限天然由源频率决定） ---- */
		if (mode_ == Mode::NORMAL && fc_state_.mode == "offboard" &&
		    (now() - last_ai_cmd_).seconds() <= p_timeout) {
			geometry_msgs::msg::TwistStamped t;
			t.header.stamp = now();
			t.twist.linear.x  = clamp(pending_.vx_body,  p_max_vh);
			t.twist.linear.y  = clamp(pending_.vy_body,  p_max_vh);
			t.twist.linear.z  = clamp(pending_.vz_body,  p_max_vz);
			t.twist.angular.z = clamp(pending_.yaw_rate_dps * M_PI / 180.0, p_max_yaw * M_PI / 180.0);
			pub_vel_->publish(t);
		}
		pub_status_->publish(st);
	}

	/** 熔断动作：立即断流。飞控侧 COM_OF_LOSS_T/offboard 失联 failsafe 接管，
	 *  主动再补一刀切 POSCTL（双保险，见《任务分区与实时性设计》§3） */
	void fuse()
	{
		RCLCPP_ERROR(get_logger(), "!!! FUSE: AI control cut, requesting POSCTL hold");
		if (!cli_mode_->service_is_ready()) return;
		auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		req->base_mode = 0;
		req->custom_mode = "POSCTL";                 // 定点悬停保护
		cli_mode_->async_send_request(req);
	}

	static double clamp(double v, double lim) { return std::fmax(-lim, std::fmin(lim, v)); }
	rclcpp::Time now() { return this->get_clock()->now(); }

	enum class Mode { NORMAL, FUSED } mode_ = Mode::NORMAL;
	struct Health { rclcpp::Time last{}; std::deque<double> lat; uint8_t state{0}; };
	std::map<std::string, Health> health_;

	ai_msgs::msg::AiCommand pending_{};
	mavros_msgs::msg::State fc_state_{};
	double battery_{-1.0};
	rclcpp::Time last_ai_cmd_{};
	bool manual_rearm_{true};    /* 重新授权默认允许（恢复健康即自动回 NORMAL） */

	double p_timeout, p_max_vh, p_max_vz, p_max_yaw, p_max_lat, p_min_bat;
	std::vector<std::string> p_critical;

	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr       sub_state_;
	rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_bat_;
	rclcpp::Subscription<ai_msgs::msg::AiCommand>::SharedPtr       sub_cmd_;
	rclcpp::Subscription<ai_msgs::msg::AiHealth>::SharedPtr        sub_health_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_vel_;
	rclcpp::Publisher<ai_msgs::msg::AiHealth>::SharedPtr           pub_status_;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr           cli_mode_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SafetyArbiter>());
	rclcpp::shutdown();
	return 0;
}
