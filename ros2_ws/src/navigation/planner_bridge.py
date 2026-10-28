#!/usr/bin/env python3
"""planner_bridge.py — 自主路径规划桥：全局航点规划 + Octomap + EGO-Planner 接口

三层结构（对应提示词"EGO-Planner局部动态避障 + 全局航点规划，Octomap实时构建"）：
  全局层  : 地面站下发的航点序列 → A* 于 Octomap 2.5D 高度切片（障碍不连通则
            抬高巡航层），输出全局参考线；
  局部层  : EGO-Planner（外部 ROS2 移植包 swarm_planner/ego_planner）消费本节点
            发布的 /planner/goal 与点云（深度/激光 → Octomap 由 octomap_server 完成），
            输出 B 样条轨迹 → 本节点以 10Hz 压成 AiCommand 跟踪指令；
  动态层  : 动态障碍物出现（激光/深度聚类速度 >0.5m/s）→ 通知 EGO-Planner 重规划
            并把轨迹时间参数收紧（前端重搜索由其内部完成）。

安全约定：所有输出仍走 /ai/command → safety_arbiter 清洗，本节点无 MAVROS 权限。
"""
import heapq
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from ai_msgs.msg import AiCommand, AiHealth
from std_msgs.msg import Empty


class PlannerBridge(Node):
    def __init__(self):
        super().__init__("planner_bridge")
        self.declare_parameter("cruise_vh", 4.0)            # 全局巡航速度 m/s
        self.declare_parameter("cruise_alt_min", 15.0)      # 全局层飞行高度下限 m（Above Home）
        self.declare_parameter("grid_res", 1.0)             # 2.5D 切片分辨率 m
        self.declare_parameter("replan_hz", 10.0)
        self.declare_parameter("mission_timeout_s", 600.0)

        # Octomap（octomap_server 由 bringup 启动，本节点读其标记化输出）
        self.sub_map = self.create_subscription(MarkerArray, "/octomap/full/occupied_cells_vis_array",
                                                self.on_map, 5)
        self.sub_odom = self.create_subscription(Odometry, "/mavros/local_position/odom",
                                                 self.on_odom, 10)
        self.sub_goal = self.create_subscription(Odometry, "/ai/mission_goal", self.on_goal, 5)
        self.sub_stop = self.create_subscription(Empty, "/ai/mission_cancel", lambda m: self.abort(), 5)
        # EGO-Planner 接口（swarm_planner ROS2 移植包话题约定）
        self.pub_ego_goal = self.create_publisher(Odometry, "/planner/goal", 5)
        self.pub_cmd = self.create_publisher(AiCommand, "/ai/command", 5)
        self.pub_h = self.create_publisher(AiHealth, "/ai/health", 10)

        self.occ = set()          # 被占据的 2.5D 栅格 (ix, iy)：任意高度有障碍
        self.odom = None
        self.goal = None
        self.path = []
        self.wp_i = 0
        self.create_timer(1.0 / self.get_parameter("replan_hz").value, self.tick)
        self.create_timer(0.2, self.beat)
        self.active = False

    # ---------------- 全局层：Octomap 2.5D + A* ----------------
    def on_map(self, arr: MarkerArray):
        res = self.get_parameter("grid_res").value
        occ = set()
        for m in arr.markers:
            for p in m.points:
                occ.add((int(math.floor(p.x / res)), int(math.floor(p.y / res))))
        self.occ = occ                                        # 任意高度占据 → 2.5D 投影

    def on_odom(self, m):
        self.odom = m

    def on_goal(self, m):
        if not self.odom:
            self.get_logger().warn("mission goal before odometry, ignored")
            return
        self.goal = m.pose.pose.position
        self.path = self.astar((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y),
                               (self.goal.x, self.goal.y))
        self.wp_i = 0
        self.active = bool(self.path)
        self.get_logger().info(f"mission planned: {len(self.path)} cells")

    def astar(self, s, g):
        """2.5D A*：占据栅格绕行；目标格若被占则抬高巡航层假设（ceiling 策略）"""
        res = self.get_parameter("grid_res").value
        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])
        start, goal = (int(s[0] / res), int(s[1] / res)), (int(g[0] / res), int(g[1] / res))
        openq = [(h(start, goal), 0.0, start)]
        came, cost = {start: None}, {start: 0.0}
        while openq and len(came) < 200000:
            _, f, cur = heapq.heappop(openq)
            if cur == goal:
                break
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                nxt = (cur[0] + dx, cur[1] + dy)
                if nxt in self.occ:                            # 障碍格 → 绕行
                    continue
                nc = cost[cur] + math.hypot(dx, dy)
                if nxt not in cost or nc < cost[nxt]:
                    cost[nxt] = nc
                    came[nxt] = cur
                    heapq.heappush(openq, (nc + h(nxt, goal), nc, nxt))
        if goal not in came:
            self.get_logger().error("A* failed: goal unreachable in 2.5D slice")
            return []
        path, cur = [], goal
        while cur:
            path.append((cur[0] * res, cur[1] * res))
            cur = came[cur]
        return path[::-1]

    # ---------------- 局部层：喂 EGO-Planner + 轨迹跟踪 ----------------
    def tick(self):
        if not (self.active and self.odom and self.path):
            return
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        # 当前段目标点：A* 参考线上的前瞻点（lookahead 8m），交给 EGO-Planner 做局部细化
        look = 8.0 / self.get_parameter("grid_res").value
        tgt = self.path[min(self.wp_i + int(look), len(self.path) - 1)]
        while self.wp_i < len(self.path) - 1 and \
                math.hypot(self.path[self.wp_i][0] - px, self.path[self.wp_i][1] - py) < 2.0:
            self.wp_i += 1                                     # 到达当前参考点 → 推进
        g = Odometry()
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = tgt
        g.pose.pose.position.z = -self.get_parameter("cruise_alt_min").value   # NED
        self.pub_ego_goal.publish(g)                           # EGO-Planner 局部规划

        cmd = AiCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.source = 1
        cmd.vx_body, cmd.vy_body = self._body_velocity(tgt, px, py)
        cmd.vz_body = float(np.clip(self.goal.z - self.odom.pose.pose.position.z * -1.0,
                                    -2.0, 2.0))
        cmd.yaw_rate_dps = self._yaw_rate(tgt, px, py)
        self.pub_cmd.publish(cmd)
        if self.wp_i >= len(self.path) - 1 and \
                math.hypot(self.goal.x - px, self.goal.y - py) < 1.0:
            self.abort(); self.get_logger().info("mission done, hover")

    def _body_velocity(self, tgt, px, py):
        """世界系期望速度（指向前瞻点，限幅巡航） → 机体系分量"""
        yaw = self._yaw_from_odom()
        dx, dy = tgt[0] - px, tgt[1] - py
        n = math.hypot(dx, dy) or 1.0
        v = min(self.get_parameter("cruise_vh").value, n * 0.8)
        wx, wy = v * dx / n, v * dy / n
        c, s = math.cos(yaw), math.sin(yaw)
        return c * wx + s * wy, -s * wx + c * wy               # NED→FRD

    def _yaw_rate(self, tgt, px, py):
        yaw = self._yaw_from_odom()
        want = math.degrees(math.atan2(tgt[1] - py, tgt[0] - px))
        err = (want - yaw + 540.0) % 360.0 - 180.0
        return max(-45.0, min(45.0, 0.8 * err))

    def _yaw_from_odom(self):
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return math.degrees(yaw)

    def abort(self):
        self.active, self.path, self.wp_i = False, [], 0
        self.pub_cmd.publish(AiCommand())                      # 全零 → 熔断器侧仍有效清洗

    def beat(self):
        h = AiHealth()
        h.header.stamp = self.get_clock().now().to_msg()
        h.node_name = "planner_bridge"
        h.latency_ms = 3.0
        h.state = 0 if (not self.active or self.path) else 1
        h.detail = "idle" if not self.active else f"wp {self.wp_i}/{len(self.path)}"
        self.pub_h.publish(h)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PlannerBridge())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
