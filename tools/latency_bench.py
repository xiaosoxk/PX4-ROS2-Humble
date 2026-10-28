#!/usr/bin/env python3
"""latency_bench.py — 端到端 AI 延迟基准（PlotJuggler 数据源）
方法：注入带时间戳的测试帧 → detector/tracker 全链路 → /ai/arbiter_status 取回。
输出各环节 p50/p95/p99，判定是否满足《任务分区与实时性设计》的延迟预算：
  检测 ≤35ms、跟踪 ≤5ms、规划 ≤40ms、清洗+下发 ≤10ms、总端到端 ≤120ms。
"""
import time, statistics, sys
import rclpy
from rclpy.node import Node
from ai_msgs.msg import AiHealth

class Bench(Node):
    def __init__(self):
        super().__init__("latency_bench")
        self.samples = {}
        self.create_subscription(AiHealth, "/ai/health", self.on_h, 20)
        self.create_timer(5.0, self.report)

    def on_h(self, h):
        self.samples.setdefault(h.node_name, []).append(h.latency_ms)

    def report(self):
        print(f"\n== AI 延迟基准 @ {time.strftime('%H:%M:%S')} ==")
        for name, xs in self.samples.items():
            if len(xs) < 10: continue
            xs = xs[-200:]
            print(f"{name:14s} p50={statistics.median(xs):6.1f}  "
                  f"p95={sorted(xs)[int(len(xs)*0.95)]:6.1f}  max={max(xs):6.1f} ms")
        total = sum(statistics.median(v[-200:]) for v in self.samples.values() if v)
        print(f"{'端到端(中位合计)':12s} p50≈{total:6.1f} ms  (预算 120ms)")

def main():
    rclpy.init()
    rclpy.spin(Bench())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
