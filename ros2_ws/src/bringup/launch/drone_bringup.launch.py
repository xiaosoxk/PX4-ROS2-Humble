"""drone_bringup.launch.py — 机载 AI 栈总启动
顺序原则：先熔断器(守门) → 感知 → 导航 → 云台/图传。
所有节点 ComposableNode 化可省 30% 启动时间，这里用独立进程便于隔离崩溃域。
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def gen():
    params = os.path.join(get_package_share_directory("bringup"), "config", "params.yaml")
    def py(pkg, exe, name, extra=None):
        p = {"params": [params]} if not extra else {"params": [params, extra]}
        return Node(package=pkg, executable=exe, name=name, output="screen", **p)

    return LaunchDescription([
        # 1) 熔断器最先起：任何 AI 节点尚未就绪时它都处于"全熔断"安全态
        Node(package="safety_arbiter", executable="safety_arbiter", name="safety_arbiter",
             output="screen", parameters=[params, {"critical_nodes": ["detector", "tracker", "semantic_nav"]}]),
        # 2) MAVROS（飞控链路, TELEM2 921600）
        Node(package="mavros", executable="mavros_node", name="mavros",
             parameters=[{"fcu_url": "/dev/ttyTHS0:921600", "tgt_system": 1}]),
        # 3) 感知链
        py("perception", "detector.py", "detector"),
        py("perception", "tracker.py", "tracker"),
        py("perception", "semantic_nav.py", "semantic_nav"),
        # 4) 导航/任务
        py("navigation", "follow_me.py", "follow_me"),
        # 5) 云台 + 健康诊断 + 地面链路
        py("gimbal", "gimbal_ctrl.py", "gimbal_ctrl"),
        py("health", "health_diag.py", "health_diag"),
        py("ground_link", "osd_overlay.py", "osd_overlay"),
        py("ground_link", "gcs_bridge.py", "gcs_bridge"),
    ])

def generate_launch_description():
    return gen()
