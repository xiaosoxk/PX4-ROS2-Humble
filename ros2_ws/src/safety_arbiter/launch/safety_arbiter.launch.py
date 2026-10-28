from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="safety_arbiter", executable="safety_arbiter", name="safety_arbiter",
             output="screen", parameters=[{
                 "ai_timeout_s": 0.5, "max_vh": 10.0, "max_vz": 3.0,
                 "max_yaw_rate_dps": 60.0, "max_latency_ms": 180.0, "min_battery": 0.20,
                 "critical_nodes": ["detector", "tracker", "semantic_nav"]}]),
    ])
