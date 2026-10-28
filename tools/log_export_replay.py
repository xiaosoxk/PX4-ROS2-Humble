#!/usr/bin/env python3
"""log_export_replay.py — 飞行日志导出与回放
导出：PX4 ulog → CSV（姿态/位置/电池/执行器），ros2 bag → CSV；
回放：按时间戳把 CSV 重投为 ROS2 话题（--replay），地面站/PlotJuggler 可复现当时画面。
用法：
  python log_export_replay.py export --ulog flight.ulg --out csv/
  python log_export_replay.py replay --csv csv/ --rate 1.0
"""
import argparse
import csv
import os
import time

def export(ulog, out):
    os.makedirs(out, exist_ok=True)
    from pyulog import ULog
    u = ULog(ulog)
    topics = {
        "vehicle_attitude_0": ["q[0]", "q[1]", "q[2]", "q[3]"],
        "vehicle_local_position_0": ["x", "y", "z", "vx", "vy", "vz"],
        "battery_status_0": ["voltage_v", "remaining"],
        "actuator_outputs_0": ["output[0]", "output[1]", "output[2]", "output[3]"],
    }
    for name, fields in topics.items():
        data = u.get_dataset(name)
        rows = zip(*([[t] for t in [data.data["timestamp"]]] +
                     [data.data.get(f.replace("[]", ""), []) for f in fields]))
        with open(os.path.join(out, name + ".csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t_us"] + [f.replace("[", "_").replace("]", "") for f in fields])
            w.writerows(rows)
    print(f"导出完成: {out}/ (topics: {len(topics)})")

def replay(csvdir, rate):
    """极简回放：读 vehicle_local_position CSV, 按时间差重投 ros2（供 GCS/PlotJuggler）"""
    import rclpy
    from nav_msgs.msg import Odometry
    path = os.path.join(csvdir, "vehicle_local_position_0.csv")
    if not os.path.exists(path):
        print("缺少 CSV, 先 export"); return
    rows = list(csv.DictReader(open(path)))
    t0 = float(rows[0]["t_us"])
    rclpy.init()
    node = rclpy.node.Node("log_replayer")
    pub = node.create_publisher(Odometry, "/mavros/local_position/odom", 10)
    print(f"回放 {len(rows)} 帧, rate={rate}")
    prev = None
    for r in rows:
        t = (float(r["t_us"]) - t0) / 1e6 / rate
        if prev is not None:
            time.sleep(max(0, t - prev))
        prev = t
        m = Odometry()
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.pose.position.x = float(r["x"])
        m.pose.pose.position.y = float(r["y"])
        m.pose.pose.position.z = float(r["z"])
        pub.publish(m)
    rclpy.shutdown()

def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("export"); e.add_argument("--ulog", required=True); e.add_argument("--out", default="csv")
    r = sub.add_parser("replay"); r.add_argument("--csv", default="csv"); r.add_argument("--rate", type=float, default=1.0)
    a = ap.parse_args()
    if a.cmd == "export": export(a.ulog, a.out)
    else: replay(a.csv, a.rate)

if __name__ == "__main__":
    main()
