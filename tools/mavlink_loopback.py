#!/usr/bin/env python3
"""mavlink_loopback.py — 不装飞控也能测 AI 链路的模拟器
模拟 PX4 发送 HEARTBEAT/ATTITUDE，并接收 AI 速度指令打印（验证 arbiter 清洗规则）。
用法：python mavlink_loopback.py --port /dev/ttyUSB0   （或 --udp 127.0.0.1:14550）
"""
import argparse, time
from pymavlink import mavutil

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None); ap.add_argument("--udp", default=None)
    a = ap.parse_args()
    url = a.port or f"udp:{a.udp}"
    m = mavutil.mavlink_connection(url, source_system=1)
    m.wait_heartbeat()
    print("heard heartbeat from system", m.target_system)
    t0 = time.time()
    while True:
        # 模拟飞控 50Hz 遥测
        m.mav.attitude_send(int((time.time()-t0)*1e3), 0,0,0,0,0,0,0,0)
        # 收 AI 链速度指令(SET_POSITION_TARGET_LOCAL_NED), 检查限幅
        msg = m.recv_match(type="SET_POSITION_TARGET_LOCAL_NED", blocking=True, timeout=0.5)
        if msg:
            vh = (msg.vx**2 + msg.vy**2) ** 0.5
            flag = "OK " if vh <= 12.0 else "VIOLATION"
            print(f"[{flag}] vx={msg.vx:.2f} vy={msg.vy:.2f} vz={msg.vz:.2f} |v_h|={vh:.2f}")

if __name__ == "__main__":
    main()
