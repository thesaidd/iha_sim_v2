#!/usr/bin/env python3
"""A4: PX4 (Manual mod, zorla arm) çubuk girdilerine karşı gz servo/motor topic'lerini oku."""
import re, subprocess, threading, time
from pymavlink import mavutil
M = "tulpar_0"
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255, source_component=190)
stop = threading.Event(); lock = threading.Lock()
stick = dict(x=0, y=0, z=0, r=0)
def hb():
    while not stop.is_set():
        with lock: m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)
def mc():
    while not stop.is_set():
        with lock: m.mav.manual_control_send(m.target_system or 1, stick["x"], stick["y"], stick["z"], stick["r"], 0)
        time.sleep(0.05)
threading.Thread(target=hb, daemon=True).start()
m.wait_heartbeat(timeout=15); print("heartbeat", m.target_system, m.target_component)
threading.Thread(target=mc, daemon=True).start()
def cmd(c, *p):
    with lock: m.mav.command_long_send(m.target_system, m.target_component, c, 0, *(list(p)+[0]*(7-len(p))))
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    return a.result if a else None
time.sleep(3)
print("manual mod:", cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 1))
time.sleep(1)
print("arm:", cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)); stick["z"]=300
time.sleep(1)
def gz(topic):
    o = subprocess.run(["gz","topic","-e","-t",topic,"-n","1"],capture_output=True,text=True,timeout=8).stdout
    return o
def servos():
    d = {}
    for n in range(4):
        o = gz(f"/model/{M}/servo_{n}"); r = re.search(r"data: ([-0-9.e]+)", o); d[n] = float(r.group(1)) if r else None
    return d
def motor():
    o = gz(f"/{M}/command/motor_speed"); r = re.search(r"velocity: ([-0-9.e]+)", o)
    return float(r.group(1)) if r else None
T=300
cases = [("nötr",0,0,T,0),("roll sağ",0,600,T,0),("roll sol",0,-600,T,0),("pitch yukarı (çubuk geri)",-600,0,T,0),("pitch aşağı",600,0,T,0),("yaw sağ",0,0,T,600),("yaw sol",0,0,T,-600),("gaz yüksek",0,0,800,0)]
print("durum            servo0(solAil) servo1(sağAil) servo2(solRud) servo3(sağRud)  motor")
for name,x,y,z,r in cases:
    stick.update(x=x,y=y,z=z,r=r); time.sleep(0.8)
    s = servos(); mo = motor()
    f = lambda v: "   None" if v is None else f"{v:+8.3f}"
    print(f"{name:26s}{f(s[0])} {f(s[1])} {f(s[2])} {f(s[3])}   {mo}")
stick.update(x=0,y=0,z=0,r=0); time.sleep(1)
print("disarm:", cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196))
stop.set()
