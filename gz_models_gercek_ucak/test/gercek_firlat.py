#!/usr/bin/env python3
"""A5: gercek_ucak el fırlatma. Parametreler → Takeoff modu + arm (launch-detect bekler) →
yerden 1,5 m'ye ışınla + hız darbesi (ApplyLinkWrench) → uçuşu izle.
Test amaçlı parametreler (SIM-ONLY) bu dosyada tek yerde: PARAMS."""
import csv, math, subprocess, sys, threading, time
from pymavlink import mavutil

V_ATIS = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0   # hedef fırlatma hızı [m/s]
T_ATIS = 0.30                                                # darbe süresi [s]
PITCH0 = math.radians(10.0)                                  # burun yukarı
M, G = 2.3, 9.81
SURE = float(sys.argv[2]) if len(sys.argv) > 2 else 90
import os
MODEL = os.environ.get("MODEL_NAME", "gercek_ucak") + "_0"

PARAMS = {  # (değer) — SIM_ONLY olanlar işaretli
    "FD_ESCS_EN": 0,            # SIM_ONLY (airframe'de de)
    "ASPD_DO_CHECKS": 0,        # SIM_ONLY (gz sıcaklık hatası)
    "ASPD_SCALE_APPLY": 0,      # SIM_ONLY
    "NAV_DLL_ACT": 0,           # SIM_ONLY
    "EKF2_IMU_CTRL": 5,         # SIM_ONLY
    "COM_DISARM_LAND": 10.0,
    "FW_LAUN_DETCN_ON": 1,
    "FW_LAUN_AC_THLD": 20.0,
    "FW_LAUN_AC_T": 0.05,
    "FW_LAUN_MOT_DEL": 0.3,
    "RWTO_TKOFF": 0,
    "MIS_TAKEOFF_ALT": 40.0,
    "FW_AIRSPD_TRIM": 15.0, "FW_AIRSPD_MIN": 11.0, "FW_AIRSPD_MAX": 24.0,
    "FW_THR_TRIM": 0.36, "FW_THR_MAX": 0.9, "FW_THR_MIN": 0.05,
    "FW_PSP_OFF": -2.0,
    "FW_TKO_PITCH_MIN": 10.0, "FW_P_LIM_MAX": 22.0, "FW_PR_I": 0.1, "FW_RR_I": 0.05, "FW_RR_P": 0.08, "FW_RR_FF": 0.12, "FW_R_LIM": 35.0,
}

m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255, source_component=190)
stop = threading.Event(); lock = threading.Lock(); S = {}
def hb():
    while not stop.is_set():
        with lock: m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(1)
def rd():
    while not stop.is_set():
        msg = m.recv_match(blocking=True, timeout=1)
        if msg is None: continue
        t = msg.get_type()
        if t == "HEARTBEAT" and msg.get_srcSystem() == 1: S["hb"] = msg
        elif t in ("VFR_HUD", "ATTITUDE", "GLOBAL_POSITION_INT", "SERVO_OUTPUT_RAW"): S[t] = msg
        elif t == "STATUSTEXT": print(f"  [PX4] {msg.text}", flush=True)
        elif t == "COMMAND_ACK": S["ack"] = msg
        elif t == "PARAM_VALUE": S.setdefault("par", {})[msg.param_id.rstrip("\x00")] = msg.param_value
threading.Thread(target=hb, daemon=True).start(); threading.Thread(target=rd, daemon=True).start()
m.wait_heartbeat(timeout=15); print("bağlandı")

INT_PARAMS = {"FD_ESCS_EN", "ASPD_DO_CHECKS", "ASPD_SCALE_APPLY", "NAV_DLL_ACT", "EKF2_IMU_CTRL", "FW_LAUN_DETCN_ON", "RWTO_TKOFF"}
def setp(k, v):
    for _ in range(8):
        with lock: m.mav.param_set_send(1, 1, k.encode(), float(v), mavutil.mavlink.MAV_PARAM_TYPE_INT32 if k in INT_PARAMS else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        for _ in range(10):
            time.sleep(0.1)
            r = S.get("par", {}).get(k)
            if r is not None and abs(r - float(v)) < 1e-3: return True
    return False
bad = [k for k, v in PARAMS.items() if not setp(k, v)]
print("param doğrulama:", "tamam" if not bad else f"BAŞARISIZ {bad}")
time.sleep(2)
time.sleep(1)
def cmd(c, *p):
    S.pop("ack", None)
    with lock: m.mav.command_long_send(1, 1, c, 0, *(list(p) + [math.nan] * (7 - len(p))))
    for _ in range(50):
        time.sleep(0.1)
        a = S.get("ack")
        if a is not None and a.command == c: return a.result
    return None
def gz(args, **kw): return subprocess.run(["gz"] + args, capture_output=True, text=True, timeout=10, **kw)

from gz.transport13 import Node
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity
a = V_ATIS / T_ATIS
Fx = M * a * math.cos(PITCH0); Fz = M * G + M * a * math.sin(PITCH0)
node = Node(); pw = node.advertise("/world/default/wrench/persistent", EntityWrench); pc = node.advertise("/world/default/wrench/clear", Entity)
time.sleep(0.5)
CGZ = 0.0265   # CG, link orijininin bu kadar üstünde → kuvvet CG'den geçmesi için tork telafisi
rx, rz = -CGZ * math.sin(PITCH0), CGZ * math.cos(PITCH0)   # CG ofseti (gövde z) dünya çerçevesinde, burun yukarı PITCH0
Ty = rz * Fx - rx * Fz     # r_cg × F (y bileşeni)
w = EntityWrench(); w.entity.name = f"{MODEL}::base_link"; w.entity.type = Entity.LINK; w.wrench.force.x = Fx; w.wrench.force.z = Fz; w.wrench.torque.y = Ty
c = Entity(); c.name = f"{MODEL}::base_link"; c.type = Entity.LINK

# 1) Takeoff modu + arm
for i in range(6):
    tk = cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
    ar = cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
    print(f"deneme {i+1}: takeoff={tk} arm={ar}")
    if ar == 0: break
    time.sleep(2)
else:
    print("HATA: arm olmadı"); stop.set(); sys.exit(1)
time.sleep(1.0)

# 2) Işınla + darbe
cp, sp = math.cos(-PITCH0 / 2), math.sin(-PITCH0 / 2)   # gz: nose-up = negatif pitch
r = gz(["service", "-s", "/world/default/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
        "--req", f'name: "{MODEL}" position {{x: 0 y: 0 z: 1.5}} orientation {{x: 0 y: {sp} z: 0 w: {cp}}}'])
print("set_pose:", r.stdout.strip() or r.stderr.strip())
t_launch = time.time(); pw.publish(w); time.sleep(T_ATIS); pc.publish(c)
print(f"darbe: a={a:.1f} m/s² {T_ATIS}s (hedef v={V_ATIS})  gerçek süre {time.time()-t_launch:.2f}s")

# 2b) ilk 3 sn ince izleme
print("  ince izleme (0,1 sn): t roll pitch q(deg/s) servo1..4")
tt = time.time()
while time.time() - tt < 3.0:
    at, sv = S.get("ATTITUDE"), S.get("SERVO_OUTPUT_RAW")
    if at: print(f"   {time.time()-tt:4.1f} roll={math.degrees(at.roll):6.1f} pitch={math.degrees(at.pitch):6.1f} q={math.degrees(at.pitchspeed):6.0f} servo={(sv.servo1_raw, sv.servo2_raw, sv.servo3_raw, sv.servo4_raw) if sv else None}", flush=True)
    time.sleep(0.1)
# 3) izle
rows = []; t0 = time.time()
print("  t   irtifa  hız(hava) pitch  roll  mod")
while time.time() - t0 < SURE:
    time.sleep(1.0)
    v, at, gp, h = S.get("VFR_HUD"), S.get("ATTITUDE"), S.get("GLOBAL_POSITION_INT"), S.get("hb")
    if not (v and at and gp and h): continue
    mode = (h.custom_mode >> 16) & 0xFF, (h.custom_mode >> 24) & 0xFF
    armed = bool(h.base_mode & 128)
    rows.append((time.time() - t0, gp.relative_alt / 1000, v.airspeed, math.degrees(at.pitch), math.degrees(at.roll), mode, armed))
    print(f"{rows[-1][0]:4.0f} {rows[-1][1]:7.1f} {v.airspeed:7.1f} {math.degrees(at.pitch):6.1f} {math.degrees(at.roll):6.1f}  {mode} {'ARMED' if armed else 'disarmed'}", flush=True)
    if not armed and time.time() - t0 > 5: print("disarm oldu → bitiyor"); break
with open("firlat_son.csv", "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "alt", "airspeed", "pitch", "roll", "mode", "armed"]); w.writerows(rows)
stop.set()
