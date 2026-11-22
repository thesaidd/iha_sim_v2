#!/usr/bin/env python3
"""A3: PX4'siz serbest uçuş. t=1s'de +x darbe (v0), servo 0, sabit motor; poz akışından hız/irtifa/pitch/roll/yaw."""
import math, re, subprocess, sys, time
M = 2.3
V0 = float(sys.argv[1]); W = float(sys.argv[2]); TSURE = float(sys.argv[3]) if len(sys.argv) > 3 else 12
PITCH0 = 0.0
STEP = int(sys.argv[4]) if len(sys.argv) > 4 else 25
OUT = "/tmp/claude-1000/-home-yusuf24/30897b40-5079-4072-b442-65588eeafe54/scratchpad/uc.txt"
f = open(OUT, "w")
p = subprocess.Popen(["gz","topic","-e","-t","/world/bos/dynamic_pose/info"], stdout=f, stderr=subprocess.DEVNULL)
time.sleep(1.5)
subprocess.run(["gz","topic","-t","/tulpar/command/motor_speed","-m","gz.msgs.Actuators","-p",f"velocity: [{W}]"], capture_output=True)
subprocess.run(["gz","service","-s","/world/bos/control","--reqtype","gz.msgs.WorldControl","--reptype","gz.msgs.Boolean","--timeout","3000","--req","pause: false"],capture_output=True)
time.sleep(0.5)
import os
F = M * V0 / 0.004
r = subprocess.run(["gz","topic","-t","/world/bos/wrench","-m","gz.msgs.EntityWrench","-p",
    f'entity: {{name: "tulpar::base_link", type: LINK}} wrench: {{force: {{x: {F}}}}}'], capture_output=True, text=True)
print("wrench:", r.stdout.strip(), r.stderr.strip())
SERVO_T = float(os.environ.get("SERVO_T", "3.0"))
time.sleep(SERVO_T)
for kv in filter(None, os.environ.get("SERVO", "").split(",")):
    n, v = kv.split(":")
    subprocess.run(["gz","topic","-t",f"/model/tulpar/servo_{n}","-m","gz.msgs.Double","-p",f"data: {v}"], capture_output=True)
time.sleep(max(0.5, TSURE - SERVO_T))
p.terminate(); f.close()
txt = open(OUT).read()
rows = []
for msg in txt.split("header {")[1:]:
    ms = re.search(r"(?<!n)sec: (\d+)", msg); mn = re.search(r"nsec: (\d+)", msg)
    ts = (int(ms.group(1)) if ms else 0) + (int(mn.group(1)) if mn else 0) * 1e-9
    b = msg.split('name: "tulpar"')
    if len(b) < 2: continue
    blk = b[1].split("pose {")[0]
    def num(sec, k):
        m = re.search(sec + r" \{(.*?)\}", blk, re.S)
        if not m: return 0.0
        r = re.search(k + r": ([-0-9.e]+)", m.group(1))
        return float(r.group(1)) if r else 0.0
    x, y, z = num("position", "x"), num("position", "y"), num("position", "z")
    qw, qx, qy, qz = num("orientation", "w"), num("orientation", "x"), num("orientation", "y"), num("orientation", "z")
    roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
    pitch = math.asin(max(-1, min(1, 2*(qw*qy-qz*qx))))
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    if x != 0.0 or z != 100.0: rows.append((ts, x, y, z, roll, pitch, yaw))
t0 = rows[0][0]
print('ilk satırlar', [tuple(round(v,2) for v in r) for r in rows[:2]], len(rows))
print(" t(s)   x     y     z    hiz   roll  pitch  yaw")
last = None
for i, r in enumerate(rows):
    if i % STEP == 0 and i >= 0:
        j = max(0, i-STEP)
        dt = r[0]-rows[j][0]
        v = math.sqrt(sum(((r[k]-rows[j][k])/dt)**2 for k in (1, 2, 3))) if dt > 0 else 0
        vx = (r[1]-rows[j][1])/dt if dt > 0 else 0
        vz = (r[3]-rows[j][3])/dt if dt > 0 else 0
        gam = math.degrees(math.atan2(vz, vx)) if vx else 0
        print(f"{r[0]-t0:5.2f} {r[1]:6.1f} {r[2]:5.1f} {r[3]:6.1f} v={v:5.1f} roll={math.degrees(r[4]):6.1f} pitch={math.degrees(r[5]):6.1f} gam={gam:6.1f} yaw={math.degrees(r[6]):6.1f}")
