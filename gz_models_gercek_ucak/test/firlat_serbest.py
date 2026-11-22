#!/usr/bin/env python3
"""PX4'siz: gercek_firlat.py'deki darbe + tork telafisinin saf etkisi (pitch/hız), 100 m'de."""
import math, re, subprocess, sys, threading, time
from gz.transport13 import Node
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity
M, G, V, T, TH = 2.3, 9.81, 12.0, 0.3, math.radians(10)
COMP = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
W = float(sys.argv[2]) if len(sys.argv) > 2 else 550
a = V / T; Fx = M*a*math.cos(TH); Fz = M*G + M*a*math.sin(TH)
CGZ = 0.0265; rx, rz = -CGZ*math.sin(TH), CGZ*math.cos(TH); Ty = COMP * (rz*Fx - rx*Fz)
node = Node(); pw = node.advertise("/world/bos/wrench/persistent", EntityWrench); pc = node.advertise("/world/bos/wrench/clear", Entity)
time.sleep(0.5)
w = EntityWrench(); w.entity.name = "gercek_ucak::base_link"; w.entity.type = Entity.LINK; w.wrench.force.x = Fx; w.wrench.force.z = Fz; w.wrench.torque.y = Ty
c = Entity(); c.name = "gercek_ucak::base_link"; c.type = Entity.LINK
f = open("/tmp/claude-1000/-home-yusuf24/30897b40-5079-4072-b442-65588eeafe54/scratchpad/fs.txt", "w")
p = subprocess.Popen(["gz","topic","-e","-t","/world/bos/dynamic_pose/info"], stdout=f, stderr=subprocess.DEVNULL)
time.sleep(1.0)
subprocess.run(["gz","service","-s","/world/bos/set_pose","--reqtype","gz.msgs.Pose","--reptype","gz.msgs.Boolean","--timeout","3000","--req",
  f'name: "gercek_ucak" position {{x: 0 y: 0 z: 100}} orientation {{x: 0 y: {math.sin(-TH/2)} z: 0 w: {math.cos(-TH/2)}}}'], capture_output=True)
subprocess.run(["gz","topic","-t","/gercek_ucak/command/motor_speed","-m","gz.msgs.Actuators","-p",f"velocity: [{W}]"], capture_output=True)
subprocess.run(["gz","service","-s","/world/bos/control","--reqtype","gz.msgs.WorldControl","--reptype","gz.msgs.Boolean","--timeout","3000","--req","pause: false"], capture_output=True)
time.sleep(0.3); pw.publish(w); time.sleep(T); pc.publish(c); time.sleep(4.0)
p.terminate(); f.close()
txt = open("/tmp/claude-1000/-home-yusuf24/30897b40-5079-4072-b442-65588eeafe54/scratchpad/fs.txt").read()
rows = []
for msg in txt.split("header {")[1:]:
    ms = re.search(r"(?<!n)sec: (\d+)", msg); mn = re.search(r"nsec: (\d+)", msg)
    ts = (int(ms.group(1)) if ms else 0) + (int(mn.group(1)) if mn else 0)*1e-9
    b = msg.split('name: "gercek_ucak"')
    if len(b) < 2: continue
    blk = b[1].split("pose {")[0]
    def num(sec, k):
        m_ = re.search(sec + r" \{(.*?)\}", blk, re.S)
        if not m_: return 0.0
        r_ = re.search(k + r": ([-0-9.e]+)", m_.group(1)); return float(r_.group(1)) if r_ else 0.0
    x, z = num("position", "x"), num("position", "z")
    qw, qx, qy, qz = (num("orientation", k) for k in "wxyz")
    rows.append((ts, x, z, math.degrees(math.asin(max(-1, min(1, 2*(qw*qy-qz*qx))))), math.degrees(math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)))))
rows = [r for r in rows if r[2] > 5]
print(f"COMP={COMP} Ty={Ty:.2f} N·m  Fx={Fx:.0f} Fz={Fz:.0f}")
t0 = rows[0][0]; last = -1
for i, r in enumerate(rows):
    if i % 10 == 0 and i > 0:
        j = i - 10; dt = r[0]-rows[j][0]
        if dt <= 0: continue
        print(f"{r[0]-t0:5.2f} x={r[1]:6.1f} z={r[2]:6.1f} vx={(r[1]-rows[j][1])/dt:5.1f} vz={(r[2]-rows[j][2])/dt:5.1f} pitch={-r[3]:6.1f}(nose-up+) roll={r[4]:5.1f}")
