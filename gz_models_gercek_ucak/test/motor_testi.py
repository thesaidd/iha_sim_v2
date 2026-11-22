#!/usr/bin/env python3
"""A2: yerçekimsiz dünyada motoru komutla, ivmeden itkiyi kestir (T = m·a, sürükleme ihmal)."""
import re, subprocess, time
M = 2.3
def pose():
    o = subprocess.run(["gz","topic","-e","-t","/world/bos/dynamic_pose/info","-n","1"],capture_output=True,text=True,timeout=10).stdout
    t = re.search(r"header \{\s*stamp \{\s*sec: (\d+)\s*(?:nsec: (\d+))?", o)
    ts = int(t.group(1)) + int(t.group(2) or 0)*1e-9 if t else None
    blk = o.split('name: "gercek_ucak"')[1].split("pose {")[0]
    pos = re.search(r"position \{(.*?)\}", blk, re.S).group(1)
    g = lambda k: float((re.search(k+r": ([-0-9.e]+)", pos) or [0,0])[1]) if re.search(k+r": ([-0-9.e]+)", pos) else 0.0
    return ts, g("x"), g("y"), g("z")
def cmd(w):
    subprocess.run(["gz","topic","-t","/gercek_ucak/command/motor_speed","-m","gz.msgs.Actuators","-p",f"velocity: [{w}]"],capture_output=True)
def olc(w, n=4, dt=0.5):
    cmd(w); time.sleep(1.0)
    s=[pose()]
    for _ in range(n):
        time.sleep(dt); s.append(pose())
    v=[(s[i+1][1]-s[i][1])/(s[i+1][0]-s[i][0]) for i in range(n)]
    a=[(v[i+1]-v[i])/((s[i+2][0]-s[i][0])/1.0*0.5+1e-9) for i in range(n-1)]
    return s, v, a
print("başlangıç", pose())
for w in (0,1000,1500):
    s,v,a=olc(w)
    am=sum(a)/len(a)
    print(f"w={w:5d} rad/s  hız={['%.2f'%x for x in v]} m/s  ivme≈{am:.2f} m/s²  itki≈{M*am:.1f} N  (k·w²={8.9e-6*w*w:.1f} N)  y,z sapma={s[-1][2]:.2f},{s[-1][3]-5:.2f}")
cmd(0)
