#!/usr/bin/env python3
"""A2 (ince): dynamic_pose akışını kaydet, komuttan sonraki ilk 0.5 sn'de x(t)'ye parabol uydur → itki."""
import re, subprocess, time, sys
M = 2.3
w = float(sys.argv[1])
f = open("/tmp/claude-1000/-home-yusuf24/30897b40-5079-4072-b442-65588eeafe54/scratchpad/mt.txt","w")
p = subprocess.Popen(["gz","topic","-e","-t","/world/bos/dynamic_pose/info"],stdout=f,stderr=subprocess.DEVNULL)
time.sleep(1.0)
subprocess.run(["gz","topic","-t","/gercek_ucak/command/motor_speed","-m","gz.msgs.Actuators","-p",f"velocity: [{w}]"],capture_output=True)
time.sleep(1.5)
p.terminate(); f.close()
txt = open("/tmp/claude-1000/-home-yusuf24/30897b40-5079-4072-b442-65588eeafe54/scratchpad/mt.txt").read()
rows=[]
for msg in txt.split("header {")[1:]:
    t = re.search(r"sec: (\d+)\s*(?:nsec: (\d+))?", msg)
    ts = int(t.group(1))+int(t.group(2) or 0)*1e-9
    blk = msg.split('name: "gercek_ucak"')
    if len(blk)<2: continue
    pos = re.search(r"position \{(.*?)\}", blk[1], re.S).group(1)
    xm = re.search(r"x: ([-0-9.e]+)", pos); zm=re.search(r"z: ([-0-9.e]+)", pos)
    rows.append((ts, float(xm.group(1)) if xm else 0.0, float(zm.group(1)) if zm else 0.0))
# komut anı: x hareket etmeye başladığı an
x0=rows[0][1]
i0=next((i for i,r in enumerate(rows) if abs(r[1]-x0)>1e-4), None)
seg=[r for r in rows[i0:] if r[0]-rows[i0][0] < 0.5]
t0=seg[0][0]
import math
# x = x0 + 0.5 a t^2  -> en küçük kareler
num=sum(0.5*(r[0]-t0)**2*(r[1]-seg[0][1]) for r in seg); den=sum((0.5*(r[0]-t0)**2)**2 for r in seg)
a=num/den
print(f"w={w:.0f} rad/s  örnek={len(seg)}  a={a:.2f} m/s²  itki≈{M*a:.1f} N  (k·w²={8.9e-6*w*w:.1f} N)  Δz={seg[-1][2]-rows[0][2]:+.3f}")
