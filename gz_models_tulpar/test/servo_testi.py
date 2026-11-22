#!/usr/bin/env python3
"""A1: her servoyu +0.3 / -0.3 rad komutla, linkin base_link'e göre dönüşünü dynamic_pose'tan ölç."""
import math, re, subprocess, sys, time

def pose_oku():
    o = subprocess.run(["gz","topic","-e","-t","/world/bos/dynamic_pose/info","-n","1"],capture_output=True,text=True,timeout=10).stdout
    d = {}
    for blk in o.split("pose {")[1:]:
        m = re.search(r'name: "([^"]+)"', blk)
        if not m: continue
        def g(sec, k):
            mm = re.search(sec + r" \{(.*?)\}", blk, re.S)
            r = re.search(k + r": ([-0-9.e]+)", mm.group(1)) if mm else None
            return float(r.group(1)) if r else 0.0
        d[m.group(1)] = (g("orientation","w") or 0.0, g("orientation","x"), g("orientation","y"), g("orientation","z"))
    return d

def qmul(a,b):
    return (a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3], a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1], a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0])
def qinv(q): return (q[0],-q[1],-q[2],-q[3])

def goreli(p, link):
    # model çerçevesine göre link dönüşü: q_base^-1 * q_link ; eksen/açı
    q = qmul(qinv(p["base_link"]), p[link])
    if q[0] < 0: q = tuple(-x for x in q)
    ang = 2*math.acos(min(1,q[0])); s = math.sqrt(max(1e-12,1-q[0]**2))
    return ang, (q[1]/s, q[2]/s, q[3]/s)

LINK = {0:"sol_aileron",1:"sag_aileron",2:"sol_ruddervator",3:"sag_ruddervator"}
def komut(n, v):
    subprocess.run(["gz","topic","-t",f"/model/tulpar/servo_{n}","-m","gz.msgs.Double","-p",f"data: {v}"],capture_output=True)

base = pose_oku()
# hareketsiz referans: link başlangıç yönelimi (servo=0)
ref = {n: qmul(qinv(base["base_link"]), base[LINK[n]]) for n in LINK}
print("n link  komut  döndürme(rad)  eksen(model)")
for n in LINK:
    for v in (0.3,-0.3):
        komut(n, v); time.sleep(2.5)
        p = pose_oku()
        q = qmul(qinv(ref[n]), qmul(qinv(p["base_link"]), p[LINK[n]]))
        # ref'e göre göreli dönüş (yerel eksen)
        if q[0] < 0: q = tuple(-x for x in q)
        ang = 2*math.acos(min(1,q[0])); s = math.sqrt(max(1e-12,1-q[0]**2))
        ax = (q[1]/s,q[2]/s,q[3]/s)
        print(f"{n} {LINK[n]:17s} {v:+.1f}  {ang:.3f}  ({ax[0]:+.2f},{ax[1]:+.2f},{ax[2]:+.2f})")
        komut(n, 0.0); time.sleep(1.5)
