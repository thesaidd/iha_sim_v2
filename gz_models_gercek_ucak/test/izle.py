import math, sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255, source_component=190)
D=float(sys.argv[1]) if len(sys.argv)>1 else 30; t0=time.time(); S={}; last=0; rmax=0; rmin=0
while time.time()-t0 < D:
    msg=m.recv_match(blocking=True,timeout=1)
    if msg is None: continue
    t=msg.get_type()
    if t in ("VFR_HUD","ATTITUDE","GLOBAL_POSITION_INT","HEARTBEAT"): S[t]=msg
    if t=="ATTITUDE": r=math.degrees(msg.roll); rmax=max(rmax,r); rmin=min(rmin,r)
    elif t=="STATUSTEXT": print("  [PX4]",msg.text)
    if time.time()-last>=2 and len(S)==4:
        last=time.time(); v,a,g,h=S["VFR_HUD"],S["ATTITUDE"],S["GLOBAL_POSITION_INT"],S["HEARTBEAT"]
        print(f"{time.time()-t0:4.0f}s alt={g.relative_alt/1000:6.1f} hiz={v.airspeed:5.1f} pitch={math.degrees(a.pitch):6.1f} roll={math.degrees(a.roll):6.1f} {'ARMED' if h.base_mode&128 else 'disarmed'}")
print(f"roll aralığı {rmin:.0f}..{rmax:.0f}")
