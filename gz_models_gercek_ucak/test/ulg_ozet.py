import sys, math, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1]); t0 = None
def d(name):
    return next((x for x in u.data_list if x.name == name), None)
att, sp, srv, asp, lp = d("vehicle_attitude"), d("vehicle_attitude_setpoint"), d("actuator_servos"), d("airspeed_validated"), d("vehicle_local_position")
mot = d("actuator_motors"); rs = d("vehicle_rates_setpoint"); ang = d("vehicle_angular_velocity")
def eul(q):
    w,x,y,z = q
    return (math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))), math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x))))))
a0 = att.data["timestamp"][0] if att else 0
T0 = float(sys.argv[2]) if len(sys.argv) > 2 else None
# başlangıç: ilk airspeed>5 anı
tt = lp.data["timestamp"]/1e6; vz = lp.data["z"]
i0 = int(np.argmax(vz < -1.0)) if (vz < -1.0).any() else 0
T = tt[i0] - 1.0
print(" t     alt   hiz   roll  rollSP  pitch pitchSP | servo0 servo1 servo2 servo3 | motor | p q r (deg/s)")
for ts in np.arange(float(sys.argv[2]) if len(sys.argv)>2 else 0, float(sys.argv[3]) if len(sys.argv)>3 else 10.5, float(sys.argv[4]) if len(sys.argv)>4 else 0.5):
    tq = (T + ts) * 1e6
    def at(dd, key, idx=None):
        k = int(np.searchsorted(dd.data["timestamp"], tq)); k = min(k, len(dd.data["timestamp"]) - 1)
        v = dd.data[key] if idx is None else dd.data[f"{key}[{idx}]"]
        return v[k]
    k = int(np.searchsorted(att.data["timestamp"], tq)); k = min(k, len(att.data["timestamp"]) - 1)
    q = [att.data[f"q[{i}]"][k] for i in range(4)]; r, p = eul(q)
    ks = int(np.searchsorted(sp.data["timestamp"], tq)); ks = min(ks, len(sp.data["timestamp"]) - 1)
    rsp, psp = eul([sp.data[f"q_d[{i}]"][ks] for i in range(4)])
    s = [at(srv, "control", i) for i in range(4)] if srv else [float("nan")]*4
    mo = at(mot, "control", 0) if mot else float("nan")
    z = -at(lp, "z"); v = at(asp, "calibrated_airspeed_m_s") if asp else float("nan")
    pq = [math.degrees(at(ang, "xyz", i)) for i in range(3)] if ang else [0,0,0]
    print(f"{ts:4.1f} {z:6.1f} {v:5.1f} {r:6.1f} {rsp:6.1f} {p:6.1f} {psp:6.1f} | {s[0]:+.2f} {s[1]:+.2f} {s[2]:+.2f} {s[3]:+.2f} | {mo:.2f} | {pq[0]:6.1f} {pq[1]:6.1f} {pq[2]:6.1f}")
