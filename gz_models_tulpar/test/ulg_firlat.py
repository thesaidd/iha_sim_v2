import math, sys, numpy as np
from pyulog import ULog
u=ULog(sys.argv[1]); d={x.name:x for x in u.data_list}
def eul(q):
    w,x,y,z=q; return math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))), math.degrees(math.asin(max(-1,min(1,2*(w*y-z*x)))))
att=d['vehicle_attitude']; ta=att.data['timestamp']/1e6
sp=d['vehicle_attitude_setpoint']; tsp=sp.data['timestamp']/1e6
sv=d['actuator_servos']; tsv=sv.data['timestamp']/1e6
mo=d['actuator_motors']; tmo=mo.data['timestamp']/1e6
rs=d['vehicle_rates_setpoint']; trs=rs.data['timestamp']/1e6
ang=d['vehicle_angular_velocity']; tan=ang.data['timestamp']/1e6
# arm sonrası: motor >0.5 ilk an
i=int(np.argmax(np.nan_to_num(mo.data['control[0]'])>0.5)); T=tmo[i]
t0=T-0.6
print("t(motor=0.9'dan)  roll pitch | pitchSP | q q_sp | servo0..3 | motor")
for t in np.arange(t0, t0+float(sys.argv[2]) if len(sys.argv)>2 else 3, 0.1):
    k=min(np.searchsorted(ta,t),len(ta)-1); ks=min(np.searchsorted(tsp,t),len(tsp)-1)
    kv=min(np.searchsorted(tsv,t),len(tsv)-1); km=min(np.searchsorted(tmo,t),len(tmo)-1); kr=min(np.searchsorted(trs,t),len(trs)-1); kn=min(np.searchsorted(tan,t),len(tan)-1)
    r,p=eul([att.data[f'q[{j}]'][k] for j in range(4)]); _,psp=eul([sp.data[f'q_d[{j}]'][ks] for j in range(4)])
    print(f"{t-T:+.1f} {r:6.1f} {p:6.1f} | {psp:5.1f} | {math.degrees(ang.data['xyz[1]'][kn]):6.0f} {math.degrees(rs.data['pitch'][kr]):6.0f} | "+" ".join(f"{sv.data[f'control[{j}]'][kv]:+.2f}" for j in range(4))+f" | {mo.data['control[0]'][km]:.2f}")
