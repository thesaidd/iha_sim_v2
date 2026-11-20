"""uav1 ulg: her OFFBOARD (nav_state 14) dilimi için alçalma evresi özeti."""
import sys, numpy as np
from pyulog import ULog
u = ULog(sys.argv[1], message_name_filter_list=['vehicle_local_position','vehicle_status','vehicle_attitude','airspeed_validated','actuator_motors','tecs_status'])
D = {}
for d in u.data_list:
    if d.multi_id == 0: D[d.name] = d.data
lp, st, at = D['vehicle_local_position'], D['vehicle_status'], D['vehicle_attitude']
asp, mo = D.get('airspeed_validated'), D['actuator_motors']
t = lp['timestamp']; nav = st['nav_state'][np.clip(np.searchsorted(st['timestamp'], t, side='right')-1, 0, len(st['timestamp'])-1)]
w,x,y,z = (np.interp(t, at['timestamp'], at[f'q[{j}]']) for j in range(4))
pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
thr = np.interp(t, mo['timestamp'], mo['control[0]'])
cas = np.interp(t, asp['timestamp'], asp['calibrated_airspeed_m_s']) if asp else np.full_like(t, np.nan, dtype=float)
alt = -lp['z']; vz = -lp['vz']; gs = np.hypot(lp['vx'], lp['vy'])
off = np.concatenate(([0], (nav == 14).astype(int), [0])); e = np.flatnonzero(np.diff(off))
segs = [(e[i], e[i+1]-1) for i in range(0, len(e), 2) if (t[e[i+1]-1]-t[e[i]])/1e6 > 15]
for a, b in segs:
    k = a + int(np.argmin(alt[a:b]))                     # dip
    if k - a < 10: continue
    # alçalma evresi: vz < −0.5 olan ilk andan dipten ~1 sn öncesine
    d0 = a + int(np.argmax(vz[a:k] < -0.5)); stop = k
    while stop > d0 and vz[stop] > -0.5: stop -= 1       # tırmanışa dönüş anı ≈ pull-up
    sl = slice(d0, stop+1); dt = (t[stop]-t[d0])/1e6
    yol = np.sum(gs[d0:stop]*np.diff(t[d0:stop+1])/1e6)
    print(f"OFFBOARD {dt and (t[b]-t[a])/1e6:.0f} sn | alçalma {dt:.1f} sn: irtifa {alt[d0]:.1f} → {alt[stop]:.1f} m (−{alt[d0]-alt[stop]:.1f}), yatay {yol:.0f} m → açı {np.degrees(np.arctan2(alt[d0]-alt[stop], yol)):.1f}°")
    print(f"   alçalma hızı ort {-(alt[stop]-alt[d0])/dt:.2f}, en büyük {-vz[sl].min():.2f} m/s | yer hızı {gs[d0]:.1f} → {gs[stop]:.1f} (maks {gs[sl].max():.1f}) | CAS {cas[d0]:.1f} → {cas[stop]:.1f} (maks {np.nanmax(cas[sl]):.1f}) | pitch ort {pitch[sl].mean():.1f}°, min {pitch[sl].min():.1f}° | gaz ort {thr[sl].mean():.2f}, min {thr[sl].min():.2f}")
    print(f"   pull-up anı irtifa {alt[stop]:.2f} m, dip {alt[k]:.2f} m (kayıp {alt[stop]-alt[k]:.2f} m, {(t[k]-t[stop])/1e6:.1f} sn); tırmanış en büyük vz {vz[k:b].max():.1f} m/s, pitch maks {pitch[k:b].max():.0f}°")
