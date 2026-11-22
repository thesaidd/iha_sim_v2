#!/usr/bin/env python3
"""ulg_hold.py <log.ulg> — fırlatma sonrası: yükselme süreleri, Hold (AUTO_LOITER) yarıçapı/hızı/irtifası, failsafe.
Hold penceresi: nav_state AUTO_LOITER (4) ve irtifa ≥ eşik. Yarıçap: yerel (x, y)'ye en küçük kareler çemberi (Kåsa).
Kullanım: ulg_hold.py <log.ulg> [hedef_irtifa=100]"""
import sys
import numpy as np
from pyulog import ULog

u = ULog(sys.argv[1]); H = float(sys.argv[2]) if len(sys.argv) > 2 else 100.0
d = {x.name: x for x in u.data_list}
lp, st, mo = d["vehicle_local_position"], d["vehicle_status"], d["actuator_motors"]
t = lp.data["timestamp"] / 1e6; z = -lp.data["z"]; x = lp.data["x"]; y = lp.data["y"]
vg = np.hypot(lp.data["vx"], lp.data["vy"])
tm = mo.data["timestamp"] / 1e6
T0 = tm[int(np.argmax(np.nan_to_num(mo.data["control[0]"]) > 0.5))]      # motor açıldı = fırlatma algılandı
print(f"fırlatma (motor>0,5): t0 = {T0:.1f} s (log zamanı)")
for h in (40, 80, H):
    i = np.argmax(z >= h - 0.5)
    print(f"  {h:5.0f} m'ye ulaşma: {t[i] - T0:5.1f} s" if z[i] >= h - 0.5 else f"  {h:5.0f} m'ye ULAŞILAMADI (tepe {z.max():.1f} m)")
print(f"  tepe irtifa: {z.max():.1f} m ({t[np.argmax(z)] - T0:.1f} s)")
nav = st.data["nav_state"]; ts = st.data["timestamp"] / 1e6
hold = np.zeros(len(t), bool)
for k in range(len(t)):
    kk = min(np.searchsorted(ts, t[k]), len(ts) - 1)
    hold[k] = (nav[kk] == 4) and z[k] > H - 15
if hold.sum() < 50:
    print(f"Hold penceresi yok/çok kısa ({hold.sum()} örnek)")
else:
    idx = np.where(hold)[0]; i0, i1 = idx[0], idx[-1]
    hs = hold & (t > t[i0] + 20)              # ilk 20 sn girişi atla
    xs, ys = x[hs], y[hs]
    A = np.c_[2 * xs, 2 * ys, np.ones(len(xs))]; b = xs**2 + ys**2
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]; r = np.sqrt(c + cx**2 + cy**2)
    rr = np.hypot(xs - cx, ys - cy)
    print(f"Hold: giriş {t[i0] - T0:.1f} s, süre {t[i1] - t[i0]:.0f} s; merkez ({cx:.0f}, {cy:.0f}) m (NED x,y)")
    print(f"  yarıçap {r:.1f} m (std {rr.std():.1f} m); yer hızı ort {vg[hs].mean():.1f} min {vg[hs].min():.1f} max {vg[hs].max():.1f} m/s; irtifa ort {z[hs].mean():.1f} ± {z[hs].std():.1f} m")
    ca = d.get("airspeed_validated")
    if ca is not None:
        tc = ca.data["timestamp"] / 1e6; m = (tc > t[i0] + 20) & (tc < t[i1]); v = ca.data["calibrated_airspeed_m_s"][m]
        print(f"  hava hızı (sensör, gürültülü) ort {np.nanmean(v):.1f} min {np.nanmin(v):.1f} max {np.nanmax(v):.1f} m/s")
fs = d.get("failsafe_flags")
if fs is not None:
    cols = [c for c in fs.data if c not in ("timestamp",) and fs.data[c].dtype == bool]
    hits = [c for c in cols if fs.data[c].any() and c not in ("offboard_control_signal_lost", "manual_control_signal_lost", "gcs_connection_lost", "home_position_invalid")]
    print("failsafe bayrağı kalkan(ihmal edilenler hariç):", hits or "yok")
print("mesajlar:", [m.message for m in u.logged_messages if m.log_level <= 4][:8])
