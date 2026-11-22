#!/usr/bin/env python3
"""zincir_analiz.py — Adım 5.4: Tulpar tarama → LoRa → uav1 dalış zinciri. Gerçek poz (gz), sonuc.jsonl, qr_online.log, ulg.
Kullanım: (ROS + ~/venv_iha) python3 zincir_analiz.py <klasör> --ulg0 X.ulg --ulg1 Y.ulg"""
import argparse, csv, json, os, re, sys
import numpy as np
sys.path.insert(0, os.path.expanduser("~/iha_sim_v2"))
from qr_pass import PYLONS  # (N, E)

ap = argparse.ArgumentParser()
ap.add_argument("klasor"); ap.add_argument("--ulg0"); ap.add_argument("--ulg1")
ap.add_argument("--hedef", type=float, nargs=2, default=(-60.0, -60.0), help="gerçek hedef (N, E) uav0 çerçevesi")
a = ap.parse_args(); K = a.klasor
T = np.array([a.hedef[1], a.hedef[0], 0.02])   # ENU
d = json.loads(open(f"{K}/sonuc.jsonl").readline())
rows = list(csv.DictReader(open(f"{K}/gercek_poz.csv")))
def seri(ad):
    r = [x for x in rows if x["ad"] == ad]
    return np.array([float(x["t_sim"]) for x in r]), np.array([[float(x["x_e"]), float(x["y_n"]), float(x["z_u"])] for x in r])
t0, P0 = seri("tulpar_0"); t1, P1 = seri("advanced_plane_1")
n = min(len(t0), len(t1)); t0, t1, P0, P1 = t0[:n], t1[:n], P0[:n], P1[:n]
# gz gerçek pozu dünya koordinatında (uav0 doğma noktası = orijin) → dönüştürme gerekmez
W1 = P1
hav = W1[:, 2] > 3.0
d3 = np.linalg.norm(W1 - T, axis=1); dy = np.linalg.norm(W1[:, :2] - T[:2], axis=1)
i = int(np.argmin(np.where(hav, d3, 1e9)))
print(f"DALIŞ (gz gerçeği): hedefe en yakın 3B {d3[i]:.2f} m, yatay {dy[i]:.2f} m, irtifa {W1[i,2]:.2f} m (t_sim {t1[i]:.0f}); node'un (PX4) durma 3B {d['durma']['3b_m']} m")
iy = int(np.argmin(np.where(hav, dy, 1e9)))
ia = [j for j in range(max(iy-400, 0), iy) if dy[j] <= d["durma"]["yatay_m"] and hav[j]]
if ia: print(f"  DURMA anı (gz gerçeği; node yatay {d['durma']['yatay_m']} m'ye indiğinde): 3B {d3[ia[0]]:.2f} m, yatay {dy[ia[0]]:.2f} m, irtifa {W1[ia[0],2]:.2f} m")
w = (t1 > t1[i] - 15) & (t1 < t1[i] + 40)
print(f"  dip irtifa (gerçek, ±bölge) {W1[w,2].min():.1f} m; tırmanış sonu {W1[w,2].max():.1f} m")
# ayrım
ay = np.linalg.norm(P0 - W1, axis=1)
ucus = (P0[:, 2] > 20) & (W1[:, 2] > 5)
print(f"AYRIM uav0–uav1 3B en küçük (ikisi havada): {ay[ucus].min():.0f} m; dalış anında {ay[i]:.0f} m")
# direk
pl = np.array([[e, n_] for n_, e in PYLONS.values()])
dd = np.min(np.linalg.norm(W1[hav, None, :2] - pl[None], axis=2), axis=1)
print(f"uav1 en yakın direk (yatay, havada): {dd.min():.1f} m")
dd0 = np.min(np.linalg.norm(P0[P0[:,2]>20, None, :2] - pl[None], axis=2), axis=1)
print(f"uav0 en yakın direk (tüm uçuş): {dd0.min():.1f} m")
# konum kestirimi
r = d["plan"]["hedef"]; est = np.array([r[0] + 200.0, r[1] - 950.0])   # uav0 NED
print(f"KONUM KESTİRİMİ (zincirin kullandığı, ilk yayın): ({est[0]:.2f}, {est[1]:.2f}) → hata {np.hypot(est[0]-a.hedef[0], est[1]-a.hedef[1]):.2f} m")
log = open(f"{K}/qr_online.log", errors="ignore").read()
print("  qr_online yayın satırları:", len(re.findall(r"yayın [1-9]|HEDEF", log)))
for k in ("tespit_kalkis_komutu_sn", "tespit_hareket_sn", "tespit_tirmanis_sonu_sn", "toplam_gorev_sn"):
    print(f"  {k}: {d.get(k)}")
print(f"  uav1 kalkış: teker kesme {d['kalkis']['teker_kesme_sn']} s, yer koşusu {d['kalkis']['yer_kosusu_m']} m, hat girişi yanal {d['hat_girisi']['yanal_m']} m, irtifa {d['hat_girisi']['irtifa_m']} m")
print(f"  ölçülen dalış açısı {d['dalis']['olculen_aci_deg']}° (plan 8.19°), iniş {d['sonuc']}, boyuna {d['inis']['boyuna_m']} m, yanal {d['inis']['yanal_m']} m")
if a.ulg0:
    from pyulog import ULog
    for ad, p in (("uav0", a.ulg0), ("uav1", a.ulg1)):
        if not p: continue
        u = ULog(p)
        def g(nm):
            return next(x for x in u.data_list if x.name == nm).data
        vs = g("vehicle_status"); fs = vs["failsafe"]
        print(f"{ad} ulg: süre {(u.last_timestamp-u.start_timestamp)/1e6:.0f} s; failsafe örnek {int((fs>0).sum())}/{len(fs)}; nav_state kümesi {sorted(set(int(x) for x in vs['nav_state']))}")
        try:
            b = g("estimator_sensor_bias"); print(f"  accel_bias maks |.| {max(np.abs(b['accel_bias[0]']).max(),np.abs(b['accel_bias[1]']).max(),np.abs(b['accel_bias[2]']).max()):.3f}")
        except Exception as e: print("  bias yok", e)
        try:
            e = g("estimator_status"); print(f"  EKF reset sayacı (pos xy) {int(g('vehicle_local_position')['xy_reset_counter'].max())}")
        except Exception: pass
