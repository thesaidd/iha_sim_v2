#!/usr/bin/env python3
"""tarama_analiz.py — Adım 5.3: tam kutu taraması (alan_tarama.py --tam-tarama) çevrimdışı analizi.

Girdi: alan_tarama sonuç JSONL'ı (son satır), iz CSV'si (5 Hz PX4 kestirimi: t, n, e, irtifa, faz, hat_sirasi, t_duvar),
gz_gercek_poz.py gerçek poz CSV'si + saat.csv (sim↔duvar), qr_online degerlendirme.csv, (isteğe bağlı) PX4 ulg.
Çıktı: hat bazlı kutu girişi yanal hatası, dönüş yayı sapmaları, GERÇEK hat–direk en yakın mesafesi (gerçek poz),
gerçek kamera ayak izinden kapsama (kutu ızgarası), QR tespitleri (kadrajda/bulundu/çözüldü), ulg özeti.
Kullanım: UAV0_MODEL=tulpar python3 tarama_analiz.py --sonuc S.jsonl --iz IZ.csv --poz poz/gercek_poz.csv --eval E.csv [--ulg L.ulg]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--sonuc", required=True); ap.add_argument("--iz", required=True)
ap.add_argument("--poz", required=True); ap.add_argument("--eval"); ap.add_argument("--ulg")
ap.add_argument("--model", default="tulpar_0"); ap.add_argument("--kutu", type=float, default=300.0)
ap.add_argument("--t-bas", type=float, default=None, help="yalnız bu sim zamanından sonrası (çoklu koşulu ayırmak için)")
a = ap.parse_args()
os.environ["UAV0_MODEL"] = a.model.rsplit("_", 1)[0]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import qr_analyze as qa  # noqa: E402
from qr_pass import PYLONS  # noqa: E402  (ROS ortamı gerekir) — (N, E)

sonuc = json.loads(open(a.sonuc).read().splitlines()[-1])
print(f"sonuç: {sonuc['sonuc']}, uçulan hat {sonuc['ucular_hat']}/9, tarama {sonuc['tarama_sn']:.0f} s, neden: {sonuc['dur_nedeni']}")
print(f"  uçulmayan: {sonuc['ucmayan_hat']}; devralma s={sonuc.get('devralma_s_m')}, x={sonuc.get('devralma_x_m')}; uav1'e en küçük 3B ayrım {sonuc.get('ayrim_tarama')} m")

# --- kutu girişi yanal hata ---
u = sonuc["ucuslar"]
xg = np.array([x["x_giris_m"] for x in u]); xm = np.array([x["x_maks_m"] for x in u])
print(f"\nKUTU GİRİŞİ yanal hata (hat başına, {len(u)} hat): " + ", ".join(f"{v:+.2f}" for v in xg))
print(f"  |x_giris| medyan {np.median(np.abs(xg)):.2f} m, ort {np.mean(np.abs(xg)):.2f}, maks {np.abs(xg).max():.2f}; kutu içi |x|maks hat başına maks {xm.max():.2f} m (medyan {np.median(xm):.2f})")
d = sonuc["donusler"]
dx = np.array([x["x_maks_m"] for x in d]); ds = np.array([x["x_son_m"] for x in d])
print(f"DÖNÜŞ yayları ({len(d)}): |x|maks ort {np.mean(np.abs(dx)):.1f} m (maks {np.abs(dx).max():.1f}); çıkışta |x_son| ort {np.mean(np.abs(ds)):.1f} m (maks {np.abs(ds).max():.1f}); planlanan dönüş–direk en küçük {min(x['direk_min_m'] for x in d):.0f} m")
print(f"PLAN hat–direk en küçük (planlanan): {min(x['direk_min_m'] for x in u):.1f} m")

# --- gerçek poz ---
T, P, Q = [], [], []
for r in csv.DictReader(open(a.poz)):
    if r["ad"] == a.model:
        T.append(float(r["t_sim"])); P.append((float(r["x_e"]), float(r["y_n"]), float(r["z_u"]))); Q.append((float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])))
T, P, Q = np.array(T), np.array(P), np.array(Q)
saat = np.array([[float(r["t_sim"]), float(r["t_duvar"])] for r in csv.DictReader(open(os.path.join(os.path.dirname(a.poz), "saat.csv")))])
iz = list(csv.DictReader(open(a.iz)))
tw = np.array([float(r["t_duvar"]) for r in iz])
t_sim_iz = np.interp(tw, saat[:, 1], saat[:, 0])           # iz satırı → sim zamanı
faz = [r["faz"] for r in iz]; hs = [int(r["hat_sirasi"]) if r["hat_sirasi"] not in ("", None) else 0 for r in iz]
t0, t1 = t_sim_iz[0], t_sim_iz[-1]
if a.t_bas is not None: t0 = max(t0, a.t_bas)
m = (T >= t0) & (T <= t1)
Tm, Pm, Qm = T[m], P[m], Q[m]
print(f"\nTARAMA penceresi (sim): {t0:.0f}–{t1:.0f} s ({t1 - t0:.0f} s), gerçek poz örneği {m.sum()}")
# faz/hat etiketi gerçek poza (en yakın iz satırı)
k = np.clip(np.searchsorted(t_sim_iz, Tm), 0, len(iz) - 1)
fz = np.array([faz[i] for i in k]); hh = np.array([hs[i] for i in k])
pyl = np.array([(e, n) for (n, e) in PYLONS.values()])            # ENU (E, N)
dist = np.min(np.hypot(Pm[:, None, 0] - pyl[None, :, 0], Pm[:, None, 1] - pyl[None, :, 1]), axis=1)
print(f"GERÇEK en yakın direk mesafesi (yatay, tüm tarama): {dist.min():.1f} m; hat fazında {dist[fz == 'hat'].min():.1f} m; dönüş fazında {dist[fz != 'hat'].min():.1f} m")
for h_ in sorted(set(hh[fz == 'hat'])):
    mm = (fz == 'hat') & (hh == h_)
    if mm.sum() > 20: print(f"    hat sırası {h_}: gerçek en yakın direk {dist[mm].min():.1f} m, irtifa {Pm[mm, 2].mean():.1f} ± {Pm[mm, 2].std():.2f} m")
hat_m = (fz == 'hat')
print(f"hat fazında irtifa {Pm[hat_m, 2].mean():.2f} ± {Pm[hat_m, 2].std():.2f} m (min {Pm[hat_m, 2].min():.1f}, maks {Pm[hat_m, 2].max():.1f})")
# --- kapsama (gerçek kamera ayak izi) ---
H = a.kutu / 2
grid = np.zeros((int(a.kutu), int(a.kutu)), bool)          # 1 m hücre, (N, E) ∈ [-H, H)
from matplotlib.path import Path  # noqa: E402
corners = [(0, 0), (qa.W, 0), (qa.W, qa.H), (0, qa.H)]
adim = max(1, int(round(0.5 / (Tm[1] - Tm[0]))) ) if len(Tm) > 2 else 1   # ≈ 0,5 sn'de bir
poly_sayi = 0
for i in range(0, len(Tm), adim):
    R_wb = qa.quat_to_R(*Qm[i]); cam = Pm[i] + R_wb @ qa.CAM_T_FLU
    pts = []
    for (u_, v_) in corners:
        ds_ = np.array([1.0, -(u_ - qa.CX) / qa.FX, -(v_ - qa.CY) / qa.FY])
        dw = R_wb @ qa.R_S @ ds_
        if dw[2] >= -1e-6: pts = []; break
        tt = -cam[2] / dw[2]; pts.append((cam[0] + tt * dw[0], cam[1] + tt * dw[1]))
    if len(pts) != 4: continue
    pts = np.array(pts)
    e0, e1, n0, n1 = int(np.floor(pts[:, 0].min())), int(np.ceil(pts[:, 0].max())), int(np.floor(pts[:, 1].min())), int(np.ceil(pts[:, 1].max()))
    if e1 < -H or e0 > H or n1 < -H or n0 > H: continue
    ee, nn = np.meshgrid(np.arange(max(e0, -H), min(e1, H)), np.arange(max(n0, -H), min(n1, H)))
    if ee.size == 0: continue
    ins = Path(pts).contains_points(np.c_[ee.ravel() + 0.5, nn.ravel() + 0.5]).reshape(ee.shape)
    grid[(nn[ins] + H).astype(int), (ee[ins] + H).astype(int)] = True; poly_sayi += 1
print(f"KAPSAMA (gerçek kamera ayak izi birleşimi, 1 m ızgara, 300×300 m kutu): %{100 * grid.mean():.1f} ({poly_sayi} ayak izi); plan geometrik kapsama %44,4")

# --- QR tespitleri ---
if a.eval:
    rows = list(csv.DictReader(open(a.eval)))
    f = lambda x: float(x) if x not in ("", None) else math.nan
    kad = [r for r in rows if r["kadrajda_gercek"] == "True" and f(r["sim_s"]) >= (a.t_bas or 0)]
    cl, cur = [], []
    for r in kad:
        if cur and f(r["sim_s"]) - f(cur[-1]["sim_s"]) > 5: cl.append(cur); cur = []
        cur.append(r)
    if cur: cl.append(cur)
    print(f"\nQR: kadrajdaki küme sayısı {len(cl)}")
    for j, g in enumerate(cl, 1):
        he = np.array([f(r["hata_yatay"]) for r in g]); hg = np.array([f(r["gercek_poz_hata_yatay"]) for r in g])
        print(f"  küme {j}: sim {f(g[0]['sim_s']):.0f}–{f(g[-1]['sim_s']):.0f} s, {len(g)} kare, bulundu {sum(r['bulundu'] == 'True' for r in g)}, çözüldü {sum(r['cozuldu'] == 'True' for r in g)}; "
              f"PX4-tam hata ort {np.nanmean(he):.2f} m, gerçek-poz {np.nanmean(hg) * 100:.1f} cm")
    yan = [r for r in rows if r["bulundu"] == "True" and r["kadrajda_gercek"] != "True"]
    print(f"  kadrajda_gercek=False iken bulunan kare: {len(yan)} (yanlış tespit/geometri payı)")

# --- ulg ---
if a.ulg:
    from pyulog import ULog
    u_ = ULog(a.ulg, message_name_filter_list=["vehicle_attitude", "actuator_servos", "failsafe_flags", "vehicle_status", "vehicle_local_position"])
    dd = {x.name: x for x in u_.data_list}
    fs = dd["failsafe_flags"]; ign = {"offboard_control_signal_lost", "manual_control_signal_lost", "gcs_connection_lost", "home_position_invalid"}
    cols = [c for c in fs.data if c != "timestamp" and fs.data[c].dtype == bool and fs.data[c].any() and c not in ign]
    print(f"\nULG failsafe bayrakları (ihmal edilenler hariç): {cols or 'yok'}")
    nav = sorted(set(int(x) for x in dd["vehicle_status"].data["nav_state"])); print(f"  nav_state değerleri: {nav} (4=AUTO_LOITER, 14=OFFBOARD, 17=TAKEOFF; 5=RTL/18=LAND olmamalı)")
    sv = dd["actuator_servos"]
    print("  servo |cmd| maks / >0,95 payı: " + ", ".join(f"servo{j} {np.abs(sv.data[f'control[{j}]']).max():.2f} / %{(np.abs(sv.data[f'control[{j}]']) > 0.95).mean() * 100:.2f}" for j in range(4)))
    at = dd["vehicle_attitude"]; ta = at.data["timestamp"] / 1e6
    q = np.stack([at.data[f"q[{j}]"] for j in range(4)], 1); mm = (ta >= t0) & (ta <= t1)
    roll = np.degrees(np.arctan2(2 * (q[:, 0] * q[:, 1] + q[:, 2] * q[:, 3]), 1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2)))[mm]
    pitch = np.degrees(np.arcsin(np.clip(2 * (q[:, 0] * q[:, 2] - q[:, 3] * q[:, 1]), -1, 1)))[mm]
    print(f"  tarama süresince roll min {roll.min():.1f} maks {roll.max():.1f}°, |roll| ort {np.abs(roll).mean():.1f}°; pitch ort {pitch.mean():.2f}° (std {pitch.std():.2f}, min {pitch.min():.1f}, maks {pitch.max():.1f})")
