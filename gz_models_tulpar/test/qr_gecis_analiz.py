#!/usr/bin/env python3
"""qr_gecis_analiz.py — Adım 5.2: QR geçişlerinin tespit/konum/kadraj analizi (çevrimdışı; simülasyon gerekmez).

Girdi: qr_online degerlendirme.csv, gz_gercek_poz.py CSV'si (gerçek poz, 50 Hz), PX4 ulg, qr_online.log.
Her geçiş (kadrajdaki karelerin kümesi) için: bulma/çözme oranı, konum hatası (PX4-tam ve gerçek-poz), QR merkezinin görüntüde
v = CY çizgisini geçtiği anki hat konumu s_c (gerçek pozdan, qr_analyze.project_enu ile), gerçek ve PX4 pitch'i,
kadraj kayması = s_c − (−kamera_x) ile 60·tan(pitch) karşılaştırması.
Kullanım: UAV0_MODEL=tulpar python3 qr_gecis_analiz.py --eval E.csv --poz P.csv --ulg L.ulg --log qr_online.log [--model tulpar_0]
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--eval", required=True); ap.add_argument("--poz", required=True)
ap.add_argument("--ulg"); ap.add_argument("--log"); ap.add_argument("--model", default="tulpar_0")
ap.add_argument("--heading", type=float, default=210.0)
a = ap.parse_args()
# kamera kolu modele göre arac_modelleri.py'den gelir: analiz edilen model, ortam değişkeninden bağımsız seçilir
os.environ["UAV0_MODEL"] = a.model.rsplit("_", 1)[0]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import qr_analyze as qa  # noqa: E402
from arac_modelleri import kamera_kol_flu  # noqa: E402

rows = list(csv.DictReader(open(a.eval)))
f = lambda x: float(x) if x not in ("", None) else math.nan
kad = [r for r in rows if r["kadrajda_gercek"] == "True"]
# geçişlere ayır (kadrajdaki kareler arasında > 5 sn boşluk)
gecisler, cur = [], []
for r in kad:
    if cur and f(r["sim_s"]) - f(cur[-1]["sim_s"]) > 5: gecisler.append(cur); cur = []
    cur.append(r)
if cur: gecisler.append(cur)

# gerçek poz (yalnız model)
t_, P_, Q_ = [], [], []
for r in csv.DictReader(open(a.poz)):
    if r["ad"] == a.model:
        t_.append(float(r["t_sim"])); P_.append((float(r["x_e"]), float(r["y_n"]), float(r["z_u"]))); Q_.append((float(r["qw"]), float(r["qx"]), float(r["qy"]), float(r["qz"])))
t_, P_, Q_ = np.array(t_), np.array(P_), np.array(Q_)

psi = math.radians(a.heading); d_enu = np.array([math.sin(psi), math.cos(psi)])   # NED (cosψ, sinψ) → ENU (E, N)
T_enu = np.array([qa.QR_E, qa.QR_N])
QR_P = np.array([qa.QR_E, qa.QR_N, qa.QR_Z])

ulg = None
if a.ulg:
    from pyulog import ULog
    u = ULog(a.ulg); dd = {x.name: x for x in u.data_list}
    at = dd["vehicle_attitude"]; ta = at.data["timestamp"] / 1e6
    qq = np.stack([at.data[f"q[{j}]"] for j in range(4)], 1)
    ulg = (ta, np.degrees(np.arcsin(np.clip(2 * (qq[:, 0] * qq[:, 2] - qq[:, 3] * qq[:, 1]), -1, 1))))   # NED, + = burun yukarı

yayin_log = open(a.log).read() if a.log else ""
print(f"model {a.model}; kamera kolu FLU {tuple(kamera_kol_flu(0))}; geçiş sayısı {len(gecisler)}")
for gi, g in enumerate(gecisler, 1):
    ts = np.array([f(r["sim_s"]) for r in g])
    bul, coz = sum(r["bulundu"] == "True" for r in g), sum(r["cozuldu"] == "True" for r in g)
    print(f"\n=== Geçiş {gi}: sim {ts[0]:.1f}–{ts[-1]:.1f} s, kadrajda {len(g)} kare ({ts[-1] - ts[0] + 0.1:.1f} s)")
    print(f"  bulma {bul}/{len(g)} (%{100 * bul / len(g):.0f}), çözme {coz}/{len(g)} (%{100 * coz / len(g):.0f}), yayın {sum(r['yayin'] == 'True' for r in g)}")
    he = np.array([f(r["hata_yatay"]) for r in g]); hg = np.array([f(r["gercek_poz_hata_yatay"]) for r in g])
    pf = np.array([f(r["px4_gercek_yatay_fark"]) for r in g]); pz = np.array([f(r["px4_gercek_dikey_fark"]) for r in g])
    print(f"  konum hatası PX4-tam: ort {np.nanmean(he):.2f} m (min {np.nanmin(he):.2f}, maks {np.nanmax(he):.2f}); medyan {np.nanmedian(he):.2f} m")
    print(f"  konum hatası gerçek-poz yöntemi: ort {np.nanmean(hg) * 100:.1f} cm (maks {np.nanmax(hg) * 100:.1f} cm)")
    print(f"  PX4 − gerçek: yatay ort {np.nanmean(pf):.2f} m, dikey ort {np.nanmean(pz):.2f} m; kamera–QR 3B {np.nanmin([f(r['mesafe_3b']) for r in g]):.1f}–{np.nanmax([f(r['mesafe_3b']) for r in g]):.1f} m")
    # kadraj kayması (gerçek poz)
    m = (t_ > ts[0] - 2.0) & (t_ < ts[-1] + 2.0)
    tt, PP, QQ = t_[m], P_[m], Q_[m]
    s = np.array([(p[:2] - T_enu) @ d_enu for p in PP])
    vv, uu = np.zeros(len(tt)), np.zeros(len(tt))
    for k in range(len(tt)):
        R = qa.quat_to_R(*QQ[k]); u_, v_, _ = qa.project_enu(R, PP[k], QR_P); uu[k], vv[k] = u_, v_
    ok = np.isfinite(vv)
    ds = np.where(np.diff(np.sign(vv[ok] - qa.CY)) != 0)[0]
    if len(ds):
        k = np.where(ok)[0][ds[0]]
        sc = s[k] + (s[k + 1] - s[k]) * (qa.CY - vv[k]) / (vv[k + 1] - vv[k]); tc = tt[k]
        R = qa.quat_to_R(*QQ[k]); pitch_gt = -math.degrees(math.asin(max(-1, min(1, 2 * (QQ[k][0] * QQ[k][2] - QQ[k][3] * QQ[k][1])))))
        h = PP[k][2] - qa.QR_Z
        pit_px4 = float(np.interp(tc, ulg[0], ulg[1])) if ulg else math.nan
        lever = kamera_kol_flu(0)[0]
        shift = sc - (-lever)
        print(f"  QR merkezi görüntü ortasından (v={qa.CY:.0f}) geçtiği an: s_c = {sc:+.2f} m (t_sim {tc:.2f}); irtifa {h:.1f} m; u = {uu[k]:.0f} px (ortadan {uu[k] - qa.CX:+.0f})")
        print(f"  pitch (burun yukarı +): gerçek {pitch_gt:+.2f}°, PX4 {pit_px4:+.2f}° → beklenen kayma {h * math.tan(math.radians(pitch_gt)):+.2f} m (60·tan(pitch)); "
              f"ölçülen kayma s_c − (−{lever:.2f}) = {shift:+.2f} m")
    else:
        print("  v=CY geçişi pencerede yok")
    # kadraj penceresi: QR merkezinin görüntü içinde olduğu s aralığı
    inside = ok & (uu > 0) & (uu < qa.W) & (vv > 0) & (vv < qa.H)
    if inside.any():
        print(f"  QR merkezi kadrajda: s ∈ [{s[inside].min():+.1f}, {s[inside].max():+.1f}] m ({(s[inside].max() - s[inside].min()):.1f} m, {tt[inside].max() - tt[inside].min():.2f} s)")
    # sim_s aralığına düşen yayınlardan kenar/modül
    kn = [float(x) for x in re.findall(r"kenar ([0-9.]+) px", yayin_log)]
print("\n(Not: kenar/modül px, qr_online.log YAYIN satırlarında; kenar/21 = modül.)")
if yayin_log:
    kn = [float(x) for x in re.findall(r"kenar ([0-9.]+) px", yayin_log)]
    if kn: print(f"tüm yayınlar: {len(kn)} adet, kenar {min(kn):.1f}–{max(kn):.1f} px → modül {min(kn) / 21:.2f}–{max(kn) / 21:.2f} px (statik 60 m: 3,69 px)")
