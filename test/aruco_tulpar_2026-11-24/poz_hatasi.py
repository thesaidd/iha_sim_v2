#!/usr/bin/env python3
"""poz_hatasi.py — PX4 kestirimi − gz gerçeği (uçuş boyunca): yaw, roll, pitch, mutlak yatay konum; rüzgâr etkisi.

Girdi: <koşu>/uav0.ulg (PX4 günlüğü), <koşu>/gercek_poz.csv (gz_gercek_poz.py: t_sim, ad, ENU konum, FLU→ENU kuaterniyon).
PX4 SITL zamanı gz sim saatidir (µs) → eşleşme t_ulg = t_sim (aynı saat; F1'de doğrulandı: düz uçuşta yaw farkı ~0).
Mutlak konum: vehicle_global_position enlem/boylam ↔ gerçek ENU'nun dünya <spherical_coordinates>'ıyla enlem/boylamı
(GNSS biası buraya düşer; EKF yerel konumu origin'le birlikte kaydığından sabit biası GÖSTERMEZ).
Rüzgâr: gerçek yer hızı ↔ PX4 airspeed_validated (gerçek TAS) farkı + PX4 wind kestirimi.
Kullanım: python3 poz_hatasi.py <koşu_klasörü> [--model tulpar_0] [--min-irtifa 30] [--json çıktı.json]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
from pyulog import ULog

SIM = os.path.expanduser("~/sim/iha_sim_v2")
sys.path.insert(0, SIM)
sys.path.insert(0, os.path.expanduser(os.environ.get("BAPP", "~/bapp")))
from aruco_degerlendir import dunya_orijini  # noqa: E402
from bap_geo.geodesy import MapProjection  # noqa: E402


def q2rpy(w, x, y, z):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def ist(x):
    x = np.abs(np.asarray(x))
    return {"medyan": round(float(np.median(x)), 3), "p95": round(float(np.percentile(x, 95)), 3),
            "maks": round(float(x.max()), 3), "n": int(x.size)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kosu")
    ap.add_argument("--model", default="tulpar_0")
    ap.add_argument("--ulg", default="uav0.ulg")
    ap.add_argument("--min-irtifa", type=float, default=30.0)
    ap.add_argument("--dunya", default=os.path.join(SIM, "gz_worlds/iha_arena_aruco.sdf"))
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    rows = [r for r in csv.DictReader(open(os.path.join(a.kosu, "gercek_poz.csv"))) if r["ad"] == a.model]
    tg = np.array([float(r["t_sim"]) for r in rows])
    P = np.array([[float(r[k]) for k in ("x_e", "y_n", "z_u")] for r in rows])
    Q = np.array([[float(r[k]) for k in ("qw", "qx", "qy", "qz")] for r in rows])
    # gerçek: FLU→ENU. Euler (ENU/FLU) → NED/FRD: roll aynı, pitch aynı (işaret: FLU pitch burun aşağı +; FRD burun yukarı +)
    r_e, p_e, y_e = q2rpy(*Q.T)
    g_roll, g_pitch, g_yaw = r_e, -p_e, wrap(np.pi / 2 - y_e)
    u = ULog(os.path.join(a.kosu, a.ulg))
    d = {x.name + (f"_{x.multi_id}" if x.multi_id else ""): x.data for x in u.data_list}
    att = d["vehicle_attitude"]
    ta = att["timestamp_sample"] * 1e-6 if "timestamp_sample" in att else att["timestamp"] * 1e-6
    pr, pp, py = q2rpy(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
    alt_g = np.interp(ta, tg, P[:, 2])
    m = (alt_g > a.min_irtifa) & (ta > tg[0]) & (ta < tg[-1])
    sonuc = {"kosu": os.path.basename(os.path.abspath(a.kosu))}
    e_yaw = np.degrees(wrap(py[m] - np.interp(ta[m], tg, np.unwrap(g_yaw))))
    e_roll = np.degrees(pr[m] - np.interp(ta[m], tg, g_roll))
    e_pitch = np.degrees(pp[m] - np.interp(ta[m], tg, g_pitch))
    sonuc["yaw_deg"] = {**ist(e_yaw), "ort_isaretli": round(float(e_yaw.mean()), 3)}
    sonuc["roll_deg"] = {**ist(e_roll), "ort_isaretli": round(float(e_roll.mean()), 3)}
    sonuc["pitch_deg"] = {**ist(e_pitch), "ort_isaretli": round(float(e_pitch.mean()), 3)}
    # yaw hatası × gerçek yön (yöne bağlı mag hatası): 8 sektör
    yon = np.degrees(np.interp(ta[m], tg, np.unwrap(g_yaw))) % 360
    sonuc["yaw_hatasi_yon_sektor"] = {f"{s * 45:03d}": round(float(e_yaw[(yon >= s * 45) & (yon < s * 45 + 45)].mean()), 2)
                                      for s in range(8) if ((yon >= s * 45) & (yon < s * 45 + 45)).sum() > 20}
    # mutlak yatay konum
    gp = d.get("vehicle_global_position")
    if gp is not None:
        tp = gp["timestamp_sample"] * 1e-6 if "timestamp_sample" in gp else gp["timestamp"] * 1e-6
        w0 = dunya_orijini(a.dunya)
        mp = MapProjection(w0[0], w0[1])
        n_p, e_p = mp.project(gp["lat"], gp["lon"])
        mm = (np.interp(tp, tg, P[:, 2]) > a.min_irtifa) & (tp > tg[0]) & (tp < tg[-1])
        en = np.asarray(n_p)[mm] - np.interp(tp[mm], tg, P[:, 1])
        ee = np.asarray(e_p)[mm] - np.interp(tp[mm], tg, P[:, 0])
        sonuc["mutlak_yatay_m"] = {**ist(np.hypot(en, ee)), "ort_N": round(float(en.mean()), 3), "ort_E": round(float(ee.mean()), 3)}
        h_p = gp["alt"][mm] - w0[2]
        sonuc["irtifa_m"] = {**ist(h_p - np.interp(tp[mm], tg, P[:, 2])), "ort_isaretli": round(float((h_p - np.interp(tp[mm], tg, P[:, 2])).mean()), 3)}
    # rüzgâr: gerçek yer hızı vektörü, PX4 rüzgâr kestirimi
    v = np.gradient(P[:, :2], tg, axis=0)
    gs = np.hypot(v[:, 0], v[:, 1])
    mh = P[:, 2] > a.min_irtifa
    sonuc["gercek_yer_hizi_m_s"] = {"p05": round(float(np.percentile(gs[mh], 5)), 2), "medyan": round(float(np.median(gs[mh])), 2),
                                    "p95": round(float(np.percentile(gs[mh], 95)), 2)}
    for ad in ("wind", "wind_estimate"):
        if ad in d:
            wd = d[ad]
            kn, ke = ("windspeed_north", "windspeed_east")
            if kn in wd:
                tw = wd["timestamp"] * 1e-6
                mw = np.interp(tw, tg, P[:, 2]) > a.min_irtifa
                sonuc["px4_ruzgar_m_s"] = {"N_medyan": round(float(np.median(wd[kn][mw])), 2),
                                           "E_medyan": round(float(np.median(wd[ke][mw])), 2), "n": int(mw.sum())}
            break
    print(json.dumps(sonuc, ensure_ascii=False, indent=1))
    if a.json:
        json.dump(sonuc, open(a.json, "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
