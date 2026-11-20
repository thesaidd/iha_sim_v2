#!/usr/bin/env python3
"""
zincir_degerlendir.py — uçtan uca ArUco zinciri: uav1'in GERÇEK yörüngesi gerçek hedefe ne kadar yaklaştı?

Girdi: <klasör>/aruco_log/gercek_poz.csv (gz_gercek_poz.py), <klasör>/sonuc.jsonl (lora_yaklasma.py).
Hesap (dünya ENU, gz gerçeği): uav1 – hedef 3B ve yatay en yakın mesafe ve o anki irtifa; uav1'in planda kullandığı
hedefin (rapor enlem/boylamı) gerçek hedefe yatay uzaklığı (= zincirin konum hatası: geolocator + LoRa kodlaması).

Kullanım: python3 zincir_degerlendir.py <klasör> [--hedef-enu -60 -60] [--dunya gz_worlds/iha_arena_aruco.sdf]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(os.environ.get("BAPP", "~/bapp")))
from bap_geo.geodesy import MapProjection  # noqa: E402

from aruco_degerlendir import dunya_orijini  # noqa: E402


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("klasor")
    ap.add_argument("--hedef-enu", type=float, nargs=2, default=(-60.0, -60.0))
    ap.add_argument("--hedef-z", type=float, default=0.02)
    ap.add_argument("--dunya", default=os.path.join(here, "gz_worlds/iha_arena_aruco.sdf"))
    ap.add_argument("--model", default="advanced_plane_1")
    a = ap.parse_args()
    K = a.klasor
    w0 = dunya_orijini(a.dunya)
    T = np.array([a.hedef_enu[0], a.hedef_enu[1], a.hedef_z])
    rows = [r for r in csv.DictReader(open(os.path.join(K, "aruco_log/gercek_poz.csv"))) if r["ad"] == a.model]
    t = np.array([float(r["t_sim"]) for r in rows])
    P = np.array([[float(r["x_e"]), float(r["y_n"]), float(r["z_u"])] for r in rows])
    d3 = np.linalg.norm(P - T, axis=1)
    dh = np.linalg.norm(P[:, :2] - T[:2], axis=1)
    i = int(np.argmin(d3))
    out = {"en_yakin_3b_m": round(float(d3[i]), 2), "o_anda_yatay_m": round(float(dh[i]), 2),
           "o_anda_irtifa_m": round(float(P[i, 2] - T[2]), 2), "t_sim": round(float(t[i]), 2),
           "en_yakin_yatay_m": round(float(dh.min()), 2),
           "yatay_en_yakinda_irtifa_m": round(float(P[int(np.argmin(dh)), 2] - T[2]), 2)}
    res = [json.loads(line) for line in open(os.path.join(K, "sonuc.jsonl")) if line.strip()]
    r = res[-1]
    out["sonuc"] = r.get("sonuc")
    out["durma_nedeni"] = r.get("durma_nedeni")
    out["durma"] = r.get("durma")
    rap = r.get("rapor")
    if rap:
        lat_t, lon_t = MapProjection(w0[0], w0[1]).reproject(T[1], T[0])
        n, e = MapProjection(lat_t, lon_t).project(rap["lat"], rap["lon"])
        out["kullanilan_rapor"] = {k: rap[k] for k in ("marker_id", "n_obs", "sigma_n", "sigma_e", "sayac") if k in rap}
        out["rapor_hedef_hatasi_m"] = {"kuzey": round(n, 2), "dogu": round(e, 2), "yatay": round(math.hypot(n, e), 2)}
        # uav1'in kendi NED'indeki hedef (planda) → gerçek hedefe göre (uav1 EKF origin'i üzerinden)
        ref = rap.get("uav1_ref")
        if ref:
            tn, te = MapProjection(ref[0], ref[1]).project(lat_t, lon_t)
            pn, pe = r["plan"]["hedef"]
            out["plan_hedefi_hatasi_m"] = round(math.hypot(pn - tn, pe - te), 2)
    # Kamikaze kamerası okuması: lora_yaklasma özeti + uav1 geolocator kare günlüğü (gerçek mesafe gz pozundan)
    if r.get("okuma"):
        out["kamikaze_okuma_ozet"] = r["okuma"]
    kp = os.path.join(K, "aruco_log_uav1", "kareler.csv")
    if os.path.exists(kp):
        kr = [x for x in csv.DictReader(open(kp)) if x["n_det"] != "0"]
        ok = {"tespitli_kare": len(kr)}
        if kr:
            ts = np.array([float(x["t_cam_s"]) for x in kr])
            idx = np.clip(np.searchsorted(t, ts), 0, len(t) - 1)
            ok["ilk_okuma_gercek_3b_m"] = round(float(d3[idx[0]]), 1)
            ok["son_okuma_gercek_3b_m"] = round(float(d3[idx[-1]]), 1)
            ok["kenar_px_aralik"] = [round(min(float(x["side_px"]) for x in kr), 1), round(max(float(x["side_px"]) for x in kr), 1)]
            ok["id"] = sorted({int(x["marker_id"]) for x in kr})
            gref = os.path.join(K, "aruco_log_uav1", "geo_ref.json")
            fn = [(float(x["fused_n"]), float(x["fused_e"])) for x in kr if x.get("fused_n")]
            if fn and os.path.exists(gref):
                g = json.load(open(gref))
                lat_t, lon_t = MapProjection(w0[0], w0[1]).reproject(T[1], T[0])
                tn, te = MapProjection(g["lat"], g["lon"]).project(lat_t, lon_t)
                ok["kamikaze_kestirim_hatasi_m"] = round(math.hypot(fn[-1][0] - tn, fn[-1][1] - te), 2)
        out["kamikaze_okuma"] = ok
    print(json.dumps(out, ensure_ascii=False, indent=1))
    json.dump(out, open(os.path.join(K, "zincir_degerlendirme.json"), "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
