#!/usr/bin/env python3
"""
tarama_video.py — kaydedilmiş tarama koşusundan izlenebilir tekrar videosu (simülasyon GEREKMEZ).

Sol: uav0 kamera karesi (kare_kaydedici.py; gs_kamera çıkışı = geolocator'ın gördüğü), o karede bapp MarkerDetector
tespiti yeşil dörtgenle. Sağ: üstten harita — arama alanı, Gazebo gerçek izi, o anki kamera ayak izi (sarı) ve o
ana dek BİRİKEN tespit kapsaması (mavi; marker tamamen ≥ kenar_px içeride görülebilen zemin), marker'ın gerçek yeri.
Üst şerit: sim zamanı, irtifa, roll, kapsama yüzdesi, tespit sayısı.

Kullanım: ~/bapp/.venv-ros/bin/python tarama_video.py --kosu test/kamera_aci_2026-11-20/b1 [--fps 8]
Girdi: <kosu>/kareler/, <kosu>/aruco_log/ (gercek_poz.csv, geo_ref.json, kamera.json), <kosu>/tarama.log.
Çıktı: <kosu>/tarama_video.mp4 (mp4v; açılamazsa .avi XVID).
"""
import argparse
import json
import math
import os

import cv2
import numpy as np

import kamera_geometri as kg
from aruco_degerlendir import Gercek, dunya_orijini, oku_csv
from bap_geo.aruco import MarkerDetector
from bap_geo.geodesy import MapProjection
from kapsama_analiz import tarama_penceresi, zemin_d
from arac_modelleri import gz_ad, kamera_kol_frd


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kosu", required=True)
    ap.add_argument("--dunya", default=os.path.join(here, "gz_worlds/iha_arena_aruco.sdf"))
    ap.add_argument("--daire", type=float, default=100.0)
    ap.add_argument("--hedef-enu", type=float, nargs=2, default=(-60.0, -60.0))
    ap.add_argument("--kenar-px", type=float, default=15.0)
    ap.add_argument("--fps", type=float, default=8.0)
    ap.add_argument("--on-pay", type=float, default=20.0, help="taramadan önce / sonra eklenen süre [s]")
    a = ap.parse_args()
    K, L = os.path.join(a.kosu, "kareler"), os.path.join(a.kosu, "aruco_log")
    ref = json.load(open(os.path.join(L, "geo_ref.json")))
    ref = (ref["lat"], ref["lon"], ref["alt"])
    w0 = dunya_orijini(a.dunya)
    gercek = Gercek(oku_csv(os.path.join(L, "gercek_poz.csv")), gz_ad(0), w0, ref)
    kam = kg.Kamera.json_oku(os.path.join(L, "kamera.json"))
    zd = zemin_d(w0, ref)
    lat, lon = MapProjection(w0[0], w0[1]).reproject(a.hedef_enu[1], a.hedef_enu[0])
    hedef = np.array(MapProjection(ref[0], ref[1]).project(lat, lon))
    t0, t1 = tarama_penceresi(os.path.join(a.kosu, "tarama.log"), os.path.join(L, "saat.csv"))
    t0, t1 = t0 - a.on_pay, t1 + a.on_pay
    det = MarkerDetector(target_ids=[0])
    R = a.daire
    S, ext = 640, R + 190.0                            # harita: 640 px kare; dönüşler alanın ~150 m dışına taşar
    g = 2 * ext / S

    def hp(ne):
        ne = np.asarray(ne, float)
        return np.stack([(ne[..., 1] + ext) / g, (ext - ne[..., 0]) / g], -1)

    taban = np.full((S, S, 3), 245, np.uint8)
    th = np.linspace(0, 2 * math.pi, 361)
    cv2.polylines(taban, [hp(np.stack([R * np.cos(th), R * np.sin(th)], -1)).astype(np.int32)], True, (60, 60, 60), 2)
    for v in range(-300, 301, 50):
        x = int(hp([0, v])[0])
        y = int(hp([v, 0])[1])
        if 0 <= x < S:
            cv2.line(taban, (x, 0), (x, S - 1), (225, 225, 225), 1)
        if 0 <= y < S:
            cv2.line(taban, (0, y), (S - 1, y), (225, 225, 225), 1)
    kaps = np.zeros((S, S), np.uint8)
    yy, xx = np.mgrid[0:S, 0:S]
    alan = np.hypot((xx + 0.5) * g - ext, ext - (yy + 0.5) * g) <= R
    A = alan.sum()
    iz = []
    rows = [r for r in oku_csv(os.path.join(K, "kareler.csv")) if t0 <= float(r["t_sim"]) <= t1]
    out = os.path.join(a.kosu, "tarama_video.mp4")
    W_, H_ = 728 + S, max(544, S) + 40
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W_, H_))
    if not vw.isOpened():
        out = out[:-4] + ".avi"
        vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"XVID"), a.fps, (W_, H_))
    n_tespit = 0
    tsec = np.array([[0, 0], [kam.w, 0], [kam.w, kam.h], [0, kam.h]], float)
    for r in rows:
        t = float(r["t_sim"])
        gg = gercek(t)
        if gg is None:
            continue
        p, R_nb = gg
        c = p + R_nb @ np.array(kamera_kol_frd(0))
        h = zd - c[2]
        roll = math.degrees(math.atan2(R_nb[2, 1], R_nb[2, 2]))
        R_nc = R_nb @ kg.R_NADIR
        im = cv2.imread(os.path.join(K, r["dosya"]))
        d = det.detect(im)
        if 0 in d:
            n_tespit += 1
            cv2.polylines(im, [d[0].astype(np.int32)], True, (0, 255, 0), 3)
            x, y = d[0].mean(0).astype(int)
            cv2.circle(im, (int(x), int(y)), 60, (0, 255, 0), 3)
            cv2.putText(im, "ArUco id 0", (int(x) + 65, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        sol = cv2.resize(im, (728, 544), interpolation=cv2.INTER_AREA)
        harita = taban.copy()
        if h > 30:
            menzil = h / max(R_nc[2, 2], 0.2)
            dp = a.kenar_px + math.sqrt(2) * kam.fx / menzil
            ic = np.array([[dp, dp], [kam.w - dp, dp], [kam.w - dp, kam.h - dp], [dp, kam.h - dp]])
            cv2.fillPoly(kaps, [hp(kg.zemine(kam, ic, h, R_nc) + c[:2]).astype(np.int32)], 1)
            ayak = hp(kg.zemine(kam, tsec, h, R_nc) + c[:2]).astype(np.int32)
        else:
            ayak = None
        mavi = kaps.astype(bool)
        harita[mavi] = (0.55 * harita[mavi] + 0.45 * np.array([230, 150, 60])).astype(np.uint8)
        harita[alan & ~mavi] = (0.8 * harita[alan & ~mavi] + 0.2 * np.array([80, 80, 230])).astype(np.uint8)
        cv2.polylines(harita, [hp(np.stack([R * np.cos(th), R * np.sin(th)], -1)).astype(np.int32)], True, (60, 60, 60), 2)
        iz.append(hp(c[:2]))
        if len(iz) > 1:
            cv2.polylines(harita, [np.array(iz, np.int32)], False, (0, 120, 255), 2)
        if ayak is not None:
            cv2.polylines(harita, [ayak], True, (0, 200, 255), 2)
        hx, hy = hp(hedef).astype(int)
        cv2.drawMarker(harita, (int(hx), int(hy)), (200, 0, 200), cv2.MARKER_STAR, 18, 2)
        x, y = iz[-1].astype(int)
        cv2.circle(harita, (int(x), int(y)), 6, (0, 0, 220), -1)
        kare = np.full((H_, W_, 3), 30, np.uint8)
        kare[40:40 + 544, :728] = sol
        kare[40:40 + S, 728:] = harita
        oran = (mavi & alan).sum() / A
        cv2.putText(kare, f"t {t:7.1f} s  irtifa {h:5.1f} m  roll {roll:+5.1f} deg  tespit kapsamasi %{100 * oran:5.1f}  "
                           f"ArUco tespitli kare {n_tespit}", (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(kare, "kamera (gs_kamera, 1456x1088 -> 1/2)", (10, 40 + 544 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 0), 1)
        cv2.putText(kare, "mavi: tespit kapsamasi, sari: ayak izi", (738, 40 + S - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (40, 40, 40), 1)
        vw.write(kare)
    vw.release()
    print(f"video: {out} ({len(rows)} kare, {len(rows) / a.fps:.0f} sn @ {a.fps:g} fps; tespitli {n_tespit})")


if __name__ == "__main__":
    main()
