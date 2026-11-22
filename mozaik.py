#!/usr/bin/env python3
"""
mozaik.py — kaydedilmiş uav0 karelerinden doğrudan jeoreferanslı ortomozaik + görüntü birleştirme (stitching) sınaması.

Her kare (kare_kaydedici.py) o anki Gazebo GERÇEK pozuyla (gercek_poz.csv; aruco_degerlendir.Gercek) düz zemine
izdüşürülür (homografi: görüntü köşeleri → zemin → mozaik pikseli; kuzey yukarı) ve ağırlıklı ortalamayla birleştirilir.
Gerçek pozla mozaikteki dikiş hatası yalnız kamera modeli / zaman damgası hatasını gösterir; uçakta poz PX4
kestirimidir (simde ~0,1–0,4 m konum, 0,2–0,7° attitude → 100 m'de 0,4–1,2 m dikiş kayması beklenir).

Birleştirme sınaması (özellik tabanlı stitcher bu örtüşmeyle çalışır mı): komşu GEÇİŞLERDEN (yan örtüşme) ve aynı geçişin
ardışık karelerinden (ön örtüşme) kare çiftleri seçilir; ikisi de aynı zemin ızgarasına çarpıtılır, örtüşme bölgesinde
ORB + oran testi + RANSAC (benzerlik) ile eşlenir → iç eşleşme sayısı ve kalan kayma [m] (gerçek pozla ≈ 0 olmalı).

Kullanım: ~/bapp/.venv-ros/bin/python mozaik.py --kareler DIR --log ARUCO_LOG [--tarama-log tarama.log] --cikis DIR
"""
import argparse
import csv
import json
import math
import os

import cv2
import numpy as np

import kamera_geometri as kg
from aruco_degerlendir import Gercek, dunya_orijini, oku_csv
from bap_geo.geodesy import MapProjection
from kapsama_analiz import tarama_penceresi, zemin_d
from arac_modelleri import gz_ad, kamera_kol_frd


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kareler", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--tarama-log", default=None)
    ap.add_argument("--dunya", default=os.path.join(here, "gz_worlds/iha_arena_aruco.sdf"))
    ap.add_argument("--model", default=gz_ad(0))
    ap.add_argument("--t-body-cam", type=float, nargs=3, default=kamera_kol_frd(0))
    ap.add_argument("--daire", type=float, default=100.0)
    ap.add_argument("--merkez", type=float, nargs=2, default=(0.0, 0.0))
    ap.add_argument("--hedef-enu", type=float, nargs=2, default=(-60.0, -60.0))
    ap.add_argument("--gsd", type=float, default=0.10, help="mozaik çözünürlüğü [m/px]")
    ap.add_argument("--max-roll", type=float, default=8.0, help="bundan çok yatık kareler mozaiğe girmez [°]")
    ap.add_argument("--cift", type=int, default=6, help="komşu geçiş çifti başına sınanan kare çifti")
    ap.add_argument("--cikis", required=True)
    a = ap.parse_args()
    os.makedirs(a.cikis, exist_ok=True)

    ref = json.load(open(os.path.join(a.log, "geo_ref.json")))
    ref = (ref["lat"], ref["lon"], ref["alt"])
    w0 = dunya_orijini(a.dunya)
    gercek = Gercek(oku_csv(os.path.join(a.log, "gercek_poz.csv")), a.model, w0, ref)
    kam = kg.Kamera.json_oku(os.path.join(a.log, "kamera.json"))
    zd = zemin_d(w0, ref)
    lat, lon = MapProjection(w0[0], w0[1]).reproject(a.hedef_enu[1], a.hedef_enu[0])
    hedef = np.array(MapProjection(ref[0], ref[1]).project(lat, lon))
    pen = tarama_penceresi(a.tarama_log, os.path.join(a.log, "saat.csv")) if a.tarama_log else (-math.inf, math.inf)
    tbc = np.array(a.t_body_cam)

    R = a.daire
    ext = R + 45.0
    N1, E0 = a.merkez[0] + ext, a.merkez[1] - ext
    S = int(round(2 * ext / a.gsd))

    def mozaige(ne):                                     # (…, 2) N, E → (…, 2) sütun, satır
        return np.stack([(ne[..., 1] - E0) / a.gsd, (N1 - ne[..., 0]) / a.gsd], -1)

    img_k = np.array([[0, 0], [kam.w, 0], [kam.w, kam.h], [0, kam.h]], np.float32)
    kareler = []
    for r in oku_csv(os.path.join(a.kareler, "kareler.csv")):
        t = float(r["t_sim"])
        if not pen[0] <= t <= pen[1]:
            continue
        g = gercek(t)
        if g is None:
            continue
        p, R_nb = g
        roll = math.degrees(math.atan2(R_nb[2, 1], R_nb[2, 2]))
        yaw = math.degrees(math.atan2(R_nb[1, 0], R_nb[0, 0]))
        c = p + R_nb @ tbc
        h = zd - c[2]
        if h < 30 or abs(roll) > a.max_roll:
            continue
        R_nc = R_nb @ kg.R_NADIR
        ne = kg.zemine(kam, img_k.astype(float), h, R_nc) + c[:2]
        # alanı görmeyen kare (dönüş / uzatma) mozaiğe girmez
        if not (np.hypot(*(ne.mean(0) - np.array(a.merkez))) < R + 40):
            continue
        kareler.append({"t": t, "dosya": r["dosya"], "ne": ne, "c": c[:2], "yaw": yaw, "roll": roll, "h": h})
    print(f"mozaik: {len(kareler)} kare (pencere {pen[0]:.0f}–{pen[1]:.0f} s)", flush=True)
    if not kareler:
        raise SystemExit("kare yok")
    # geçişler: yön 90°'den çok değişirse ya da 3 sn boşluk → yeni geçiş
    gecis, son = 0, None
    for k in kareler:
        if son is not None and (k["t"] - son["t"] > 3.0 or abs(math.remainder(k["yaw"] - son["yaw"], 360)) > 90):
            gecis += 1
        k["gecis"] = gecis
        son = k

    acc = np.zeros((S, S, 3), np.float32)
    wsum = np.zeros((S, S), np.float32)
    # kenara doğru azalan ağırlık (dikişleri yumuşatır, ortalamada keskinliği korur)
    yy, xx = np.mgrid[0:kam.h, 0:kam.w].astype(np.float32)
    agirlik = (np.minimum(xx, kam.w - 1 - xx) / (kam.w / 2)) * (np.minimum(yy, kam.h - 1 - yy) / (kam.h / 2)) + 1e-3
    for k in kareler:
        im = cv2.imread(os.path.join(a.kareler, k["dosya"]))
        H = cv2.getPerspectiveTransform(img_k, mozaige(k["ne"]).astype(np.float32))
        k["H"] = H
        x0, y0 = np.floor(mozaige(k["ne"]).min(0)).astype(int)
        x1, y1 = np.ceil(mozaige(k["ne"]).max(0)).astype(int)
        x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, S), min(y1, S)
        if x1 <= x0 or y1 <= y0:
            continue
        T = np.array([[1, 0, -x0], [0, 1, -y0], [0, 0, 1.0]])
        sz = (x1 - x0, y1 - y0)
        wi = cv2.warpPerspective(im, T @ H, sz, flags=cv2.INTER_LINEAR).astype(np.float32)
        ww = cv2.warpPerspective(agirlik, T @ H, sz, flags=cv2.INTER_LINEAR)
        acc[y0:y1, x0:x1] += wi * ww[..., None]
        wsum[y0:y1, x0:x1] += ww
    moz = np.where(wsum[..., None] > 0, acc / np.maximum(wsum[..., None], 1e-6), 255).astype(np.uint8)
    # çizim: alan çemberi, marker, geçiş izleri
    th = np.linspace(0, 2 * math.pi, 721)
    ci = mozaige(np.stack([a.merkez[0] + R * np.cos(th), a.merkez[1] + R * np.sin(th)], -1)).astype(np.int32)
    ciz = moz.copy()
    cv2.polylines(ciz, [ci], True, (0, 0, 0), 3)
    renk = [(0, 140, 255), (255, 80, 0), (0, 200, 0), (200, 0, 200), (0, 200, 200), (100, 100, 255)]
    for k in kareler:
        x, y = mozaige(k["c"]).astype(int)
        cv2.circle(ciz, (int(x), int(y)), 4, renk[k["gecis"] % len(renk)], -1)
    hx, hy = mozaige(hedef).astype(int)
    cv2.circle(ciz, (int(hx), int(hy)), 30, (255, 0, 255), 3)
    cv2.imwrite(os.path.join(a.cikis, "mozaik.jpg"), ciz, [cv2.IMWRITE_JPEG_QUALITY, 88])
    kes = moz[max(hy - 60, 0):hy + 60, max(hx - 60, 0):hx + 60]
    cv2.imwrite(os.path.join(a.cikis, "mozaik_marker.png"), cv2.resize(kes, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))

    # ── birleştirme sınaması ──
    orb = cv2.ORB_create(4000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)

    def esle(ki, kj):
        pi = mozaige(ki["ne"]).astype(np.float32)
        pj = mozaige(kj["ne"]).astype(np.float32)
        alan_, ort = cv2.intersectConvexConvex(pi, pj)
        if ort is None or alan_ <= 0:
            return None
        ort = ort.reshape(-1, 2)
        x0, y0 = np.floor(ort.min(0)).astype(int)
        x1, y1 = np.ceil(ort.max(0)).astype(int)
        # örtüşmeyi yerel ölçekte (≈ GSD 6 cm) çarpıt: mozaik GSD'sinden bağımsız
        s = a.gsd / 0.06
        T = np.array([[s, 0, -x0 * s], [0, s, -y0 * s], [0, 0, 1.0]])
        sz = (int((x1 - x0) * s), int((y1 - y0) * s))
        if min(sz) < 40:
            return None
        g = []
        for k in (ki, kj):
            im = cv2.imread(os.path.join(a.kareler, k["dosya"]), cv2.IMREAD_GRAYSCALE)
            g.append(cv2.warpPerspective(im, T @ k["H"], sz, flags=cv2.INTER_LINEAR))
        m = ((g[0] > 0) & (g[1] > 0)).astype(np.uint8) * 255
        k1, d1 = orb.detectAndCompute(g[0], m)
        k2, d2 = orb.detectAndCompute(g[1], m)
        if d1 is None or d2 is None or len(k1) < 10 or len(k2) < 10:
            return {"ortusme_m2": round(alan_ * a.gsd ** 2, 1), "ic": 0, "kayma_m": None}
        ms = [x[0] for x in bf.knnMatch(d1, d2, k=2) if len(x) == 2 and x[0].distance < 0.8 * x[1].distance]
        if len(ms) < 8:
            return {"ortusme_m2": round(alan_ * a.gsd ** 2, 1), "ic": len(ms), "kayma_m": None}
        A_, inl = cv2.estimateAffinePartial2D(np.float32([k1[x.queryIdx].pt for x in ms]),
                                             np.float32([k2[x.trainIdx].pt for x in ms]), method=cv2.RANSAC,
                                             ransacReprojThreshold=3.0)
        n_ic = int(inl.sum()) if inl is not None else 0
        kay = None if A_ is None else round(float(np.hypot(A_[0, 2], A_[1, 2])) * 0.06, 3)
        return {"ortusme_m2": round(alan_ * a.gsd ** 2, 1), "ic": n_ic, "kayma_m": kay}

    gecisler = sorted({k["gecis"] for k in kareler})
    merk = {}
    for gp in gecisler:
        q = np.array([k["c"] for k in kareler if k["gecis"] == gp])
        merk[gp] = q.mean(0)
    sonuc = {"kare": len(kareler), "gecis": len(gecisler), "yan": [], "on": []}
    # yan örtüşme: her geçişin yanal olarak en yakın komşusu
    y0 = math.radians(np.median([k["yaw"] for k in kareler if k["gecis"] == gecisler[0]]))
    nrm = np.array([-math.sin(y0), math.cos(y0)])
    sira = sorted(gecisler, key=lambda gp: float(merk[gp] @ nrm))
    for ga, gb in zip(sira, sira[1:]):
        A_ = [k for k in kareler if k["gecis"] == ga]
        B_ = [k for k in kareler if k["gecis"] == gb]
        aday = []
        for ka in A_[:: max(1, len(A_) // (3 * a.cift))]:
            kb = min(B_, key=lambda k: np.linalg.norm(k["ne"].mean(0) - ka["ne"].mean(0)))
            aday.append((ka, kb))
        aday = sorted(aday, key=lambda p_: abs(float((p_[0]["ne"].mean(0) - np.array(a.merkez)) @ np.array([math.cos(y0), math.sin(y0)]))))[:a.cift]
        for ka, kb in aday:
            r_ = esle(ka, kb)
            if r_:
                r_.update(gecisler=[ga, gb], aralik_m=round(abs(float((merk[gb] - merk[ga]) @ nrm)), 1))
                sonuc["yan"].append(r_)
    for gp in gecisler[:3]:
        G = [k for k in kareler if k["gecis"] == gp]
        for ka, kb in list(zip(G, G[1:]))[:: max(1, len(G) // a.cift)][:a.cift]:
            r_ = esle(ka, kb)
            if r_:
                r_.update(gecis=gp, aralik_m=round(float(np.linalg.norm(kb["c"] - ka["c"])), 1))
                sonuc["on"].append(r_)
    for tur in ("yan", "on"):
        ic = [r_["ic"] for r_ in sonuc[tur]]
        kay = [r_["kayma_m"] for r_ in sonuc[tur] if r_["kayma_m"] is not None]
        sonuc[f"{tur}_ozet"] = {"cift": len(ic), "ic_medyan": float(np.median(ic)) if ic else None,
                                "ic_min": int(min(ic)) if ic else None,
                                "basarili_cift(ic≥30)": int(sum(i >= 30 for i in ic)),
                                "kayma_medyan_m": round(float(np.median(kay)), 3) if kay else None,
                                "kayma_maks_m": round(float(max(kay)), 3) if kay else None}
    json.dump(sonuc, open(os.path.join(a.cikis, "birlestirme.json"), "w"), ensure_ascii=False, indent=1)
    print(json.dumps({k: v for k, v in sonuc.items() if k.endswith("ozet") or k in ("kare", "gecis")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
