#!/usr/bin/env python3
"""
kapsama_analiz.py — uçulmuş taramanın GERÇEK kamera kapsaması (uçuş sonrası; Gazebo gerçek pozuyla).

Plan kapsaması (tarama_plan.kapsama) hat merkezlerini ve sabit W'yi varsayar; bu araç her kamera karesinin
o anki gerçek pozla (gz_gercek_poz.py → gercek_poz.csv, roll/pitch dahil) zemindeki ayak izini çıkarır:

  - görüntü kapsaması: alanın en az bir karede GÖRÜLEN payı (görüntü birleştirme / haritalama için),
  - tespit kapsaması: 2 m marker'ın (her yaw'da) tamamen ≥ kenar_px içeride görüleceği hücrelerin payı
    (kamera_geometri tespit ölçütü; köşe ≥ kenar_px, kenar ≥ min_kenar_px),
  - ATLANAN bölgeler: tespit kapsaması 0 olan alan hücreleri (m², en büyük boşluğun yeri),
  - TEKRAR: hücreyi tespit edilebilir gören ayrı geçiş sayısı (geçiş = alanı gören ardışık kare dizisi);
    ≥ 2 geçişte görülen alan payı = şerit örtüşmesi (tespit için gereksiz, birleştirme için gerekli),
  - geçişler arası yan örtüşme (komşu geçişlerin görüntü bantları), kare başına hat boyu örtüşme,
  - süre: ilk alan karesinden son alan karesine; alan üstünde geçen süre payı (yol verimi),
  - marker: geometriye göre tespit edilebilir kare sayısı ↔ geolocator'ın tespit ettiği (kareler.csv n_det).

Girdi: aruco_geolocator --log dizini (kareler.csv kare zamanları, geo_ref.json) + gercek_poz.csv.
Kamera: --kamera-json (varsayılan <log>/kamera.json), montaj nadir, --t-body-cam.
Çıktı: --json özet, --png harita (tespit geçiş sayısı + boşluklar + gerçek iz).

Kullanım: ~/bapp/.venv-ros/bin/python kapsama_analiz.py --log test/.../aruco_log --daire 100 --png k.png --json k.json
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
from arac_modelleri import gz_ad, kamera_kol_frd


def zemin_d(w0, ref, z_enu=0.0):
    return ref[2] - (w0[2] + z_enu)


def tarama_penceresi(tarama_log, saat_csv):
    """alan_tarama günlüğündeki OFFBOARD devralma ve TARAMA DURDU anları (yerel HH:MM:SS) → sim saati (t0, t1)."""
    import datetime
    rows = oku_csv(saat_csv)
    ts = np.array([float(r["t_sim"]) for r in rows])
    tw = np.array([float(r["t_duvar"]) for r in rows])
    gun = datetime.datetime.fromtimestamp(tw[0]).date()
    t0 = t1 = None
    for ln in open(tarama_log, encoding="utf-8"):
        hm = ln[:8]
        if t0 is None and "OFFBOARD ✔" in ln:
            t0 = hm
        elif "TARAMA DURDU" in ln:
            t1 = hm
    if t0 is None or t1 is None:
        raise SystemExit(f"{tarama_log}: 'OFFBOARD ✔' / 'TARAMA DURDU' satırı yok")

    def sim(hm):
        dt = datetime.datetime.combine(gun, datetime.time.fromisoformat(hm)).timestamp()
        return float(np.interp(dt, tw, ts))
    return sim(t0), sim(t1) + 1.0


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True)
    ap.add_argument("--dunya", default=os.path.join(here, "gz_worlds/iha_arena_aruco.sdf"))
    ap.add_argument("--model", default=gz_ad(0))
    ap.add_argument("--kamera-json", default=None)
    ap.add_argument("--t-body-cam", type=float, nargs=3, default=kamera_kol_frd(0))
    ap.add_argument("--daire", type=float, default=100.0, help="alan: merkez çevresinde daire yarıçapı [m]; 0 → --kutu")
    ap.add_argument("--kutu", type=float, default=300.0)
    ap.add_argument("--merkez", type=float, nargs=2, default=(0.0, 0.0), metavar=("N", "E"))
    ap.add_argument("--hedef-enu", type=float, nargs=2, default=(-60.0, -60.0), metavar=("X_E", "Y_N"))
    ap.add_argument("--boy", type=float, default=2.0, help="marker kenarı [m]")
    ap.add_argument("--kenar-px", type=float, default=5.0)
    ap.add_argument("--min-kenar-px", type=float, default=20.0)
    ap.add_argument("--hucre", type=float, default=0.5, help="ızgara hücresi [m]")
    ap.add_argument("--min-irtifa", type=float, default=50.0, help="bu irtifanın altındaki kareler sayılmaz [m]")
    ap.add_argument("--gecis-boslugu", type=float, default=2.0, help="alanı görmeyen bu kadar sn → yeni geçiş")
    ap.add_argument("--tarama-log", default=None,
                    help="alan_tarama.py günlüğü: yalnız 'OFFBOARD ✔' → 'TARAMA DURDU' arası kareler (duvar saati → sim "
                         "saati <log>/saat.csv ile). Verilmezse tüm uçuş (Hold çemberleri dahil)")
    ap.add_argument("--t-aralik", type=float, nargs=2, default=None, metavar=("T0", "T1"), help="sim saati penceresi [s]")
    ap.add_argument("--etiket", default="")
    ap.add_argument("--json", default=None)
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    L = a.log
    ref = json.load(open(os.path.join(L, "geo_ref.json")))
    ref = (ref["lat"], ref["lon"], ref["alt"])
    w0 = dunya_orijini(a.dunya)
    gercek = Gercek(oku_csv(os.path.join(L, "gercek_poz.csv")), a.model, w0, ref)
    kam = kg.Kamera.json_oku(a.kamera_json or os.path.join(L, "kamera.json"))
    zd = zemin_d(w0, ref)
    lat, lon = MapProjection(w0[0], w0[1]).reproject(a.hedef_enu[1], a.hedef_enu[0])
    hedef = np.array(MapProjection(ref[0], ref[1]).project(lat, lon))
    kareler = oku_csv(os.path.join(L, "kareler.csv"))
    t_body_cam = np.array(a.t_body_cam)
    pencere = tuple(a.t_aralik) if a.t_aralik else (
        tarama_penceresi(a.tarama_log, os.path.join(L, "saat.csv")) if a.tarama_log else (-math.inf, math.inf))

    # ızgara (N satır, E sütun)
    R = a.daire if a.daire > 0 else a.kutu / 2
    ext = R + 60.0
    n_h = int(math.ceil(2 * ext / a.hucre))
    gn = a.merkez[0] - ext + (np.arange(n_h) + 0.5) * a.hucre
    ge = a.merkez[1] - ext + (np.arange(n_h) + 0.5) * a.hucre
    GN, GE = np.meshgrid(gn, ge, indexing="ij")
    if a.daire > 0:
        alan = np.hypot(GN - a.merkez[0], GE - a.merkez[1]) <= R
    else:
        alan = (np.abs(GN - a.merkez[0]) <= R) & (np.abs(GE - a.merkez[1]) <= R)

    def hucreye(ne):
        return np.stack([(ne[:, 1] - (a.merkez[1] - ext)) / a.hucre, (ne[:, 0] - (a.merkez[0] - ext)) / a.hucre], -1)

    n_img = np.zeros((n_h, n_h), np.int32)          # kare sayısı (görüntü)
    n_det = np.zeros((n_h, n_h), np.int32)          # kare sayısı (tespit edilebilir)
    gecis_det = np.zeros((n_h, n_h), np.int32)      # tespit edilebilir gören geçiş sayısı
    son_gecis = np.full((n_h, n_h), -1, np.int32)
    gecisler, iz, hler = [], [], []
    gecis, son_alan_t = -1, -1e9
    t_ilk = t_son = None
    n_kare = n_alan_kare = 0
    hedef_beklenen, hedef_tespit = 0, 0
    sinir = np.array([[0, 0], [kam.w, 0], [kam.w, kam.h], [0, kam.h]], float)
    for r in kareler:
        try:
            t = float(r["t_cam_s"])
        except (TypeError, ValueError):
            continue
        if not pencere[0] <= t <= pencere[1]:
            continue
        g = gercek(t)
        if g is None:
            continue
        p, R_nb = g
        c_ned = p + R_nb @ t_body_cam
        h = zd - c_ned[2]
        if h < a.min_irtifa:
            continue
        n_kare += 1
        R_nc = R_nb @ kg.R_NADIR
        iz.append((t, c_ned[0], c_ned[1]))
        hler.append(h)
        # görüntü ayak izi
        poly = kg.ayak_izi(kam, h, R_nc, 4)
        if not np.isfinite(poly).all():
            continue
        poly = poly + c_ned[:2]
        m_img = np.zeros((n_h, n_h), np.uint8)
        cv2.fillPoly(m_img, [np.round(hucreye(poly) * 8).astype(np.int32)], 1, shift=3)
        # tespit bölgesi: kenarlardan (kenar_px + marker yarı köşegeni px) içerideki dikdörtgenin zemin izdüşümü
        menzil = h / max(R_nc[2, 2], 0.2)              # optik eksen boyunca zemine uzaklık
        d_px = a.kenar_px + (a.boy / math.sqrt(2)) * kam.fx / menzil
        m_det = np.zeros((n_h, n_h), np.uint8)
        if a.boy * kam.fx / menzil >= a.min_kenar_px and 2 * d_px < min(kam.w, kam.h):
            ic = np.array([[d_px, d_px], [kam.w - d_px, d_px], [kam.w - d_px, kam.h - d_px], [d_px, kam.h - d_px]])
            pd = kg.zemine(kam, ic, h, R_nc) + c_ned[:2]
            cv2.fillPoly(m_det, [np.round(hucreye(pd) * 8).astype(np.int32)], 1, shift=3)
        alanda = bool((m_det.astype(bool) & alan).any())
        if alanda:
            n_alan_kare += 1
            if t - son_alan_t > a.gecis_boslugu:
                gecis += 1
                gecisler.append({"t0": t, "t1": t, "n": 0, "iz": []})
            son_alan_t = t
            gecisler[-1]["t1"] = t
            gecisler[-1]["n"] += 1
            gecisler[-1]["iz"].append((c_ned[0], c_ned[1]))
            t_ilk = t if t_ilk is None else t_ilk
            t_son = t
            md = m_det.astype(bool)
            n_det += md
            yeni = md & (son_gecis != gecis)
            gecis_det += yeni
            son_gecis[yeni] = gecis
        n_img += m_img
        # marker: geometrik olarak tespit edilebilir mi (her iki yaw) ↔ geolocator tespiti
        ok = True
        for y in (0.0, 45.0):
            ic_, k_ = kg.marker_durumu(kam, (hedef - c_ned[:2])[None], h, R_nc, a.boy, math.radians(y), a.kenar_px)
            ok &= bool(ic_[0] and k_[0] >= a.min_kenar_px)
        if ok:
            hedef_beklenen += 1
            try:
                hedef_tespit += int(float(r.get("n_det") or 0) > 0)
            except ValueError:
                pass

    A = alan.sum()
    hucre_m2 = a.hucre ** 2
    det_ok = (n_det > 0) & alan
    bos = alan & ~det_ok
    n_lab, lab, st, cen = cv2.connectedComponentsWithStats(bos.astype(np.uint8), connectivity=8)
    bosluklar = sorted(({"alan_m2": round(float(st[i, cv2.CC_STAT_AREA]) * hucre_m2, 1),
                         "merkez_ne": [round(float(gn[int(round(cen[i][1]))]), 1), round(float(ge[int(round(cen[i][0]))]), 1)]}
                        for i in range(1, n_lab)), key=lambda b: -b["alan_m2"])
    # geçişler: ortalama doğrultu ve yanal konum (komşu geçiş aralığı → yan örtüşme)
    ozet_g = []
    for k, gs in enumerate(gecisler):
        q = np.array(gs["iz"])
        yon = math.degrees(math.atan2(q[-1, 1] - q[0, 1], q[-1, 0] - q[0, 0])) if len(q) > 1 else float("nan")
        ozet_g.append({"no": k, "sure_s": round(gs["t1"] - gs["t0"], 1), "kare": gs["n"], "yon_deg": round(yon, 1),
                       "_q": q})
    yan = []
    if len(ozet_g) >= 2:
        y0 = math.radians(ozet_g[0]["yon_deg"])
        nrm = np.array([-math.sin(y0), math.cos(y0)])
        lat_ = []
        for g_ in ozet_g:
            lat_.append(float(np.median((g_["_q"] - np.array(a.merkez)) @ nrm)))
            g_["yanal_m"] = round(lat_[-1], 1)
        srt = sorted(lat_)
        # görüntü bant genişliği (seviye uçuş) ve tespit şeridi ≈ görüntü − 2·(kenar_px + marker yarı köşegeni)
        h_med = float(np.median(hler))
        Wg = 2 * h_med * math.tan(kam.hfov / 2)
        Wd = Wg - 2 * (a.kenar_px * h_med / kam.fx + a.boy / math.sqrt(2))
        for x0, x1 in zip(srt, srt[1:]):
            d = x1 - x0
            yan.append({"aralik_m": round(d, 1), "goruntu_yan_ortusme": round(1 - d / Wg, 3),
                        "tespit_serit_ortusmesi_m": round(Wd - d, 1)})
    for g_ in ozet_g:
        g_.pop("_q")
    alan_ustu = sum(g_["sure_s"] for g_ in ozet_g)
    out = {
        "etiket": a.etiket, "log": os.path.abspath(L),
        "pencere_sim_s": [round(x, 1) for x in pencere] if np.isfinite(pencere[0]) else None,
        "alan": f"daire R {R:g} m" if a.daire > 0 else f"kutu {a.kutu:g} m", "alan_m2": round(float(A) * hucre_m2),
        "kare_sayisi": n_kare, "alan_karesi": n_alan_kare, "gecis_sayisi": len(gecisler),
        "goruntu_kapsamasi": round(float(((n_img > 0) & alan).sum() / A), 4),
        "tespit_kapsamasi": round(float(det_ok.sum() / A), 4),
        "atlanan_m2": round(float(bos.sum()) * hucre_m2, 1), "bosluklar": bosluklar[:5],
        "tekrar_2+_gecis_payi": round(float(((gecis_det >= 2) & alan).sum() / A), 4),
        "tekrar_3+_gecis_payi": round(float(((gecis_det >= 3) & alan).sum() / A), 4),
        "hucre_basina_tespit_kare_medyan": float(np.median(n_det[det_ok])) if det_ok.any() else 0.0,
        "gecisler": ozet_g, "komsu_gecisler": yan,
        "alan_suresi_s": None if t_ilk is None else round(t_son - t_ilk, 1),
        "alan_ustu_sure_s": round(alan_ustu, 1),
        "alan_ustu_sure_payi": None if t_ilk is None else round(alan_ustu / max(t_son - t_ilk, 1e-9), 3),
        "hedef_ne": [round(float(hedef[0]), 2), round(float(hedef[1]), 2)],
        "hedef_tespit_edilebilir_kare": hedef_beklenen, "hedef_tespit_edilen_kare": hedef_tespit,
    }
    print(json.dumps(out, ensure_ascii=False, indent=1))
    if a.json:
        json.dump(out, open(a.json, "w"), ensure_ascii=False, indent=1)
    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
        fig, ax = plt.subplots(1, 2, figsize=(13, 6.2))
        x0, x1 = a.merkez[1] - ext, a.merkez[1] + ext
        y0, y1 = a.merkez[0] - ext, a.merkez[0] + ext
        g = np.where(alan | (gecis_det > 0), gecis_det, -1).astype(float)
        g[g < 0] = np.nan
        cm = ListedColormap(["#d62728", "#9ecae1", "#3182bd", "#08519c", "#08306b"])
        im = ax[0].imshow(np.clip(g, 0, 4), origin="lower", extent=(x0, x1, y0, y1), cmap=cm, vmin=-0.5, vmax=4.5,
                          interpolation="nearest")
        cb = fig.colorbar(im, ax=ax[0], ticks=range(5), fraction=0.046)
        cb.ax.set_yticklabels(["0 (atlanan)", "1 geçiş", "2", "3", "4+"])
        ax[0].set_title("Tespit edilebilir gören geçiş sayısı")
        nimg = np.where(n_img > 0, n_img, np.nan).astype(float)
        im2 = ax[1].imshow(nimg, origin="lower", extent=(x0, x1, y0, y1), cmap="viridis", interpolation="nearest")
        fig.colorbar(im2, ax=ax[1], fraction=0.046, label="kare sayısı")
        ax[1].set_title("Görüntü: hücreyi gören kare sayısı")
        q = np.array([(n, e) for _, n, e in iz])
        th = np.linspace(0, 2 * np.pi, 361)
        for x in ax:
            if len(q):
                x.plot(q[:, 1], q[:, 0], "-", color="orange", lw=1, label="gerçek iz (kamera)")
            if a.daire > 0:
                x.plot(a.merkez[1] + R * np.sin(th), a.merkez[0] + R * np.cos(th), "k--", lw=1, label="alan")
            x.plot(hedef[1], hedef[0], "m*", ms=12, label="marker")
            x.set_xlim(x0, x1)
            x.set_ylim(y0, y1)
            x.set_xlabel("Doğu [m]")
            x.set_ylabel("Kuzey [m]")
            x.set_aspect("equal")
        ax[0].legend(loc="lower right", fontsize=8)
        fig.suptitle(f"{a.etiket}  tespit kapsaması %{100 * out['tespit_kapsamasi']:.1f}, görüntü %"
                     f"{100 * out['goruntu_kapsamasi']:.1f}, atlanan {out['atlanan_m2']:.0f} m², 2+ geçiş %"
                     f"{100 * out['tekrar_2+_gecis_payi']:.1f}, alan süresi {out['alan_suresi_s']} s")
        fig.tight_layout()
        fig.savefig(a.png, dpi=110)


if __name__ == "__main__":
    main()
