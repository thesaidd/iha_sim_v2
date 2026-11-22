#!/usr/bin/env python3
"""
aruco_degerlendir.py — bapp aruco_geolocator günlüğünü Gazebo gerçeğiyle karşılaştırır (uçuş sonrası).

Girdi (--log dizini; aruco_geolocator --log + gz_gercek_poz.py çıktıları):
  kareler.csv, rapor.csv, kamera.json, geo_ref.json, gercek_poz.csv
Gerçek hedef: dünya ENU (--hedef-enu, varsayılan −60 −60; z = --hedef-z) → enlem/boylam (dünya
spherical_coordinates, --dunya SDF'inden) → geolocator NED'i (geo_ref = uav0 EKF origin).

Hata ayrıştırması (kare başına):
  - tahmin hatası: ray-cast / PnP / füzyon − gerçek (yatay, m),
  - PX4 poz hatası: PX4 konum/attitude (kareler.csv) − gerçek poz (aynı sim anında, enterpolasyonlu),
  - "gerçek pozla" ray-cast/PnP: aynı köşeler + GERÇEK poz → kalan hata = tespit + kamera modeli
    (+ zemin düzlemi) hatası; tahmin hatası − bu ≈ poz kaynaklı hata.
Tespit oranı: gerçek pozla marker köşeleri görüntüdeyse "beklenen" kare; tespit / beklenen, kenar px'e göre.
Zaman: kareler.csv t_px4_s (PX4 saati) = gz sim saati varsayılır (UXRCE_DDS_SYNCT=0). Doğrulama: PX4–gerçek
konum farkını en küçükleyen gecikme taranır ve raporlanır.

Kullanım: python3 aruco_degerlendir.py --log /tmp/aruco_log [--dunya gz_worlds/iha_arena_aruco.sdf] [--md sonuc.md]
"""
import argparse
import csv
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(os.environ.get("BAPP", "~/bapp")))
from bap_geo.camera import CameraModel  # noqa: E402
from bap_geo.frames import R_body_cam_mount, R_ned_body_to_euler, quat_to_R  # noqa: E402
from bap_geo.geodesy import MapProjection  # noqa: E402
from bap_geo.pnp import pnp_target  # noqa: E402
from bap_geo.pose import AircraftPose, marker_corners_ned, project_ned  # noqa: E402
from bap_geo.raycast import raycast_target  # noqa: E402
from arac_modelleri import gz_ad, kamera_kol_frd  # noqa: E402

T_NED_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
F_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def oku_csv(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def dunya_orijini(sdf):
    t = open(sdf, encoding="utf-8").read()
    m = re.search(r"<spherical_coordinates>(.*?)</spherical_coordinates>", t, re.S)
    lat = float(re.search(r"<latitude_deg>([^<]+)", m.group(1)).group(1))
    lon = float(re.search(r"<longitude_deg>([^<]+)", m.group(1)).group(1))
    el = re.search(r"<elevation>([^<]+)", m.group(1))
    return lat, lon, float(el.group(1)) if el else 0.0


class Gercek:
    """gercek_poz.csv → sim zamanında enterpolasyonlu NED/FRD poz (geolocator NED'inde)."""

    def __init__(self, rows, ad, w0, ref):
        rows = [r for r in rows if r["ad"] == ad]
        self.t = np.array([f(r["t_sim"]) for r in rows])
        enu = np.array([[f(r["x_e"]), f(r["y_n"]), f(r["z_u"])] for r in rows])
        self.q = np.array([[f(r["qw"]), f(r["qx"]), f(r["qy"]), f(r["qz"])] for r in rows])
        mw, mr = MapProjection(w0[0], w0[1]), MapProjection(ref[0], ref[1])
        ned = []
        for e, n, u in enu:
            lat, lon = mw.reproject(n, e)
            x, y = mr.project(lat, lon)
            ned.append((x, y, ref[2] - (w0[2] + u)))
        self.p = np.array(ned)

    def __call__(self, t):
        if len(self.t) < 2 or not (self.t[0] <= t <= self.t[-1]):
            return None
        i = int(np.clip(np.searchsorted(self.t, t) - 1, 0, len(self.t) - 2))
        u = (t - self.t[i]) / max(self.t[i + 1] - self.t[i], 1e-9)
        p = (1 - u) * self.p[i] + u * self.p[i + 1]
        q0, q1 = self.q[i], self.q[i + 1]
        if np.dot(q0, q1) < 0:
            q1 = -q1
        q = (1 - u) * q0 + u * q1
        R_enu_flu = quat_to_R(q / np.linalg.norm(q))
        return p, T_NED_ENU @ R_enu_flu @ F_FLU_FRD


def yuzdelik(a, q):
    a = np.asarray([x for x in a if np.isfinite(x)])
    return float(np.percentile(a, q)) if a.size else float("nan")


def ozet(a):
    a = np.asarray([x for x in a if np.isfinite(x)])
    if not a.size:
        return "—"
    return f"medyan {np.median(a):.2f}, %95 {np.percentile(a, 95):.2f}, en çok {a.max():.2f} (n={a.size})"


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", default="/tmp/aruco_log")
    ap.add_argument("--dunya", default=os.path.join(here, "gz_worlds/iha_arena_aruco.sdf"))
    ap.add_argument("--hedef-enu", type=float, nargs=2, default=(-60.0, -60.0), metavar=("X_E", "Y_N"))
    ap.add_argument("--hedef-z", type=float, default=0.02)
    ap.add_argument("--hedef-yaw", type=float, default=0.0, help="marker yaw (NED, derece); 0 = üst kenar kuzey")
    ap.add_argument("--model", default=gz_ad(0))
    ap.add_argument("--t-body-cam", type=float, nargs=3, default=kamera_kol_frd(0))
    ap.add_argument("--marker-size", type=float, default=2.0)
    ap.add_argument("--md", default=None, help="özet Markdown dosyası")
    ap.add_argument("--csv", default=None, help="kare başına değerlendirme CSV (varsayılan <log>/degerlendirme.csv)")
    a = ap.parse_args()

    L = a.log
    ref = json.load(open(os.path.join(L, "geo_ref.json")))
    ref = (ref["lat"], ref["lon"], ref["alt"])
    cam = CameraModel.load_json(os.path.join(L, "kamera.json"))
    cam_pinhole = CameraModel(K=cam.K, dist=np.zeros(5), width=cam.width, height=cam.height)
    ayar = json.load(open(os.path.join(L, "ayarlar.json")))
    tilt, roll, yaw = np.radians(ayar.get("mount_deg", (0, 0, 0)))
    R_bc = R_body_cam_mount(tilt, roll, yaw)
    t_bc = np.asarray(a.t_body_cam, float)
    w0 = dunya_orijini(a.dunya)
    lat_t, lon_t = MapProjection(w0[0], w0[1]).reproject(a.hedef_enu[1], a.hedef_enu[0])
    n_t, e_t = MapProjection(ref[0], ref[1]).project(lat_t, lon_t)
    hedef = np.array([n_t, e_t, ref[2] - (w0[2] + a.hedef_z)])
    gercek = Gercek(oku_csv(os.path.join(L, "gercek_poz.csv")), a.model, w0, ref)
    kareler = oku_csv(os.path.join(L, "kareler.csv"))
    rapor = oku_csv(os.path.join(L, "rapor.csv")) if os.path.exists(os.path.join(L, "rapor.csv")) else []
    corners_t = marker_corners_ned(hedef, a.marker_size, math.radians(a.hedef_yaw))
    ground_d = 0.0 if ayar.get("ground_alt") is None else ref[2] - float(ayar["ground_alt"])

    # --- zaman hizası doğrulaması: PX4 konumu ile gerçek arasındaki gecikme taraması
    px = [(f(r["t_px4_s"]), np.array([f(r["pose_n"]), f(r["pose_e"]), f(r["pose_d"])])) for r in kareler
          if r.get("t_px4_s") and np.isfinite(f(r["pose_n"]))]
    lag_best, lag_err = float("nan"), float("nan")
    if len(px) > 5:
        best = []
        for lag in np.arange(-0.5, 0.5001, 0.01):
            errs = [np.linalg.norm(p[:2] - g[0][:2]) for t, p in px if (g := gercek(t + lag)) is not None]
            if len(errs) > 5:
                best.append((float(np.median(errs)), float(lag)))
        if best:
            lag_err, lag_best = min(best)

    rows = []
    for r in kareler:
        t = f(r.get("t_px4_s")) if r.get("t_px4_s") else f(r["t_cam_s"])
        g = gercek(t)
        ev = {"t": t, "n_det": int(r["n_det"]), "side_px": f(r.get("side_px")), "reason": r.get("reason") or ""}
        if g is None:
            rows.append(ev)
            continue
        p_g, R_g = g
        pose_g = AircraftPose(p_ned=p_g, R_ned_body=R_g, R_body_cam=R_bc, t_body_cam=t_bc)
        uv = project_ned(cam, pose_g, corners_t)
        # Kadraj denetimi distorsiyonsuz: polinom distorsiyon (ör. k3 < 0) görüş alanı DIŞINDAKİ noktaları
        # görüntünün içine katlayabiliyor (15° yatışta uzak hedef "kadrajda" sanılıyordu, 2026-11-20).
        uv0 = project_ned(cam_pinhole, pose_g, corners_t)
        ev["beklenen"] = bool(np.all(cam.in_image(uv0)) and np.all(np.isfinite(uv0)) and np.all(cam.in_image(uv)))
        ev["kenar_beklenen_px"] = float(np.mean(np.linalg.norm(uv - np.roll(uv, 1, 0), axis=1)))
        ev["irtifa"] = float(-p_g[2] + hedef[2])
        rg, pg, yg = (math.degrees(v) for v in R_ned_body_to_euler(R_g))
        ev.update(g_roll=rg, g_pitch=pg, g_yaw=yg)
        if np.isfinite(f(r.get("pose_n"))):
            ev["px4_poz_hata_yatay"] = math.hypot(f(r["pose_n"]) - p_g[0], f(r["pose_e"]) - p_g[1])
            ev["px4_poz_hata_d"] = f(r["pose_d"]) - p_g[2]
            ev["px4_roll_hata"] = f(r["roll_deg"]) - rg
            ev["px4_pitch_hata"] = f(r["pitch_deg"]) - pg
            ev["px4_yaw_hata"] = (f(r["yaw_deg"]) - yg + 180.0) % 360.0 - 180.0
        if ev["n_det"] and np.isfinite(f(r.get("u0"))):
            c = np.array([[f(r[f"u{k}"]), f(r[f"v{k}"])] for k in range(4)])
            ev["kose_hata_px"] = float(np.sqrt(np.mean(np.sum((c - uv) ** 2, axis=1)))) if ev["beklenen"] else float("nan")
            for key in ("rc", "pnp", "fused"):
                if np.isfinite(f(r.get(f"{key}_n"))):
                    ev[f"{key}_hata"] = math.hypot(f(r[f"{key}_n"]) - hedef[0], f(r[f"{key}_e"]) - hedef[1])
            o = raycast_target(cam, pose_g, c, ground_d=ground_d, compute_cov=False)
            if o is not None:
                ev["rc_gercekpoz_hata"] = math.hypot(o.p_ned[0] - hedef[0], o.p_ned[1] - hedef[1])
            o = pnp_target(cam, pose_g, c, a.marker_size, compute_cov=False)
            if o is not None:
                ev["pnp_gercekpoz_hata"] = math.hypot(o.p_ned[0] - hedef[0], o.p_ned[1] - hedef[1])
        rows.append(ev)

    out_csv = a.csv or os.path.join(L, "degerlendirme.csv")
    keys = sorted({k for r in rows for k in r})
    with open(out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, keys)
        w.writeheader()
        w.writerows(rows)

    det = [r for r in rows if r["n_det"]]
    bek = [r for r in rows if r.get("beklenen")]
    lines = [f"# ArUco simülasyon değerlendirmesi — {L}", ""]
    lines += [f"- Gerçek hedef: ENU {tuple(a.hedef_enu)} → ({lat_t:.7f}, {lon_t:.7f}); geolocator NED "
              f"({hedef[0]:.2f}, {hedef[1]:.2f}, {hedef[2]:.2f}); EKF origin {ref[0]:.7f}, {ref[1]:.7f}, {ref[2]:.2f} m",
              f"- Kare: {len(rows)} işlenen, {len(bek)} beklenen (marker tam görüntüde), {len(det)} tespitli; "
              f"sahte tespit (beklenmezken): {sum(1 for r in det if r.get('beklenen') is False)}",
              f"- Zaman hizası: PX4–gerçek yatay konum farkını en küçükleyen gecikme {lag_best * 1e3:+.0f} ms "
              f"(medyan fark {lag_err:.2f} m); 0 ms civarı → kare/PX4 saatleri aynı", ""]
    # tespit oranı: kenar px kutuları
    lines += ["## Tespit oranı (beklenen karelerde)", "", "| kenar px | irtifa m | beklenen | tespit | oran |", "|---|---|---|---|---|"]
    bins = [(0, 25), (25, 30), (30, 35), (35, 40), (40, 50), (50, 80), (80, 1e9)]
    for lo, hi in bins:
        b = [r for r in bek if lo <= r["kenar_beklenen_px"] < hi]
        if b:
            n_d = sum(1 for r in b if r["n_det"])
            alt = np.median([r["irtifa"] for r in b])
            lines.append(f"| {lo:g}–{hi:g} | {alt:.0f} | {len(b)} | {n_d} | {100 * n_d / len(b):.1f} % |")
    lines += ["", "## Kare başına hata (yatay, m)", "",
              f"- ray-cast: {ozet([r.get('rc_hata', np.nan) for r in det])}",
              f"- PnP: {ozet([r.get('pnp_hata', np.nan) for r in det])}",
              f"- ray-cast GERÇEK pozla (tespit+kamera modeli): {ozet([r.get('rc_gercekpoz_hata', np.nan) for r in det])}",
              f"- PnP GERÇEK pozla: {ozet([r.get('pnp_gercekpoz_hata', np.nan) for r in det])}",
              f"- köşe hatası (gerçek izdüşüme göre, RMS px): {ozet([r.get('kose_hata_px', np.nan) for r in det])}",
              "", "## PX4 poz hatası (tespitli karelerde)", "",
              f"- yatay konum: {ozet([r.get('px4_poz_hata_yatay', np.nan) for r in det])} m",
              f"- düşey (D): {ozet([abs(r.get('px4_poz_hata_d', np.nan)) for r in det])} m",
              f"- |roll|: {ozet([abs(r.get('px4_roll_hata', np.nan)) for r in det])}°",
              f"- |pitch|: {ozet([abs(r.get('px4_pitch_hata', np.nan)) for r in det])}°",
              f"- |yaw|: {ozet([abs(r.get('px4_yaw_hata', np.nan)) for r in det])}°", ""]
    if rapor:
        lines += ["## LoRa raporları (füzyon tahmini, gönderilen)", "", "| # | n_obs | hata m | σ95 m | hata/σ95 |", "|---|---|---|---|---|"]
        mp = MapProjection(lat_t, lon_t)
        errs = []
        for i, r in enumerate(rapor):
            n, e = mp.project(f(r["lat"]), f(r["lon"]))
            err = math.hypot(n, e)
            errs.append((err, f(r["s95"]), int(r["n_obs"])))
        idx = sorted({0, len(errs) // 4, len(errs) // 2, (3 * len(errs)) // 4, len(errs) - 1})
        for i in idx:
            err, s95, n = errs[i]
            lines.append(f"| {i + 1} | {n} | {err:.2f} | {s95:.2f} | {err / s95:.2f} |")
        lines += ["", f"- Son rapor hatası: **{errs[-1][0]:.2f} m** (σ95 {errs[-1][1]:.2f} m, {errs[-1][2]} gözlem); "
                  f"raporların %{100 * np.mean([e <= s for e, s, _ in errs]):.0f}'inde hata ≤ σ95"]
    text = "\n".join(lines) + "\n"
    print(text)
    if a.md:
        open(a.md, "w", encoding="utf-8").write(text)
    print(f"kare CSV: {out_csv}")


if __name__ == "__main__":
    main()
