#!/usr/bin/env python3
"""
qr_analyze.py — QR geçiş kayıtlarının (qr_recorder.py) çevrimdışı analizi.

Her kare için:
  - Kadraj: QR kodunun (1,00 m, kenar boşluğu hariç) dört köşesi Gazebo gerçek pozu + kamera modeliyle
    görüntüye izdüşürülür; dördü de 1456×1088 içindeyse "kadrajda".
  - Ham görüntü + cv2.QRCodeDetector ve cv2.QRCodeDetectorAruco: bulma (detect), çözme ("HEDEF-01"),
    piksel merkezi, kenar (modül pikseli = kenar / 21), izdüşümle piksel farkı, kameradan QR'a 3B mesafe.
  - Konum tahmini (bulunan karelerde, QRCodeDetector merkezi): pikselden ışın → zemin düzlemi.
      poz kaynağı: PX4 (vehicle_odometry, varış zamanıyla eşleşmiş; yalnızca geçiş 2–3, geçiş 1'de
                   eşleşme PX4 damgasıyla yapılmıştı ve 1–2 sn kayık) ve Gazebo gerçek pozu (tüm geçişler)
      model:      tam yönelim (roll, pitch, yaw) ve yalnızca yaw (kamera tam dik varsayımı)
    Zemin: PX4'te D = 0 (ev irtifası; gerçekte QR ~9 cm aşağıda), gerçek pozda z = 0,05 (levha).

Kamera modeli: Gazebo kamerası +X'e bakar, görüntü sağı −Y, aşağısı −Z; sensör base_link'e göre
(0,07; 0; −0,1) m, pitch +π/2 (dik aşağı, görüntü üstü = burun). fx = fy = 4637,81 (camera_info),
asal nokta (727,5; 543,5) (statik taramada (728, 544)'e göre ~0,7 px sapma: yarım piksel kuralı).
Gövde FRD'de piksel ışını: (−(v−cy)/fy, (u−cx)/fx, 1); kamera konumu FRD (0,07; 0; 0,1).
Dünya: Gazebo ENU (x = Doğu, y = Kuzey); PX4 yerel NED (N = y, E = x). QR merkezi E −60, N −60, z 0,05;
doku kuzey yukarı ve aynasız (pencereden doğrulandı).

Kullanım: python3 qr_analyze.py <kayit klasörü> [--yon 210] → <kayit>/analiz/ (kare CSV'leri, ozet.md, ozet.json)
"""

import argparse
import csv
import json
import math
import os
import time

import cv2
import numpy as np

W, H = 1456, 1088
FX = FY = 4637.81
CX, CY = 727.5, 543.5
CAM_T_FLU = np.array([0.07, 0.0, -0.1])
R_S = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])     # Ry(+π/2): sensör → gövde (FLU)
QR_E, QR_N, QR_Z = -60.0, -60.0, 0.05
CODE = 1.0
TEXT = "HEDEF-01"
FLU_FRD = np.diag([1.0, -1.0, -1.0])
ENU_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])   # simetrik: NED↔ENU


def quat_to_R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def euler_to_R(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def project_enu(R_wb, t_wb, P):
    """Dünya ENU noktası → (u, v, derinlik). R_wb: gövde FLU → ENU."""
    p_b = R_wb.T @ (P - t_wb)
    p_s = R_S.T @ (p_b - CAM_T_FLU)
    if p_s[0] <= 0:
        return math.nan, math.nan, p_s[0]
    return CX - FX * p_s[1] / p_s[0], CY - FY * p_s[2] / p_s[0], p_s[0]


def ray_ground_ned(R_nb_frd, pos_ned, u, v, ground_d):
    """Piksel (u, v) ışınının D = ground_d düzlemiyle kesişimi (N, E). R_nb_frd: gövde FRD → NED."""
    d_body = np.array([-(v - CY) / FY, (u - CX) / FX, 1.0])
    cam = pos_ned + R_nb_frd @ (FLU_FRD @ CAM_T_FLU)
    d = R_nb_frd @ d_body
    if d[2] <= 1e-6:
        return math.nan, math.nan
    t = (ground_d - cam[2]) / d[2]
    q = cam + t * d
    return q[0], q[1]


def yaw_only(R):
    y = yaw_of(R)
    return np.array([[math.cos(y), -math.sin(y), 0.0], [math.sin(y), math.cos(y), 0.0], [0.0, 0.0, 1.0]])


def stats(errs):
    e = np.array([x for x in errs if np.all(np.isfinite(x))])
    if len(e) == 0:
        return None
    h = np.hypot(e[:, 0], e[:, 1])
    return {"n": int(len(e)), "ort_dN": round(float(e[:, 0].mean()), 3), "ort_dE": round(float(e[:, 1].mean()), 3),
            "yatay_ort": round(float(h.mean()), 3), "yatay_rms": round(float(math.sqrt((h ** 2).mean())), 3),
            "yatay_std": round(float(h.std()), 3), "yatay_maks": round(float(h.max()), 3)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kayit")
    ap.add_argument("--yon", type=float, default=210.0, help="Geçiş hattı yönü ψ [°]")
    ap.add_argument("--px4-gecisler", type=int, nargs="+", default=[2, 3],
                    help="PX4 pozu güvenilir eşleşen geçişler (varış zamanı eşleşmesi)")
    args = ap.parse_args()
    out = os.path.join(args.kayit, "analiz")
    os.makedirs(out, exist_ok=True)
    psi = math.radians(args.yon)
    dvec = np.array([math.cos(psi), math.sin(psi)])
    rvec = np.array([-math.sin(psi), math.cos(psi)])
    qr_center = np.array([QR_E, QR_N, QR_Z])
    corners = [np.array([QR_E + dx, QR_N + dy, QR_Z]) for dx, dy in ((-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5))]
    dets = {"QR": cv2.QRCodeDetector(), "Aruco": cv2.QRCodeDetectorAruco()}
    summary = {}
    est_all = {}
    for gdir in sorted(d for d in os.listdir(args.kayit) if d.startswith("gecis_")):
        gid = int(gdir.split("_")[1])
        rows = list(csv.DictReader(open(os.path.join(args.kayit, gdir, "kareler.csv"))))
        recs = []
        t_start = time.time()
        for r in rows:
            gt_t = np.array([float(r["gt_x"]), float(r["gt_y"]), float(r["gt_z"])])
            R_wb = quat_to_R(float(r["gt_qw"]), float(r["gt_qx"]), float(r["gt_qy"]), float(r["gt_qz"]))
            uvc = project_enu(R_wb, gt_t, qr_center)
            uvs = [project_enu(R_wb, gt_t, c) for c in corners]
            in_frame = all(math.isfinite(u) and 0 <= u < W and 0 <= v < H for u, v, _ in uvs)
            cam_world = gt_t + R_wb @ CAM_T_FLU
            dist3 = float(np.linalg.norm(cam_world - qr_center))
            pn, pe = gt_t[1] - QR_N, gt_t[0] - QR_E
            s_gt, x_gt = float(np.dot([pn, pe], dvec)), float(np.dot([pn, pe], rvec))
            img = cv2.imread(os.path.join(args.kayit, gdir, r["png"]), cv2.IMREAD_GRAYSCALE)
            rec = {"kare": int(r["kare"]), "sim_s": float(r["img_sim_s"]), "s_gt": s_gt, "x_gt": x_gt,
                   "irtifa_gt": float(gt_t[2]), "kadrajda": in_frame, "izdusum_u": uvc[0], "izdusum_v": uvc[1],
                   "mesafe_3b": dist3}
            for name, det in dets.items():
                t0 = time.perf_counter()
                ok, pts = det.detect(img)
                text = det.detectAndDecode(img)[0]
                rec[f"{name}_ms"] = 1000 * (time.perf_counter() - t0)
                q = np.asarray(pts, dtype=float).reshape(-1, 4, 2)[0] if ok and pts is not None else None
                rec[f"{name}_bulma"] = q is not None
                rec[f"{name}_cozme"] = text == TEXT
                rec[f"{name}_yanlis"] = bool(text) and text != TEXT
                if q is not None:
                    c = q.mean(axis=0)
                    side = float(np.mean([np.linalg.norm(q[(i + 1) % 4] - q[i]) for i in range(4)]))
                    rec.update({f"{name}_u": float(c[0]), f"{name}_v": float(c[1]), f"{name}_kenar": side,
                                f"{name}_modul": side / 21,
                                f"{name}_izdusum_farki": float(math.hypot(c[0] - uvc[0], c[1] - uvc[1]))
                                if math.isfinite(uvc[0]) else math.nan})
            # Konum tahmini (QRCodeDetector merkezi)
            if rec["QR_bulma"]:
                u, v = rec["QR_u"], rec["QR_v"]
                R_nb_gt = ENU_NED @ R_wb @ FLU_FRD                         # gövde FRD → NED (gerçek)
                pos_ned_gt = ENU_NED @ gt_t
                truth_ne = np.array([QR_N, QR_E])
                for model, Rm in (("tam", R_nb_gt), ("yaw", yaw_only(R_nb_gt))):
                    n, e = ray_ground_ned(Rm, pos_ned_gt, u, v, -QR_Z)
                    rec[f"tahmin_gercek_{model}"] = (n - truth_ne[0], e - truth_ne[1])
                if gid in args.px4_gecisler and r["n"]:
                    R_px4 = euler_to_R(*(math.radians(float(r[k])) for k in ("yaw_deg", "pitch_deg", "roll_deg")))
                    pos = np.array([float(r["n"]), float(r["e"]), float(r["d"])])
                    for model, Rm in (("tam", R_px4), ("yaw", yaw_only(R_px4))):
                        n, e = ray_ground_ned(Rm, pos, u, v, 0.0)
                        rec[f"tahmin_px4_{model}"] = (n - truth_ne[0], e - truth_ne[1])
                    rec["px4_gt_yatay_fark"] = float(math.hypot(pos[0] - pos_ned_gt[0], pos[1] - pos_ned_gt[1]))
                    rec["px4_gt_dikey_fark"] = float(pos[2] - pos_ned_gt[2])
                    rec["px4_gt_roll_fark"] = float(r["roll_deg"]) - math.degrees(
                        math.atan2(R_nb_gt[2, 1], R_nb_gt[2, 2]))
                    rec["px4_gt_pitch_fark"] = float(r["pitch_deg"]) - math.degrees(math.asin(-R_nb_gt[2, 0]))
                    rec["dt_odo_ms"] = float(r["dt_odo_ms"])
                rec["roll_gt"] = math.degrees(math.atan2(R_nb_gt[2, 1], R_nb_gt[2, 2]))
                rec["pitch_gt"] = math.degrees(math.asin(-R_nb_gt[2, 0]))
            recs.append(rec)
        # Kare CSV
        keys = sorted({k for rec in recs for k in rec})
        with open(os.path.join(out, f"{gdir}_kareler.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for rec in recs:
                w.writerow({k: (json.dumps(v) if isinstance(v, tuple) else v) for k, v in rec.items()})
        # Özet
        inf = [x for x in recs if x["kadrajda"]]
        g = {"kare": len(recs), "kadrajda": len(inf), "analiz_sn": round(time.time() - t_start, 1)}
        for name in dets:
            found = [x for x in recs if x[f"{name}_bulma"]]
            dec = [x for x in recs if x[f"{name}_cozme"]]
            g[name] = {"bulunan": len(found), "cozulen": len(dec),
                       "kadrajda_bulunan": sum(1 for x in inf if x[f"{name}_bulma"]),
                       "kadraj_disi_bulunan": sum(1 for x in found if not x["kadrajda"]),
                       "yanlis_icerik": sum(1 for x in recs if x[f"{name}_yanlis"]),
                       "ms_ort": round(float(np.mean([x[f"{name}_ms"] for x in recs])), 1)}
            if dec:
                g[name].update(ilk_cozum_sim_s=dec[0]["sim_s"], son_cozum_sim_s=dec[-1]["sim_s"],
                               ilk_cozum_s_m=round(dec[0]["s_gt"], 2), son_cozum_s_m=round(dec[-1]["s_gt"], 2),
                               cozum_suresi_s=round(dec[-1]["sim_s"] - dec[0]["sim_s"], 2))
            if found:
                g[name].update(modul_px_ort=round(float(np.mean([x[f"{name}_modul"] for x in found])), 2),
                               izdusum_farki_px_ort=round(float(np.nanmean([x[f"{name}_izdusum_farki"] for x in found])), 2),
                               mesafe_3b_ort=round(float(np.mean([x["mesafe_3b"] for x in found])), 2))
        if inf:
            g["kadraj_s_m"] = [round(inf[0]["s_gt"], 2), round(inf[-1]["s_gt"], 2)]
            g["kadraj_suresi_s"] = round(inf[-1]["sim_s"] - inf[0]["sim_s"], 2)
        near = min(recs, key=lambda x: abs(x["s_gt"]))
        g["hedef_ustu_gercek"] = {"x_m": round(near["x_gt"], 3), "irtifa_m": round(near["irtifa_gt"], 3),
                                  "s_m": round(near["s_gt"], 3)}
        g["en_kucuk_yanal_gercek_m"] = round(min(abs(x["x_gt"]) for x in recs), 3)
        g["kadrajda_yanal_gercek_m"] = [round(min(x["x_gt"] for x in inf), 3), round(max(x["x_gt"] for x in inf), 3)] if inf else None
        g["kamera_teorik_modul_px"] = round(FX / 21 / float(np.mean([x["mesafe_3b"] for x in inf])), 2) if inf else None
        for src in ("gercek", "px4"):
            for model in ("tam", "yaw"):
                key = f"tahmin_{src}_{model}"
                errs = [np.array(x[key]) for x in recs if key in x]
                st = stats(errs)
                if st:
                    g[key] = st
                    est_all.setdefault(key, []).extend(errs)
        px = [x for x in recs if "px4_gt_yatay_fark" in x]
        if px:
            g["px4_vs_gercek"] = {"yatay_m": round(float(np.mean([x["px4_gt_yatay_fark"] for x in px])), 3),
                                  "dikey_m": round(float(np.mean([x["px4_gt_dikey_fark"] for x in px])), 3),
                                  "roll_fark_deg": round(float(np.mean([x["px4_gt_roll_fark"] for x in px])), 3),
                                  "pitch_fark_deg": round(float(np.mean([x["px4_gt_pitch_fark"] for x in px])), 3),
                                  "dt_odo_ms": round(float(np.mean([x["dt_odo_ms"] for x in px])), 2)}
        rp = [x for x in recs if "roll_gt" in x]
        if rp:
            g["bulunan_karelerde_roll_pitch_gercek_deg"] = {
                "roll": [round(min(x["roll_gt"] for x in rp), 2), round(max(x["roll_gt"] for x in rp), 2)],
                "pitch": [round(min(x["pitch_gt"] for x in rp), 2), round(max(x["pitch_gt"] for x in rp), 2)]}
        summary[gdir] = g
        print(f"{gdir}: {json.dumps(g, ensure_ascii=False)}", flush=True)
    summary["toplam_tahmin"] = {k: stats(v) for k, v in est_all.items()}
    json.dump(summary, open(os.path.join(out, "ozet.json"), "w"), indent=1, ensure_ascii=False)
    print("toplam tahmin:", json.dumps(summary["toplam_tahmin"], ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
