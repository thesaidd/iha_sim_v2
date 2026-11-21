#!/usr/bin/env python3
"""
kalib_degerlendir.py — Simüle kamera kalibrasyonunu ayrılmış test görünümlerinde doğrular.

Girdi : kareler/egitim/*.png, kareler/test/*.png, kareler/pozlar.json, w_nadir_10_sim.json
        (w_nadir_10_sim.json bapp scripts/camera/calibrate.py ile eğitim karelerinden üretilir; README).
Yapılan:
  1. Eğitim karelerini bapp kalibrasyon fonksiyonuyla (bap_geo.calibration.calibrate) sim ölçekli tahtayla
     yeniden kalibre eder; calibrate.py'nin (varsayılan 35 mm tahta) JSON'uyla aynı K, D'yi verdiğini
     doğrular (iç parametreler tahta ölçeğinden bağımsız).
  2. Test görünümlerinde her model için solvePnP (görünüm başına) → yeniden yansıtma RMS:
       kalibre   : w_nadir_10_sim.json
       nominal   : fx=fy=1739.13, cx=728, cy=544, D=[-0.08, 0.02, 0, 0, 0] (SDF'teki parametreler)
       pinhole   : nominal K, D=0 (gz distorsiyonu uygulamıyorsa bu da kalibre kadar iyi olmalı)
  3. İki modelin görüntü üzerindeki farkı: piksel ızgarası kalibre modelle 5 m derinliğe geri yansıtılır,
     nominal (ve pinhole) modelle yeniden yansıtılır → |Δ| px (en çok, köşe, merkez).
  4. Taze kare denetimi: her görünümde PnP'den kameranın tahta merkezine uzaklığı ↔ planlanan uzaklık.
Çıktı : degerlendirme.json + ekrana tablo.

Kullanım:  ~/bapp/.venv-ros/bin/python kalib_degerlendir.py
"""
import glob
import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(Path.home() / "bapp"))
sys.path.insert(0, str(HERE))
from bap_geo.calibration import BOARD_SQUARES, calibrate, detect_view  # noqa: E402
from bap_geo.camera import CameraModel  # noqa: E402
from make_kalib_world import BOARD_Z, SQUARE_M, board  # noqa: E402

W, H = 1456, 1088
NOMINAL = CameraModel(K=np.array([[1739.13, 0, 728.0], [0, 1739.13, 544.0], [0, 0, 1]]),
                      dist=np.array([-0.08, 0.02, 0, 0, 0]), width=W, height=H)
PINHOLE = CameraModel(K=NOMINAL.K, dist=np.zeros(5), width=W, height=H)
DEPTH_M = 5.0


def views(folder, det, b):
    out = []
    for f in sorted(glob.glob(str(HERE / "kareler" / folder / "*.png"))):
        v = detect_view(det, b, cv2.imread(f))
        out.append((Path(f).name, v))
    return out


def pnp_rms(v, cam):
    ok, r, t = cv2.solvePnP(v.obj, v.img, cam.K, cam.dist, flags=cv2.SOLVEPNP_ITERATIVE)
    r, t = cv2.solvePnPRefineLM(v.obj, v.img, cam.K, cam.dist, r, t)
    proj, _ = cv2.projectPoints(v.obj, r, t, cam.K, cam.dist)
    e2 = np.sum((proj.reshape(-1, 2) - v.img) ** 2, axis=1)
    return e2, r, t


def displacement(cam_a, cam_b, n=(57, 43)):
    """cam_a pikselleri → DEPTH_M'de 3B nokta → cam_b'ye yansıt; |Δ| (ny, nx) px."""
    u, v = np.meshgrid(np.linspace(0, W - 1, n[0]), np.linspace(0, H - 1, n[1]))
    uv = np.column_stack([u.ravel(), v.ravel()])
    rays = cam_a.pixel_to_ray(uv)
    P = rays / rays[:, 2:3] * DEPTH_M
    d = np.linalg.norm(cam_b.project(P) - uv, axis=1).reshape(n[1], n[0])
    return d


def main():
    b = board()
    det = cv2.aruco.CharucoDetector(b)
    cal = CameraModel.load_json(HERE / "w_nadir_10_sim.json")
    poz = {Path(p["dosya"]).name: p for p in json.loads((HERE / "kareler/pozlar.json").read_text())}

    tr = views("egitim", det, b)
    te = views("test", det, b)
    # calibrate.py ile aynı seçim: >= 12 köşe (AUTO_MIN_POINTS)
    tr_ok = [v for _, v in tr if v is not None and len(v.ids) >= 12]
    res = calibrate(tr_ok, (W, H))
    dK = np.abs(res.camera.K - cal.K).max()
    dD = np.abs(res.camera.dist - cal.dist).max()
    print(f"eğitim: {len(tr)} kare, {len(tr_ok)} kullanıldı; RMS {res.rms_px:.4f} px "
          f"(sim ölçekli tahta; JSON'a göre max|ΔK|={dK:.2e}, max|ΔD|={dD:.2e})")

    # Kapsama: eğitim köşelerinin 8×6 görüntü hücresine dağılımı
    allpts = np.vstack([v.img for v in tr_ok])
    cells = np.zeros((6, 8), int)
    np.add.at(cells, (np.minimum(allpts[:, 1] * 6 // H, 5).astype(int),
                      np.minimum(allpts[:, 0] * 8 // W, 7).astype(int)), 1)
    print(f"kapsama: 8×6 hücrenin {np.count_nonzero(cells)}'i dolu; en az dolu hücre {cells.min()} köşe")
    print(cells)

    # Test görünümleri
    models = {"kalibre": cal, "nominal": NOMINAL, "pinhole": PINHOLE}
    sonuc = {m: [] for m in models}
    rows = []
    c_board = np.array([BOARD_SQUARES[0] * SQUARE_M / 2, BOARD_SQUARES[1] * SQUARE_M / 2, 0.0])
    for name, v in te:
        if v is None:
            print(f"{name}: tespit yok / az köşe")
            continue
        row = [name, int(len(v.ids))]
        for m, cam in models.items():
            e2, r, t = pnp_rms(v, cam)
            sonuc[m].append(e2)
            row.append(float(np.sqrt(e2.mean())))
            if m == "kalibre":
                R, _ = cv2.Rodrigues(r)
                cam_pos_board = -R.T @ t.ravel()
                d_pnp = np.linalg.norm(cam_pos_board - c_board)
                p = poz[name]
                d_plan = np.linalg.norm(np.array(p["C"]) - np.array([0, 0, BOARD_Z]))
                row.append(float(d_pnp - d_plan))
        rows.append(row)
    print("\ntest görünümü        köşe  kalibre  nominal  pinhole   (RMS px)   PnP−plan uzaklık [m]")
    for r in rows:
        print(f"{r[0]:18s} {r[1]:5d} {r[2]:8.3f} {r[4]:8.3f} {r[5]:8.3f}             {r[3]:+.4f}")
    pooled = {m: float(np.sqrt(np.concatenate(sonuc[m]).mean())) for m in models}
    print("toplu RMS        " + "  ".join(f"{m} {pooled[m]:.3f}" for m in models))

    # Tüm görünümlerde taze kare denetimi
    taze = []
    for name, v in tr + te:
        if v is None:
            continue
        _, r, t = pnp_rms(v, cal)
        R, _ = cv2.Rodrigues(r)
        d_pnp = np.linalg.norm(-R.T @ t.ravel() - c_board)
        d_plan = np.linalg.norm(np.array(poz[name]["C"]) - np.array([0, 0, BOARD_Z]))
        taze.append(d_pnp - d_plan)
    taze = np.abs(np.array(taze))
    print(f"taze kare denetimi: {len(taze)} görünüm, |PnP − plan uzaklığı| en çok {taze.max() * 100:.2f} cm")

    # Model farkı haritası
    disp = {}
    for m, cam in (("nominal", NOMINAL), ("pinhole", PINHOLE)):
        d = displacement(cal, cam)
        disp[m] = {"max_px": float(d.max()), "kose_px": float(d[0, 0]), "merkez_px": float(d[d.shape[0] // 2, d.shape[1] // 2]),
                   "kenar_orta_x_px": float(d[d.shape[0] // 2, 0]), "rms_px": float(np.sqrt((d ** 2).mean()))}
        print(f"kalibre ↔ {m}: |Δ| en çok {d.max():.2f} px, köşe {d[0, 0]:.2f}, sol kenar ortası "
              f"{d[d.shape[0] // 2, 0]:.2f}, merkez {d[d.shape[0] // 2, d.shape[1] // 2]:.2f} px ({DEPTH_M} m derinlik)")
    cal_d0 = CameraModel(K=cal.K, dist=np.zeros(5), width=W, height=H)
    d_d0 = displacement(cal, cal_d0)
    disp["kalibre_D_etkisi"] = {"max_px": float(d_d0.max()), "kose_px": float(d_d0[0, 0]),
                                "kenar_orta_x_px": float(d_d0[d_d0.shape[0] // 2, 0])}
    print(f"kalibre D'nin kendi etkisi (aynı K, D ↔ 0): en çok {d_d0.max():.2f} px, köşe {d_d0[0, 0]:.2f}, "
          f"sol kenar ortası {d_d0[d_d0.shape[0] // 2, 0]:.2f} px")
    d_np = displacement(PINHOLE, NOMINAL)
    print(f"(başvuru) nominal SDF distorsiyonunun beklenen etkisi (pinhole ↔ nominal): en çok {d_np.max():.2f} px")

    out = {
        "egitim": {"kare": len(tr), "kullanilan": len(tr_ok), "rms_px": float(res.rms_px),
                   "gorunum_rms_px": res.per_view_rms_px.tolist(), "kapsama_8x6_dolu": int(np.count_nonzero(cells)), "kapsama_8x6": cells.tolist()},
        "test": {"kare": len(te), "kullanilan": len(rows), "toplu_rms_px": pooled,
                 "gorunum": [{"dosya": r[0], "kose": r[1], "kalibre": r[2], "nominal": r[4], "pinhole": r[5],
                              "pnp_plan_uzaklik_farki_m": r[3]} for r in rows]},
        "model_farki_px_5m": disp,
        "nominal_distorsiyon_etkisi_max_px": float(d_np.max()),
        "taze_kare_max_uzaklik_farki_m": float(taze.max()),
    }
    (HERE / "degerlendirme.json").write_text(json.dumps(out, indent=1))
    print("degerlendirme.json yazıldı")


if __name__ == "__main__":
    main()
