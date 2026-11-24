#!/usr/bin/env python3
"""gercekci_olcum.py — gs_kamera gerçekçilik ayarlarının ArUco köşe hatasına etkisi (statik Gazebo kamerası, uçuşsuz).

aruco_aci_tarama.py düzeni: kamera_test_W (W_nadir_10 ile aynı 6 mm kamera) marker'ın 100 m üstüne konur; her pozda
KEEP kare alınır, her kare HER varyantla işlenir (aynı gz karesi → varyantlar arası fark yalnız işleme). Ölçüler:
  - titreşim σ [px]: köşe koordinatının kareler arası std'si (köşe ve eksen başına, havuzlanmış kök ortalama kare) —
    bapp `noise.SIGMA_PX` (laptop `live_aruco.py --jitter` 0,30–0,35 px) ile aynı tanım,
  - bias [px]: ortalama köşe − beklenen köşe (beklenen = kamera pozu + varyantın kalibrasyonu, distorsiyon dahil),
  - içe bias [px/köşe]: (beklenen kenar − ölçülen kenar)/2.
Kullanım (yalnız gz sunucusu yeter; iha_arena_aruco, aynı GZ_PARTITION):
  ~/bapp/.venv-ros/bin/python test/aruco_tulpar_2026-11-24/gercekci_olcum.py <çıktı.json>
"""
import json
import math
import os
import sys
import time

import cv2
import numpy as np

SIM = os.path.expanduser("~/sim/iha_sim_v2")
sys.path.insert(0, os.path.join(SIM, "gz_models_cam_hw"))
sys.path.insert(0, SIM)
import aruco_aci_tarama as t  # noqa: E402
import gs_kamera as g  # noqa: E402
from bap_geo.aruco import MarkerDetector  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.transport13 import Node  # noqa: E402

t.SKIP, t.KEEP = 3, 12
KAL_SIM = json.load(open(os.path.join(SIM, "kalibrasyon/w_nadir_10_sim.json")))
K = np.array(KAL_SIM["K"])
W, H = KAL_SIM["width"], KAL_SIM["height"]
D_G = np.array(g.GERCEKCI_DISTORSIYON)
GEO = g.Geometri(K, D_G, W, H)                     # boresight 0: titreşim/bias ölçümü (boresight ayrı geometrik kayma)

VARYANTLAR = {   # ad: (geo, bulanıklık σ, shot, read, beklenen D)
    "taban": (None, 0.0, 0.0, 0.0, None),
    "dist": (GEO, 0.0, 0.0, 0.0, D_G),
    "dist+b0.5": (GEO, 0.5, 0.0, 0.0, D_G),
    "dist+b0.7": (GEO, 0.7, 0.0, 0.0, D_G),
    "dist+b0.7+g4/2": (GEO, 0.7, 4.0, 2.0, D_G),
    "dist+b1.0+g4/2": (GEO, 1.0, 4.0, 2.0, D_G),
    "dist+b0.7+g8/3": (GEO, 0.7, 8.0, 3.0, D_G),
    "dist+b1.0+g8/3": (GEO, 1.0, 8.0, 3.0, D_G),
}


def beklenen(C, R_wc, D):
    k = t.MARKER + np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0.0]]) * t.BOY / 2
    p = (k - C) @ R_wc                                  # OpenCV kamera çerçevesi
    uv, _ = cv2.projectPoints(p.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D if D is not None else np.zeros(5))
    return uv.reshape(-1, 2)


def kareler(h, egim, dl, psi):
    C, R_model, R_wc = t.duzen(h, egim, dl, psi)
    w, x, y, z = t.quat(R_model)
    with t.lock:
        t.state.update(want=False, frames=[], skip=0, after=None)
    t.gz_service(f"/world/{t.WORLD}/set_pose", "gz.msgs.Pose",
                 f'name: "{t.NAME}", position: {{x: {C[0]:.4f}, y: {C[1]:.4f}, z: {C[2]:.4f}}}, '
                 f'orientation: {{w: {w:.9f}, x: {x:.9f}, y: {y:.9f}, z: {z:.9f}}}')
    with t.lock:
        t.state.update(after=t.state["last"], want=True)
    t0 = time.time()
    while time.time() - t0 < 40:
        with t.lock:
            if len(t.state["frames"]) >= t.KEEP:
                break
        time.sleep(0.02)
    with t.lock:
        fr, t.state["want"] = list(t.state["frames"]), False
    return fr, C, R_wc


def main():
    out = sys.argv[1]
    det = MarkerDetector(target_ids=[0])
    C, R_model, _ = t.duzen(100, 0, 0)
    w, x, y, z = t.quat(R_model)
    ok, msg = t.gz_service(f"/world/{t.WORLD}/create", "gz.msgs.EntityFactory",
                           f'sdf_filename: "{t.MODEL_SDF}", name: "{t.NAME}", allow_renaming: false, '
                           f'pose: {{position: {{x: {C[0]}, y: {C[1]}, z: {C[2]}}}, orientation: {{w: {w}, x: {x}, y: {y}, z: {z}}}}}')
    print("model:", ok, msg, flush=True)
    node = Node()
    topic = None
    for _ in range(30):
        topic = next((tp for tp in node.topic_list() if f"/model/{t.NAME}/" in tp and tp.endswith("/image")), None)
        if topic:
            break
        time.sleep(1)
    node.subscribe(Image, topic, t.on_image)
    rng = np.random.default_rng(7)
    sonuc = {"varyantlar": {k: {"bulaniklik": v[1], "shot": v[2], "read": v[3], "dist": v[4] is not None}
                            for k, v in VARYANTLAR.items()}, "pozlar": []}
    # kadraj merkezi → kenar, iki yön; 0,37/0,61 m kaydırmalar köşeleri farklı alt piksel fazlarına koyar
    pozlar = [(0, 0), (0.37, 0), (15, 0), (15.61, 30), (30, 0), (30.37, 30), (0, 30), (8.61, 45)]
    for dl, psi in pozlar:
        fr, C, R_wc = kareler(100, 0, dl, psi)
        for ad, (geo, b, sh, rd, D) in VARYANTLAR.items():
            uv_b = beklenen(C, R_wc, D)
            cs = []
            for rgb in fr:
                d = det.detect(g.isle(rgb, sh, rd, rng, geo, b))
                if 0 in d:
                    c = d[0]
                    k = min(range(4), key=lambda k: float(np.sum((np.roll(c, k, 0) - uv_b) ** 2)))
                    cs.append(np.roll(c, k, 0))
            r = {"dl": dl, "psi": psi, "varyant": ad, "kare": len(fr), "tespit": len(cs)}
            if len(cs) >= 3:
                A = np.array(cs)                               # (n, 4, 2)
                r["titresim_px"] = float(np.sqrt(np.mean(A.std(0, ddof=1) ** 2)))
                ort = A.mean(0)
                r["bias_px"] = float(np.sqrt(np.mean(np.sum((ort - uv_b) ** 2, 1))))
                kb = np.linalg.norm(uv_b - np.roll(uv_b, -1, 0), axis=1).mean()
                ko = np.linalg.norm(ort - np.roll(ort, -1, 0), axis=1).mean()
                r["ice_bias_px"] = float((kb - ko) / 2)
                r["kenar_px"] = float(ko)
                r["merkez_u"] = float(uv_b[:, 0].mean())
            sonuc["pozlar"].append(r)
            print(json.dumps(r), flush=True)
        json.dump(sonuc, open(out, "w"), indent=1)
    # özet: varyant başına medyan
    ozet = {}
    for ad in VARYANTLAR:
        rr = [r for r in sonuc["pozlar"] if r["varyant"] == ad and "titresim_px" in r]
        ozet[ad] = {k: round(float(np.median([r[k] for r in rr])), 3) for k in ("titresim_px", "bias_px", "ice_bias_px", "kenar_px")}
        ozet[ad]["tespit"] = f'{sum(r["tespit"] for r in sonuc["pozlar"] if r["varyant"] == ad)}/' \
                             f'{sum(r["kare"] for r in sonuc["pozlar"] if r["varyant"] == ad)}'
    sonuc["ozet"] = ozet
    json.dump(sonuc, open(out, "w"), indent=1)
    for ad, o in ozet.items():
        print(f"{ad:18s} {o}")
    t.gz_service(f"/world/{t.WORLD}/remove", "gz.msgs.Entity", f'name: "{t.NAME}", type: MODEL')


if __name__ == "__main__":
    main()
