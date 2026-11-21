#!/usr/bin/env python3
"""
aruco_aci_tarama.py — 6 mm GS kamerasının montaj açısı / yanal konum / irtifa ile ArUco tespit sınırını Gazebo'da ölçer
ve kamera_geometri.py'nin teorik tespit şeridiyle karşılaştırır (uçuşsuz; 2026-11-20).

Yol (qr_irtifa_tarama.py deseni): kamera_test_W modeli /world/default/create ile eklenir, her poz /world/default/set_pose
ile kurulur (`gz service` alt süreci), görüntü gz.transport13 aboneliğiyle alınır (ROS köprüsü yok). Her pozda set_pose'tan
sonra damgası ilerleyen ilk kareden itibaren SKIP kare atılır, KEEP kare işlenir: gs_kamera.isle (RGGB Bayer + demozaik,
uçuştaki zincirle aynı) → bapp MarkerDetector (aruco3 kapalı + subpix, id 0). Beklenen köşeler: kamera pozu + sim
kalibrasyonu (kalibrasyon/w_nadir_10_sim.json) ile izdüşüm.

Kamera düzeni (dünya ENU): ψ = 0'da görüntü üstü = kuzey (uçuş yönü), sağ = doğu (yanal, sağ kanat). Yan eğim θ:
optik eksen yanal eksen yönünde θ kadar eğik (kamera_geometri R_body_cam(yan=θ) ile aynı). Δl: marker'ın kameranın
DİK İZDÜŞÜM noktasına göre yanal uzaklığı [m] (+ = sağ). Deneyler:
  A) θ ∈ {0, 10, 20, 30}°, h 100 m, Δl taraması (kaba 4 m + teorik şerit kenarlarında 0,5 m),
  B) θ 0, ψ 45° (marker kadraja göre 45° dönük — yanal genişlik en büyük), kenarlarda 0,5 m,
  C) θ 0, Δl 0, irtifa 80–200 m (marker piksel boyu → tespit alt sınırı).
Kullanım (sim açık, ör. CAM=W10 ./launch_all.sh; aynı GZ_PARTITION):
  ~/bapp/.venv-ros/bin/python gz_models_cam_hw/aruco_aci_tarama.py <çıktı_klasörü> [--hizli]
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
sys.path.insert(0, SIM)
sys.path.insert(0, os.path.expanduser(os.environ.get("BAPP", "~/bapp")))
import gs_kamera  # noqa: E402
import kamera_geometri as kg  # noqa: E402
from bap_geo.aruco import MarkerDetector  # noqa: E402

MODEL_SDF = os.path.join(HERE, "kamera_test_W", "model.sdf")
NAME = "kamera_test_w"
WORLD = "default"
MARKER = np.array([-60.0, -60.0, 0.02])     # ENU (make_iha_arena.py --hedef aruco), yaw 0, siyah kare 2 m
BOY = 2.0
SKIP, KEEP = 3, 6


def gz_service(service, reqtype, req, reptype="gz.msgs.Boolean", timeout=5000):
    r = subprocess.run(["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", reptype,
                        "--timeout", str(timeout), "--req", req], capture_output=True, text=True)
    return "data: true" in r.stdout, (r.stdout + r.stderr).strip()


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return w, x, y, z


def duzen(h, egim_deg, dl, psi_deg=0.0):
    """(model konumu C, model dönüşü R_model, kamera R_wc (OpenCV eksenleri dünya ENU'da))."""
    psi = math.radians(psi_deg)
    R_model = Rz(psi) @ Ry(-math.radians(egim_deg)) @ Rz(math.pi / 2)
    l = Rz(psi) @ np.array([1.0, 0, 0])                 # yanal (sağ)
    C = MARKER - dl * l + np.array([0, 0, h])
    z_cv = R_model @ np.array([0, 0, -1.0])
    y_cv = -(R_model @ np.array([1.0, 0, 0]))
    x_cv = -(R_model @ np.array([0, 1.0, 0]))
    return C, R_model, np.stack([x_cv, y_cv, z_cv], 1)


def beklenen(kam, C, R_wc):
    k = MARKER + np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0.0]]) * BOY / 2
    p = (k - C) @ R_wc
    uv, z = kam.izdus(p)
    return uv, z


state = {"after": None, "skip": 0, "frames": [], "last": 0.0, "want": False}
lock = threading.Lock()


def on_image(msg):
    st = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    with lock:
        state["last"] = st
        if not state["want"] or state["after"] is None or st <= state["after"]:
            return
        if state["skip"] < SKIP:
            state["skip"] += 1
            return
        if len(state["frames"]) < KEEP:
            state["frames"].append(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy())


def olc(kam, det, h, egim, dl, psi=0.0, kaydet=None):
    C, R_model, R_wc = duzen(h, egim, dl, psi)
    w, x, y, z = quat(R_model)
    with lock:
        state.update(want=False, frames=[], skip=0, after=None)
    ok, _ = gz_service(f"/world/{WORLD}/set_pose", "gz.msgs.Pose",
                       f'name: "{NAME}", position: {{x: {C[0]:.4f}, y: {C[1]:.4f}, z: {C[2]:.4f}}}, '
                       f'orientation: {{w: {w:.9f}, x: {x:.9f}, y: {y:.9f}, z: {z:.9f}}}')
    with lock:
        state.update(after=state["last"], want=True)
    t0 = time.time()
    while time.time() - t0 < 30:
        with lock:
            if len(state["frames"]) >= KEEP:
                break
        time.sleep(0.02)
    with lock:
        frames, state["want"] = list(state["frames"]), False
    uv_b, zb = beklenen(kam, C, R_wc)
    marj = float(min(uv_b[:, 0].min(), kam.w - uv_b[:, 0].max(), uv_b[:, 1].min(), kam.h - uv_b[:, 1].max()))
    kenar_b = float(np.linalg.norm(uv_b - np.roll(uv_b, -1, 0), axis=1).min())
    n_det, hata, kenar = 0, [], []
    for i, rgb in enumerate(frames):
        bgr = gs_kamera.isle(rgb)
        d = det.detect(bgr)
        if kaydet and i == 0:
            import cv2
            cv2.imwrite(kaydet, bgr)
        if 0 in d:
            c = d[0]
            n_det += 1
            # köşe eşleşmesi: döngüsel kaydırmaların en iyisi (id 0'ın köşe sırası dünyaya göre sabit ama emin olmayalım)
            e = min(float(np.sqrt(np.mean(np.sum((np.roll(c, k, 0) - uv_b) ** 2, 1)))) for k in range(4))
            hata.append(e)
            kenar.append(float(np.linalg.norm(c - np.roll(c, -1, 0), axis=1).min()))
    return {"h": h, "egim_deg": egim, "psi_deg": psi, "dl_m": round(dl, 2), "set_pose": ok, "kare": len(frames),
            "tespit": n_det, "oran": round(n_det / len(frames), 3) if frames else None,
            "beklenen_kenar_px": round(kenar_b, 2), "beklenen_marj_px": round(marj, 1),
            "kose_rms_px": round(float(np.median(hata)), 3) if hata else None,
            "kenar_px": round(float(np.median(kenar)), 2) if kenar else None,
            "u_merkez": round(float(uv_b[:, 0].mean()), 1)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cikis")
    ap.add_argument("--hizli", action="store_true", help="kaba adım 8 m, ince adım 1 m (deneme)")
    a = ap.parse_args()
    os.makedirs(a.cikis, exist_ok=True)
    kam = kg.Kamera.json_oku(os.path.join(SIM, "kalibrasyon", "w_nadir_10_sim.json"))
    det = MarkerDetector(target_ids=[0])
    kaba, ince = (8.0, 1.0) if a.hizli else (4.0, 0.5)
    C, R_model, _ = duzen(100, 0, 0)
    w, x, y, z = quat(R_model)
    ok, out = gz_service(f"/world/{WORLD}/create", "gz.msgs.EntityFactory",
                         f'sdf_filename: "{MODEL_SDF}", name: "{NAME}", allow_renaming: false, '
                         f'pose: {{position: {{x: {C[0]}, y: {C[1]}, z: {C[2]}}}, orientation: {{w: {w}, x: {x}, y: {y}, z: {z}}}}}')
    print(f"model ekleme: {ok} ({out})", flush=True)
    if not ok:
        sys.exit(1)
    node = Node()
    topic = None
    for _ in range(30):
        topic = next((t for t in node.topic_list() if f"/model/{NAME}/" in t and t.endswith("/image")), None)
        if topic:
            break
        time.sleep(1)
    print(f"görüntü topic'i: {topic}", flush=True)
    node.subscribe(Image, topic, on_image)
    sonuc = {"teori": {}, "olcum": []}
    H = kam.hfov / 2

    def kos(h, egim, dl, psi=0.0, deney="", kaydet=None):
        r = olc(kam, det, h, egim, dl, psi, kaydet)
        r["deney"] = deney
        sonuc["olcum"].append(r)
        print(f"{time.strftime('%H:%M:%S')} {deney} h {h:5.0f} θ {egim:4.0f}° ψ {psi:3.0f}° Δl {dl:+7.2f} m: "
              f"{r['tespit']}/{r['kare']}  kenar {r['kenar_px']} (beklenen {r['beklenen_kenar_px']}) px  marj "
              f"{r['beklenen_marj_px']} px  köşe rms {r['kose_rms_px']} px", flush=True)
        json.dump(sonuc, open(os.path.join(a.cikis, "sonuc.json"), "w"), ensure_ascii=False, indent=1)

    try:
        # A) yan eğim × yanal konum (roll/pitch payı 0: sabit kamera; teori ile birebir karşılaştırma)
        for egim in (0, 10, 20, 30):
            s = kg.tespit_seridi(kam, 100, yan=egim, roll_pay=0, pitch_pay=0, yawlar=(0.0,))
            sonuc["teori"][f"A_{egim}"] = s
            lo, hi = 100 * math.tan(math.radians(egim) - H), 100 * math.tan(math.radians(egim) + H)
            noktalar = set(np.round(np.arange(lo - 2, hi + 2 + 1e-9, kaba), 2))
            for kenar_ in (s["alt"], s["ust"]):
                if kenar_ is not None:
                    noktalar |= set(np.round(np.arange(kenar_ - 2, kenar_ + 2 + 1e-9, ince), 2))
            for dl in sorted(noktalar):
                kos(100, egim, float(dl), deney="A",
                    kaydet=os.path.join(a.cikis, f"A_{egim}_nadir.png") if abs(dl - 100 * math.tan(math.radians(egim))) < kaba / 2 else None)
        # B) marker kadraja 45° dönük (ψ 45): yanal genişlik √2 m → şerit daralır
        s = kg.tespit_seridi(kam, 100, roll_pay=0, pitch_pay=0, yawlar=(45.0,))
        sonuc["teori"]["B_45"] = s
        for kenar_ in (s["alt"], s["ust"]):
            for dl in np.arange(kenar_ - 2, kenar_ + 2 + 1e-9, ince):
                kos(100, 0, float(dl), psi=45.0, deney="B")
        # C) irtifa (Δl 0, nadir)
        for h in (80, 100, 120, 140, 160, 180, 200, 220):
            kos(float(h), 0, 0.0, deney="C", kaydet=os.path.join(a.cikis, f"C_{h}m.png") if h in (100, 160, 200) else None)
    finally:
        ok, out = gz_service(f"/world/{WORLD}/remove", "gz.msgs.Entity", f'name: "{NAME}", type: MODEL')
        print(f"model silme: {ok} ({out})", flush=True)
        json.dump(sonuc, open(os.path.join(a.cikis, "sonuc.json"), "w"), ensure_ascii=False, indent=1)
    print(f"bitti: {len(sonuc['olcum'])} poz", flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
