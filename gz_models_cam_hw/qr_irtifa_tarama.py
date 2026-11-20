#!/usr/bin/env python3
"""
qr_irtifa_tarama.py — sabit test kamerasıyla QR (HEDEF-01, (−60, −60)) bulma/çözme irtifa taraması.

Yol: kamera_test modeli /world/default/create ile eklenir, /world/default/set_pose ile taşınır,
/world/default/remove ile silinir — hepsi `gz service` CLI (alt süreç) ile. Neden: gz.transport13 Python
bağında abonelik açıkken request() yanıtı teslim alınamıyor (CLAUDE.md); görüntü aboneliği sürekli açık.
Görüntü doğrudan gz.transport13 aboneliğiyle (ROS köprüsü yok). Poz her adımda `gz model -p` ile doğrulanır.

Her (irtifa, yaw): poz → set_pose yanıtından sonra stamp'i ilerleyen ilk kareden itibaren 5 kare atılır,
sonraki 20 kare analiz edilir. Varyantlar: ham, adaptif eşik, 2× büyütme, 2× + adaptif eşik.
Dedektörler: cv2.QRCodeDetector, cv2.QRCodeDetectorAruco (pyzbar kurulu değil → yok).
Bulma: detect() dörtgen döndürdü ve merkezi beklenen noktaya ≤ max(10, 0,5 × teorik kenar) px.
Çözme: detectAndDecode() == "HEDEF-01". Süre: ön işleme + detect + detectAndDecode (ms).

Kullanım: qr_irtifa_tarama.py <çıktı_klasörü> <dur_dosyası>
"""
import json
import math
import os
import re
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_SDF = os.path.join(HERE, "kamera_test", "model.sdf")
NAME = "kamera_test"
WORLD = "default"
QR_XY = (-60.0, -60.0)
QR_Z = 0.05                       # levha yüksekliği (make_iha_arena.py QR_Z)
CODE_M = 1.00                     # kodun kenarı (kenar boşluğu hariç)
TEXT = "HEDEF-01"
FX = 4637.81                      # camera_info (T_nadir, 2026-10-30)
W, H = 1456, 1088
CX, CY = W / 2, H / 2
ALTITUDES = [20, 30, 40, 50, 60, 70, 80, 90, 100, 120]
YAWS = [0, 45]
SKIP, KEEP = 5, 20
SAVE_AT = {40, 80, 100}
VARIANTS = ["ham", "adaptif", "2x", "2x+adaptif"]
DETECTORS = ["QRCodeDetector", "QRCodeDetectorAruco"]

OUT, STOP_FILE = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)


def gz_service(service, reqtype, req, reptype="gz.msgs.Boolean", timeout=5000):
    r = subprocess.run(["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", reptype,
                        "--timeout", str(timeout), "--req", req], capture_output=True, text=True)
    return "data: true" in r.stdout, (r.stdout + r.stderr).strip()


def pose_req(alt, yaw_deg):
    y = math.radians(yaw_deg)
    return (f'name: "{NAME}", position: {{x: {QR_XY[0]}, y: {QR_XY[1]}, z: {QR_Z + alt}}}, '
            f'orientation: {{w: {math.cos(y / 2):.9f}, x: 0, y: 0, z: {math.sin(y / 2):.9f}}}')


def model_pose():
    r = subprocess.run(["gz", "model", "-m", NAME, "-p"], capture_output=True, text=True).stdout
    # "Model: [<varlık no>]" satırını atla; konum/rpy "Pose [ XYZ (m) ] [ RPY (rad) ]" başlığından sonra gelir
    nums = re.findall(r"\[([-\d.e ]+)\]", r.split("RPY (rad) ]", 1)[-1])
    if len(nums) < 2:
        return None
    xyz, rpy = [float(v) for v in nums[0].split()], [float(v) for v in nums[1].split()]
    return xyz, rpy


# --- görüntü aboneliği -------------------------------------------------------------------------
lock = threading.Lock()
state = {"after": None, "skip": 0, "frames": [], "last_stamp": 0.0, "count": 0, "want": False}


def on_image(msg):
    stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    with lock:
        state["last_stamp"] = stamp
        state["count"] += 1
        if not state["want"] or state["after"] is None or stamp <= state["after"]:
            return
        if state["skip"] < SKIP:
            state["skip"] += 1
            return
        if len(state["frames"]) < KEEP:
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
            state["frames"].append((stamp, rgb))


def preprocess(gray, variant):
    if variant == "ham":
        return gray, 1.0
    if variant == "adaptif":
        return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, 5), 1.0
    up = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
    if variant == "2x":
        return up, 2.0
    return cv2.adaptiveThreshold(up, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 101, 5), 2.0


def quad_of(pts, scale):
    if pts is None:
        return None
    q = np.asarray(pts, dtype=np.float64).reshape(-1, 4, 2)
    return q[0] / scale if len(q) else None


def analyse(frames, alt, yaw):
    side_th = FX * CODE_M / alt
    tol = max(10.0, 0.5 * side_th)
    dets = {"QRCodeDetector": cv2.QRCodeDetector(), "QRCodeDetectorAruco": cv2.QRCodeDetectorAruco()}
    res = {}
    for v in VARIANTS:
        for dn in DETECTORS:
            res[f"{v}|{dn}"] = {"bulma": 0, "cozme": 0, "yanlis_icerik": 0, "merkez": [], "kenar": [], "ms": []}
    grays = [cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY) for _, rgb in frames]
    diff = max(int(np.abs(g.astype(np.int16) - grays[0]).max()) for g in grays)
    for gray in grays:
        for v in VARIANTS:
            t0 = time.perf_counter()
            img, s = preprocess(gray, v)
            t_pre = time.perf_counter() - t0
            for dn, det in dets.items():
                r = res[f"{v}|{dn}"]
                t1 = time.perf_counter()
                ok, pts = det.detect(img)
                text, _, _ = det.detectAndDecode(img)
                r["ms"].append((t_pre + time.perf_counter() - t1) * 1000)
                q = quad_of(pts, s) if ok else None
                if q is not None:
                    c = q.mean(axis=0)
                    off = float(math.hypot(c[0] - CX, c[1] - CY))
                    if off <= tol:
                        r["bulma"] += 1
                        r["merkez"].append(off)
                        r["kenar"].append(float(np.mean([np.linalg.norm(q[(i + 1) % 4] - q[i]) for i in range(4)])))
                if text == TEXT:
                    r["cozme"] += 1
                elif text:
                    r["yanlis_icerik"] += 1
    summary = {}
    for k, r in res.items():
        summary[k] = {
            "bulma_%": 100.0 * r["bulma"] / len(frames), "cozme_%": 100.0 * r["cozme"] / len(frames),
            "yanlis_icerik": r["yanlis_icerik"],
            "merkez_px": float(np.mean(r["merkez"])) if r["merkez"] else None,
            "kenar_px": float(np.mean(r["kenar"])) if r["kenar"] else None,
            "ms": float(np.mean(r["ms"])),
        }
    measured = summary["ham|QRCodeDetector"]["kenar_px"]
    src = "ham|QRCodeDetector"
    if measured is None:
        for k, s in summary.items():
            if s["kenar_px"] is not None:
                measured, src = s["kenar_px"], k
                break
    return {"irtifa": alt, "yaw": yaw, "kenar_teorik_px": side_th, "modul_teorik_px": side_th / 21,
            "modul_olculen_px": measured / 21 if measured else None, "olcum_kaynagi": src if measured else None,
            "kareler_arasi_maks_fark": diff, "kare": len(frames), "sonuc": summary}


def save_frames(frames, alt):
    side = FX * CODE_M / alt
    rgb = frames[0][1]
    cv2.imwrite(os.path.join(OUT, f"kare_{alt:03d}m.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    half = int(max(40, 1.2 * side))
    crop = rgb[int(CY) - half:int(CY) + half, int(CX) - half:int(CX) + half]
    k = max(1, round(480 / crop.shape[0]))
    crop = cv2.resize(crop, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(os.path.join(OUT, f"kesit_{alt:03d}m_x{k}.png"), cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))


def main():
    ok, out = gz_service(f"/world/{WORLD}/create", "gz.msgs.EntityFactory",
                         f'sdf_filename: "{MODEL_SDF}", name: "{NAME}", allow_renaming: false, '
                         f'pose: {{position: {{x: {QR_XY[0]}, y: {QR_XY[1]}, z: {QR_Z + ALTITUDES[0]}}}}}')
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
    results, aborted = [], None
    try:
        for alt in ALTITUDES:
            for yaw in YAWS:
                if os.path.exists(STOP_FILE):
                    aborted = open(STOP_FILE).read().strip()
                    raise StopIteration
                with lock:
                    state.update(want=False, frames=[], skip=0, after=None)
                ok, out = gz_service(f"/world/{WORLD}/set_pose", "gz.msgs.Pose", pose_req(alt, yaw))
                with lock:
                    state.update(after=state["last_stamp"], want=True)
                pose = model_pose()
                t0 = time.time()
                while True:
                    with lock:
                        n = len(state["frames"])
                    if n >= KEEP or time.time() - t0 > 90:
                        break
                    time.sleep(0.05)
                with lock:
                    frames, state["want"] = list(state["frames"]), False
                if len(frames) < KEEP:
                    print(f"{alt} m / {yaw}°: yalnız {len(frames)} kare (zaman aşımı)", flush=True)
                if not frames:
                    continue
                t_cap = time.time() - t0
                r = analyse(frames, alt, yaw)
                r.update(set_pose=ok, poz=pose, toplama_sn=t_cap)
                results.append(r)
                if yaw == 0 and alt in SAVE_AT:
                    save_frames(frames, alt)
                s = r["sonuc"]
                print(f"{time.strftime('%H:%M:%S')} {alt:3d} m {yaw:2d}° set_pose={ok} poz={pose} "
                      f"modül teorik {r['modul_teorik_px']:.2f} ölçülen "
                      f"{r['modul_olculen_px'] if r['modul_olculen_px'] is None else round(r['modul_olculen_px'], 2)} "
                      f"| fark {r['kareler_arasi_maks_fark']} | " +
                      " ".join(f"{k.split('|')[0]}/{'QR' if k.endswith('Detector') else 'Ar'}="
                               f"{v['bulma_%']:.0f}/{v['cozme_%']:.0f}" for k, v in s.items()), flush=True)
                with open(os.path.join(OUT, "sonuclar.json"), "w") as f:
                    json.dump(results, f, indent=1, ensure_ascii=False)
    except StopIteration:
        print(f"DURDURULDU: {aborted}", flush=True)
    finally:
        ok, out = gz_service(f"/world/{WORLD}/remove", "gz.msgs.Entity", f'name: "{NAME}", type: MODEL')
        print(f"model silme: {ok} ({out})", flush=True)
    with open(os.path.join(OUT, "sonuclar.json"), "w") as f:
        json.dump(results, f, indent=1, ensure_ascii=False)
    print(f"bitti: {len(results)} poz{'; DURDURULDU' if aborted else ''}", flush=True)
    sys.stdout.flush()
    os._exit(3 if aborted else 0)


if __name__ == "__main__":
    main()
