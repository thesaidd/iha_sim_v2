"""Statik kamera_test ile QR bulma/çözme (tek irtifa listesi). Kullanım: qr_statik.py <çıktı klasörü> <etiket> [irtifalar...]"""
import json, math, os, subprocess, sys, threading, time
import cv2, numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.world_stats_pb2 import WorldStatistics
MODEL = os.path.expanduser("~/iha_sim_v2/gz_models_cam_hw/kamera_test/model.sdf")
NAME, QR, FX, W, H = "kamera_test", (-60.0, -60.0), 4637.81, 1456, 1088
OUT, TAG = sys.argv[1], sys.argv[2]; ALTS = [float(a) for a in sys.argv[3:]] or [60.0]
os.makedirs(OUT, exist_ok=True)
def svc(s, t, req):
    r = subprocess.run(["gz", "service", "-s", s, "--reqtype", t, "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req], capture_output=True, text=True)
    return "data: true" in r.stdout
def pose(alt): return f'name: "{NAME}", position: {{x: {QR[0]}, y: {QR[1]}, z: {alt + 0.05}}}, orientation: {{w: 1, x: 0, y: 0, z: 0}}'
frames, lock, want = [], threading.Lock(), [False]
def on_img(m):
    if want[0]:
        with lock: frames.append(np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3).copy())
st = []
node = Node()
node.subscribe(WorldStatistics, "/world/default/stats", lambda m: st.append((m.sim_time.sec + m.sim_time.nsec * 1e-9, m.real_time.sec + m.real_time.nsec * 1e-9)))
print("create:", svc("/world/default/create", "gz.msgs.EntityFactory", f'sdf_filename: "{MODEL}", name: "{NAME}", pose: {{position: {{x: {QR[0]}, y: {QR[1]}, z: {ALTS[0] + 0.05}}}}}'))
time.sleep(3)
node.subscribe(Image, f"/world/default/model/{NAME}/link/link/sensor/camera/image", on_img)
res = []
for alt in ALTS:
    svc("/world/default/set_pose", "gz.msgs.Pose", pose(alt)); time.sleep(2.0)
    with lock: frames.clear()
    n0 = len(st); want[0] = True
    t = time.time()
    while len(frames) < 20 and time.time() - t < 30: time.sleep(0.1)
    want[0] = False
    with lock: fr = list(frames)
    rtf = (st[-1][0] - st[n0][0]) / (st[-1][1] - st[n0][1]) if len(st) > n0 + 1 else float("nan")
    qd, ar = cv2.QRCodeDetector(), cv2.QRCodeDetectorAruco()
    r = {"etiket": TAG, "irtifa": alt, "kare": len(fr), "rtf": round(rtf, 3), "teorik_kenar_px": round(FX / alt, 1)}
    for ad, det in (("QRCodeDetector", qd), ("QRCodeDetectorAruco", ar)):
        bul = coz = 0; ms = []; kenar = []
        for rgb in fr:
            g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY); t0 = time.perf_counter()
            ok, pts = det.detect(g); txt, p2, _ = det.detectAndDecode(g); ms.append(1000 * (time.perf_counter() - t0))
            if ok and pts is not None:
                q = np.asarray(pts).reshape(-1, 4, 2)[0]; c = q.mean(axis=0)
                if math.hypot(c[0] - W / 2, c[1] - H / 2) <= max(10, 0.5 * FX / alt):
                    bul += 1; kenar.append(float(np.mean([np.linalg.norm(q[i] - q[(i + 1) % 4]) for i in range(4)])))
            coz += txt == "HEDEF-01"
        r[ad] = {"bulma": bul, "cozme": coz, "ms_ort": round(float(np.mean(ms)), 1) if ms else None, "kenar_px": round(float(np.mean(kenar)), 1) if kenar else None}
    if fr:
        g = cv2.cvtColor(fr[-1], cv2.COLOR_RGB2GRAY)
        r["kare_gri_ort"] = round(float(g.mean()), 1); r["kare_gri_std"] = round(float(g.std()), 1)
        cv2.imwrite(os.path.join(OUT, f"{TAG}_{int(alt)}m.png"), cv2.cvtColor(fr[-1], cv2.COLOR_RGB2BGR))
        k = int(max(120, 3 * FX / alt)); cx, cy = W // 2, H // 2
        cv2.imwrite(os.path.join(OUT, f"{TAG}_{int(alt)}m_kirpma.png"), cv2.resize(cv2.cvtColor(fr[-1][cy - k:cy + k, cx - k:cx + k], cv2.COLOR_RGB2BGR), (480, 480), interpolation=cv2.INTER_NEAREST))
    res.append(r); print(json.dumps(r, ensure_ascii=False), flush=True)
print("remove:", svc("/world/default/remove", "gz.msgs.Entity", f'name: "{NAME}", type: MODEL'))
with open(os.path.join(OUT, f"{TAG}.json"), "w") as f: json.dump(res, f, ensure_ascii=False, indent=1)
