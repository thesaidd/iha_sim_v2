#!/usr/bin/env python3
"""
qr_online.py — çevrimiçi QR algılama + dünya konumu tahmini (uav0 aşağı bakan kamera).

Girdi: /uav0/camera/image (camera_bridge.yaml, best effort), /fmu/out/vehicle_odometry (PX4 tahmini; tek poz
kaynağı), gz /world/default/clock (kare sim damgası → duvar saati). Çıktı: /hedef_konum (std_msgs/String, JSON):
  damga_sim, x_kuzey, y_dogu (uav0 yerel NED, m), icerik, kenar_px, modul_px, kare_no, bulma_no,
  ort_x_kuzey/ort_y_dogu/ort_n (bu geçişteki yayınların ortalaması), poz_dt_ms, yontem.
Özel mesaj paketi derlemek yerine JSON: istenen alanları (içerik, kenar, kare no) taşıyan hazır tip yok.

Algılama (2026-10-31, kayıtlı 678 karede ölçüldü): tam karede QRCodeDetectorAruco.detect (QR yokken 3,6 ms,
varken 21 ms; 25/25 buldu) → yalnızca bulununca QR çevresindeki ROI'de (kenar kadar pay) QRCodeDetector
.detectAndDecode (7,9 ms, 25/25); çözülemezse ROI'de Aruco (3,3 ms). Önceki karede bulunup bu karede Aruco
bulamazsa yedek olarak tam karede QRCodeDetector.detect (~40 ms). Tam karede QRCodeDetector.detectAndDecode
73–134 ms sürüyordu (10 Hz'de sınırda). İşleme ayrı iş parçacığında; kuyruk (2) doluysa kare atlanır ve sayılır.
Köprüde/DDS'te kaybolan kareler sim damgası aralığından ayrıca sayılır.

Konum: qr_analyze.py yöntemi — QR merkezi pikseli → gövde FRD ışını (kamera ofseti (0,07; 0; 0,1) FRD, dik aşağı)
→ PX4 tam yönelimi (roll, pitch, yaw) → zemin D = 0 (ev). Odometri, ROS'a VARIŞ zamanıyla eşleşir (PX4
timestamp_sample sim zamanı değil ve RTF < 1'de 1–2 sn geride, 2026-10-30).

Değerlendirme (yalnızca log, algılamayı etkilemez): Gazebo gerçek pozu (/world/default/dynamic_pose/info) ile
QR'ın geometrik olarak kadrajda olup olmadığı, gerçek pozla tahmin ve hatalar → <log>/degerlendirme.csv.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 qr_online.py --log <klasör>                      # çevrimiçi
  python3 qr_online.py --cevrimdisi <kayit> --log <klasör> # kayıtlı kareler (qr_recorder.py) üzerinde aynı mantık
"""

import argparse
import bisect
import collections
import csv
import json
import math
import os
import queue
import signal
import threading
import time

import cv2
import numpy as np

from qr_analyze import (CX, CY, FX, H, QR_E, QR_N, QR_Z, W, ENU_NED, FLU_FRD, euler_to_R, project_enu, quat_to_R,
                        ray_ground_ned)

TEXT = "HEDEF-01"
QR_CORNERS = [np.array([QR_E + dx, QR_N + dy, QR_Z]) for dx, dy in ((-0.5, 0.5), (0.5, 0.5), (0.5, -0.5), (-0.5, -0.5))]
PROC_COLS = ["kare_no", "sim_s", "kuyruk_ms", "islem_ms", "aruco_ms", "roi_ms", "yedek_ms", "bulundu", "cozuldu",
             "yontem", "u", "v", "kenar_px", "yayin", "x_kuzey", "y_dogu", "poz_dt_ms"]
EVAL_COLS = ["kare_no", "sim_s", "kadrajda_gercek", "bulundu", "cozuldu", "yayin", "x_kuzey", "y_dogu",
             "hata_n", "hata_e", "hata_yatay", "gercek_poz_tahmin_n", "gercek_poz_tahmin_e", "gercek_poz_hata_yatay",
             "px4_gercek_yatay_fark", "px4_gercek_dikey_fark", "dt_gt_ms", "mesafe_3b"]


def log(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


class QrIsleyici:
    """Bulma + çözme (tek kare). Durum: bir önceki karede bulundu mu (yedek tam kare dedektörü için)."""

    def __init__(self):
        self.aruco = cv2.QRCodeDetectorAruco()
        self.qr = cv2.QRCodeDetector()
        self.prev_found = False

    def isle(self, gray):
        out = {"bulundu": False, "cozuldu": False, "icerik": "", "yontem": "", "aruco_ms": 0.0, "roi_ms": 0.0,
               "yedek_ms": 0.0}
        t0 = time.perf_counter()
        ok, pts = self.aruco.detect(gray)
        out["aruco_ms"] = 1000 * (time.perf_counter() - t0)
        quad, method = (pts.reshape(-1, 4, 2)[0] if ok and pts is not None else None), "aruco"
        if quad is None and self.prev_found:
            t0 = time.perf_counter()
            ok, pts = self.qr.detect(gray)
            out["yedek_ms"] = 1000 * (time.perf_counter() - t0)
            quad, method = (pts.reshape(-1, 4, 2)[0] if ok and pts is not None else None), "yedek_qr"
        self.prev_found = quad is not None
        if quad is None:
            return out
        side = float(np.mean([np.linalg.norm(quad[(i + 1) % 4] - quad[i]) for i in range(4)]))
        x0, y0 = np.maximum((quad.min(axis=0) - side).astype(int), 0)
        x1, y1 = np.minimum((quad.max(axis=0) + side).astype(int), [gray.shape[1], gray.shape[0]])
        roi = gray[y0:y1, x0:x1]
        t0 = time.perf_counter()
        text, rpts, _ = self.qr.detectAndDecode(roi)
        if text:
            method += "+roi_qr"
            quad_dec = np.asarray(rpts, dtype=float).reshape(-1, 4, 2)[0] + [x0, y0]
        else:
            text, rpts, _ = self.aruco.detectAndDecode(roi)
            method += "+roi_aruco" if text else "+cozulemedi"
            quad_dec = np.asarray(rpts, dtype=float).reshape(-1, 4, 2)[0] + [x0, y0] if text and rpts is not None else None
        out["roi_ms"] = 1000 * (time.perf_counter() - t0)
        if quad_dec is not None:                 # çözümün köşeleri daha hassas; merkez/kenar bunlardan
            quad = quad_dec
            side = float(np.mean([np.linalg.norm(quad[(i + 1) % 4] - quad[i]) for i in range(4)]))
        c = quad.mean(axis=0)
        out.update(bulundu=True, cozuldu=text == TEXT, icerik=text, yontem=method, u=float(c[0]), v=float(c[1]),
                   kenar_px=side, quad=quad)
        return out


def estimate_px4(u, v, n, e, d, yaw_deg, pitch_deg, roll_deg):
    R = euler_to_R(math.radians(yaw_deg), math.radians(pitch_deg), math.radians(roll_deg))
    return ray_ground_ned(R, np.array([n, e, d]), u, v, 0.0)


def truth_eval(gt, u=None, v=None):
    """gt = (x, y, z, qw, qx, qy, qz) ENU. (kadrajda, gerçek pozla tahmin (n, e) ya da None, kamera–QR 3B mesafe)."""
    t = np.array(gt[:3])
    R_wb = quat_to_R(*gt[3:])
    in_frame = all(math.isfinite(pu) and 0 <= pu < W and 0 <= pv < H for pu, pv, _ in
                   (project_enu(R_wb, t, c) for c in QR_CORNERS))
    dist = float(np.linalg.norm(t + R_wb @ np.array([0.07, 0.0, -0.1]) - np.array([QR_E, QR_N, QR_Z])))
    est = None
    if u is not None:
        est = ray_ground_ned(ENU_NED @ R_wb @ FLU_FRD, ENU_NED @ t, u, v, -QR_Z)
    return in_frame, est, dist


class Ortalama:
    """Geçiş içi yayın ortalaması; 5 sn (sim) yayın yoksa sıfırlanır."""

    def __init__(self):
        self.vals, self.last = [], -1e9

    def add(self, t, n, e):
        if t - self.last > 5.0:
            self.vals = []
        self.vals.append((n, e))
        self.last = t
        a = np.mean(self.vals, axis=0)
        return float(a[0]), float(a[1]), len(self.vals)


class CsvLog:
    def __init__(self, path, cols):
        self.f = open(path, "w", newline="")
        self.w = csv.DictWriter(self.f, fieldnames=cols, extrasaction="ignore")
        self.w.writeheader()

    def row(self, d):
        self.w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in d.items()})
        self.f.flush()


# ── Çevrimdışı ───────────────────────────────────────────────────────────────────────────────────────
def offline(args):
    isl = QrIsleyici()
    summary = {}
    for gdir in sorted(d for d in os.listdir(args.cevrimdisi) if d.startswith("gecis_")):
        rows = list(csv.DictReader(open(os.path.join(args.cevrimdisi, gdir, "kareler.csv"))))
        plog = CsvLog(os.path.join(args.log, f"{gdir}_islem.csv"), PROC_COLS)
        elog = CsvLog(os.path.join(args.log, f"{gdir}_degerlendirme.csv"), EVAL_COLS)
        avg = Ortalama()
        pubs, procs, inframe, found, dec = [], [], 0, 0, 0
        for k, r in enumerate(rows, 1):
            gray = cv2.imread(os.path.join(args.cevrimdisi, gdir, r["png"]), cv2.IMREAD_GRAYSCALE)
            t0 = time.perf_counter()
            o = isl.isle(gray)
            sim = float(r["img_sim_s"])
            rec = {"kare_no": k, "sim_s": sim, "kuyruk_ms": 0.0, "bulundu": o["bulundu"], "cozuldu": o["cozuldu"],
                   "yontem": o["yontem"], "aruco_ms": o["aruco_ms"], "roi_ms": o["roi_ms"], "yedek_ms": o["yedek_ms"],
                   "yayin": False}
            gt = tuple(float(r[c]) for c in ("gt_x", "gt_y", "gt_z", "gt_qw", "gt_qx", "gt_qy", "gt_qz"))
            if o["bulundu"]:
                found += 1
                dec += o["cozuldu"]
                n, e = estimate_px4(o["u"], o["v"], *(float(r[c]) for c in ("n", "e", "d", "yaw_deg", "pitch_deg", "roll_deg")))
                rec.update(u=o["u"], v=o["v"], kenar_px=o["kenar_px"], yayin=True, x_kuzey=n, y_dogu=e,
                           poz_dt_ms=float(r["dt_odo_ms"]))
                avg.add(sim, n, e)
                pubs.append((n, e))
            rec["islem_ms"] = 1000 * (time.perf_counter() - t0)
            procs.append(rec["islem_ms"])
            plog.row(rec)
            in_frame, est_gt, dist = truth_eval(gt, o.get("u"), o.get("v"))
            inframe += in_frame
            ev = dict(rec, kadrajda_gercek=in_frame, mesafe_3b=dist)
            if o["bulundu"]:
                ev.update(hata_n=rec["x_kuzey"] - QR_N, hata_e=rec["y_dogu"] - QR_E,
                          hata_yatay=math.hypot(rec["x_kuzey"] - QR_N, rec["y_dogu"] - QR_E),
                          gercek_poz_tahmin_n=est_gt[0], gercek_poz_tahmin_e=est_gt[1],
                          gercek_poz_hata_yatay=math.hypot(est_gt[0] - QR_N, est_gt[1] - QR_E))
            elog.row(ev)
        err = np.array([(n - QR_N, e - QR_E) for n, e in pubs])
        h = np.hypot(err[:, 0], err[:, 1]) if len(err) else np.array([])
        summary[gdir] = {"kare": len(rows), "kadrajda": inframe, "bulunan": found, "cozulen": dec,
                         "islem_ms_ort": round(float(np.mean(procs)), 1), "islem_ms_maks": round(float(np.max(procs)), 1),
                         "ort_hata_dN_dE": [round(float(err[:, 0].mean()), 3), round(float(err[:, 1].mean()), 3)] if len(err) else None,
                         "yatay_ort": round(float(h.mean()), 3) if len(h) else None,
                         "yatay_maks": round(float(h.max()), 3) if len(h) else None}
        log(f"{gdir}: {json.dumps(summary[gdir], ensure_ascii=False)}")
    json.dump(summary, open(os.path.join(args.log, "cevrimdisi_ozet.json"), "w"), indent=1, ensure_ascii=False)


# ── Çevrimiçi ────────────────────────────────────────────────────────────────────────────────────────
def online(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from px4_msgs.msg import VehicleOdometry
    from gz.transport13 import Node as GzNode
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("qr_online")
    pub = node.create_publisher(String, "/hedef_konum", 10)
    lock = threading.Lock()
    clock = collections.deque(maxlen=4000)     # (sim, duvar)
    truth = collections.deque(maxlen=1000)     # (sim, x, y, z, qw, qx, qy, qz)
    odo = collections.deque(maxlen=400)        # (varış duvar, n, e, d, yaw, pitch, roll)
    work = queue.Queue(maxsize=2)
    st = {"gelen": 0, "atlanan": 0, "kopru_kayip": 0, "son_sim": None, "kare_no": 0, "bulunan": 0, "cozulen": 0,
          "yayin": 0, "islem": [], "info": None}

    def on_clock(m):
        with lock:
            clock.append((m.sim.sec + m.sim.nsec * 1e-9, time.time()))

    def on_pose(m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        for p in m.pose:
            if p.name == "advanced_plane_0":
                o = p.orientation
                with lock:
                    truth.append((t, p.position.x, p.position.y, p.position.z, o.w, o.x, o.y, o.z))
                return

    def on_odo(m):
        w, x, y, z = m.q
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        with lock:
            odo.append((time.time(), m.position[0], m.position[1], m.position[2], yaw, pitch, roll))

    def on_info(m):
        if st["info"] is None:
            st["info"] = (m.width, m.height, m.k[0], m.k[2], m.k[5])
            log(f"camera_info: {m.width}×{m.height}, fx {m.k[0]:.2f}, cx {m.k[2]:.1f}, cy {m.k[5]:.1f} "
                f"(kullanılan fx {FX}, asal nokta {CX}, {CY} — qr_analyze ile aynı)")

    def on_image(msg):
        arrival = time.time()
        sim = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        st["gelen"] += 1
        if st["son_sim"] is not None and sim - st["son_sim"] > 0.15:
            st["kopru_kayip"] += int(round((sim - st["son_sim"]) / 0.1)) - 1
        st["son_sim"] = sim
        st["kare_no"] += 1
        gray = cv2.cvtColor(np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2GRAY)
        try:
            work.put_nowait((st["kare_no"], sim, arrival, gray))
        except queue.Full:
            st["atlanan"] += 1

    def sim_to_wall(t_sim):
        with lock:
            c = list(clock)
        sims = [x[0] for x in c]
        i = bisect.bisect_left(sims, t_sim)
        if i <= 0 or i >= len(c):
            return math.nan
        (s0, w0), (s1, w1) = c[i - 1], c[i]
        return w0 + (w1 - w0) * (t_sim - s0) / (s1 - s0) if s1 > s0 else w0

    def nearest(dq, key_t):
        with lock:
            items = list(dq)
        return min(items, key=lambda x: abs(x[0] - key_t)) if items and math.isfinite(key_t) else None

    os.makedirs(args.log, exist_ok=True)
    plog = CsvLog(os.path.join(args.log, "islem.csv"), PROC_COLS + ["yayin_gecikme_ms"])
    elog = CsvLog(os.path.join(args.log, "degerlendirme.csv"), EVAL_COLS)
    isl = QrIsleyici()
    avg = Ortalama()

    def worker():
        while not stop["flag"]:
            try:
                k, sim, arrival, gray = work.get(timeout=0.2)
            except queue.Empty:
                continue
            t_start = time.time()
            t0 = time.perf_counter()
            o = isl.isle(gray)
            rec = {"kare_no": k, "sim_s": sim, "kuyruk_ms": 1000 * (t_start - arrival), "bulundu": o["bulundu"],
                   "cozuldu": o["cozuldu"], "yontem": o["yontem"], "aruco_ms": o["aruco_ms"], "roi_ms": o["roi_ms"],
                   "yedek_ms": o["yedek_ms"], "yayin": False}
            if o["bulundu"]:
                st["bulunan"] += 1
                st["cozulen"] += o["cozuldu"]
                t_wall = sim_to_wall(sim)
                od = nearest(odo, t_wall)
                rec.update(u=o["u"], v=o["v"], kenar_px=o["kenar_px"])
                if od is not None and abs(od[0] - t_wall) < 0.05:
                    n, e = estimate_px4(o["u"], o["v"], *od[1:])
                    an, ae, cnt = avg.add(sim, n, e)
                    msg = String()
                    msg.data = json.dumps({
                        "damga_sim": round(sim, 4), "x_kuzey": round(n, 3), "y_dogu": round(e, 3),
                        "cerceve": "uav0 yerel NED (x=Kuzey, y=Doğu)", "icerik": o["icerik"],
                        "kenar_px": round(o["kenar_px"], 2), "modul_px": round(o["kenar_px"] / 21, 2), "kare_no": k,
                        "bulma_no": cnt, "ort_x_kuzey": round(an, 3), "ort_y_dogu": round(ae, 3), "ort_n": cnt,
                        "poz_dt_ms": round(1000 * (od[0] - t_wall), 1), "yontem": o["yontem"]}, ensure_ascii=False)
                    pub.publish(msg)
                    st["yayin"] += 1
                    rec.update(yayin=True, x_kuzey=n, y_dogu=e, poz_dt_ms=1000 * (od[0] - t_wall),
                               yayin_gecikme_ms=1000 * (time.time() - t_wall) if math.isfinite(t_wall) else math.nan)
                    log(f"YAYIN kare {k}: ({n:.2f}, {e:.2f}) içerik='{o['icerik']}' kenar {o['kenar_px']:.1f} px, "
                        f"ort ({an:.2f}, {ae:.2f}) n={cnt}, {o['yontem']}")
                else:
                    log(f"kare {k}: QR bulundu ama eşleşen odometri yok (fark "
                        f"{'?' if od is None else f'{1000 * (od[0] - t_wall):.0f} ms'}) — yayın yok")
            rec["islem_ms"] = 1000 * (time.perf_counter() - t0)
            st["islem"].append(rec["islem_ms"])
            plog.row(rec)
            # Değerlendirme (yalnızca log)
            gt = nearest(truth, sim)
            if gt is not None:
                in_frame, est_gt, dist = truth_eval(gt[1:], o.get("u"), o.get("v"))
                ev = dict(rec, kadrajda_gercek=in_frame, mesafe_3b=dist, dt_gt_ms=1000 * (gt[0] - sim))
                if rec["yayin"]:
                    ev.update(hata_n=rec["x_kuzey"] - QR_N, hata_e=rec["y_dogu"] - QR_E,
                              hata_yatay=math.hypot(rec["x_kuzey"] - QR_N, rec["y_dogu"] - QR_E))
                    od = nearest(odo, sim_to_wall(sim))
                    gt_ned = ENU_NED @ np.array(gt[1:4])
                    ev.update(px4_gercek_yatay_fark=math.hypot(od[1] - gt_ned[0], od[2] - gt_ned[1]),
                              px4_gercek_dikey_fark=od[3] - gt_ned[2])
                if est_gt is not None:
                    ev.update(gercek_poz_tahmin_n=est_gt[0], gercek_poz_tahmin_e=est_gt[1],
                              gercek_poz_hata_yatay=math.hypot(est_gt[0] - QR_N, est_gt[1] - QR_E))
                elog.row(ev)

    gz = GzNode()
    gz.subscribe(Clock, "/world/default/clock", on_clock)
    gz.subscribe(Pose_V, "/world/default/dynamic_pose/info", on_pose)
    node.create_subscription(Image, "/uav0/camera/image", on_image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/uav0/camera/camera_info", on_info, qos_profile_sensor_data)
    node.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", on_odo, qos_profile_sensor_data)
    th = threading.Thread(target=worker, daemon=True)
    th.start()
    log(f"qr_online hazır: /uav0/camera/image → /hedef_konum; log {args.log}")
    last = time.time()
    while not stop["flag"]:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last >= 30:
            last = time.time()
            pr = st["islem"][-300:]
            log(f"durum: gelen {st['gelen']}, atlanan (kuyruk dolu) {st['atlanan']}, köprü kaybı (damga) "
                f"{st['kopru_kayip']}, bulunan {st['bulunan']}, çözülen {st['cozulen']}, yayın {st['yayin']}, "
                f"işlem ort {np.mean(pr) if pr else math.nan:.1f} ms maks {max(pr) if pr else math.nan:.1f} ms")
    th.join(timeout=3)
    pr = st["islem"]
    log(f"ÖZET: gelen {st['gelen']}, atlanan {st['atlanan']}, köprü kaybı {st['kopru_kayip']}, bulunan "
        f"{st['bulunan']}, çözülen {st['cozulen']}, yayın {st['yayin']}, işlem ort "
        f"{np.mean(pr) if pr else math.nan:.1f} ms, p95 {np.percentile(pr, 95) if pr else math.nan:.1f} ms, "
        f"maks {max(pr) if pr else math.nan:.1f} ms")
    os._exit(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", required=True)
    ap.add_argument("--cevrimdisi", help="qr_recorder.py kayıt klasörü (gecis_NN/)")
    args = ap.parse_args()
    os.makedirs(args.log, exist_ok=True)
    offline(args) if args.cevrimdisi else online(args)


if __name__ == "__main__":
    main()
