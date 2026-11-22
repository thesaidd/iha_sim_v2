#!/usr/bin/env python3
"""
qr_recorder.py — uav0 kamerasının karelerini QR geçiş penceresinde kayıpsız PNG olarak kaydeder,
her kare için konum/hız/yönelim ve zaman damgası eşleşmesini CSV'ye yazar.

Pencere: araç OFFBOARD'da ve hedefe göre ilerleme yönündeki mesafe s = (P − T)·v̂ ∈ [−250, +100] m
(hedefe 250 m kala başlar, 100 m geçince durur). Her yeni pencere yeni bir geçiş klasörü açar
(<çıktı>/gecis_NN/). Hold çemberindeyken (OFFBOARD değil) kayıt yapılmaz.

Zaman damgaları (2026-10-30 doğrulandı):
  - Kare: header.stamp = Gazebo sim zamanı.
  - Gazebo gerçek pozu (/world/default/dynamic_pose/info, ~50 Hz): sim zamanı → kare ile doğrudan eşleşir.
  - PX4 vehicle_odometry: uXRCE-DDS istemcisi PX4 zamanını Agent'ın sistem saatine çevirir; sim zamanı
    değildir ve RTF < 1'de saat eşitlemesi kayıyor (timestamp_sample duvar saatinin 1–2 sn gerisinde,
    2026-10-30 geçiş 1). Bu yüzden odometri ROS'a VARIŞ zamanıyla (duvar saati) eşleştirilir: karenin sim
    damgası gz /world/default/clock örneklerinden (sim → duvar) duvar saatine çevrilir, en yakın varış
    seçilir. CSV: odo_ts_s (PX4 damgası, bilgi), odo_varis_s, dt_odo_ms (varış − kare duvar saati).
Kayıt: RGB → BGR, PNG (kayıpsız, sıkıştırma seviyesi --png-level). Yazma ayrı iş parçacığında;
kuyruk doluysa kare düşer ve sayılır. Toplam disk > --max-gb olursa kayıt kalıcı olarak durur.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 qr_recorder.py --out <klasör>       (köprü: camera_bridge.yaml çalışıyor olmalı)
SIGTERM/Ctrl+C: kuyruk boşaltılır, özet yazılır.
"""

import argparse
import bisect
import collections
import csv
import math
import os
import queue
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from px4_msgs.msg import VehicleControlMode, VehicleOdometry
from gz.transport13 import Node as GzNode
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from arac_modelleri import gz_ad

COLUMNS = ["gecis", "kare", "png", "img_sim_s", "img_duvar_s", "varis_duvar_s",
           "odo_ts_s", "odo_varis_s", "dt_odo_ms", "n", "e", "d", "vn", "ve", "vd", "yaw_deg", "pitch_deg", "roll_deg",
           "gt_sim_s", "dt_gt_ms", "gt_x", "gt_y", "gt_z", "gt_qw", "gt_qx", "gt_qy", "gt_qz",
           "s_m", "yatay_hedefe_m"]


def euler_deg(q):
    """PX4 q (w, x, y, z; FRD → NED) → (yaw, pitch, roll) [°]."""
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def log(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


class Recorder:
    def __init__(self, args):
        self.a = args
        self.lock = threading.Lock()
        self.clock = collections.deque(maxlen=4000)      # (sim, duvar)
        self.truth = collections.deque(maxlen=1000)      # (sim, x, y, z, qw, qx, qy, qz)
        self.odo = collections.deque(maxlen=400)         # (varış [duvar s], ts_sample [s], msg alanları)
        self.offboard = False
        self.recording = False
        self.pass_id = len([d for d in os.listdir(args.out) if d.startswith("gecis_")])   # yeniden başlatmada devam
        self.pass_dir = None
        self.frame_idx = 0
        self.bytes = 0
        self.disk_full = False
        self.drops = 0
        self.frames_total = 0
        self.window_log = []                              # (geçiş, başlangıç duvar, bitiş duvar, kare, düşen)
        self.q = queue.Queue(maxsize=args.queue)
        self.csv_file = None
        self.csv = None
        self.writer = threading.Thread(target=self._write_loop, daemon=True)
        self.writer.start()

    # --- gz ------------------------------------------------------------------------------------
    def on_clock(self, m):
        with self.lock:
            self.clock.append((m.sim.sec + m.sim.nsec * 1e-9, time.time()))

    def on_pose(self, m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        for p in m.pose:
            if p.name == gz_ad(0):
                o = p.orientation
                with self.lock:
                    self.truth.append((t, p.position.x, p.position.y, p.position.z, o.w, o.x, o.y, o.z))
                return

    def sim_to_wall(self, t_sim):
        with self.lock:
            c = list(self.clock)
        if len(c) < 2:
            return math.nan
        sims = [x[0] for x in c]
        i = bisect.bisect_left(sims, t_sim)
        if i <= 0 or i >= len(c):
            return math.nan
        (s0, w0), (s1, w1) = c[i - 1], c[i]
        return w0 + (w1 - w0) * (t_sim - s0) / (s1 - s0) if s1 > s0 else w0

    # --- ROS -----------------------------------------------------------------------------------
    def on_mode(self, m):
        self.offboard = bool(m.flag_control_offboard_enabled)

    def on_odo(self, m):
        n, e, d = m.position
        vn, ve, vd = m.velocity
        with self.lock:
            self.odo.append((time.time(), m.timestamp_sample * 1e-6, (n, e, d, vn, ve, vd, list(m.q))))
        self._update_window(n, e, vn, ve)

    def _update_window(self, n, e, vn, ve):
        dn, de = n - self.a.target_n, e - self.a.target_e
        sp = math.hypot(vn, ve)
        s = (dn * vn + de * ve) / sp if sp > 1e-3 else math.nan
        inside = (self.offboard and sp > 5.0 and math.isfinite(s)
                  and -self.a.start <= s <= self.a.stop and not self.disk_full)
        if inside and not self.recording:
            self.pass_id += 1
            self.frame_idx = 0
            self.pass_dir = os.path.join(self.a.out, f"gecis_{self.pass_id:02d}")
            os.makedirs(self.pass_dir, exist_ok=True)
            self.q.put(("csv_open", os.path.join(self.pass_dir, "kareler.csv")))
            self.win_start, self.win_drops0, self.win_frames0 = time.time(), self.drops, self.frames_total
            self.recording = True
            log(f"KAYIT BAŞLADI: geçiş {self.pass_id}, s={s:+.0f} m, hedefe yatay {math.hypot(dn, de):.0f} m")
        elif not inside and self.recording:
            self.recording = False
            nf, nd = self.frames_total - self.win_frames0, self.drops - self.win_drops0
            self.window_log.append((self.pass_id, self.win_start, time.time(), nf, nd))
            self.q.put(("csv_close", None))
            log(f"KAYIT BİTTİ: geçiş {self.pass_id}, {nf} kare, {nd} düşen, s={s:+.0f} m, "
                f"{time.time() - self.win_start:.1f} sn, disk toplam {self.bytes / 2**20:.0f} MB")

    def on_image(self, msg):
        if not self.recording:
            return
        arrival = time.time()
        t_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        self.frame_idx += 1
        item = ("frame", (self.pass_id, self.frame_idx, self.pass_dir, t_img, arrival, rgb.copy()))
        try:
            self.q.put_nowait(item)
            self.frames_total += 1
        except queue.Full:
            self.drops += 1

    # --- yazıcı --------------------------------------------------------------------------------
    def _nearest_truth(self, t):
        with self.lock:
            tr = list(self.truth)
        if not tr:
            return None
        ts = [x[0] for x in tr]
        i = bisect.bisect_left(ts, t)
        cands = [tr[j] for j in (i - 1, i) if 0 <= j < len(tr)]
        return min(cands, key=lambda x: abs(x[0] - t))

    def _nearest_odo(self, t_wall):
        with self.lock:
            od = list(self.odo)
        if not od or not math.isfinite(t_wall):
            return None
        return min(od, key=lambda x: abs(x[0] - t_wall))

    def _write_loop(self):
        while True:
            kind, payload = self.q.get()
            if kind == "stop":
                break
            if kind == "csv_open":
                self.csv_file = open(payload, "w", newline="")
                self.csv = csv.writer(self.csv_file)
                self.csv.writerow(COLUMNS)
                continue
            if kind == "csv_close":
                if self.csv_file:
                    self.csv_file.close()
                    self.csv_file = self.csv = None
                continue
            pid, idx, pdir, t_img, arrival, rgb = payload
            if self.disk_full:
                continue
            png = os.path.join(pdir, f"kare_{idx:04d}.png")
            cv2.imwrite(png, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, self.a.png_level])
            self.bytes += os.path.getsize(png)
            if self.bytes > self.a.max_gb * 2**30:
                self.disk_full = True
                log(f"DİSK SINIRI: {self.bytes / 2**30:.2f} GB > {self.a.max_gb} GB — kayıt kalıcı olarak durdu")
            t_wall = self.sim_to_wall(t_img)
            od = self._nearest_odo(t_wall)
            gt = self._nearest_truth(t_img)
            row = [pid, idx, os.path.basename(png), f"{t_img:.4f}", f"{t_wall:.4f}", f"{arrival:.4f}"]
            if od:
                (arr, ts, (n, e, d, vn, ve, vd, q)) = od
                yaw, pitch, roll = euler_deg(q)
                row += [f"{ts:.4f}", f"{arr:.4f}", f"{1000 * (arr - t_wall):.1f}", f"{n:.3f}", f"{e:.3f}", f"{d:.3f}",
                        f"{vn:.3f}", f"{ve:.3f}", f"{vd:.3f}", f"{yaw:.3f}", f"{pitch:.3f}", f"{roll:.3f}"]
                sp = math.hypot(vn, ve)
                dn, de = n - self.a.target_n, e - self.a.target_e
                s_along = (dn * vn + de * ve) / sp if sp > 1e-3 else math.nan
                horiz = math.hypot(dn, de)
            else:
                row += [""] * 12
                s_along = horiz = math.nan
            if gt:
                row += [f"{gt[0]:.4f}", f"{1000 * (gt[0] - t_img):.1f}"] + [f"{v:.5f}" for v in gt[1:]]
            else:
                row += [""] * 9
            row += [f"{s_along:.2f}", f"{horiz:.2f}"]
            if self.csv:
                self.csv.writerow(row)
                self.csv_file.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--target-n", type=float, default=-60.0)
    ap.add_argument("--target-e", type=float, default=-60.0)
    ap.add_argument("--start", type=float, default=250.0, help="Hedefe bu kadar kala kayıt başlar [m]")
    ap.add_argument("--stop", type=float, default=100.0, help="Hedefi bu kadar geçince kayıt durur [m]")
    ap.add_argument("--max-gb", type=float, default=3.0)
    ap.add_argument("--png-level", type=int, default=1)
    ap.add_argument("--queue", type=int, default=120)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    rec = Recorder(args)
    gz = GzNode()
    gz.subscribe(Clock, "/world/default/clock", rec.on_clock)
    gz.subscribe(Pose_V, "/world/default/dynamic_pose/info", rec.on_pose)
    node = rclpy.create_node("qr_recorder")
    node.create_subscription(Image, "/uav0/camera/image", rec.on_image, qos_profile_sensor_data)
    node.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", rec.on_odo, qos_profile_sensor_data)
    node.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", rec.on_mode, qos_profile_sensor_data)
    log(f"kaydedici hazır: {args.out}; pencere s ∈ [−{args.start:.0f}, +{args.stop:.0f}] m, hedef "
        f"({args.target_n:.0f}, {args.target_e:.0f}), disk sınırı {args.max_gb} GB, PNG seviye {args.png_level}")
    last = time.time()
    while not stop["flag"]:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last >= 30:
            last = time.time()
            log(f"durum: kayıt={'AÇIK' if rec.recording else 'kapalı'} offboard={rec.offboard} "
                f"kare={rec.frames_total} düşen={rec.drops} kuyruk={rec.q.qsize()} disk={rec.bytes / 2**20:.0f} MB")
    log("durduruluyor — kuyruk boşaltılıyor")
    if rec.recording:
        rec.q.put(("csv_close", None))
    rec.q.put(("stop", None))
    rec.writer.join(timeout=60)
    for pid, t0, t1, nf, nd in rec.window_log:
        log(f"ÖZET geçiş {pid}: {time.strftime('%H:%M:%S', time.localtime(t0))}–"
            f"{time.strftime('%H:%M:%S', time.localtime(t1))}, {nf} kare, {nd} düşen")
    log(f"ÖZET toplam: {rec.frames_total} kare, {rec.drops} düşen, disk {rec.bytes / 2**20:.0f} MB")
    os._exit(0)


if __name__ == "__main__":
    main()
