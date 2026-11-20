#!/usr/bin/env python3
"""
canli_kamera.py — uav0'ın aşağı bakan kamerasını canlı gösterir; ArUco tespitini ve geolocator tahminini çizer.

Yalnızca görsel (hiçbir şey yayınlamaz). Görüntü: /uav0/camera/image_gs (gs_kamera çıkışı = geolocator'ın gördüğü
kare). Tespit burada AYRICA yapılır (bapp MarkerDetector, aynı ayarlar) — çizim için; geolocator'ı etkilemez.
Bilgi satırları: /hedef_konum (geolocator JSON: tahmin, σ95, gözlem sayısı), /fmu/out/vehicle_local_position_v1
(irtifa, hız), /bap/hedef_rapor_role (uav1'e ulaşan LoRa raporu sayısı). --gercek-ne verilirse tahminin gerçek
hedefe yatay hatası da yazılır (yalnız sim değerlendirmesi).

Pencere: OpenCV (WSLg). Tuşlar: q / Esc çıkış, s kareyi PNG kaydet, + / − ölçek.
Kullanım: python3 canli_kamera.py [--olcek 0.6] [--gercek-ne -60 -60]
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.expanduser(os.environ.get("BAPP", "~/bapp")))
from bap_geo.aruco import MarkerDetector, draw_markers  # noqa: E402


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleLocalPosition
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, UInt8MultiArray

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uav", type=int, default=0, choices=(0, 1), help="0: keşif (aşağı), 1: kamikaze (ileri 15° aşağı)")
    ap.add_argument("--goruntu", default=None, help="varsayılan /uav<N>/camera/image_gs")
    ap.add_argument("--olcek", type=float, default=0.6)
    ap.add_argument("--marker-id", type=int, default=0)
    ap.add_argument("--gercek-ne", type=float, nargs=2, default=None, metavar=("N", "E"),
                    help="gerçek hedef (geolocator NED'i ≈ uav0 yerel NED) — yalnız sim")
    ap.add_argument("--kayit-dizini", default="/tmp/canli_kamera")
    a = ap.parse_args()
    a.goruntu = a.goruntu or f"/uav{a.uav}/camera/image_gs"
    fmu = "/fmu" if a.uav == 0 else "/px4_1/fmu"
    json_konu = "/hedef_konum" if a.uav == 0 else "/bap/uav1/okuma"

    det = MarkerDetector(target_ids=[a.marker_id])
    L = threading.Lock()
    st = {"img": None, "stamp": 0.0, "json": None, "pos": None, "rapor": 0, "yeni": False}

    rclpy.init()
    n = rclpy.create_node(f"canli_kamera_uav{a.uav}")

    def on_img(m):
        h, w = m.height, m.width
        ch = 3 if m.encoding in ("bgr8", "rgb8") else 1
        img = np.frombuffer(m.data, np.uint8).reshape(h, m.step)[:, : w * ch].reshape(h, w, ch)
        if m.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        with L:
            st["img"], st["stamp"], st["yeni"] = img.copy(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, True

    def on_json(m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        with L:
            st["json"] = d

    def on_pos(m):
        with L:
            st["pos"] = m

    def on_rapor(_m):
        with L:
            st["rapor"] += 1

    n.create_subscription(Image, a.goruntu, on_img, qos_profile_sensor_data)
    n.create_subscription(String, json_konu, on_json, 10)
    n.create_subscription(VehicleLocalPosition, f"{fmu}/out/vehicle_local_position_v1", on_pos, qos_profile_sensor_data)
    n.create_subscription(UInt8MultiArray, "/bap/hedef_rapor_role", on_rapor, 10)
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    win = "uav0 kesif kamera (6 mm, asagi) - ArUco" if a.uav == 0 else "uav1 kamikaze kamera (6 mm, ileri 15 asagi) - ArUco"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    olcek = a.olcek
    n_kare = n_tespit = 0
    son_tespit_t = None
    t_fps, fps = time.monotonic(), 0.0
    while rclpy.ok():   # rclpy SIGINT/SIGTERM'i yakalayıp kapatır → döngü biter
        with L:
            img, yeni, stamp = st["img"], st["yeni"], st["stamp"]
            js, pos, n_rapor = st["json"], st["pos"], st["rapor"]
            st["yeni"] = False
        if img is None:
            blank = np.zeros((300, 700, 3), np.uint8)
            cv2.putText(blank, "kamera bekleniyor: " + a.goruntu, (20, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(win, blank)
            if cv2.waitKey(200) & 0xFF in (ord("q"), 27):
                break
            continue
        if not yeni:
            k = cv2.waitKey(10) & 0xFF
        else:
            n_kare += 1
            vis = img.copy()
            found = det.detect(img)
            if found:
                n_tespit += 1
                son_tespit_t = time.monotonic()
                draw_markers(vis, found)
                for mid, c in found.items():
                    side = float(np.mean(np.linalg.norm(np.roll(c, -1, 0) - c, axis=1)))
                    ctr = c.mean(0)
                    cv2.circle(vis, (int(ctr[0]), int(ctr[1])), 60, (0, 255, 255), 3)
                    cv2.putText(vis, f"ID {mid}  {side:.0f} px", (int(ctr[0]) + 70, int(ctr[1])),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            dt = time.monotonic() - t_fps
            if dt > 1.0:
                fps, t_fps = n_kare / dt, time.monotonic()
                n_kare = 0
            satirlar = [f"sim t {stamp:7.1f} s   ekran {fps:4.1f} Hz   tespitli kare {n_tespit}"]
            if pos is not None:
                satirlar.append(f"uav{a.uav} irtifa {-pos.z:5.1f} m   hiz {math.hypot(pos.vx, pos.vy):4.1f} m/s   "
                                f"N {pos.x:7.1f}  E {pos.y:7.1f}")
            if js is not None:
                s = (f"TAHMIN N {js['x_kuzey']:7.2f}  E {js['y_dogu']:7.2f}   s95 {js.get('s95', float('nan')):.2f} m   "
                     f"n {js.get('n_obs', '?')}")
                if a.gercek_ne:
                    s += f"   HATA {math.hypot(js['x_kuzey'] - a.gercek_ne[0], js['y_dogu'] - a.gercek_ne[1]):.2f} m"
                satirlar.append(s)
            satirlar.append(f"uav1'e ulasan LoRa raporu: {n_rapor}")
            if son_tespit_t and time.monotonic() - son_tespit_t < 1.0:
                satirlar.append("ARUCO GORULUYOR")
            y = 50
            for i, t in enumerate(satirlar):
                renk = (0, 255, 0) if t.startswith("ARUCO") else (255, 255, 255)
                cv2.putText(vis, t, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 6)
                cv2.putText(vis, t, (20, y), cv2.FONT_HERSHEY_SIMPLEX, 1.2, renk, 2)
                y += 48
            cv2.imshow(win, cv2.resize(vis, None, fx=olcek, fy=olcek, interpolation=cv2.INTER_AREA))
            k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("s") and img is not None:
            os.makedirs(a.kayit_dizini, exist_ok=True)
            yol = os.path.join(a.kayit_dizini, f"kare_{stamp:08.1f}.png")
            cv2.imwrite(yol, img)
            print("kaydedildi:", yol, flush=True)
        if k in (ord("+"), ord("=")):
            olcek = min(1.0, olcek + 0.1)
        if k == ord("-"):
            olcek = max(0.3, olcek - 0.1)
    cv2.destroyAllWindows()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
