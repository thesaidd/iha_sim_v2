#!/usr/bin/env python3
"""
qr_live.py — uav0 aşağı bakan kamerasını canlı OpenCV penceresinde gösterir, QR'ı bulup çerçeveler.
Yalnızca GÖRSELLEŞTİRME (komut/abort yok). camera_bridge.yaml (/uav0/camera/image, best effort) çalışmalı.

Yarı çözünürlük (728×544), ~10 Hz. QR bulunursa dörtgen + metin ("HEDEF-01" yeşil / "bulundu, cozulemedi" sarı),
yoksa "QR yok". Algılama qr_online ile aynı ucuz yol: QRCodeDetectorAruco.detect → ROI'de QRCodeDetector.
Çıkış: 'q' tuşu, SIGTERM, ya da --stop-file oluşunca. (cv2 Hershey ASCII; Türkçe karakter yok.)

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 qr_live.py [--stop-file /tmp/qr_live.stop]
"""
import argparse
import os
import signal
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

W, H = 728, 544
PENCERE = "uav0 canli kamera (yari coz, QR)"


class CanliPencere(Node):
    def __init__(self, stop_file):
        super().__init__("qr_live")
        self.stop_file = stop_file
        self.aruco = cv2.QRCodeDetectorAruco()
        self.qr = cv2.QRCodeDetector()
        self.son_goster = 0.0
        self.kare = 0
        self.son_bulma = 0
        self.create_subscription(Image, "/uav0/camera/image", self.on_img, qos_profile_sensor_data)
        cv2.namedWindow(PENCERE, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(PENCERE, W, H)

    def bul_ciz(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        ok, pts = self.aruco.detect(gray)
        etiket, renk = "QR yok", (180, 180, 180)
        if ok and pts is not None:
            p = pts.reshape(-1, 2).astype(np.float32)
            x0, y0 = p[:, 0].min(), p[:, 1].min()
            x1, y1 = p[:, 0].max(), p[:, 1].max()
            pad = 0.6 * max(x1 - x0, y1 - y0)
            rx0, ry0 = max(int(x0 - pad), 0), max(int(y0 - pad), 0)
            rx1, ry1 = min(int(x1 + pad), bgr.shape[1]), min(int(y1 + pad), bgr.shape[0])
            icerik = ""
            if rx1 > rx0 and ry1 > ry0:
                icerik, _, _ = self.qr.detectAndDecode(gray[ry0:ry1, rx0:rx1])
            cozme = icerik == "HEDEF-01"
            renk = (0, 220, 0) if cozme else (0, 215, 255)
            etiket = "HEDEF-01" if cozme else "bulundu, cozulemedi"
            self.son_bulma = self.kare
            cv2.polylines(bgr, [p.astype(np.int32)], True, renk, 2, cv2.LINE_AA)
            tx, ty = int(x0), int(y0) - 6 if y0 > 16 else int(y1) + 16
            cv2.putText(bgr, etiket, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 2, cv2.LINE_AA)
        else:
            cv2.putText(bgr, etiket, (W - 150, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, renk, 2, cv2.LINE_AA)
        return bgr

    def on_img(self, msg):
        self.kare += 1
        now = time.time()
        if now - self.son_goster < 0.1:          # ~10 Hz görüntüleme
            return
        self.son_goster = now
        try:
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        except ValueError:
            return
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if bgr.shape[1] != W:
            bgr = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
        bgr = self.bul_ciz(bgr)
        bar = bgr.copy()
        cv2.rectangle(bar, (0, 0), (W, 26), (0, 0, 0), -1)
        cv2.addWeighted(bar, 0.55, bgr, 0.45, 0, bgr)
        cv2.putText(bgr, f"CANLI uav0  kare {self.kare}  {time.strftime('%H:%M:%S')}",
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(PENCERE, bgr)
        if (cv2.waitKey(1) & 0xFF) == ord("q") or (self.stop_file and os.path.exists(self.stop_file)):
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stop-file", default="/tmp/qr_live.stop")
    args = ap.parse_args()
    if os.path.exists(args.stop_file):
        os.remove(args.stop_file)
    rclpy.init()
    node = CanliPencere(args.stop_file)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    print(f"{time.strftime('%H:%M:%S')}  qr_live hazır: /uav0/camera/image → pencere; stop-file {args.stop_file}",
          flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        print(f"{time.strftime('%H:%M:%S')}  qr_live kapandı (kare {node.kare}, son bulma {node.son_bulma})",
              flush=True)
        os._exit(0)


if __name__ == "__main__":
    main()
