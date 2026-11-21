#!/usr/bin/env python3
"""
kare_kaydedici.py — uav0 kamera karelerini seyrek kaydeder (görüntü birleştirme / mozaik ve görsel kontrol için).

Yalnız dinler, hiçbir şey yayınlamaz. Varsayılan giriş /uav0/camera/image_gs (geolocator'ın gördüğü kare, bgr8).
Her --aralik sn'de (sim saati, header.stamp) bir kare JPEG olarak <dizin>/k_<damga_ms>.jpg'ye yazılır; damga
kareler.csv t_cam_s ve gercek_poz.csv t_sim ile aynı saattir (UXRCE_DDS_SYNCT=0, ARUCO=1). İndeks: <dizin>/kareler.csv.
10 Hz × 1456×1088 tam kayıt ~4 MB/sn olurdu; 1 sn aralık hat boyunca ~%78 ön örtüşme verir (100 m, 14 m/s).

Kullanım: python3 kare_kaydedici.py --dizin /tmp/kareler [--aralik 1.0] [--min-irtifa 0]
"""
import argparse
import os
import time

import cv2
import numpy as np


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--goruntu", default="/uav0/camera/image_gs")
    ap.add_argument("--dizin", default="/tmp/kareler")
    ap.add_argument("--aralik", type=float, default=1.0, help="kayıt aralığı [s, sim saati]")
    ap.add_argument("--kalite", type=int, default=90)
    a = ap.parse_args()
    os.makedirs(a.dizin, exist_ok=True)
    idx = open(os.path.join(a.dizin, "kareler.csv"), "a")
    if idx.tell() == 0:
        idx.write("t_sim,dosya,t_duvar\n")
    rclpy.init()
    n = rclpy.create_node("kare_kaydedici")
    son = [-1e9]
    say = [0]

    def on_img(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - son[0] < a.aralik - 1e-6:
            return
        son[0] = t
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        if m.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ad = f"k_{int(round(t * 1000)):09d}.jpg"
        cv2.imwrite(os.path.join(a.dizin, ad), img, [cv2.IMWRITE_JPEG_QUALITY, a.kalite])
        idx.write(f"{t:.3f},{ad},{time.time():.3f}\n")
        idx.flush()
        say[0] += 1
        if say[0] % 60 == 1:
            n.get_logger().info(f"{say[0]} kare ({ad})")

    n.create_subscription(Image, a.goruntu, on_img, qos_profile_sensor_data)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        idx.close()


if __name__ == "__main__":
    main()
