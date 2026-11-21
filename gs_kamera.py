#!/usr/bin/env python3
"""
gs_kamera.py — Gazebo kamera karesini renkli Raspberry Pi Global Shutter sensörüne (IMX296LQR-C) benzetir.

Gazebo (ogre2) her pikselde tam RGB üretir; gerçek renkli sensörde her piksel yalnız bir rengi ölçer (RGGB
Bayer) ve ISP demozaikle tam RGB'yi kestirir. Demozaik kenarlarda renk/ton hatası yapar; bapp sentetik
deneyinde (Faz 0 adım 3) renkli sensörde ArUco köşe σ'sı monoya göre ~2–4 kat çıktı (noise.SYNTH_CORNER_TABLE).
Bu düğüm o etkiyi simülasyona taşır:

  1. RGB → RGGB mozaik (her piksel tek kanal; satır çift/sütun çift = R, ... ),
  2. isteğe bağlı ek gürültü: foton (shot) gürültüsü ~ √sinyal (--shot, tam ölçekte DN σ) + okuma gürültüsü
     (--read, DN σ) — gz <noise> (Gauss, kanal başına) zaten kare üstünde; varsayılan ek gürültü kapalı,
  3. 8 bit nicemleme, OpenCV bilineer demozaik (RGGB = COLOR_BayerBG2BGR; bapp synth ile aynı) → bgr8.

Zaman damgası (header.stamp) DEĞİŞMEZ: kare anı = Gazebo render anı (sim saati). camera_info aynen
yeniden yayınlanır (CameraInfo konusu ayrı tutulur ki tüketici hangi akışa baktığını bilsin).

Giriş : /uav0/camera/image (rgb8, best-effort), /uav0/camera/camera_info
Çıkış : /uav0/camera/image_gs (bgr8, best-effort), /uav0/camera/camera_info_gs

Kullanım:
  source /opt/ros/jazzy/setup.bash && python3 gs_kamera.py [--shot 0] [--read 0] [--seed N]
"""
import argparse
import time

import cv2
import numpy as np


def bayer_rggb(rgb: np.ndarray) -> np.ndarray:
    """RGB (H, W, 3) uint8 → RGGB Bayer ham kare (H, W) uint8."""
    raw = np.empty(rgb.shape[:2], np.uint8)
    raw[0::2, 0::2] = rgb[0::2, 0::2, 0]   # R
    raw[0::2, 1::2] = rgb[0::2, 1::2, 1]   # G (R satırı)
    raw[1::2, 0::2] = rgb[1::2, 0::2, 1]   # G (B satırı)
    raw[1::2, 1::2] = rgb[1::2, 1::2, 2]   # B
    return raw


def gurultu(raw: np.ndarray, shot: float, read: float, rng: np.random.Generator) -> np.ndarray:
    """Ham kareye shot (σ = shot·√(x/255)) + okuma (σ = read) gürültüsü, DN; 0–255'e kırpar."""
    if shot <= 0 and read <= 0:
        return raw
    x = raw.astype(np.float32)
    sigma = np.sqrt((shot ** 2) * (x / 255.0) + read ** 2)
    x += rng.standard_normal(x.shape, dtype=np.float32) * sigma
    return np.clip(np.rint(x), 0, 255).astype(np.uint8)


def demozaik(raw: np.ndarray) -> np.ndarray:
    """RGGB ham → BGR (OpenCV bilineer)."""
    # OpenCV Bayer kodları desenin (1,1)–(1,2) piksellerine göre adlandırılır: RGGB deseni = COLOR_BayerBG2BGR.
    return cv2.cvtColor(raw, cv2.COLOR_BayerBG2BGR)


def isle(rgb: np.ndarray, shot=0.0, read=0.0, rng=None) -> np.ndarray:
    rng = rng or np.random.default_rng()
    return demozaik(gurultu(bayer_rggb(rgb), shot, read, rng))


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--giris", default="/uav0/camera/image")
    ap.add_argument("--info", default="/uav0/camera/camera_info")
    ap.add_argument("--cikis", default="/uav0/camera/image_gs")
    ap.add_argument("--info-cikis", default="/uav0/camera/camera_info_gs")
    ap.add_argument("--shot", type=float, default=0.0, help="tam ölçekte shot gürültüsü σ [DN]")
    ap.add_argument("--read", type=float, default=0.0, help="okuma gürültüsü σ [DN]")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--ad", default="gs_kamera", help="ROS düğüm adı (uav1 için gs_kamera_uav1)")
    a = ap.parse_args()

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=2)

    class GsKamera(Node):
        def __init__(self):
            super().__init__(a.ad)
            self.rng = np.random.default_rng(a.seed)
            self.pub = self.create_publisher(Image, a.cikis, qos)
            self.pub_info = self.create_publisher(CameraInfo, a.info_cikis, qos)
            self.create_subscription(Image, a.giris, self.on_img, qos)
            self.create_subscription(CameraInfo, a.info, self.pub_info.publish, qos)
            self.n = 0
            self.t_ms = []
            self.create_timer(10.0, self.durum)

        def on_img(self, m):
            t0 = time.perf_counter()
            if m.encoding != "rgb8":
                self.get_logger().error(f"beklenmeyen kodlama {m.encoding} (rgb8 bekleniyor)")
                return
            h, w = m.height, m.width
            rgb = np.frombuffer(m.data, np.uint8).reshape(h, m.step)[:, : 3 * w].reshape(h, w, 3)
            bgr = isle(rgb, a.shot, a.read, self.rng)
            out = Image(header=m.header, height=h, width=w, encoding="bgr8", is_bigendian=0, step=3 * w,
                        data=bgr.tobytes())
            self.pub.publish(out)
            self.n += 1
            self.t_ms.append((time.perf_counter() - t0) * 1e3)

        def durum(self):
            if self.t_ms:
                t = np.array(self.t_ms)
                self.get_logger().info(f"{self.n} kare; işlem {np.median(t):.1f} ms medyan, {t.max():.1f} ms en çok")
                self.t_ms.clear()

    rclpy.init()
    node = GsKamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
