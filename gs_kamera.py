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

Gerçekçilik (2026-11-24, varsayılan KAPALI; launch_all.sh GERCEKCI=1 → --gercekci): gz ogre2 lens distorsiyonu ve optik
bulanıklık uygulamıyor, kamera montajı kusursuz; gerçek GS + 6 mm CS lensinde üçü de var. Mozaikten ÖNCE:
  0a. geometri (tek remap): Brown–Conrady distorsiyonu (--distorsiyon k1 k2 p1 p2 k3, K --kalib JSON'undan) ve
      bilinmeyen montaj (boresight) kayması (--boresight-deg rx ry rz, OpenCV kamera eksenlerinde: rx > 0 görüntüyü
      aşağı, ry > 0 sola kaydırır, rz optik eksen etrafında). Çıkış pikseli p_d → ışın K⁻¹·undistort(p_d) → R_err → gz
      ideal (iğne deliği) karesinde K ile izdüşüm. Fıçı distorsiyonda köşelerin bir kısmı gz kadrajının dışına düşer →
      siyah (gerçek kamerada görünen alan; simde kapsama biraz temkinli).
  0b. optik bulanıklık: Gauss σ (--bulaniklik, px).
Bu kareyi işleyen bapp'e distorsiyonlu kalibrasyon (kalibrasyon/w_nadir_10_gercekci.json) verilir; boresight bapp'e
VERİLMEZ (gerçek uçuştaki ölçülmemiş montaj hatası). --gercekci hazır değerleri VARSAYIMDIR (GS kamerada ölçülecek).

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


# --gercekci hazır değerleri (VARSAYIM; GS + 6 mm CS lens kalibrasyonu ve bias ölçümüyle değiştirilecek). Distorsiyon:
# fıçı, köşede (r ≈ 0,52) ≈ −%3,5 radyal (~30 px) — CCTV 6 mm M12/CS lenslerinde tipik −%2…−5. Boresight 0,5°/0,5°/0,3°
# (toplam eğim ~0,7° → 100 m'de ~1,2 m yer kayması). Bulanıklık + gürültü: statik gz ölçümünde (100 m, ~34 px kenar,
# test/aruco_tulpar_2026-11-24/gercekci_olcum.json) köşe titreşimi σ 0,003 → 0,22 px (laptop 0,30–0,35 px; daha fazla
# gürültüde tespit oranı düşüyor: bu ayarda 86/96), içe bias 0,41 → 0,55 px/köşe.
GERCEKCI_DISTORSIYON = (-0.15, 0.06, 0.0, 0.0, 0.0)
GERCEKCI_BORESIGHT_DEG = (0.5, 0.5, 0.3)
GERCEKCI_BULANIKLIK = 1.0
GERCEKCI_SHOT = 8.0
GERCEKCI_READ = 3.0


def rot_xyz_deg(rx, ry, rz) -> np.ndarray:
    """Küçük açılı dönme (rodrigues vektörü, derece) — kamera eksenlerinde."""
    R, _ = cv2.Rodrigues(np.radians([rx, ry, rz]).astype(np.float64))
    return R


class Geometri:
    """Gz ideal karesinden distorsiyonlu + boresight kaymalı kareye remap haritası (bir kez kurulur)."""

    def __init__(self, K, D, w, h, boresight_deg=(0.0, 0.0, 0.0)):
        self.K = np.asarray(K, np.float64)
        self.D = np.asarray(D if D is not None else (0, 0, 0, 0, 0), np.float64)
        self.R = rot_xyz_deg(*boresight_deg)
        uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        pts = np.stack([uu.ravel(), vv.ravel()], 1).reshape(-1, 1, 2)
        crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 1e-9)
        if hasattr(cv2, "undistortPointsIter"):          # OpenCV 4.x
            n = cv2.undistortPointsIter(pts, self.K, self.D, None, None, crit)
        else:                                            # OpenCV 5: criteria undistortPoints'in son argümanı
            n = cv2.undistortPoints(pts, self.K, self.D, None, None, None, crit)
        n = n.reshape(-1, 2)
        ray = np.concatenate([n, np.ones((n.shape[0], 1))], 1) @ self.R.T      # gerçek kamera → nominal (gz) kamera
        uv = ray[:, :2] / ray[:, 2:3] * np.array([self.K[0, 0], self.K[1, 1]]) + self.K[:2, 2]
        self.mx = uv[:, 0].reshape(h, w).astype(np.float32)
        self.my = uv[:, 1].reshape(h, w).astype(np.float32)
        disari = (self.mx < 0) | (self.mx > w - 1) | (self.my < 0) | (self.my > h - 1)
        self.bos_oran = float(disari.mean())

    def uygula(self, rgb: np.ndarray) -> np.ndarray:
        return cv2.remap(rgb, self.mx, self.my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def isle(rgb: np.ndarray, shot=0.0, read=0.0, rng=None, geo: "Geometri | None" = None, bulaniklik=0.0) -> np.ndarray:
    rng = rng or np.random.default_rng()
    if geo is not None:
        rgb = geo.uygula(rgb)
    if bulaniklik > 0:
        rgb = cv2.GaussianBlur(rgb, (0, 0), bulaniklik)
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
    ap.add_argument("--gercekci", action="store_true",
                    help="hazır gerçekçilik değerleri (distorsiyon, boresight, bulanıklık, shot/read); ayrı bayraklar bunları ezer")
    ap.add_argument("--distorsiyon", type=float, nargs=5, default=None, metavar=("K1", "K2", "P1", "P2", "K3"))
    ap.add_argument("--boresight-deg", type=float, nargs=3, default=None, metavar=("RX", "RY", "RZ"))
    ap.add_argument("--bulaniklik", type=float, default=None, help="optik Gauss bulanıklık σ [px]")
    ap.add_argument("--kalib", default=None,
                    help="iğne deliği K'si için kalibrasyon JSON'u (varsayılan kalibrasyon/w_nadir_10_sim.json)")
    ap.add_argument("--ad", default="gs_kamera", help="ROS düğüm adı (uav1 için gs_kamera_uav1)")
    a = ap.parse_args()
    if a.gercekci:
        a.distorsiyon = a.distorsiyon if a.distorsiyon is not None else list(GERCEKCI_DISTORSIYON)
        a.boresight_deg = a.boresight_deg if a.boresight_deg is not None else list(GERCEKCI_BORESIGHT_DEG)
        a.bulaniklik = a.bulaniklik if a.bulaniklik is not None else GERCEKCI_BULANIKLIK
        a.shot = a.shot or GERCEKCI_SHOT
        a.read = a.read or GERCEKCI_READ
    a.bulaniklik = a.bulaniklik or 0.0
    geo_gerekli = a.distorsiyon is not None or a.boresight_deg is not None
    if geo_gerekli:
        import json
        import os
        kp = a.kalib or os.path.join(os.path.dirname(os.path.abspath(__file__)), "kalibrasyon", "w_nadir_10_sim.json")
        kal = json.load(open(kp))
        GEO_ARGS = (kal["K"], a.distorsiyon, kal["width"], kal["height"], a.boresight_deg or (0.0, 0.0, 0.0))

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=2)

    class GsKamera(Node):
        def __init__(self):
            super().__init__(a.ad)
            self.rng = np.random.default_rng(a.seed)
            self.pub = self.create_publisher(Image, a.cikis, qos)
            self.pub_info = self.create_publisher(CameraInfo, a.info_cikis, qos)
            self.create_subscription(Image, a.giris, self.on_img, qos)
            self.create_subscription(CameraInfo, a.info, self.on_info, qos)
            self.geo = None
            if geo_gerekli:
                t0 = time.perf_counter()
                self.geo = Geometri(*GEO_ARGS)
                self.get_logger().info(
                    f"gerçekçilik: distorsiyon {a.distorsiyon}, boresight {a.boresight_deg}° (bapp'e verilmez), "
                    f"bulanıklık σ {a.bulaniklik} px, shot {a.shot} read {a.read} DN; boş kadraj %{100 * self.geo.bos_oran:.2f}; "
                    f"harita {1e3 * (time.perf_counter() - t0):.0f} ms")
            self.n = 0
            self.t_ms = []
            self.create_timer(10.0, self.durum)

        def on_info(self, m):
            # distorsiyon uygulanıyorsa camera_info D'si de onu söylesin (bapp yine --camera-json kullanır)
            if a.distorsiyon is not None:
                m.distortion_model = "plumb_bob"
                m.d = [float(x) for x in a.distorsiyon]
            self.pub_info.publish(m)

        def on_img(self, m):
            t0 = time.perf_counter()
            if m.encoding != "rgb8":
                self.get_logger().error(f"beklenmeyen kodlama {m.encoding} (rgb8 bekleniyor)")
                return
            h, w = m.height, m.width
            rgb = np.frombuffer(m.data, np.uint8).reshape(h, m.step)[:, : 3 * w].reshape(h, w, 3)
            bgr = isle(rgb, a.shot, a.read, self.rng, self.geo, a.bulaniklik)
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
