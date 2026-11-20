#!/usr/bin/env python3
"""
make_aruco_hedef.py — arena ArUco hedefinin dokusunu (aruco_hedef.png) üretir ve doğrular.

Marker: DICT_4X4_50, varsayılan id 0. 4×4 veri + 1 modül siyah çerçeve = 6 modül; siyah dış kenar
--boyut m (varsayılan 2,00 m → modül 1/3 m). Çevresinde --kenar modül beyaz boşluk (varsayılan 1)
→ levha (6 + 2·kenar) modül = 2,00·8/6 = 2,667 m. INTER_NEAREST ile büyütülür (yalnızca 0/255).
Doğrulama: cv2.aruco.ArucoDetector (bapp ile aynı sözlük) id'yi ve köşe sırasını bulmalı.

Görüntü yönü: dokunun üstü levhanın +y (kuzey) kenarına gelir (make_iha_arena.py yaw 0). ArUco köşe 0
(sol-üst) = kuzeybatı köşe.

Kullanım: python3 make_aruco_hedef.py [--id 0] [--boyut 2.0] [--kenar 1] [--px-modul 64]
          → aruco_hedef.png (aynı klasör) + levha kenarı (m) ekrana
"""

import argparse
import os
import sys

import cv2
import numpy as np

MARKER_MODULES = 6     # 4×4 veri + 2 çerçeve
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "aruco_hedef.png")


def levha_kenari(boyut_m: float, kenar_modul: int) -> float:
    """Beyaz boşluk dahil levha kenarı (m)."""
    return boyut_m * (MARKER_MODULES + 2 * kenar_modul) / MARKER_MODULES


def uret(marker_id: int, kenar_modul: int, px_modul: int) -> np.ndarray:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    m = cv2.aruco.generateImageMarker(d, marker_id, MARKER_MODULES)      # 6×6, 1 px/modül
    m = cv2.copyMakeBorder(m, kenar_modul, kenar_modul, kenar_modul, kenar_modul, cv2.BORDER_CONSTANT, value=255)
    n = m.shape[0] * px_modul
    return cv2.resize(m, (n, n), interpolation=cv2.INTER_NEAREST)


def dogrula(img: np.ndarray, marker_id: int) -> None:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(img)
    if ids is None or list(ids.ravel()) != [marker_id]:
        raise SystemExit(f"doğrulama başarısız: ids={None if ids is None else ids.ravel().tolist()}")
    c = corners[0].reshape(4, 2)
    if not (c[0, 0] < c[1, 0] and c[0, 1] < c[3, 1]):          # köşe 0 sol-üst, 1 sağ-üst, 3 sol-alt
        raise SystemExit(f"köşe sırası beklenmedik: {c.tolist()}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--boyut", type=float, default=2.0, help="siyah dış kenar [m]")
    ap.add_argument("--kenar", type=int, default=1, help="beyaz boşluk [modül]")
    ap.add_argument("--px-modul", type=int, default=64)
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    if not 0 <= a.id < 50:
        sys.exit("DICT_4X4_50: id 0–49")
    img = uret(a.id, a.kenar, a.px_modul)
    dogrula(img, a.id)
    cv2.imwrite(a.out, img)
    print(f"yazıldı: {a.out} ({img.shape[1]}×{img.shape[0]} px, id {a.id}, marker {a.boyut:.3f} m, "
          f"levha {levha_kenari(a.boyut, a.kenar):.4f} m, modül {a.boyut / MARKER_MODULES:.4f} m)")


if __name__ == "__main__":
    main()
