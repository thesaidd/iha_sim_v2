#!/usr/bin/env python3
"""
make_qr_hedef.py — arena QR hedefinin dokusunu (qr_hedef01.png) üretir ve doğrular.

QR: sürüm 1 (21×21 modül), hata düzeltme M, içerik "HEDEF-01", 4 modül beyaz kenar → 29×29 modül.
cv2.QRCodeEncoder 2 modül kenar üretir; kod kırpılır, 4 modül kenarla yeniden çerçevelenir,
INTER_NEAREST ile büyütülür (yumuşatma yok, yalnızca 0/255).
Doğrulama: format bitlerinden hata düzeltme seviyesi, cv2.QRCodeDetector ile içerik.
Levha: kod 1,00 m → modül 1/21 m, kenarla birlikte 29/21 m (make_iha_arena.py QR_SIZE ile aynı).

Kullanım: python3 make_qr_hedef.py   → qr_hedef01.png (aynı klasör)
"""

import os
import sys

import cv2
import numpy as np

TEXT = "HEDEF-01"
MODULES = 21          # sürüm 1
QUIET = 4             # modül
PX_PER_MODULE = 16    # ≥ 16; OpenCV 4.9 QRCodeDetector ≥ 26 px/modülde bulucu desenleri bulamıyor (2026-10-30)
CODE_SIZE_M = 1.00
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "qr_hedef01.png")


def ec_level(modules):
    """Format bilgisi (satır 8, sütun 0–5, 7, 8 + sütun 8, satır 7, 5–0) → hata düzeltme harfi."""
    bits = [modules[8, c] for c in (0, 1, 2, 3, 4, 5, 7, 8)] + [modules[r, 8] for r in (7, 5, 4, 3, 2, 1, 0)]
    value = int("".join(str(b) for b in bits), 2) ^ 0b101010000010010
    return {0b01: "L", 0b00: "M", 0b11: "Q", 0b10: "H"}[value >> 13]


def main():
    params = cv2.QRCodeEncoder_Params()
    params.version = 1
    params.correction_level = cv2.QRCODE_ENCODER_CORRECT_LEVEL_M
    raw = cv2.QRCodeEncoder.create(params).encode(TEXT)
    dark = np.argwhere(raw == 0)
    (r0, c0), (r1, c1) = dark.min(0), dark.max(0)
    code = raw[r0:r1 + 1, c0:c1 + 1]
    if code.shape != (MODULES, MODULES):
        sys.exit(f"beklenmeyen kod boyutu {code.shape} (sürüm 1 = 21×21)")
    level = ec_level((code == 0).astype(int))
    if level != "M":
        sys.exit(f"hata düzeltme seviyesi {level}, M bekleniyordu")

    framed = np.pad(code, QUIET, constant_values=255)
    side = framed.shape[0] * PX_PER_MODULE
    img = cv2.resize(framed, (side, side), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(OUT, cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))   # 3 kanal: tek kanal ogre2'de R8 (kırmızı) yüklenebilir

    back = cv2.imread(OUT, cv2.IMREAD_GRAYSCALE)
    text, points, _ = cv2.QRCodeDetector().detectAndDecode(back)
    module_cm = CODE_SIZE_M / MODULES * 100
    print(f"yazıldı: {OUT}")
    print(f"  {framed.shape[0]}×{framed.shape[0]} modül (kod {MODULES} + 2×{QUIET} kenar), "
          f"{side}×{side} px, {PX_PER_MODULE} px/modül, değerler {sorted(np.unique(back).tolist())}")
    print(f"  hata düzeltme: {level}")
    print(f"  levha: kod {CODE_SIZE_M:.2f} m, modül {module_cm:.3f} cm, kenarla {framed.shape[0] * module_cm / 100:.4f} m")
    print(f"  cv2.QRCodeDetector: {text!r} → {'DOĞRU' if text == TEXT else 'YANLIŞ'}")
    if text != TEXT:
        sys.exit(1)


if __name__ == "__main__":
    main()
