#!/usr/bin/env python3
"""
make_zemin_doku.py — iha_arena_v2 zemini için prosedürel, DİKİŞSİZ (döşenebilir) çim/toprak dokusu üretir.

İnternet/indirme yok: numpy FFT ile 1/f^β gürültüsü (periyodik → karo kenarlarında dikiş olmaz).
  - düşük frekans maskesi: çim (yeşil) ↔ toprak (kahverengi) yamaları
  - orta/yüksek frekans parlaklık gürültüsü: ince doku
--kontrast parlaklık gürültüsünün genliğini, --parlaklik ortalama albedoyu ölçekler (QR tespiti
bozulursa düşürülür). Çıktı sRGB PNG; make_iha_arena.py --zemin dokulu bunu PBR albedo olarak kullanır.

Kullanım: python3 make_zemin_doku.py [--boyut 2048] [--kontrast 1.0] [--parlaklik 1.0] [--out zemin_doku.png]
"""
import argparse
import os

import cv2
import numpy as np


def gurultu(n, beta, rng, f_min=0.0, f_max=None):
    """Periyodik 1/f^beta gürültüsü, [0, 1] aralığına sıkıştırılmış (ort 0,5)."""
    fy = np.fft.fftfreq(n)[:, None] * n
    fx = np.fft.rfftfreq(n)[None, :] * n
    f = np.hypot(fx, fy)
    amp = np.zeros_like(f)
    m = f > 0
    amp[m] = f[m] ** (-beta)
    amp[f < f_min] = 0.0
    if f_max:
        amp[f > f_max] = 0.0
    spec = amp * np.exp(2j * np.pi * rng.random(f.shape))
    g = np.fft.irfft2(spec, s=(n, n))
    g = (g - g.mean()) / (g.std() + 1e-12)
    return np.clip(0.5 + g / 6.0, 0.0, 1.0)          # ±3σ → [0, 1]


def uret(n=2048, kontrast=1.0, parlaklik=1.0, seed=20261111):
    rng = np.random.default_rng(seed)
    maske = gurultu(n, 2.2, rng, f_max=24)           # büyük yamalar (karo başına ≤ 24 çevrim)
    maske = np.clip((maske - 0.5) * 3.0 + 0.45, 0.0, 1.0)[..., None]   # çim ağırlıklı, toprak adacıkları
    ince = gurultu(n, 1.0, rng, f_min=8)             # ince doku (parlaklık)
    orta = gurultu(n, 1.6, rng, f_min=2, f_max=96)
    cim = np.array([0.20, 0.33, 0.12])               # doğrusal-benzeri albedo (R, G, B)
    cim2 = np.array([0.30, 0.38, 0.16])              # kurumuş çim tonu
    toprak = np.array([0.38, 0.29, 0.20])
    ton = orta[..., None]
    renk = maske * (cim * (1 - ton) + cim2 * ton) + (1 - maske) * toprak
    lum = 1.0 + kontrast * (0.55 * (ince - 0.5) + 0.35 * (orta - 0.5))
    img = np.clip(renk * lum[..., None] * parlaklik, 0.0, 1.0)
    return (img * 255 + 0.5).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boyut", type=int, default=2048)
    ap.add_argument("--kontrast", type=float, default=1.0)
    ap.add_argument("--parlaklik", type=float, default=1.0)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "zemin_doku.png"))
    a = ap.parse_args()
    img = uret(a.boyut, a.kontrast, a.parlaklik)
    cv2.imwrite(a.out, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    print(f"yazıldı: {a.out}  {a.boyut}×{a.boyut}, {os.path.getsize(a.out) / 1e6:.1f} MB; gri ort {g.mean():.0f}, "
          f"std {g.std():.1f}, min {g.min()}, maks {g.max()}; dikiş farkı (sol-sağ) "
          f"{np.abs(img[:, 0].astype(int) - img[:, -1].astype(int)).mean():.1f}")


if __name__ == "__main__":
    main()
