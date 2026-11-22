#!/usr/bin/env python3
"""
make_zemin_gercek.py — iha_arena_gercek zemini için "kuşbakışı ortofoto" benzeri prosedürel doku (2×2 karo).

İnternet/indirme yok; numpy + cv2 + scipy. 700×700 m tek bir tuval üretilir, 4 karoya bölünüp
zemin_gercek_{i}{j}.jpg olarak yazılır (i = x/Doğu indeksi, j = y/Kuzey indeksi; make_iha_arena.py ile aynı;
görüntünün üstü = Kuzey, solu = Batı). İçerik:
  - Tarla parselleri (bozulmuş Voronoi): buğday, yeşil ekin, sürülmüş toprak, anız, çayır. Her parselde
    ekin sırası çizgileri (rastgele yön/aralık), parsel başına ton farkı.
  - Toprak yollar (açık renk + iki tekerlek izi), parsel sınırlarında ağaç/çit şeritleri + öbekler,
    ağaçların gölgesi (güneş +y yönüne doğru ışık → gölge kuzeye düşer).
  - Büyük ölçekli ton değişimi (nem/toprak lekeleri).
QR hedefi (−60, −60) çevresinde (R = 25 m) ve doğma noktasında (R = 40 m) düz çayır: tespit koşulları
düz zemin/dokulu zemin testleriyle aynı kalsın diye.

Kullanım: python3 make_zemin_gercek.py [--boyut 6144] [--seed N] [--kalite 92] [--onizleme onizleme.jpg]
"""
import argparse
import os

import cv2
import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
ALAN = 700.0                       # m, tuval kenarı (−350 … +350)
QR = (-60.0, -60.0)
DUZ_BOLGELER = [(QR[0], QR[1], 25.0), (0.0, 0.0, 40.0)]   # (x, y, yarıçap m) — düz çayır

EKINLER = {   # ad: (ana renk, ağırlık, sıra genliği, sıra periyodu aralığı m)
    "bugday":   ((0.55, 0.45, 0.20), 3, 0.10, (0.6, 1.2)),
    "yesil":    ((0.17, 0.33, 0.09), 4, 0.12, (0.7, 1.6)),
    "genc":     ((0.30, 0.44, 0.15), 3, 0.09, (0.5, 1.0)),
    "suruldu":  ((0.35, 0.26, 0.18), 2, 0.16, (0.9, 2.0)),
    "aniz":     ((0.50, 0.43, 0.27), 2, 0.07, (0.5, 1.0)),
    "cayir":    ((0.22, 0.34, 0.13), 3, 0.00, (1.0, 1.0)),
}


def gurultu(n, beta, rng, f_min=0.0, f_max=None):
    """Periyodik 1/f^beta gürültüsü, ort 0,5, ±3σ → [0, 1]."""
    fy = np.fft.fftfreq(n)[:, None] * n
    fx = np.fft.rfftfreq(n)[None, :] * n
    f = np.hypot(fx, fy)
    amp = np.zeros_like(f)
    m = f > 0
    amp[m] = f[m] ** (-beta)
    amp[f < f_min] = 0.0
    if f_max:
        amp[f > f_max] = 0.0
    g = np.fft.irfft2(amp * np.exp(2j * np.pi * rng.random(f.shape)), s=(n, n))
    g = (g - g.mean()) / (g.std() + 1e-12)
    return np.clip(0.5 + g / 6.0, 0.0, 1.0).astype(np.float32)


def uret(n=6144, seed=20261121):
    rng = np.random.default_rng(seed)
    px = n / ALAN                                           # piksel / m
    # tuval koordinatları (m): satır 0 = +Y (Kuzey), sütun 0 = −X (Batı)
    kx = (np.arange(n, dtype=np.float32) + 0.5) / px - ALAN / 2
    ky = ALAN / 2 - (np.arange(n, dtype=np.float32) + 0.5) / px
    X, Y = np.meshgrid(kx, ky)

    # --- parseller: bozulmuş Voronoi (düşük çözünürlükte etiket, sonra büyüt) ---
    kn = 1536
    sx = (np.arange(kn, dtype=np.float32) + 0.5) / kn * ALAN - ALAN / 2
    sy = ALAN / 2 - (np.arange(kn, dtype=np.float32) + 0.5) / kn * ALAN
    gx, gy = np.meshgrid(sx, sy)
    warp = 28.0
    wx = (gurultu(kn, 2.4, rng, f_max=10) - 0.5) * 2 * warp
    wy = (gurultu(kn, 2.4, rng, f_max=10) - 0.5) * 2 * warp
    tohum = rng.uniform(-ALAN / 2 - 60, ALAN / 2 + 60, size=(46, 2))
    _, etiket = cKDTree(tohum).query(np.stack([(gx + wx).ravel(), (gy + wy).ravel()], 1))
    etiket = etiket.reshape(kn, kn).astype(np.int32)
    etiket_b = cv2.resize(etiket.astype(np.float32), (n, n), interpolation=cv2.INTER_NEAREST).astype(np.int32)

    adlar = list(EKINLER)
    agirlik = np.array([EKINLER[a][1] for a in adlar], dtype=float)
    agirlik /= agirlik.sum()
    tur = rng.choice(len(adlar), size=len(tohum), p=agirlik)
    ton_f = rng.normal(1.0, 0.07, size=len(tohum))
    aci_f = rng.uniform(0, np.pi, size=len(tohum))
    per_f = np.array([rng.uniform(*EKINLER[adlar[t]][3]) for t in tur])
    gen_f = np.array([EKINLER[adlar[t]][2] for t in tur])
    renk_f = np.array([EKINLER[adlar[t]][0] for t in tur], dtype=np.float32)

    renk = renk_f[etiket_b] * ton_f[etiket_b][..., None]
    # ekin sıraları (parsel başına yön/periyot)
    u = X * np.cos(aci_f)[etiket_b] + Y * np.sin(aci_f)[etiket_b]
    sira = np.sin(2 * np.pi * u / per_f[etiket_b])
    renk *= (1.0 + gen_f[etiket_b] * sira)[..., None]
    del u, sira

    # --- ince doku + büyük ölçekli ton ---
    ince = gurultu(n, 1.1, rng, f_min=n / 60)
    orta = gurultu(n, 1.8, rng, f_min=4, f_max=200)
    buyuk = gurultu(n, 2.2, rng, f_max=8)
    lum = 1.0 + 0.35 * (ince - 0.5) + 0.30 * (orta - 0.5) + 0.35 * (buyuk - 0.5)
    renk *= lum[..., None]
    del ince, orta

    # --- parsel sınırı (kenar) maskesi ---
    kenar = np.zeros((n, n), np.uint8)
    kenar[:, 1:] |= (etiket_b[:, 1:] != etiket_b[:, :-1]).astype(np.uint8)
    kenar[1:, :] |= (etiket_b[1:, :] != etiket_b[:-1, :]).astype(np.uint8)

    # --- toprak yollar ---
    yol = np.zeros((n, n), np.uint8)
    iz = np.zeros((n, n), np.uint8)

    def m2p(x, y):
        return int((x + ALAN / 2) * px), int((ALAN / 2 - y) * px)

    yollar = []
    for _ in range(3):
        a0 = rng.uniform(0, 2 * np.pi)
        p0 = np.array([np.cos(a0), np.sin(a0)]) * 480.0
        p1 = -p0 + rng.normal(0, 60, 2)
        kayma = rng.uniform(140, 260) * rng.choice([-1, 1])
        nn = np.array([-(p1 - p0)[1], (p1 - p0)[0]]); nn /= np.linalg.norm(nn)
        p0 = p0 + nn * kayma; p1 = p1 + nn * kayma
        t = np.linspace(0, 1, 60)[:, None]
        ort = p0 * (1 - t) + p1 * t
        dik = np.array([-(p1 - p0)[1], (p1 - p0)[0]]); dik /= np.linalg.norm(dik)
        sap = (gurultu(64, 2.0, rng)[0, :60] - 0.5) * 2 * 90.0
        yollar.append(ort + dik * sap[:, None])
    # QR ve doğma bölgesinden geçen yolu itele (R + 12 m payı)
    for pts in yollar:
        for (cx, cy, r) in DUZ_BOLGELER:
            d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
            yakin = d < r + 12
            if yakin.any():
                yon = (pts[yakin] - np.array([cx, cy])) / np.maximum(d[yakin], 1e-3)[:, None]
                pts[yakin] = np.array([cx, cy]) + yon * (r + 12)
        def poli_of(q):
            return np.array([m2p(x, y) for x, y in q], np.int32).reshape(-1, 1, 2)
        tg = np.gradient(pts, axis=0); tg /= np.linalg.norm(tg, axis=1, keepdims=True) + 1e-9
        nr = np.stack([-tg[:, 1], tg[:, 0]], 1)
        cv2.polylines(yol, [poli_of(pts)], False, 255, int(3.6 * px), cv2.LINE_AA)
        for sgn in (-0.85, 0.85):                                     # tekerlek izleri
            cv2.polylines(iz, [poli_of(pts + nr * sgn)], False, 255, max(2, int(0.5 * px)), cv2.LINE_AA)
    yol_f = cv2.GaussianBlur(yol.astype(np.float32) / 255, (0, 0), 0.5 * px)
    iz_f = cv2.GaussianBlur(iz.astype(np.float32) / 255, (0, 0), 0.25 * px)
    yol_renk = np.array([0.52, 0.44, 0.32], np.float32)
    toz = 1.0 + 0.25 * (gurultu(n, 1.2, rng, f_min=n / 80) - 0.5)
    renk = renk * (1 - yol_f[..., None]) + (yol_renk * toz[..., None]) * yol_f[..., None]
    renk *= (1.0 - 0.20 * iz_f * yol_f)[..., None]

    # --- ağaç/çit şeritleri ve öbekler ---
    ag_gur = gurultu(n, 1.4, rng, f_min=n / 120, f_max=n / 6)
    kenar_g = cv2.dilate(kenar, np.ones((int(3.0 * px), int(3.0 * px)), np.uint8))
    yaban = ((kenar_g > 0) & (ag_gur > 0.54)).astype(np.uint8)
    obek = (gurultu(n, 2.0, rng, f_max=40) > 0.715).astype(np.uint8) & (ag_gur > 0.48)
    ag = (yaban | obek.astype(np.uint8)).astype(np.uint8)
    # düz bölgeler ve yol: ağaç yok
    for (cx, cy, r) in DUZ_BOLGELER:
        cv2.circle(ag, m2p(cx, cy), int((r + 6) * px), 0, -1)
    ag[yol > 60] = 0
    ag_f = cv2.GaussianBlur(ag.astype(np.float32), (0, 0), 0.5 * px)
    # gölge: ağacın +y (kuzey) tarafına ~5 m (satırlar yukarı = −satır)
    gol = np.roll(ag_f, -int(5.0 * px), axis=0)
    gol = cv2.GaussianBlur(gol, (0, 0), 1.2 * px)
    renk *= (1.0 - 0.38 * np.clip(gol - ag_f, 0, 1))[..., None]
    ag_ton = np.array([0.075, 0.15, 0.06], np.float32) * (0.7 + 0.8 * gurultu(n, 0.9, rng, f_min=n / 25))[..., None]
    renk = renk * (1 - ag_f[..., None]) + ag_ton * ag_f[..., None]
    del ag_gur

    # --- düz bölgeler: QR ve doğma çevresi sade çayır (ton ortalaması korunur) ---
    cayir = np.array(EKINLER["cayir"][0], np.float32)
    sade = cayir * (0.95 + 0.10 * gurultu(n, 1.4, rng, f_min=n / 40))[..., None]
    for (cx, cy, r) in DUZ_BOLGELER:
        mx = np.hypot(X - cx, Y - cy)
        w = np.clip((r + 10 - mx) / 10.0, 0, 1)[..., None]        # 10 m yumuşak geçiş
        renk = renk * (1 - w) + sade * w

    # --- gama + hafif atmosferik ton ---
    renk = np.clip(renk, 0, 1) ** 0.95
    return (renk * 255 + 0.5).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boyut", type=int, default=6144)
    ap.add_argument("--seed", type=int, default=20261121)
    ap.add_argument("--kalite", type=int, default=92)
    ap.add_argument("--klasor", default=HERE)
    ap.add_argument("--onizleme", default=None, help="1024 px önizleme JPEG yolu")
    a = ap.parse_args()
    img = uret(a.boyut, a.seed)
    if a.onizleme:
        cv2.imwrite(a.onizleme, cv2.cvtColor(cv2.resize(img, (1024, 1024), interpolation=cv2.INTER_AREA), cv2.COLOR_RGB2BGR))
    t = a.boyut // 2
    toplam = 0
    for i in range(2):
        for j in range(2):
            # karo (i, j): X aralığı i'ye göre, Y aralığı j'ye göre; satır 0 = +Y → j = 1 üst yarı
            parca = img[(1 - j) * t:(2 - j) * t, i * t:(i + 1) * t]
            yol = os.path.join(a.klasor, f"zemin_gercek_{i}{j}.jpg")
            cv2.imwrite(yol, cv2.cvtColor(parca, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, a.kalite])
            toplam += os.path.getsize(yol)
            print(f"yazıldı: {yol}  {t}×{t}")
    g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    print(f"toplam {toplam / 1e6:.1f} MB; tuval {a.boyut}² ({ALAN / a.boyut * 100:.1f} cm/px); gri ort {g.mean():.0f}, std {g.std():.1f}")


if __name__ == "__main__":
    main()
