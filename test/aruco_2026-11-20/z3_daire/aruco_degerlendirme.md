# ArUco simülasyon değerlendirmesi — /home/bap/sim/iha_sim_v2/test/aruco_2026-11-20/z3_daire/aruco_log

- Gerçek hedef: ENU (-60.0, -60.0) → (47.3974315, 8.5453666); geolocator NED (-59.99, -59.99, 0.07); EKF origin 47.3979710, 8.5461636, 0.09 m
- Kare: 10257 işlenen, 35 beklenen (marker tam görüntüde), 35 tespitli; sahte tespit (beklenmezken): 0
- Zaman hizası: PX4–gerçek yatay konum farkını en küçükleyen gecikme -10 ms (medyan fark 0.05 m); 0 ms civarı → kare/PX4 saatleri aynı

## Tespit oranı (beklenen karelerde)

| kenar px | irtifa m | beklenen | tespit | oran |
|---|---|---|---|---|
| 30–35 | 100 | 31 | 31 | 100.0 % |
| 35–40 | 100 | 4 | 4 | 100.0 % |

## Kare başına hata (yatay, m)

- ray-cast: medyan 1.43, %95 1.67, en çok 1.70 (n=35)
- PnP: medyan 1.27, %95 1.67, en çok 1.69 (n=35)
- ray-cast GERÇEK pozla (tespit+kamera modeli): medyan 0.04, %95 0.06, en çok 0.07 (n=35)
- PnP GERÇEK pozla: medyan 0.64, %95 0.78, en çok 1.07 (n=35)
- köşe hatası (gerçek izdüşüme göre, RMS px): medyan 0.97, %95 1.11, en çok 1.18 (n=35)

## PX4 poz hatası (tespitli karelerde)

- yatay konum: medyan 0.11, %95 0.16, en çok 0.17 (n=35) m
- düşey (D): medyan 0.03, %95 0.04, en çok 0.04 (n=35) m
- |roll|: medyan 0.55, %95 0.56, en çok 0.56 (n=35)°
- |pitch|: medyan 0.31, %95 0.32, en çok 0.32 (n=35)°
- |yaw|: medyan 0.66, %95 0.68, en çok 0.68 (n=35)°

## LoRa raporları (füzyon tahmini, gönderilen)

| # | n_obs | hata m | σ95 m | hata/σ95 |
|---|---|---|---|---|
| 1 | 14 | 1.06 | 0.94 | 1.13 |
| 100 | 70 | 1.33 | 0.41 | 3.23 |
| 200 | 70 | 1.33 | 0.41 | 3.23 |
| 299 | 70 | 1.33 | 0.41 | 3.23 |
| 398 | 70 | 1.33 | 0.41 | 3.23 |

- Son rapor hatası: **1.33 m** (σ95 0.41 m, 70 gözlem); raporların %0'inde hata ≤ σ95
