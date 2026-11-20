# ArUco simülasyon değerlendirmesi — /home/bap/sim/iha_sim_v2/test/aruco_2026-11-20/z1/aruco_log

- Gerçek hedef: ENU (-60.0, -60.0) → (47.3974315, 8.5453666); geolocator NED (-59.98, -60.01, 0.11); EKF origin 47.3979709, 8.5461639, 0.13 m
- Kare: 10734 işlenen, 38 beklenen (marker tam görüntüde), 33 tespitli; sahte tespit (beklenmezken): 0
- Zaman hizası: PX4–gerçek yatay konum farkını en küçükleyen gecikme -20 ms (medyan fark 0.30 m); 0 ms civarı → kare/PX4 saatleri aynı

## Tespit oranı (beklenen karelerde)

| kenar px | irtifa m | beklenen | tespit | oran |
|---|---|---|---|---|
| 30–35 | 101 | 38 | 33 | 86.8 % |

## Kare başına hata (yatay, m)

- ray-cast: medyan 0.55, %95 0.64, en çok 0.66 (n=33)
- PnP: medyan 1.02, %95 1.18, en çok 1.24 (n=33)
- ray-cast GERÇEK pozla (tespit+kamera modeli): medyan 0.06, %95 0.08, en çok 0.08 (n=33)
- PnP GERÇEK pozla: medyan 1.07, %95 1.31, en çok 1.44 (n=33)
- köşe hatası (gerçek izdüşüme göre, RMS px): medyan 1.04, %95 1.14, en çok 1.18 (n=33)

## PX4 poz hatası (tespitli karelerde)

- yatay konum: medyan 0.39, %95 0.43, en çok 0.44 (n=33) m
- düşey (D): medyan 0.07, %95 0.08, en çok 0.08 (n=33) m
- |roll|: medyan 0.09, %95 0.10, en çok 0.10 (n=33)°
- |pitch|: medyan 0.11, %95 0.13, en çok 0.13 (n=33)°
- |yaw|: medyan 0.21, %95 0.22, en çok 0.22 (n=33)°

## LoRa raporları (füzyon tahmini, gönderilen)

| # | n_obs | hata m | σ95 m | hata/σ95 |
|---|---|---|---|---|
| 1 | 4 | 0.87 | 1.91 | 0.46 |
| 100 | 66 | 0.79 | 0.45 | 1.74 |
| 199 | 66 | 0.79 | 0.45 | 1.74 |
| 298 | 66 | 0.79 | 0.45 | 1.74 |
| 396 | 66 | 0.79 | 0.45 | 1.74 |

- Son rapor hatası: **0.79 m** (σ95 0.45 m, 66 gözlem); raporların %0'inde hata ≤ σ95
