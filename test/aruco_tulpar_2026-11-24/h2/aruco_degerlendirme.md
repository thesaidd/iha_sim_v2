# ArUco simülasyon değerlendirmesi — /home/bap/sim/iha_sim_v2/test/aruco_tulpar_2026-11-24/h2/aruco_log

- Gerçek hedef: ENU (-60.0, -60.0) → (47.3974315, 8.5453666); geolocator NED (-61.48, -60.89, -0.52); EKF origin 47.3979844, 8.5461755, -0.50 m
- Kare: 7474 işlenen, 317 beklenen (marker tam görüntüde), 247 tespitli; sahte tespit (beklenmezken): 1
- Zaman hizası: PX4–gerçek yatay konum farkını en küçükleyen gecikme -90 ms (medyan fark 1.49 m); 0 ms civarı → kare/PX4 saatleri aynı

## Tespit oranı (beklenen karelerde)

| kenar px | irtifa m | beklenen | tespit | oran |
|---|---|---|---|---|
| 25–30 | 100 | 46 | 14 | 30.4 % |
| 30–35 | 100 | 244 | 213 | 87.3 % |
| 35–40 | 99 | 27 | 19 | 70.4 % |

## Kare başına hata (yatay, m)

- ray-cast: medyan 2.89, %95 5.81, en çok 6.25 (n=244)
- PnP: medyan 2.59, %95 5.88, en çok 6.42 (n=235)
- ray-cast GERÇEK pozla (tespit+kamera modeli): medyan 1.29, %95 1.95, en çok 2.29 (n=247)
- PnP GERÇEK pozla: medyan 1.24, %95 4.80, en çok 9.62 (n=247)
- köşe hatası (gerçek izdüşüme göre, RMS px): medyan 22.25, %95 24.44, en çok 25.90 (n=246)

## PX4 poz hatası (tespitli karelerde)

- yatay konum: medyan 2.04, %95 2.25, en çok 2.36 (n=247) m
- düşey (D): medyan 0.52, %95 0.63, en çok 0.63 (n=247) m
- |roll|: medyan 0.41, %95 0.64, en çok 0.66 (n=247)°
- |pitch|: medyan 0.41, %95 1.43, en çok 1.46 (n=247)°
- |yaw|: medyan 1.56, %95 3.30, en çok 3.35 (n=247)°

## LoRa raporları (füzyon tahmini, gönderilen)

| # | n_obs | hata m | σ95 m | hata/σ95 |
|---|---|---|---|---|
| 1 | 2 | 6.15 | 5.40 | 1.14 |
| 191 | 97 | 3.74 | 4.53 | 0.83 |
| 381 | 132 | 3.80 | 4.62 | 0.82 |
| 571 | 191 | 3.10 | 4.57 | 0.68 |
| 760 | 244 | 2.70 | 4.59 | 0.59 |

- Son rapor hatası: **2.70 m** (σ95 4.59 m, 244 gözlem); raporların %95'inde hata ≤ σ95
