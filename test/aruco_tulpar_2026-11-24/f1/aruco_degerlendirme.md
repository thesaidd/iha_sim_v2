# ArUco simülasyon değerlendirmesi — /home/bap/sim/iha_sim_v2/test/aruco_tulpar_2026-11-24/f1/aruco_log

- Gerçek hedef: ENU (-60.0, -60.0) → (47.3974315, 8.5453666); geolocator NED (-60.02, -60.00, -0.05); EKF origin 47.3979712, 8.5461637, -0.03 m
- Kare: 1195 işlenen, 12 beklenen (marker tam görüntüde), 12 tespitli; sahte tespit (beklenmezken): 0
- Zaman hizası: PX4–gerçek yatay konum farkını en küçükleyen gecikme -20 ms (medyan fark 0.06 m); 0 ms civarı → kare/PX4 saatleri aynı

## Tespit oranı (beklenen karelerde)

| kenar px | irtifa m | beklenen | tespit | oran |
|---|---|---|---|---|
| 30–35 | 100 | 10 | 10 | 100.0 % |
| 35–40 | 100 | 2 | 2 | 100.0 % |

## Kare başına hata (yatay, m)

- ray-cast: medyan 0.83, %95 0.99, en çok 1.00 (n=12)
- PnP: medyan 0.92, %95 1.05, en çok 1.14 (n=12)
- ray-cast GERÇEK pozla (tespit+kamera modeli): medyan 0.05, %95 0.07, en çok 0.07 (n=12)
- PnP GERÇEK pozla: medyan 0.69, %95 0.88, en çok 0.96 (n=12)
- köşe hatası (gerçek izdüşüme göre, RMS px): medyan 1.02, %95 1.12, en çok 1.15 (n=12)

## PX4 poz hatası (tespitli karelerde)

- yatay konum: medyan 0.34, %95 0.40, en çok 0.41 (n=12) m
- düşey (D): medyan 0.06, %95 0.08, en çok 0.08 (n=12) m
- |roll|: medyan 0.10, %95 0.15, en çok 0.15 (n=12)°
- |pitch|: medyan 0.29, %95 0.31, en çok 0.31 (n=12)°
- |yaw|: medyan 0.07, %95 0.17, en çok 0.17 (n=12)°

## LoRa raporları (füzyon tahmini, gönderilen)

| # | n_obs | hata m | σ95 m | hata/σ95 |
|---|---|---|---|---|
| 1 | 1 | 0.83 | 3.88 | 0.21 |
| 111 | 12 | 0.87 | 1.89 | 0.46 |
| 221 | 12 | 0.87 | 1.89 | 0.46 |
| 331 | 12 | 0.87 | 1.89 | 0.46 |
| 441 | 12 | 0.87 | 1.89 | 0.46 |

- Son rapor hatası: **0.87 m** (σ95 1.89 m, 12 gözlem); raporların %100'inde hata ≤ σ95
