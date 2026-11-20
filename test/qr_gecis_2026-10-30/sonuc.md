# QR geçiş testi — 2026-10-30 (uav0, 60 m, ψ 210°, 10 Hz kamera)

Akış: `CAM=T10 ./launch_all.sh` → `camera_bridge.yaml` köprüsü → eşiksiz RTF kaydı (`uzun_monitor.py` kopyası,
5 sn) → `mavlink_takeoff.py --uav 0` (bir kez; 100 m Hold) → `qr_recorder.py` → her geçişte `qr_watch.py` +
`qr_pass.py` → `qr_analyze.py kayit`. 15 Hz ile ilk deneme RTF 0,692 < 0,7 ile abort edildi (`deneme1_15hz/`).

## Uçuş (PX4 tahmini, `gecis_sonuclari.jsonl`)

| Geçiş | Bias | Devralma (s, x) | Hedef üstü yanal / dikey | s ∈ [−400, +150] yanal ort ± std, maks | Dikey ort, maks | Yer / hava hızı |
|---|---|---|---|---|---|---|
| 1 | 0 | −670 m, +3,0 m | −0,30 / +0,21 m | −0,25 ± 0,05, 0,33 m | +0,26, 0,67 m | 14,7 / 14,2 m/s |
| 2 | 0 | −670 m, +2,9 m | −0,47 / +0,41 m | −0,46 ± 0,04, 0,55 m | +0,36, 0,47 m | 14,4 / 14,0 m/s |
| 3 | +0,4 | −670 m, +3,0 m | +0,22 / −0,26 m | +0,23 ± 0,05, 0,36 m | −0,39, 0,73 m | 14,5 / 14,0 m/s |

Gerçek (Gazebo) yanal sapma hedef üstünde: −0,18 / −0,36 / +0,35 m.

## Görüntü (`kayit/analiz/ozet.json`)

| Geçiş | Kare | Kadrajda | QRCodeDetector bulma / çözme | Aruco bulma / çözme | İlk–son çözüm (s, gerçek) | Süre | Modül px | 3B mesafe |
|---|---|---|---|---|---|---|---|---|
| 1 | 228 | 9 | 9 / 9 | 9 / 7 | −8,0 … +3,7 m | 0,8 sn | 3,66 | 60,3 m |
| 2 | 224 | 8 | 8 / 8 | 8 / 6 | −6,1 … +4,0 m | 0,7 sn | 3,67 | 60,5 m |
| 3 | 226 | 8 | 8 / 8 | 8 / 6 | −7,9 … +3,7 m | 0,8 sn | 3,70 | 60,0 m |

Kadraj dışı bulma 0, yanlış içerik 0. Ortalama süre/kare (tüm kareler): QR 73 ms, Aruco 12 ms.
Statik tarama (60 m): modül 3,67 px, iki yönde %100 bulma/çözme → uçuşla aynı.

## Konum tahmini (QR merkezi pikselinden, bulunan karelerde)

| Poz kaynağı | Model | n | Ortalama hata (dN, dE) | Yatay ort / RMS / maks |
|---|---|---|---|---|
| Gazebo gerçek | tam (roll, pitch, yaw) | 25 | (−0,048, −0,028) m | 0,056 / 0,057 / 0,063 m |
| Gazebo gerçek | yalnızca yaw | 25 | (+1,661, +0,472) m | 1,73 / 1,74 / 1,97 m |
| PX4 (geçiş 2–3) | tam | 16 | (−0,815, −0,869) m | 1,29 / 1,32 / 1,82 m |
| PX4 (geçiş 2–3) | yalnızca yaw | 16 | (+0,649, +0,122) m | 0,82 / 0,85 / 1,21 m |

Geçiş 1'de PX4 odometrisi PX4 damgasıyla eşleştirilmişti (1–2 sn kayık) → PX4 tahmininden çıkarıldı.

## RTF

Hold 0,952 (10 Hz), Offboard kayıt öncesi 0,86–0,88, kayıt penceresi 0,77–0,78 (gz CPU %290 → %390),
en düşük 0,750 (kayıt dışında). Kaydedici %21 CPU, 151 MB; 678 kare 27 MB, düşen 0.
