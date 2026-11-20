# Tarama → dalış zinciri kuru testi (uçuş yok)

## 1. Plan taraması — tarama planının her (şerit, yön) × 5 hedef konumu

| direk payı | durum | uygun plan | NO-GO | en küçük direk (m) | en küçük uav0 çember kenarı (m) | 40 m altı koridor noktası | yön sapması ort / maks (°) |
|---|---|---|---|---|---|---|---|
| 40 | 45 | 42 | 3 | 41.5 | 60.7 | 321 (Hold merkezi koridor üstünde: 4 plan) | 49 / 150 |

NO-GO (pay 40): şerit -40 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-4.4 m)); şerit -100 yön +1 t=+0: hiçbir yön açık değil (ilk ret: hizalanma çemberi ↔ direk_D (-0.9 m)); şerit +81 yön -1 t=+60: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-1.5 m))


Çıkış Hold merkezleri (pay 40, uav0 NED): [(-310, -40), (-310, -26), (-310, -13), (-310, 1), (310, -20), (310, -5), (310, 10), (310, 25), (310, 40)]

| 20 | 80 | 57 | 23 | 41.3 | 62.4 | 580 (Hold merkezi koridor üstünde: 11 plan) | 37 / 155 |

NO-GO (pay 20): şerit +47 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-1.1 m)); şerit -120 yön -1 t=-120: hiçbir yön açık değil (ilk ret: hizalanma çemberi ↔ direk_D (-1.1 m)); şerit -120 yön -1 t=+0: hiçbir yön açık değil (ilk ret: hat ↔ direk_B (-3.4 m)); şerit +61 yön +1 t=+60: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-2.1 m)); şerit +61 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-0.5 m)); şerit -106 yön -1 t=+0: hiçbir yön açık değil (ilk ret: tırmanış ↔ direk_B (-3.9 m)); şerit +74 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-2.0 m)); şerit +88 yön +1 t=+60: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-0.2 m)); şerit +88 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-0.2 m)); şerit -77 yön -1 t=-120: hiçbir yön açık değil (ilk ret: geçiş yolu yok); şerit +101 yön +1 t=+60: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-1.6 m)); şerit +101 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-0.0 m)); şerit -63 yön -1 t=-120: hiçbir yön açık değil (ilk ret: geçiş yolu yok); şerit +34 yön +1 t=-120: hiçbir yön açık değil (ilk ret: hizalanma çemberi ↔ direk_D (-9.3 m)); şerit +34 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-1.3 m)); şerit -49 yön -1 t=-120: hiçbir yön açık değil (ilk ret: hat ↔ direk_G (-0.6 m)); şerit +142 yön +1 t=+60: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-1.3 m)); şerit -34 yön -1 t=-120: hiçbir yön açık değil (ilk ret: hizalanma çemberi ↔ direk_D (-2.7 m)); şerit -34 yön -1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-3.6 m)); şerit +20 yön +1 t=-120: hiçbir yön açık değil (ilk ret: hizalanma çemberi ↔ direk_D (-10.3 m)); şerit +20 yön +1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-1.3 m)); şerit -20 yön -1 t=-120: hiçbir yön açık değil (ilk ret: hizalanma çemberi ↔ direk_D (-4.0 m)); şerit -20 yön -1 t=+120: hiçbir yön açık değil (ilk ret: hat ↔ direk_D (-3.6 m))


Çıkış Hold merkezleri (pay 20, uav0 NED): [(-310, -200), (-310, -186), (-310, -171), (-310, -157), (-310, -143), (-310, -129), (-310, -114), (-310, -100), (310, 100), (310, 114), (310, 127), (310, 141), (310, 154), (310, 168), (310, 181), (310, 222)]


## 2. Eski sabit ön-hesap (210° / 150 m) aynı 45 durumda

31 uygun plan. Gerçek çıkış çemberine 60 m'den fazla yaklaşan: **0**. Devralma kapısında takılan (uav0 öngörülen çemberde değil ve yola < 200 m): **16**. Var olmayan çember yüzünden dalış yönü değişen: **10**. Canlı kaynakta uygun olup eskide NO-GO: **11**.


## 4. Akış (sahte araç — MODEL, uçuş değil)

| senaryo | şerit E / yön | uav0 çıkış merkezi (uav0 NED) | uav0 bekleme (sn) | ölçülen merkez hatası (m) | dalış hat yönü (°) | sapma (°) | durma 3B (m) | dip (m) | en küçük 3B ayrım (m) | en küçük direk (m) | toplam (sn) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Z1 batı şeridi, kuzeye uçuş → kuzey çıkış | -70.0 / K | (310, 10) | 37 | 0.0 | 284 | +15 | 19.6 | 13.8 | 310 | 42 | 133 |
| Z2 doğu şeridi, güneye uçuş → güney çıkış | +67.5 / G | (-310, -13) | 37 | 0.0 | 344 | +55 | 19.4 | 13.8 | 148 | 43 | 140 |
| Z3 gerçek QR (−60, −60), E −55 şeridi, kuzeye → kuzey çıkış | -55.0 / K | (310, 25) | 37 | 0.0 | 349 | +80 | 20.0 | 13.8 | 140 | 43 | 154 |
