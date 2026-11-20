# dalis profili kuru test — 2026-11-13

Uçuş YOK. Hedef uav1 NED (-75.0, -45.0). Birim testleri: 66 assert ✔


## bugünkü PX4 (2,0 m/s)

Profil (başlangıçtan bağımsız): istenen 12.0° → **etkin 6.84°** (alçalma 1.8 m/s), hat 250 m = düz giriş 75 + dalış 161.8 m (−19.4 m, 10.8 sn) + durma payı 13.2 m; giriş irtifası 34.4 m; durma 3B 20 m @ 15 m; pull-up dip tahmini 12.3 m (taban 8 m). Açık yönler: 3° (280–282°).

| başlangıç                          | yatay | geliş° | yön° | sapma° | giriş | ara nokta | açı° | giriş irt. | dalış kaybı | 3B dur / irt. | geçiş+çember+hat+tırm. = toplam sn | en küçük açıklık (pay üstü) | uçulabilir |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 çok yakın (50 m K)              | 50 | 180 | 280 | +100 | hizalan | — | 6.84 | 34.4 | 19.4 | 20 / 15 | 17+57+16+26 = 116 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S2 belgeleme koşusu (82 m)         | 81 | 227 | 282 | +55 | hizalan | — | 6.84 | 34.4 | 19.4 | 20 / 15 | 14+53+16+26 = 108 | direk_B +1.0, uav0_bekleme +2.5 | evet |
| S3 gösterim koşusu (110 m)         | 105 | 20 | 280 | -100 | hizalan | (-232, 140) | 6.84 | 34.4 | 19.4 | 20 / 15 | 29+54+16+26 = 124 | uav0_bekleme +1.3, direk_D +3.1 | evet |
| S4 orta (250 m GD)                 | 250 | 315 | 280 | -35 | hizalan | — | 6.84 | 34.4 | 19.4 | 20 / 15 | 15+57+16+26 = 114 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S5 deneme 2 (410 m D)              | 409 | 255 | 280 | +25 | hizalan | — | 6.84 | 34.4 | 19.4 | 20 / 15 | 10+58+16+26 = 110 | direk_D +0.2, uav0_bekleme +0.6 | evet |
| S6 uzak (600 m B)                  | 600 | 90 | 280 | -170 | hizalan | (-76, -330) | 6.84 | 34.4 | 19.4 | 20 / 15 | 57+57+16+26 = 156 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S7 uzak (600 m KD)                 | 600 | 225 | 280 | +55 | hizalan | — | 6.84 | 34.4 | 19.4 | 20 / 15 | 28+57+16+26 = 127 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S8 çok uzak, hat üstünde (1300 m)  | 1300 | 280 | 280 | +0 | dogrudan | — | 6.84 | 34.4 | 19.4 | 20 / 15 | 70+0+16+26 = 112 | uav0_bekleme +1.0, direk_B +3.7 | evet |

### Kinematik benzetim (bugünkü PX4 (2,0 m/s)) — MODEL, uçuş değil

| durum | durma nedeni | 3B dur | irtifa | s | ölçülen açı° | dip | tırmanış sn | çıkış m |
|---|---|---|---|---|---|---|---|---|
| nominal (öne alma 3,5 sn, τ 1,0) | durma-mesafesi (3b) | 19.81 | 14.86 | -13.1 | 6.06 | 14.35 | 25.5 | 382 |
| öne alma YOK | durma-mesafesi (3b) | 19.93 | 19.76 | -2.6 | 4.86 | 19.27 | 24.5 | 368 |
| yavaş tepki τ 2,0 | durma-mesafesi (3b) | 19.81 | 15.12 | -12.8 | 6.18 | 14.09 | 24.4 | 366 |
| hızlı tepki τ 0,5 | durma-mesafesi (3b) | 19.95 | 14.78 | -13.4 | 5.97 | 14.52 | 26.4 | 396 |
| devralma yanal 10 m | durma-mesafesi (3b) | 19.81 | 14.86 | -13.1 | 6.06 | 14.35 | 25.5 | 382 |
| devralma yanal 19 m | durma-mesafesi (3b) | 19.81 | 14.86 | -13.1 | 6.06 | 14.35 | 25.5 | 382 |

## önerilen sim param (4,0 m/s)

Profil (başlangıçtan bağımsız): istenen 12.0° → **etkin 12.0°** (alçalma 3.19 m/s), hat 250 m = düz giriş 75 + dalış 161.8 m (−34.4 m, 10.8 sn) + durma payı 13.2 m; giriş irtifası 49.4 m; durma 3B 20 m @ 15 m; pull-up dip tahmini 10.2 m (taban 8 m). Açık yönler: 3° (280–282°).

| başlangıç                          | yatay | geliş° | yön° | sapma° | giriş | ara nokta | açı° | giriş irt. | dalış kaybı | 3B dur / irt. | geçiş+çember+hat+tırm. = toplam sn | en küçük açıklık (pay üstü) | uçulabilir |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 çok yakın (50 m K)              | 50 | 180 | 280 | +100 | hizalan | — | 12.0 | 49.4 | 34.4 | 20 / 15 | 17+47+16+26 = 106 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S2 belgeleme koşusu (82 m)         | 81 | 227 | 282 | +55 | hizalan | — | 12.0 | 49.4 | 34.4 | 20 / 15 | 14+43+16+26 = 99 | direk_B +1.0, uav0_bekleme +2.5 | evet |
| S3 gösterim koşusu (110 m)         | 105 | 20 | 280 | -100 | hizalan | (-232, 140) | 12.0 | 49.4 | 34.4 | 20 / 15 | 29+44+16+26 = 115 | uav0_bekleme +1.3, direk_D +3.1 | evet |
| S4 orta (250 m GD)                 | 250 | 315 | 280 | -35 | hizalan | — | 12.0 | 49.4 | 34.4 | 20 / 15 | 15+47+16+26 = 104 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S5 deneme 2 (410 m D)              | 409 | 255 | 280 | +25 | hizalan | — | 12.0 | 49.4 | 34.4 | 20 / 15 | 10+48+16+26 = 100 | direk_D +0.2, uav0_bekleme +0.6 | evet |
| S6 uzak (600 m B)                  | 600 | 90 | 280 | -170 | hizalan | (-76, -330) | 12.0 | 49.4 | 34.4 | 20 / 15 | 57+47+16+26 = 146 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S7 uzak (600 m KD)                 | 600 | 225 | 280 | +55 | hizalan | — | 12.0 | 49.4 | 34.4 | 20 / 15 | 28+47+16+26 = 117 | uav0_bekleme +1.0, direk_D +1.7 | evet |
| S8 çok uzak, hat üstünde (1300 m)  | 1300 | 280 | 280 | +0 | dogrudan | — | 12.0 | 49.4 | 34.4 | 20 / 15 | 70+0+16+26 = 112 | uav0_bekleme +1.0, direk_B +3.7 | evet |

### Kinematik benzetim (önerilen sim param (4,0 m/s)) — MODEL, uçuş değil

| durum | durma nedeni | 3B dur | irtifa | s | ölçülen açı° | dip | tırmanış sn | çıkış m |
|---|---|---|---|---|---|---|---|---|
| nominal (öne alma 3,5 sn, τ 1,0) | durma-mesafesi (3b) | 19.83 | 15.39 | -12.5 | 10.4 | 14.73 | 25.4 | 381 |
| öne alma YOK | hedef geçildi (3B tetiklenmedi) | 22.89 | 22.89 | 0.1 | 8.62 | 21.71 | 23.9 | 358 |
| yavaş tepki τ 2,0 | durma-mesafesi (3b) | 19.99 | 15.61 | -12.5 | 10.7 | 13.91 | 24.3 | 364 |
| hızlı tepki τ 0,5 | durma-mesafesi (3b) | 19.93 | 15.52 | -12.5 | 10.12 | 15.26 | 26.2 | 394 |
| devralma yanal 10 m | durma-mesafesi (3b) | 19.83 | 15.39 | -12.5 | 10.4 | 14.73 | 25.4 | 381 |
| devralma yanal 19 m | durma-mesafesi (3b) | 19.83 | 15.39 | -12.5 | 10.4 | 14.73 | 25.4 | 381 |

## Sahte düğümle akış duman testi — MODEL, uçuş değil

| durum | kod | sonuç | hizalama sn | devralma s / x / irtifa | durma nedeni | 3B / irtifa | ölçülen açı° | dip | en küçük irtifa | engel ihlali | toplam sn |
|---||---||---||---||---||---||---||---||---||---||---||---|
| D1 çok yakın (S1), bugünkü PX4 | 0 | tamam | 70 | -270 / +2.4 / 34.5 | durma-mesafesi (3b) | 19.5 / 14.8 | 6.1 | 14.0 | 14.0 | yok | 114 |
| D2 gösterim (S3, ara noktalı), bugünkü PX4 | 0 | tamam | 84 | -270 / +2.3 / 34.4 | durma-mesafesi (3b) | 19.8 / 14.9 | 6.1 | 14.0 | 14.0 | yok | 128 |
| D3 deneme 2 (S5), önerilen sim param | 0 | tamam | 52 | -270 / +2.4 / 38.3 | durma-mesafesi (3b) | 19.8 / 14.8 | 7.3 | 13.8 | 13.8 | yok | 97 |
| D4 çok uzak hat üstü (S8, dogrudan), önerilen | 0 | tamam | 14 | -1339 / -149.9 / 120.0 | durma-mesafesi (3b) | 19.8 / 14.9 | 7.5 | 13.8 | 13.8 | yok | 135 |
| D5 uav0 hedefin üstünde kaldı → kapı geçilmez, NO-GO | 3 | NO-GO: hizalama zaman aşımı (150 sn; engel: uav0 çember dışında ve yola 2 m ( | — | — | — | — | — | — | 34.4 | yok | — |
| D6 uav0 konumu gelmiyor → NO-GO | 3 | NO-GO: hizalama zaman aşımı (150 sn; engel: uav0 konumu yok) | — | — | — | — | — | — | 34.4 | yok | — |
| D7 açık yön yok (uav0 payı 400) → plan NO-GO | 3 | NO-GO: hiçbir yön açık değil (ilk ret: hizalanma çemberi ↔ uav0_bekleme (-60. | — | — | — | — | — | — | 120.0 | yok | — |
