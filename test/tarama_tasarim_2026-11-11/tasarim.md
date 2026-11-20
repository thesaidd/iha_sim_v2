# Alan taraması tasarımı — atlamalı sıralı tek yönlü boustrophedon (2026-11-11, uçuş yok)

Amaç: uav0, hedefin yerini bilmeden arena merkezli 300 × 300 m kutuyu 60 m'de tarar; tarama boyunca
`qr_online.py` çalışır; ilk tespitte tarama durur ve mevcut zincir (röle → uav1 → dalış) sürer.
Kod: `tarama_plan.py` (plan), `alan_tarama.py` (düğüm), `alan_tarama_test.py` (kuru test).
Sayıları yeniden üretmek: `tasarim_hesap.py` (çıktı `tasarim_hesap.txt`), `egik_ozet.py`.

## Özet

- **Tespit şeridi W = 16 m** (60 m irtifa, ±8 m). Statik ölçümde sınır ±8,5 m; 9 m'de hiç tespit yok.
- **Ana bulgu — direkler kutunun İÇİNDE.** Dört direk merkezden 140 m'de, kutu kenarı 150 m'de: K, G, B
  kenardan 10 m, D 29 m içeride; boyları 140 m, tarama irtifası 60 m. Direk dönüş bölgesini değil, düz
  hatların kendisini kapatıyor. Projenin 40 m direk payıyla 300 m'lik yanal eksenin 167 m'si uçulamıyor.
- **Varsayılan plan (direk payı 40 m): 9 hat, geometrik kapsama %44, 441 sn.** Atlamalı sıra aynı hatları
  numara sırasına göre %19 daha kısa uçuyor; atlamasız seyrek tarama %11 kapsıyor.
- **Kapsamayı direk payı belirliyor, sıra değil:** pay 20 m → 16 hat, %76, 766 sn; pay 15 m → %85.
  Payı düşürmek kullanıcı kararı (aşağıda "Açık kararlar").
- 300 m'lik kutu 4·R'den (320–340 m) dar olduğu için bütün geçişler ≥ 2·R olamıyor; olmayanlar "ampul"
  dönüşle (üç yay, hepsi 85 m) çözüldü. Hiçbir dönüş R_min'den sıkı değil.

## Adım 1 — eğik geçiş tespit karakterizasyonu (statik)

Yöntem: `egik_gecis.py`. Sabit `kamera_test` (T_nadir kamerası, 1456 × 1088, dik aşağı, 10 Hz) 60 m'de.
Dört uçuş yönü ψ (0°, 45°, 90°, 135°; görüntü üstü = burun) × 19 yanal kaydırma (−15 … +15 m). Her
(ψ, kaydırma) için kamera hat boyunca −9 … +9 m, 1,5 m adımla taşındı (15 m/s × 10 Hz = bir geçişin kare
aralığı) → o kaydırmada bir düz geçişin göreceği 13 kare. Tespit, uçuşta kullanılan boru hattıyla
(`qr_online.QrIsleyici`: Aruco bul → ROI çöz, kareler sırayla) ve ayrıca tam kare `QRCodeDetector` /
`QRCodeDetectorAruco` ile ölçüldü. Geometri: kodun dört köşesi kadrajda → "tam".
Görüntü yatayı (yarı genişlik 9,41 m) yanal, dikeyi (7,04 m) hat boyu.

Hücre: tam kadraj kare sayısı / bulunan / çözülen (geçiş başına).

### Dokulu zemin (`iha_arena_v2`) — 796 poz, etkin RTF 0,80

| yanal kaydırma (m) | geometri | ψ 0° | ψ 45° | ψ 90° | ψ 135° | en az çözme / geçiş |
|---|---|---|---|---|---|---|
| ±15, ±12,5 | kadraj dışı | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| −10 | yalnız kısmi | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| −9 | yalnız kısmi | 0 / 0 / 0 | 0 / 9 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| −8,5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 9 / 9 / 9 | 8 |
| −8 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 9 / 9 / 7 | 7 |
| −7 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 8 |
| −5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 9 / 9 / 9 | 8 |
| −2,5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 7 | 7 |
| 0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 8 |
| +2,5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +7 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +8 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 8 | 8 |
| +8,5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +9, +10 | yalnız kısmi | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |

- Tam kadrajdaki 396 karenin 396'sı bulundu, 386'sı çözüldü (%97,5). Yöne göre kare başı çözme:
  ψ 0° %100, 45° %97, 90° %98, 135° %95. Yön, geçiş başına sonucu değiştirmiyor (her geçişte ≥ 7/9).
- Kısmi kadrajda çözme 0/212; yanlış içerik 0. −9 m / ψ 45°'te kod bulundu ama çözülmedi (köşe kadraj dışı).
- Tam kare dedektörler daha zayıf: `QRCodeDetector` 357/396, Aruco 249/396 çözme. Uçuş boru hattının ROI
  adımı fark yaratıyor (işlem ort. 17 ms).
- Kareler statik sahnede gürültüsüz; aynı poz yinelense aynı sonucu verir. Kare başı kaçırmalar piksel
  fazına bağlı, rastgele değil.

### Düz zemin (`iha_arena`)

796 poz, etkin RTF 0,82. Sınır aynı (±8,5 m tam, 9 m'de 0); tam kadrajda çözme 389/396 (%98,2; dokulu
%97,5), geçiş başına ≥ 8/9. Yöne göre %100 / 96 / 98 / 99. Zemin dokusu tespit şeridini değiştirmiyor.

| yanal kaydırma (m) | geometri | ψ 0° | ψ 45° | ψ 90° | ψ 135° | en az çözme / geçiş |
|---|---|---|---|---|---|---|
| -15.0 | kadraj dışı | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| -12.5 | kadraj dışı | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| -10.0 | yalnız kısmi | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| -9.0 | yalnız kısmi | 0 / 0 / 0 | 0 / 9 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| -8.5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| -8.0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 9 / 9 / 9 | 8 |
| -7.0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 8 | 9 / 9 / 9 | 8 |
| -5.0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 9 / 9 / 9 | 8 |
| -2.5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 8 | 8 |
| +0.0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +2.5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +5.0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 9 / 9 / 9 | 8 |
| +7.0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 8 | 9 / 9 / 9 | 8 |
| +8.0 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +8.5 | tam kadraj | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 / 9 / 9 | 9 |
| +9.0 | yalnız kısmi | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| +10.0 | yalnız kısmi | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| +12.5 | kadraj dışı | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |
| +15.0 | kadraj dışı | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 |

### Önceki kadraj toleransı testiyle karşılaştırma

| | Uçuş (2026-10-31, ψ 210°, 4 geçiş) | Statik (bu tur, 4 yön) |
|---|---|---|
| Tam kadraj sınırı | ±7,5 m'de 9/9 kare; +9,5 m'de 6 kare, QR erken çıkıyor | ±8,5 m'de 9/9 tam; 9 m'de 0 |
| Kadrajda kalma | 0,8 sn / 9 kare | 9 kare (1,5 m adım) |
| Çözme | 9/9 (tam kare `QRCodeDetector`) | 7–9 / 9 (uçuş boru hattı) |
| Teorik yarı genişlik | 9,41 m | 9,41 − kod yarı köşegeni ≈ 8,7 m |

İkisi tutarlı. Statik test, uçuşta denenmemiş 8–9 m aralığını ve öteki yönleri doldurdu: sınır keskin
(8,5 m var, 9 m yok) ve yönden bağımsız.

### W seçimi

Statik sınır ±8,5 m (17 m). Uçuşta ölçülen hat izleme hatası ≤ 0,55 m, PX4 konum hatası ~1,2 m; 1° yatış
60 m'de görüş alanını 1,05 m kaydırır. **W = 16 m (±8 m)** alındı; hat aralığı ≤ 16 m. Komşu iki hattın
tam ortasındaki hedef her ikisine 8 m'dedir, yani iki geçişte de görülür. Uçuş doğrulamasında kutu
girişindeki yanal hata 1 m'yi aşarsa hat aralığı 14 m'ye çekilmeli (`--hat-araligi 14`).

## Adım 2 — geometri, atlamalı sıra, fizibilite

### Arena (okuma)

- `NAV_LOITER_RAD` 80 m (PX4 varsayılanı; Hold çemberleri bununla ölçüldü). R_min = 80 m.
- Direkler (N, E): K (+140, 0), G (−140, 0), B (0, −140), D (+70, +121,2); 140 m boy.
  Kutu kenarı ±150 m → **K, G, B 10 m, D 28,8 m içeride.**
- Binalar hepsi kutu dışında, en yükseği kule 24 m (N 210, E −80; kuzey dönüş bölgesinin altında):
  60 m'de dikey boşluk 36 m. Binalar plan denetiminde engel sayılmadı (qr_pass'te de sayılmıyor).
- Kalkış koridoru: 0 < E < 300, |N| < 40. Kutunun doğu yarısındaki K–G hatlar bunu keser.
- Kalkış Hold'ları: uav0 ≈ (−30, 290) @100 m, uav1 ≈ (45, 334) @119 m. Tarama 60 m'de: uav1'e dikey
  boşluk 49 m (≥ 40), yatayda en yakın tarama noktası çember diskine 92 m.
- Dokulu zemin ±350 m; dönüşlerin en uzak noktası |N| ≈ 390–410 m (düz zemin, sorun değil).

### Hat yönü

K–G hatlar (ψ0 = 0°; yanal eksen = Doğu) seçildi. D–B hatlarda (90°) doğu ucundaki dönüşler kalkış
koridoruna ve kalkış Hold'larına giriyor; hiçbir direk payında uçulabilir sıra çıkmadı.

### Hat yerleşimi ve k

- Hat uzunluğu 300 m + iki uçta 40 m uzatma (giriş/taşma payı; "taşma ≈ hat aralığı" kuralının 2,5 katı,
  dönüşten çıkışta yanal hatanın oturması için). Dönüşler |N| ≥ 190 m'de.
- k = ceil(2·R_min / d) = ceil(160 / 16) = **10**; 10 m güvenlik payıyla (dönüş yayı 85 m, eşik 170 m)
  **k = 11** (176 m). İleri k / geri k−1 düzeni için 2k = 22 hat gerekir; kutuya 18–19 hat sığıyor
  (300 m < 4·R). Yani ortadaki hatlar hiçbir komşuya ≥ 2R uzak değil — tam atlamalı sıra bu kutuda
  kuramsal olarak da yok.
- Direk yasağı: düz hat direğe < pay geçemez. K ve G aynı yanal konumda (E = 0) ve hattın iki ucunda
  olduğu için o bant iki yönden de kapalı.
- Hatlar serbest yanal aralıklara ≤ 16 m adımla, bant kenarına tam pay mesafesinde yerleştirilir.

| direk payı (m) | kapalı bantlar (E, m) | hat | geometrik kapsama | C | POD Koopman | toplam (m) | süre (sn) | stadyum / ampul | en küçük D (m) |
|---|---|---|---|---|---|---|---|---|---|
| **40 (varsayılan)** | −150…−100, −40…+40, +81…+150 | 9 | %44,4 | 0,48 | %38 | 6622 | 441 | 1 / 7 | 80 |
| 25 | −150…−115, ±25, +96…+146 | 13 | %64,4 | 0,69 | %50 | 9566 | 638 | 3 / 9 | 64 |
| 20 | −150…−120, ±20, +101…+141 | 16 | %76,4 | 0,85 | %57 | 11497 | 766 | 6 / 9 | 40 |
| 15 | −150…−125, ±15, +106…+136 | 17 | %85,0 | 0,91 | %60 | 11847 | 790 | 9 / 7 | 30 |

Hatların direğe en küçük mesafesi her satırda paya eşit; dönüşlerde 52–64 m (≥ 55 m denetimi; hat ucuna
değen ilk 10 m hat payıyla denetlenir).

### Dönüşler

- **Stadyum** (D ≥ 170 m): yay 90° + düz (D − 170) + yay 90°, yarıçap 85 m. Uzunluk 267 + (D − 170) m.
- **Ampul / dış bağlantı** (D < 170 m): dışa α, içe π + 2α, dışa α; üçü de 85 m.
  α = acos((R + D/2) / 2R). Uzunluk R·(π + 4α): D = 165 m'de 322 m, 80 m'de 520 m, 15 m'de ~610 m.
- Yaylar PX4'e standart yol setpoint'i olarak verilir (konum + teğet hız + merkezcil ivme → eğrilik;
  `FixedWingModeManager` kaynak kodundan doğrulandı). Özel güdüm yok.
- Her dönüş 5 m örneklemeyle direk (≥ 55 m), koridor ve alan sınırı için denetlenir; gerekirse hat
  doğrultusunda dışarı ötelenir (bu planlarda öteleme gerekmedi).

### Sıra

Hatların her biri bir kez, yön her hatta değişir. Sıra, toplam dönüş uzunluğunu en küçük yapan
deterministik katmanlı aramayla bulunur (durum = uçulan küme + son hat; 13 hatta kadar tam, üstünde
4000 durumla kırpılır). Başlangıç maliyeti: uçağın konumundan ilk hattın hizalama Hold'una uzaklık.

**Varsayılan plan (pay 40 m)** — giriş Hold merkezi (−340, +40), çember–direk 124 m:

| # | hat E (m) | yön | sonraki hatta D (m) | ≥ 160 | ≥ 170 | bağlantı | uzunluk (m) | dönüş–direk (m) |
|---|---|---|---|---|---|---|---|---|
| 1 | −40,0 | K | 80,0 | hayır | hayır | ampul | 520 | 64 |
| 2 | +40,0 | G | 110,0 | hayır | hayır | ampul | 472 | 64 |
| 3 | −70,0 | K | 137,5 | hayır | hayır | ampul | 417 | 84 |
| 4 | +67,5 | G | 167,5 | evet | hayır | ampul | 309 | 84 |
| 5 | −100,0 | K | 181,2 | evet | evet | stadyum | 278 | 95 |
| 6 | +81,2 | G | 166,2 | evet | hayır | ampul | 318 | 95 |
| 7 | −85,0 | K | 138,7 | hayır | hayır | ampul | 414 | 73 |
| 8 | +53,7 | G | 108,7 | hayır | hayır | ampul | 474 | 73 |
| 9 | −55,0 | K | — | | | | | |

Bu planda uçulabilir hatlar iki dar kümede (E −100…−40 ve +40…+81); kümeler arası en büyük mesafe
181 m. Bu yüzden 8 geçişin yalnız 1'i paylı eşiği (170 m), 3'ü çıplak 2·R_min'i (160 m) sağlıyor;
kalanı ampul. Ders kitabı sırası (0, k, 1, k+1, …) bu hat kümesinde hiçbir geçişi sağlamıyor (8/8 < 170).
Gerçek QR (−60, −60) E −55 hattının şeridinde (5 m) ve o hat sırada 9/9 — tespit ancak taramanın sonunda.

**Pay 20 m planı** (16 hat) ayrıntısı `tasarim_hesap.txt` içinde: 15 geçişin 11'i ≥ 160 m, 6'sı ≥ 170 m;
desen +47 → −120 → +61 → −106 → … (ileri ~180 / geri ~166 m), sonda ortadaki hatlar ampulle.

### Karşılaştırma: atlamalı / sıralı / atlamasız

| yöntem | direk payı | hat | hat aralığı (m) | geometrik kapsama | C | POD Koopman 1−e^−C | hat (m) | dönüş (m) | toplam (m) | süre (sn) |
|---|---|---|---|---|---|---|---|---|---|---|
| **Atlamalı** (dar aralık, atlamalı sıra) | 40 | 9 | 13,7–15 (+ bantlar) | %44,4 | 0,48 | %38 | 3420 | 3202 | 6622 | 441 |
| Sıralı dar (aynı hatlar, numara sırası) | 40 | 9 | aynı | %44,4 | 0,48 | %38 | 3420 | 4763 | 8183 | 546 |
| Atlamasız seyrek (aralık 2R = 170 m) | 40 | 2 | 170 | %10,7 | 0,11 | %10 | 760 | 267 | 1027 | 68 |
| **Atlamalı** | 20 | 16 | 13,5–14,3 (+ bantlar) | %76,4 | 0,85 | %57 | 6080 | 5417 | 11497 | 766 |
| Sıralı dar | 20 | 16 | aynı | %76,4 | 0,85 | %57 | 6080 | 9037 | 15117 | 1008 |
| Atlamasız seyrek | 20 | 2 | 170 | %10,7 | 0,11 | %10 | 760 | 267 | 1027 | 68 |

- C = W · (kutu içi hat uzunluğu) / alan. Geometrik kapsama = şeritlerin birleşimi / kutu genişliği;
  hatlar W'den sık olduğu yerde C biraz daha büyük.
- Koopman formülü rastgele arama varsayar ve alt sınırdır. Paralel hat + keskin sınırlı sensörde (Adım 1:
  şerit içinde geçiş başına tespit ≈ 1, dışında 0) beklenen tespit olasılığı geometrik kapsamaya eşittir:
  atlamalı %44 / %76, seyrek %11.
- Atlamalı sıra aynı kapsamayı sıralı dardan %19 (pay 40) / %24 (pay 20) kısa yoldan veriyor. Fark, bu
  kutuda kitaptaki kadar büyük değil çünkü kutu 4R'den dar ve direk bantları hatları kümeliyor.
- Atlamasız seyrek tarama 7–11 kat kısa ama hedefi ~%11 olasılıkla buluyor.

### Giriş, çıkış, öteki araç

- **Giriş:** `qr_pass.transit()` ile ilk hattın hizalama Hold'una (hattın 80 m sağı, hat başının 150 m
  gerisi); rota hat yönüne döndüğünde OFFBOARD (qr_pass deseni).
- **Çıkış (tespit ya da bitiş):** uçulan hat bitirilir, 60–100 m daha düz uçulur, Hold'a geçilir, sonra
  açık Hold merkezine DO_REPOSITION. Her hat/yön için çıkış çemberleri (80 + 25 m) direklerden ≥ 40 m,
  koridor dışı — plan aşamasında denetli. Arena içinde, direklerin arasında Hold'a girilmez.
- **Öteki araç (`ayrim.py`):** her hat ve dönüş, uçulmadan önce canlı denetlenir (en kötü durum 3B ≥ 40 m).
  Uygun olmayan hat ertelenir, uygun olan öne alınır; hiçbiri uygun değilse tarama kesilir ve çıkış
  Hold'una gidilir. Tarama sırasında gerçekleşen ayrım 40 m'nin altına inerse uçulan hat bitince çıkılır.
- **Koridor:** doğu hatları koridoru keser; yalnız öteki araç havadaysa (ya da hiç yoksa) uçulur.

## Adım 3 — düğüm ve kuru test

`alan_tarama_test.py`: 43 doğrulama geçti (`kuru_test.log`). Sahte uçak kinematik bir modeldir; PX4'ün
gerçek izleme hatasını temsil etmez.

- Geometri: 400 rastgele dönüş kapanıyor ve sürekli; yay setpoint'i merkezcil.
- Plan (pay 40 ve 20): her hat bir kez, direk/koridor/çıkış denetimleri, yarıçap ≥ R_min.
- Uçtan uca tespitsiz: 9 hat sırayla, izden ölçülen kapsama %42,4 (plan %44,4), çıkış Hold'u.
- Tespit: gerçek QR 9/9. hatta (442 sn); (−60, −68)'deki hedef 3. hatta (138 sn), kalan 6 hat açılmadı,
  hat bitirilip kutu dışında Hold. Direk bandındaki hedef (0, 0) → "bulunamadi".
- Ayrım: uav1 kalkış Hold'unda → erteleme yok; doğu hatlarının üstünde 65 m'de → doğu hatları ertelendi,
  batı hatları uçuldu, "kesildi"; yeri belirsiz ve aynı irtifada → tarama başlamadı (kod 3).
- Koridor (uav1 yerde), OFFBOARD kaybı (kod 2, akış kesildi), OFFBOARD reddi, taşıma reddi (kod 3).

## Açık kararlar ve riskler

1. **Direk payı / kapsama (kullanıcı kararı).** 40 m ile %44. Seçenekler: (a) %44'ü kabul et;
   (b) ilk uçuşta kutu girişindeki yanal hatayı ölç, küçükse düz hat payını 20 m'ye indir (%76) —
   `--hat-payi 20`; (c) direkleri üretici betikle kutunun dışına (r ≥ 200 m) taşıyan yeni bir dünya
   (kapsama ~%100, tam atlamalı sıraya yakın). Kod (a) ile varsayılan; (b) ve (c) parametre/dünya değişimi.
2. **Yay izleme uçuşta ölçülmedi.** Offboard'da eğrilikli yol setpoint'i bu projede yalnız orbit
   düğümünde (R 100 m, +6 m dışa kayma) kullanıldı. 85 m yayda ve dönüşten çıkıştaki 40 m uzatmada yanal
   hata bilinmiyor; ilk uçuşun ölçeceği şey bu.
3. **`lora_yaklasma.py` uav0'ın yerini sabit varsayıyor** (`--uav0-gecis-yonu 210 --uav0-gecis-bitis`).
   Taramada uav0 tespitten sonra hat doğrultusunda (0° / 180°) kutu dışındaki çıkış Hold'una gider; bu
   varsayım tutmaz. Zincir uçuşundan önce yaklaşma düğümü uav0'ı canlı izlemeli (`ayrim.py`) ya da
   tarama düğümünün yazdığı çıkış Hold'unu almalı.
4. **Süre.** 441–790 sn tarama + kalkış/taşıma. Uzun uçuşta görülen ivmeölçer ofseti uyarısı (15–25 dk)
   pay 15–20 planlarında sınıra yaklaşır.
5. Kule (24 m) kuzey dönüş bölgesinin altında: dikey boşluk 36 m.

## Uçuş ölçümü ve direk payı kararı (2026-11-11 21:16–21:31, tek uçuş, abort yok)

Veri: `test/zincir_genel_2026-11-11/ucus/` (`k1_*` ölçüm dilimi, `k2_*` tespitli dilim + zincir, `iz_iki_arac.csv`).
Yanal hata PX4 kestirimine göredir (ayrıca ~1,2 m konum hatası var).

**Koşu 1 — 4 hat, direk payı 40 m, giriş payı (uzatma) 40 m:** +40↑ → −100↓ → +81↑ → −40↓; bir stadyum (D 181 m), iki ampul (D 140 ve 121 m).

| hat | önceki dönüş | dönüş çıkışında yanal (m) | uzatmada en büyük (m) | kutu girişinde (m) | kutu içinde en büyük (m) | direğe en yakın (m; plan 40) |
|---|---|---|---|---|---|---|
| +40 ↑ | Hold'dan hizalama | +2,3 (devralma) | 3,4 | −0,8 | 1,2 | 39,4 |
| −100 ↓ | ampul D 140 | +5,5 | 5,4 | −0,6 | 2,7 | 37,5 |
| +81 ↑ | stadyum D 181 | +13,5 | 13,5 | +3,0 | 3,0 | 39,2 |
| −40 ↓ | ampul D 121 | −7,2 | 9,8 | −5,6 | 5,6 | 35,6 |

- **Yaylar:** 85 m komut edilen yayda yanal hata ortalama 11,4 m, en çok 19,1 m (üç dönüşte 15–19 m). Dönüş–direk en yakın 60 m (plan 64 m).
- **40 m giriş payı yetmiyor:** uçak hatta 6–14 m hatayla giriyor, kutu kenarına kadar oturmuyor.

**Koşu 2 — 2 hat, direk payı 40 m, giriş payı 100 m:** +67↑ → (ampul D 122) → −55↓ (gerçek QR şeridi).

| hat | uzatmada en büyük (m) | kutu girişinde (m) | kutu içinde ort ± std / en büyük (m) | direğe en yakın (m) |
|---|---|---|---|---|
| +67 ↑ | 3,6 | 0,0 | +0,06 ± 0,26 / 0,41 | 54,0 |
| −55 ↓ | 7,8 | −0,1 | +0,48 ± 0,33 / 0,87 | 55,0 |

**Karar: düz hat direk payı 40 m'de KALDI.** Gerekçe:
1. Kutu girişinde ölçülen hata 0,6–5,6 m (40 m giriş payıyla); bu 20 m'lik payın dörtte birinden fazla ve
   örnek yalnız 4 hat. Önceden koyduğum ölçüt (giriş ve kutu içi ≤ 3 m) sağlanmadı.
2. K, G, B direkleri kutu kenarından yalnız 10 m içeride, yani hatanın en büyük olduğu yerde duruyor.
3. 100 m giriş payıyla hata 0,9 m'nin altına indi, ama bu yalnız 2 hatta ve direklerden 54 m uzakta ölçüldü.

**Yapılan değişiklik:** varsayılan giriş payı 40 → **100 m** (`--uzatma`). Varsayılan plan aynı 9 hat ve %44 kapsama;
yol 6622 → 7702 m, süre 441 → 513 sn. Dönüş alanı sınırı 450 → 520 m.

**Payı 20 m'ye indirmek için gereken:** 100 m giriş payıyla en az bir tam tarama (9 hat) uçup kutu girişindeki
hatanın her hatta ≤ 1 m kaldığını görmek. O zaman 20 m pay, direkte ≥ 17 m gerçek boşluk bırakır.

**Tespit şeridi (W) için sonuç:** 100 m giriş payıyla kutu içi hata ≤ 0,9 m → 16 m hat aralığı geçerli.
Gerçek QR, 5,0 m yanal kaydırmayla geçilen şeritte 10 karede bulundu ve çözüldü (10 yayın, röle 10/10).
