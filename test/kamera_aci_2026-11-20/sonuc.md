# Kamera geometrisi + alan taraması (çim biçme) iyileştirmesi — 2026-11-20

Kapsam: uav0 keşif kamerası (RPi GS IMX296 + 6 mm, aşağı bakan, 10 Hz) ve `alan_tarama.py` R = 100 m daire taraması
(100 m irtifa, 2 m ArUco id 0 @ (−60, −60)). Tüm uçuşlar Gazebo SITL'de bu oturumda yapıldı; kapsama ve mozaik **Gazebo
gerçek pozuyla** (gz_gercek_poz.py) ölçüldü, plan değerleriyle değil. uav1 uzak üste yerde (zincir.sh spawn'ı), dalış yok.

## 1. Kamera geometrisi (teori: `kamera_geometri.py`, bağımsız alt ajan hesabıyla çapraz denetlendi)

| 100 m, nadir | değer |
|---|---|
| Ayak izi | 83,7 m (yanal) × 62,6 m (boyuna); HFOV 45,4°, VFOV 34,7° |
| GSD | 5,75 cm/px, **görüntü boyunca sabit** (nadir pinhole + düz zemin; cos² açısal GSD'dir, piksel değil) |
| Marker | 34,8 px kenar (her yerde), hücre 5,8 px |
| Ön örtüşme (10 Hz, 14 m/s) | %97,8; bir zemin noktası ~44 karede; %80 ön örtüşme için kare aralığı 0,89 sn |
| Tespit şeridi W (marker her yaw, kenar payı 5 px) | roll ±0° 80,3 m · ±1° 76,2 · **±2° 72,2** · ±3° 68,3 (her 1° roll ≈ 4 m) |
| Yan örtüşmeye göre hat aralığı (roll ±2°) | %30 → 52,9 m · %60 → 30,3 m · %70 → 22,7 m |

Ölçülen roll (z1 + z3 düz hatlar, gerçek poz): p50 0,3°, p95 0,6–0,7°, p99 1,3–1,5° (rüzgârsız) → plan payı ±2°.
Pitch düz uçuşta sabit +1,6° (trim): ayak izini 2,8 m öne kaydırır, yanal şeridi etkilemez.

### Montaj açısı — Gazebo statik doğrulaması (`gz_models_cam_hw/aruco_aci_tarama.py`, 199 poz, `statik/`)

Sabit test kamerası (aynı 6 mm blok + gs_kamera Bayer + bapp MarkerDetector), 100 m, roll/pitch 0:

| yan eğim | ölçülen güvenilir bant | teori (kenar 15 px) | not |
|---|---|---|---|
| 0° (nadir) | −39,4 … +39,9 m → **79,2 m** | 79,8 m | teori ≈ ölçüm |
| 10° | −20,6 … +62,3 → 82,8 m | 82,8 m | +%4,5 şerit; uzak kenarda marker 28 px, 1° roll hatası 2,0 → 2,35 m |
| 20° | −2,7 … +53,3 → **56,0 m** | 92,8 m | uzak tarafta (≤ 27 px) tespit kesik kesik (ara tespit +90 m'ye kadar) |
| 30° | +14,8 … +58,8 → **44,0 m** | 77,5 m | aynı; eğik + basık marker güvenilmez |
| 0°, marker 45° dönük | 80,2 m | 80,2 m (kenar 5 px) | |

İrtifa (nadir): 160 m'de 20,3 px ✔ 6/6, 180 m'de 19,3 px ✘ 0/6 (gürültüsüz sim — gerçek alt sınır daha yüksek).
Kenar payı: köşe kadraj kenarına 13–18 px'ten yakınken tespit kararsız (5,6 px'te tespit, 18,3 px'te tespitsiz poz var)
→ planlamada **kenar payı 15 px**. Köşe RMS (gerçek izdüşüme) 1,06 px, ölçülen/beklenen kenar 0,973 (bilinen içe bias).

**Sonuç: en uygun montaj nadir (0°).** Teoride yan eğim şeridi ~24°'ye kadar büyütüyor (alt ajan: 89,7 m @ 20 px eşiği)
ama Gazebo'da 20–30°'de uzak taraf güvenilmez, şerit nadirden **dar**; 10° yalnız +%4,5 kazandırıyor, karşılığında
GSD anizotropik (mozaik için kötü) ve attitude hatasının konum etkisi büyüyor. İleri eğim de yanal şeridi ancak uzak, kaba
çözünürlüklü bölgede genişletiyor. Montaj değiştirilmedi.

## 2. Tarama — eski ve yeni (gerçek pozla ölçülen kapsama; `kapsama_analiz.py`)

| | B0 taban (z3 ayarı) | **B1 yeni** | B2 haritalama (%60 yan) |
|---|---|---|---|
| Argümanlar | `--serit 70 --hat-araligi 70 --sira cim` (uzatma 100) | `--serit-kamera … --kenar-px 15 --uzatma 60` (sıra: en kısa) | B1 + `--yan-ortusme 0.6` |
| Plan | 3 hat (+59 → 0 → −59), 2300 m | 3 hat (0 → −59 → +59), **1967 m** | 5 hat, 3660 m |
| Tarama süresi (sim, devralma → son hat) | 185,3 sn | **160,8 sn (−%13,2)** | 289,9 sn |
| Duvar saati (RTF ~0,85) | 212 sn | 188 sn | 333 sn |
| Alan üstünde geçen süre payı | %33,3 | **%39,4** | %34,6 |
| Tespit kapsaması (gerçek) | %100 | %99,92 (24 m² kenar şeridi) | %99,94 (19 m²) |
| Görüntü kapsaması | %100 | %100 | %100 |
| ≥ 2 geçişte görülen alan (tekrar) | %28,0 | %32,4 | %83 (≥ 3: %45) |
| Ölçülen yan örtüşme (görüntü) | %29,8 | %29,5–29,9 | %64,6–64,8 |
| Marker: geometriye göre görülebilir / tespit | 40 / 40 | 38 / 38 | 41 / 41 |
| Son LoRa raporu hatası | 0,19 m | 0,42 m | 1,36 m |
| Gerçek pozla ray-cast (kamera + tespit) | 0,05 m | 0,05 m | 0,05 m |

- **Konum hatası farkı taramadan değil, PX4 poz kestiriminden** (koşudan koşuya EKF yaw/roll: B0 0,51°, B1 0,07°, B2 0,79°
  yaw); gerçek pozla ray-cast üçünde de 0,05 m.
- **Atlanan bölge**: yalnız daire kenarında (−70, −70) / (70, 70) ince şerit. Neden geometrik: direk payı 40 m dış hatları
  |c| ≤ 59 m'ye sabitliyor (135°'de direkler c = ±99), gerçek yarı şerit ~39,6 m → 98,6 m < 100 m. Plan da %98,5'i garanti
  ediyordu; B0'da dış hat ~1 m dışarıdan geçtiği için şerit kapanmıştı. Direk payı güvenlik kuralı, değiştirilmedi.
- **Gereksiz tekrar**: B0/B1'de alanın ~%30'u iki geçişte (3 hat zorunlu: 2 hat 200 m çapı örtemez); hat boyunca her nokta
  ~40 karede (10 Hz) — tespit füzyonu için yararlı, mozaik için 1 sn'de bir kare yeter.
- Dönüş sonrası hatta giriş (z1, z3 izleri): ampul dönüşten çıkışta yanal hata 12,9 m → 40 m'de 4,5, 50 m'de 2,4, **60 m'de
  ≤ 1,0 m**, 70 m'de 0,2 m. 100 m uzatmanın ~40 m'si gereksizdi → daire modunda `--uzatma 60` (dış hatlarda kirişe kadar
  +20 m daha). Kutu modu varsayılanı (100 m, 2026-11-11 ölçümü) değişmedi.

## 3. Görüntü birleştirme (`mozaik.py`)

Doğrudan jeoreferanslı mozaik (gerçek poz) dikişsiz: dünya ızgara çizgileri ve arena halkası şeritler arasında hizalı
(`*/mozaik/mozaik.jpg`). Özellik tabanlı eşleme (ORB + RANSAC, zemine çarpıtılmış örtüşme):

| | ardışık kareler (1 sn, ~%78 ön) | komşu geçişler (yan) |
|---|---|---|
| B0 (%30 yan) | 18/18 başarılı, kayma medyan 0,01 m | **0/12** |
| B1 (%30 yan) | 18/18, 0,01 m | **1/12** |
| B2 (%65 yan) | 17/18, 0,01 m | **13/24**, kayma medyan 0,10 m (en çok 0,61) |

→ %30 yan örtüşme hedef araması için yeterli, **özellik tabanlı mozaik için yetersiz**; haritalama gerekiyorsa
`--yan-ortusme 0.6` (5 hat, +%86 yol). Uyarı: sim zemini düşük frekanslı, az özellikli doku — gerçek arazide eşleşme
büyük olasılıkla daha kolay; uçakta poz PX4 kestirimi olacağından doğrudan jeoreferansta 0,4–1,2 m dikiş kayması beklenir.

## 4. Değişen / eklenen dosyalar (iha_sim_v2)

- `kamera_geometri.py` (yeni): `Kamera`, `R_body_cam`, `zemine`, `goruntuye`, `ayak_izi`, `gsd`, `marker_durumu`,
  `tespit_seridi`, `goruntu_seridi`, `hat_araligi`, `on_ortusme`, `konum_duyarliligi`, `aci_tarama`, `ozet`; saf numpy.
  Test: `kamera_geometri_test.py` (20 ✔).
- `alan_tarama.py`: `serit_kameradan()` + `--serit-kamera/--roll-pay/--pitch-pay/--kenar-px/--min-kenar-px/--yan-ortusme`;
  `param_olustur(args, kayit)`; `plan_kur` varsayılan sıra en kısa (`atlamali`; çim sırası da aranan sıralardan biri,
  `--sira cim` duruyor); `--tam-tarama` (`Yurutucu(tam_tarama=…)`, `_hat_bitti`, `tespit_bildir`); `Yurutucu.adim`
  daire modunda hat girişini daireye girişte ölçer (`tp.alan_icinde`); iz CSV'ye `t_duvar` sütunu.
  `alan_tarama_test.py` 43 → 47 ✔.
- `kapsama_analiz.py` (yeni): gerçek pozla kapsama / boşluk / tekrar / yan örtüşme / süre, `--tarama-log` penceresi.
- `mozaik.py` (yeni): ortomozaik + ORB birleştirme sınaması. `kare_kaydedici.py` (yeni): seyrek kare kaydı.
- `tarama_video.py` (yeni): kamera + harita tekrar videosu. `gz_models_cam_hw/kamera_test_W/`, `aruco_aci_tarama.py` (yeni).
- `test/kamera_aci_2026-11-20/tarama.sh`: yalnız keşif taraması senaryosu (kayıt + değerlendirme).

## 5. Yeniden üretim

```
cd ~/sim/iha_sim_v2/test/kamera_aci_2026-11-20
KARE_ARALIK=0.5 TARAMA_ARGS="--alt 100 --serit-kamera kalibrasyon/w_nadir_10_sim.json --kenar-px 15 --daire 100 \
  --uzatma 60 --icerik ARUCO-0" bash tarama.sh b1_yeni           # ~5 dk; GOR=1 ile pencereli, sim açık kalır
~/bapp/.venv-ros/bin/python ~/sim/iha_sim_v2/tarama_video.py --kosu b1_yeni --fps 8
```

## Açık / denenmedi

- Dönüş yarıçapı: `--r-min/--r-donus 60` planı 1967 → 1543 m kısaltır ama NAV_LOITER_RAD (80 m) kuralının altında; uçuşta
  izleme hatası ölçülmedi → uygulanmadı.
- Rüzgâr yok: gerçek uçuşta roll p99 ölçülüp `--roll-pay` güncellenmeli (her 1° ≈ 4 m şerit).
- Tek koşu / yapılandırma; konum hatası koşular arasında PX4 kestirimiyle 0,2–1,4 m oynuyor.
- Gerçek GS kamerada kenar payı ve tespit alt sınırı (sim: ~20 px) ayrıca ölçülmeli.

## Ek A — Bağımsız geometri hesabı (alt ajan çapraz denetimi)

Proje dosyalarını okumadan, ilk ilkelerden ayrı bir betikle yapıldı (`ajan_capraz/hesap.py`, eğim × eşik taraması
`ajan_capraz/ek.py`; saf numpy, tam koşu ~6 dk). Amaç `kamera_geometri.py`'yi bağımsız bir uygulamayla sınamak.

**Model:** pinhole, f = 6,0 mm / 3,45 µm = 1739,13 px, cx 728, cy 544, distorsiyon yok; nadir montaj (görüntü üstü = burun).
Tespit ölçütü: 4 köşe kenarlardan ≥ 5 px içeride ve en kısa kenar ≥ 20 px. W = marker yaw 0–90° (5° adım) × roll
{−m, 0, +m} (× pitch) üzerinden kesişim; kenarlar ikiye bölmeyle 5 mm'ye. Nadir kapalı biçim denetimi:
W = 2·[h·tan(atan(723/f) − m) − √2] (√2 m = 45°'deki marker yarı köşegeni; 20 px kuralı nadirde bağlamıyor).

### A.1 Ayak izi (seviye uçuş)

| h | yanal | boyuna | GSD nadir | GSD yanal kenar (radyal / teğet) | marker nadir / kenar |
|---|---|---|---|---|---|
| 80 m | 66,98 m | 50,05 m | 4,60 cm/px | 4,60 / 4,60 | 43,5 / 43,5 px |
| 100 m | 83,72 m | 62,56 m | 5,75 cm/px | 5,75 / 5,75 | 34,8 / 34,8 px |
| 120 m | 100,46 m | 75,07 m | 6,90 cm/px | 6,90 / 6,90 | 29,0 / 29,0 px |

Not: nadir pinhole + düz zeminde görüntü düzlemi zemine paraleldir → piksel başına GSD sabittir. 100 m'de kenar ışınında
(θ 22,7°, eğik menzil 108,4 m) açısal GSD 6,23 / 6,76 cm olur ama piksel aralığının 1/cos θ çarpanı bunu söndürür.

### A.2 Garantili tespit şeridi W [m] (marker her yaw, roll ±m)

| h | m = 0 | 1° | 2° | 3° | roll ±2 + pitch ±2 | ayak izi |
|---|---|---|---|---|---|---|
| 80 | 63,69 | 60,43 | 57,23 | 54,06 | 57,23 | 66,98 |
| 100 | 80,31 | 76,25 | **72,24** | 68,28 | 72,24 | 83,72 |
| 120 | 96,95 | 92,06 | 87,25 | 82,51 | 87,25 | 100,46 |

Pitch ayak izini yalnız boyuna kaydırır, W'yi azaltmaz. `kamera_geometri.py` (adım 0,25 m): 72,0 m — uyumlu.

### A.3 Hat boyunca (14 m/s, 10 Hz, kare başına 1,40 m)

| h | noktayı gören kare | marker tam tespit edilebilir (yaw 45 / 0) | 10 Hz ön örtüşme | %80 ön için tetik | %70 için |
|---|---|---|---|---|---|
| 80 | 35,7 | 33,4 / 34,0 | %97,2 | 0,715 sn (10,0 m) | 1,07 sn |
| 100 | 44,7 | 42,3 / 42,8 | %97,8 | 0,894 sn (12,5 m) | 1,34 sn |
| 120 | 53,6 | 51,1 / 51,7 | %98,1 | 1,07 sn (15,0 m) | 1,61 sn |

### A.4 Yan örtüşmeye göre hat aralığı (100 m, aralık = (1 − örtüşme) × 83,72)

| %30 | %60 | %70 |
|---|---|---|
| 58,6 m | 33,5 m | 25,1 m |

(`kamera_geometri.hat_araligi` aynı oranları roll ±2° altında her zaman görülen 75,6 m'lik görüntü şeridine uygular:
52,9 / 30,3 / 22,7 m — bilinçli olarak daha temkinli.)

### A.5 Yan eğim t (100 m, roll ±2°)

| t | ayak izi yakın / uzak | bant alt / üst | W | en küçük kenar | en kötü GSD (radyal/teğet, cm) | θ uzak | 1° roll / pitch / yaw hatası |
|---|---|---|---|---|---|---|---|
| 0° | −41,9 / 41,9 | −36,1 / 36,1 | 72,2 | 34,0 px | 5,9 / 5,8 | 19,9° | 1,99 / 1,75 / 0,63 m |
| 5° | −31,9 / 52,5 | −26,5 / 46,4 | 72,9 | 31,6 | 6,3 / 6,0 | 24,9° | 2,14 / 1,75 / 0,81 |
| 10° | −22,6 / 64,2 | −17,3 / 57,7 | 74,9 | 28,9 | 6,9 / 6,3 | 30,0° | 2,35 / 1,75 / 1,01 |
| 15° | −13,5 / 77,3 | −8,3 / 70,1 | 78,5 | 25,9 | 7,8 / 6,7 | 35,0° | 2,64 / 1,75 / 1,22 |
| 20° | −4,7 / 92,3 | 0,4 / 84,2 | 83,8 | 22,7 | 8,9 / 7,1 | 40,1° | 3,03 / 1,75 / 1,47 |
| 25° | 4,0 / 110,0 | 9,2 / 96,5 | 87,3 | 20,4 | 10,3 / 7,6 | 44,0° | 3,43 / 1,75 / 1,68 |
| 30° | 12,8 / 131,3 | 18,0 / 91,5 | 73,4 | 20,2 | 10,3 / 7,7 | 42,4° | 3,26 / 1,75 / 1,60 |
| 35° | 21,8 / 158,3 | 27,1 / 89,4 | 62,3 | 20,1 | 10,5 / 7,7 | 41,8° | 3,19 / 1,75 / 1,56 |

- Teorik en büyük W ≈ t 24°'de 89,7 m (+%24); t ≥ 25°'te uzak kenarı 20 px kuralı keser.
- Sonuç eşiğe çok duyarlı: eşik 25 px olunca W = 72,2 / 78,5 / 68,3 / 58,2 / 56,0 m (t 0 / 15 / 20 / 24 / 25°), optimum ~15°.

### A.6 İleri eğim t (100 m)

| t | W (roll ±2) | W (roll ±2 + pitch ±2) | ayak izi arka / ön genişlik | merkez öne kayma | merkezde GSD (u/v) | bantta en küçük kenar | en kötü GSD (u/v) |
|---|---|---|---|---|---|---|---|
| 0° | 72,2 | 72,2 | 83,7 / 83,7 | 0 m | 5,75 / 5,75 | 34,0 px | 6,0 / 6,0 |
| 10° | 78,0 | 76,7 | 80,6 / 90,0 | 17,6 m | 5,84 / 5,93 | 31,2 | 6,5 / 7,0 |
| 20° | 87,9 | 85,7 | 80,0 / 100,5 | 36,4 m | 6,12 / 6,51 | 25,4 | 7,3 / 9,0 |
| 30° | 97,7 | 97,2 | 81,9 / 118,0 | 57,7 m | 6,64 / 7,67 | 20,1 | 8,7 / 12,7 |

Yamuk ayak izi öne doğru genişliyor; ek genişlik yalnız uzak, kaba çözünürlüklü (boyuna 12,7 cm'ye kadar) bölgede ve
pitch hatası uzak kenarda konum hatasını büyütüyor.

### A.7 Alt ajanın önerisi ve bu oturumdaki ölçümle karşılaştırma

Ajan önerisi: arama/geo-lokalizasyon için 0–10°, ortomozaik için 0° (nadir); şerit genişliği eğimle değil hat aralığıyla
ayarlanmalı (arama ≤ 72 m, %60–70 yan örtüşme için 25–34 m).

**Gazebo ile çelişki ve çözümü:** teori 20–25° yan eğimde W'nin büyüdüğünü söylüyor (20 px eşiğiyle). Statik Gazebo
ölçümünde (bölüm 1) ise 20°'de güvenilir bant 56 m, 30°'de 44 m — uzak tarafta marker 24–27 px'te bile tespit kesik
kesik. Yani eğik bakışta gerçek eşik "en kısa kenar ≥ 20 px"ten sıkı (basıklık + eğik perspektif + Bayer). Nadirde ise
teori ile ölçüm örtüşüyor (79,8 ↔ 79,2 m). İki kaynak aynı karara varıyor: **nadir montaj**; ajanın "eşiğe duyarlı"
uyarısı ölçümle doğrulandı.
