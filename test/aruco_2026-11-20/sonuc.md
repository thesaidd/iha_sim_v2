# ArUco geo-lokalizasyonu + LoRa + uav1 zinciri — Ubuntu 24.04 / Jazzy / PX4 v1.17.0 (2026-11-20)

bapp (`~/bapp`, `bap_geo` + `bap_ros/aruco_geolocator_node.py`) bu simülasyonda gerçek ürün
konfigürasyonuyla denendi. Ürünler: RPi Global Shutter renkli IMX296 (1456×1088, 3,45 µm) + 6 mm CS lens
(45,4°×34,7°), aşağı bakan sabit montaj; 2×2 m DICT_4X4_50 id 0 (+1 modül beyaz boşluk); arama irtifası
100 m; LoRa 24 bayt `TargetReport`.

## Ortam (bu depoda yeni / değişen)

| Parça | Dosya |
|---|---|
| 6 mm aşağı bakan kamera (PX4 v1.17 `advanced_plane` + kamera bloğu, 10 Hz, Gauss σ 0,012) | `gz_models_cam_hw/W_nadir_10` |
| ArUco dünyası (2 m marker, (−60, −60), dokulu zemin, göreli doku yolları) | `gz_worlds/make_aruco_hedef.py`, `make_iha_arena.py --zemin dokulu --hedef aruco` → `iha_arena_aruco.sdf` |
| Renkli sensör taklidi (RGGB mozaik + OpenCV bilineer demozaik, damga değişmez; ~9 ms/kare) | `gs_kamera.py`, `gs_kamera_test.py` |
| Sim kamerasının ChArUco kalibrasyonu (bapp aracıyla; RMS 0,066 / test 0,051 px) | `kalibrasyon/` (README) |
| LoRa paket modu: ToA (Semtech AN1200.13; SF7/125 kHz/4:5, 24+4 B = 66,8 ms), duty %10, yarı çift yönlü, kayıp %3 | `lora_relay.py --mod paket` |
| uav1: rapordaki enlem/boylam → **kendi** EKF origin'i (spawn farkı hedef zincirinde kullanılmaz) | `lora_yaklasma.py --girdi paket` |
| Başlatma: `CAM=W10 ARUCO=1 [YAKLASMA=1]`, Jazzy yolları, `SIM_DIR`/`PX4_DIR`/`BAPP` | `launch_all.sh` |
| Gerçek poz kaydı + değerlendirme (hata ayrıştırması) | `gz_gercek_poz.py`, `aruco_degerlendir.py`, `zincir_degerlendir.py` |
| Uçtan uca senaryo | `test/aruco_2026-11-20/zincir.sh` |
| PX4 yaması (yalnız sim modülü `gz_bridge`) | `px4_yamalar/gz_bridge_magnetometre.patch` |

Kurulum (24.04): PX4 `v1.17.0` (`Tools/setup/ubuntu.sh --no-nuttx`, `make px4_sitl`) + yama
(`cd ~/PX4-Autopilot && git apply ~/sim/iha_sim_v2/px4_yamalar/gz_bridge_magnetometre.patch`), ROS 2 Jazzy +
`ros-jazzy-ros-gz-bridge`, Micro-XRCE-DDS-Agent v2.4.3, `px4_msgs` release/1.17 içine PX4'ün `msg/` + `msg/versioned/`
kopyası (birebir eşleşme → `px4_durum.py` ham CDR çözümüne gerek kalmadı), bapp `.venv-ros` (`--system-site-packages`
+ opencv-contrib-python 5.0).

## Simülatörde bulunan ve düzeltilen hatalar

Hepsi ölçümle bulundu (gz gerçek pozu ↔ PX4 kestirimi); hiçbiri bapp kodu değil ama hepsi geo-lokalizasyonu bozuyordu.

1. **Magnetometre eksen eşlemesi (PX4 v1.17 `GZBridge::magnetometerCallback`)** — gz-sim 8.15 alanı tutarlı
   döndürüyor (uçuşta 258° yön boyunca `R_enu_flu · m` sabit, std 1e-4 G; bileşenler NED sırasında: N 0,216, E 0,010,
   D 0,429 G), PX4'ün sabit yansıtması (x=−y, y=−x) ise yalnız bazı yönlerde doğru. Sonuç: yöne bağlı ±10° yaw ve
   ±4° roll/pitch hatası; EKF bunu sahte rüzgârla uzlaştırıp düz uçuşta **18° yaw** hatasına ve uçağı 7° yengeç
   uçurmaya götürdü (ilk koşu r1: hedef hatası 1–5 m). Yerde uav1: PX4 104,4° / gerçek 90°. **Düzeltme:** köprü gerçek
   pozla vektörü dünyaya çevirip FRD'ye doğru indiriyor (yama). Sonrası: yerde 90,5° / 91,1°, uçuşta |yaw| 0,21°,
   |roll|,|pitch| ≈ 0,1°. **Bu, README "bilinen açık konular" 3'teki 9–10 m anomalinin (PX4 − gerçek yaw −8…−10°)
   büyük olasılıkla kök nedenidir.**
2. **`EKF2_GPS_DELAY` 110 ms, gz NavSat gecikmesiz** → PX4 konumu gerçeğin ~100 ms önünde (15 m/s'de 1,3–1,5 m,
   uçuş yönünde). Simde `EKF2_GPS_DELAY=0` (`launch_all.sh`, `SIM_EKF_DUZELT=0` ile kapatılır). Sonrası: gecikme
   taraması −10…−20 ms, PX4 yatay konum hatası 0,3–0,4 m.
3. **gz ogre2 `<distortion>`'ı uygulamıyor ama `camera_info`'ya yazıyor** → D ile undistort kenarda ~19 px hata.
   Modelden kaldırıldı; geolocator sim ChArUco kalibrasyonunu kullanıyor (`kalibrasyon/w_nadir_10_sim.json`).
4. **Kalkış irtifası yarışı** (v1.17): `MIS_TAKEOFF_ALT` ayarlanır ayarlanmaz gelen `NAV_TAKEOFF` eski 25 m'yi
   kullanıyordu → `mavlink_takeoff.py` 2 sn bekliyor.
5. **Jazzy'de `gz sim` görünmüyor**: `gz_tools_vendor` `gz`'yi ve `GZ_CONFIG_PATH`'i gölgeliyor →
   `launch_all.sh` `/usr/share/gz`'yi öne alıyor.
6. **`stop` kendi kabuğunu / başka bölümdeki gz'yi öldürüyordu** (`pkill -f "gz sim"`) → yalnız aynı
   `GZ_PARTITION`'daki ruby/gz süreçleri (`gz_pids`).
7. v1.17 konu sürümleri: `vehicle_command_ack` (sürüm eki yok), `vehicle_status_v1`.
8. `alan_tarama.py --koridor-yok`: doğu kalkış koridoru kuralı uav1 uzak üsteyken anlamsız; eskiden `--hat-sec`
   ile **hedefin olduğu hat seçilerek** aşılıyordu. Bu koşuda hat seçilmedi.

## Sonuçlar

### r2 → z1: düzeltmelerden sonra uçtan uca zincir (`z1/`)

uav0 100 m'de 4 hatlı tarama (W 70 m, hat sırası +40 → −100 → +81 → −40; hedef konumu planda yok). Marker
**2. hatta, kadrajın kenarında** (hatta 40 m yanal, görüntü u ≈ 22–66 px) bulundu; tarama hattı bitirip Hold'a çıktı.

| Ölçü | Değer |
|---|---|
| Tespit (marker tam kadrajda, ~33 px kenar) | 33 / 38 kare (%86,8; kadraj kenarı); sahte tespit 0 / 10 734 kare |
| PX4 poz hatası (tespitli karelerde) | yatay 0,39 m, yaw 0,21°, roll/pitch ~0,1° |
| Kare başına ray-cast hatası | medyan 0,55 m, %95 0,64 m (gerçek pozla 0,06 m → kalan PX4 konumu) |
| Kare başına PnP hatası | medyan 1,02 m (gerçek pozla da 1,07 m → köşe içe biası, aşağıda) |
| Gönderilen son rapor (füzyon, 66 gözlem) | **0,79 m** hata; bildirilen σ95 0,45 m (iyimser, ×1,74) |
| LoRa | 393 rapor, 13 kayıp (%3,3), 3 meşgul kanalda ezilen; gecikme 66,8–85,8 ms (ort. 74,8) |
| uav1 | yerden kalkış (tespitten 49 sn sonra, üs 950 m), plan hedefi hatası 0,76 m, 3B 20,0 m'de durma (kendi kestirimiyle) |
| **uav1 GERÇEK en yakın geçiş** | **yatay 0,29 m, 14,4 m irtifada**; tırmanış, üsse dönüş, iniş ✔ |

Nadir geçişte (r1, düzeltmesiz ortam) tespit 93/94 (%98,9; 25–40 px kenar), sahte 0.

### z3: R = 100 m halka içinde çim biçme taraması + kamikaze kamerasıyla okuma (`z3_daire/`, görsel mod)

Yeni: `alan_tarama.py --daire 100` (hatlar çapı boydan boya geçer, dönüşler daire dışında; `--sira cim` = komşu hatlar
sırayla; yön verilmezse 0–165° arasından doğrulanmış en iyi kapsama → **135°**, direklerden en uzak çapraz; 0°'de merkez
şerit direk payına giriyordu), uav1'e burunda 15° aşağı bakan aynı GS + 6 mm kamera (`kamera_ileri`, `kamikaze_bridge.yaml`)
ve bapp geolocator'ın 2. örneği (`/bap/uav1/okuma`; dalış hedefi yine LoRa raporu).

| Ölçü | Değer |
|---|---|
| Plan | 3 hat (−59 → 0 → +59 m, yön 135°), 2300 m ≈ 153 sn, kapsama %98,2 (kutu planı: %80, 3141 m) |
| Keşif tespiti | 3. hatta; 35/35 beklenen kare (%100), sahte 0 |
| PX4 poz hatası | konum 0,11 m, roll 0,55°, pitch 0,31°, yaw 0,66° (gerçekçi EKF düzeyi) |
| Kare başına | ray-cast 1,43 m (gerçek pozla 0,04 → hata attitude'dan), PnP 1,27 m |
| LoRa son rapor | 1,33 m (σ95 0,41 — iyimser); uav1 plan hedefi hatası 1,34 m (çoğu hat boyunca) |
| **Kamikaze okuması** | **19 kare, id 0 doğru**; ilk okuma gerçek 3B 72 m (22 m irtifa, kenar 14 px), son 29,5 m (73 px; sonra marker kadrajın altından çıkıyor); kendi kestirimi 1,35 m |
| Kamikaze gerçek en yakın geçiş | yatay 0,25 m, 13,8 m irtifada; tırmanış, üsse dönüş, iniş ✔ |

Bu koşuda bulunup düzeltilen ekip-kodu yarışları: (1) `lora_yaklasma.Uav0Izleyici` 400 örneklik geçmiş, ~100+ Hz bekleme
döngüsünde < 4 sn kalıyordu → uav0 Hold'u hiç "kararlı" sayılmadı, uav1 240 sn sonra NO-GO (z2_gorsel); örnekler 0,05 sn
seyreltildi (yeniden üretim: eski hiç, yeni 12 sn'de kararlı). (2) `alan_tarama.py` zincir denetimi DDS keşfi bitmeden
yayıncı sayıyordu (yayıncı varken 0 → HATA); 15 sn keşif beklemesi. (3) `launch_all.sh stop` kamera izleyicilerini
kapatmıyordu. Etkin RTF iki kamera + pencere + izleyicilerle ~0,85 (`GPU=1`, D3D12/RTX 4050).

### Bulgular (bapp için)

- **PnP içe köşe biası**: Gazebo + `gs_kamera` görüntüsünde tespit edilen marker %1,4–2,3 küçük (köşe başına
  ~0,33 px nadirde, ~0,54 px kadraj kenarında); bapp sentetik modeli 0,30 px öngörüyor. Aynı z1 köşeleriyle
  çevrimdışı: telafisiz PnP 1,54 m / füzyon 1,53 m; model telafisi (`--corner-sensor color`) 1,02 / 0,79 m;
  ölçülen 0,54 px (`--inward-bias-px 0.54`) 0,65 / 0,60 m; yalnız ray-cast 0,54 m. Ray-cast bu biastan etkilenmiyor.
  → Gerçek GS kamerada bias **ölçülmeli**; ölçülmeden PnP füzyona sokulmamalı.
- **H1 bu senaryoda desteklenmedi**: tek geçiş, kadraj kenarı, aynı PX4 konum hatası → füzyon ray-cast'ten iyi
  değil, kovaryans iyimser (ortak/biaslı hatalar bağımsız sayılıyor). H1/H2 için daire deseni + çok geçiş gerekir.
- Kare damgası ile PX4 `timestamp_sample` aynı saatte (`UXRCE_DDS_SYNCT=0`); kare, son attitude örneğinden ~28 ms
  sonra ulaşıyor → poz tamponu kare anını iki yandan kapsıyor.

### Simülasyonun gerçekçi OLMAYAN yanları (sonuçlar bu yüzden iyimser)

- GNSS çok iyi: PX4 konum hatası 0,3–0,4 m (gerçekte RTK'sız 1–2,5 m bias + gürültü). **Mutlak hata gerçekte
  büyük olasılıkla 1–3 m**; uav1 de aynı tür GNSS kullandığından ortak bias kısmen söner (CLAUDE.md notu).
- Rüzgâr yok (yengeç açısı yok), lens distorsiyonu yok (gz desteklemiyor), optik bulanıklık/MTF yok, köşe gürültüsü
  ~0,05–0,07 px (gerçek ~0,33 px). Işık düzgün, gölge yok, zemin düz.
- Uçuş tek koşu; istatistik için tekrar gerekir.
- Kamikaze okuması sığ dalış açısında (~7°) marker çok basık görünüyor (ilk okuma 14 px kenar); daha dik dalış okumayı
  erkene çeker. Okuma dalış hedefini değiştirmiyor (terminal güdüm yok).

## Sonraki adımlar

1. `gs_kamera.py`'ye Brown distorsiyonu (cv2.remap) + optik bulanıklık ekleyip kalibrasyonu tekrarlamak.
2. gz `WindEffects` ile rüzgâr; GNSS gürültü/bias'ını gerçek alıcıya göre artırmak.
3. H1/H2: hedef çevresinde daire deseni, çok geçiş, Monte Carlo ile karşılaştırma.
4. Gerçek GS kamerada köşe biası ölçümü → `--inward-bias-px`.
