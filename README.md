# İHA Sim V2

TÜBİTAK kapsamında geliştirilen **İHA tespit/takip sistemi**nin simülasyon çalışma alanı.
Hedef: simülasyonda iki sabit kanatlı İHA'yı birlikte uçurmak; biri alanı tarayıp yer hedefini (QR) bulur,
konumu LoRa benzeri bir röleyle ikinci İHA'ya iletir, ikinci İHA hedefe kontrollü dalış yapar.

> **Durum (2026-11-20):** Simülasyon aşaması kullanıcı kararıyla **tamamlandı** (2026-11-13), ardından
> "yerden kalkış" senaryosu da uçuşla doğrulandı (2026-11-16). Sıradaki aşama: gerçek donanım.
> Ayrıntılı kayıt: [`v2_proje_gunlugu.md`](v2_proje_gunlugu.md) (en yeni kayıt en üstte) ve [`CLAUDE.md`](CLAUDE.md).

## Senaryo (uçtan uca zincir)

1. **uav0** 60 m'de 300×300 m alanı boustrophedon tarar (aşağı bakan kamera).
2. `qr_online.py` karede QR hedefini (`HEDEF-01`) bulur, piksel → ışın → zemin ile konumunu kestirir → `/hedef_konum`.
3. `lora_relay.py` mesajı gecikme (Gauss 50 ± 20 ms) ve kayıp (%3) ekleyerek `/hedef_konum_role`'ye aktarır.
4. **uav1** pistte yerde bekler; ilk röle mesajı kalkış komutudur. Kalkış → yaklaşma hattı → 3B 20 m'de duran dalış →
   tırmanış → üse dönüş → görev deseniyle iniş → disarm (`lora_yaklasma.py`, `yer_gorevi.py`).

Doğrulanan sonuçlar: 6/6 abortsuz zincir, QR tespiti kadrajdayken 53/54, dalış 3B 19,8–19,9 m'de duruş,
yerden kalkış zinciri 2/2. Tablolar `test/*/sonuc.md` dosyalarında.

## ArUco / bapp zinciri (2026-11-20, Ubuntu 24.04 + Jazzy + PX4 v1.17.0)

bapp geo-lokalizasyonu (2×2 m ArUco, RPi GS + 6 mm) bu simülasyonda uçtan uca denendi: tespit → 24 bayt LoRa raporu →
uav1 yerden kalkış → dalış; uav1 gerçek hedefin 0,29 m yanından geçti. Kurulum, simülatör hataları (magnetometre
eşlemesi, GPS gecikmesi, gz distorsiyonu), sonuçlar ve sınırlar: [`test/aruco_2026-11-20/sonuc.md`](test/aruco_2026-11-20/sonuc.md).

```bash
CAM=W10 ARUCO=1 ./launch_all.sh                     # 6 mm kamera + ArUco dünyası + bapp düğümü + LoRa (paket)
bash test/aruco_2026-11-20/zincir.sh <çıktı>        # uçtan uca: R=100 m halkada çim biçme taraması + kamikaze okuması (~20 dk)
GOR=1 bash test/aruco_2026-11-20/zincir.sh <çıktı>  # aynısı görsel: Gazebo penceresi + iki kamera penceresi (GPU)
```
PX4'e `px4_yamalar/gz_bridge_magnetometre.patch` uygulanmış olmalı.

### bapp kurulumu (ArUco zinciri ve `bap_geo` kullanan analiz araçları için)

`ARUCO=1`, `gs_kamera.py`, `aruco_degerlendir.py`, `zincir_degerlendir.py`, `kapsama_analiz.py`, `mozaik.py`,
`lora_yaklasma.py --girdi paket` bapp deposunun `bap_geo` paketini kullanır (bulunamazsa bu araçlar çalışmaz — 2026-11-22
Tulpar testlerinde `kapsama_analiz`/`aruco_degerlendir` bu yüzden atlanmıştı). Bir kez:

```bash
git clone https://github.com/muhammetgumus55/aruco-geolocalization.git ~/bapp     # başka yere: BAPP=<yol> ver
cd ~/bapp && python3 -m venv --system-site-packages .venv-ros                       # rclpy/px4_msgs sistemden gelir
.venv-ros/bin/pip install --no-deps opencv-contrib-python==5.0.0.93                # numpy sistemde yoksa: pip install numpy
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
.venv-ros/bin/python -c "import sys; sys.path.insert(0, '.'); import bap_geo.aruco, rclpy, px4_msgs; print('tamam')"
.venv-ros/bin/python -m pytest -q                                                   # bapp testleri (isteğe bağlı)
```
`bap_geo` pip ile kurulmaz: araçlar `BAPP` (varsayılan `~/bapp`) yolunu `sys.path`'e ekler; `launch_all.sh` düğümleri
`$BAPP/.venv-ros/bin/python` (`PY_BAPP`) ile başlatır. Analiz araçlarını da bu yorumlayıcıyla çalıştırın:
`~/bapp/.venv-ros/bin/python aruco_degerlendir.py …`.

### Tulpar + BAP kamerası + ArUco (2026-11-24)

`UAV0=tulpar CAM=W10 ARUCO=1 YAKLASMA=1 ./launch_all.sh` — Tulpar'a 6 mm GS kamerası (`gz_models_tulpar/W10`, T10 ile aynı
kütle/konum). Uçtan uca: `bash test/aruco_tulpar_2026-11-24/zincir.sh <çıktı>` (el fırlatma → 100 m → B1 taraması → bapp →
LoRa paket (868 MHz g1, duty %1) → uav1 yerden kalkış → dalış). `GERCEKCI=1` ile distorsiyon, boresight, bulanıklık, GNSS
biası, magnetometre biası ve rüzgâr açılır (`launch_all.sh` başı). Sonuçlar: `test/aruco_tulpar_2026-11-24/sonuc.md`.
PX4'te Tulpar airframe'i kurulu olmalı (`px4_yamalar/README.md`, "Tulpar airframe'i").

## Yığın

> **Tek sürüm kararı (2026-11-21):** proje Ubuntu 24.04 + ROS 2 Jazzy üzerinde birleşir (Harmonic'in resmî eşi, destek 2029;
> Humble 2027'de biter). Geçiş bitene kadar `launch_all.sh` kurulu dağıtımı otomatik seçer (jazzy varsa jazzy, yoksa humble);
> eski Humble ortamı yalnız geçiş süresince desteklenir ve doğrulama Jazzy'de yeniden yapılmalıdır.

**Hedef yığın: Ubuntu 24.04 (WSL2) · ROS 2 Jazzy · Gazebo Sim 8 (Harmonic) · PX4 SITL (v1.17.0)** ·
MicroXRCE-DDS Agent · `px4_msgs` · Python (OpenCV 4.9 + contrib, numpy, pymavlink; V1'de ultralytics/YOLO11).
Model: PX4 `advanced_plane` (sabit kanat). MAVROS / ArduPilot kullanılmıyor.

## Gereksinimler (bu depoda YOK)

- `~/PX4-Autopilot` (PX4 SITL; `Tools/simulation/gz` elle değiştirilmiş — güncelleme/reset yapmayın)
- `~/ros2_ws` içinde derlenmiş `px4_msgs` (`source ~/ros2_ws/install/setup.bash`)
- `MicroXRCEAgent`, `ros_gz_bridge`, `gz sim`
- Eski V1 kodu ve YOLO ağırlıkları ayrı depoda/klasörde: `~/iha_tespit_sim` (arşiv)

## Hızlı başlangıç

```bash
# Günlük başlatma (kameralı, özel dünya, LoRa röle + tespit düğümleri)
CAM=T10 LORA=1 WORLD=iha_arena_v2 ./launch_all.sh

# Kalkış (QGC'siz) — iki araç
python3 mavlink_takeoff.py --uav 0 1 --ucus-dinamigi

# Kapatma (komut metninde betik adı geçirmeyin: pkill kabuğu öldürür)
./launch_all.sh stop
```

Seçenekler: `CAM=1|T|T15|T10` (kamera/Hz), `GUI=1` (pencere; WSLg'de RTF düşürür), `VIZ=1` (işaretçiler),
`LORA=1` (köprü + qr_online + röle). Ayrıntı için `launch_all.sh` başı ve `CLAUDE.md`.

## Önemli dosyalar

| Alan | Dosyalar |
|---|---|
| Başlatma / kalkış | `launch_all.sh`, `mavlink_takeoff.py`, `ros_takeoff.py`, `start_sim.sh` |
| Orbit (V2 ilk aşama) | `v2_orbit_mission.py`, `orbit_approach.py`, `orbit_watch.py`, `orbit_abort` |
| Alan taraması | `tarama_plan.py`, `alan_tarama.py` |
| Tespit / konum | `qr_online.py`, `qr_pass.py`, `qr_recorder.py`, `qr_analyze.py`, `qr_video.py` |
| Röle + uav1 | `lora_relay.py`, `lora_yaklasma.py`, `yer_gorevi.py`, `ayrim.py` |
| PX4 durum okuma | `px4_durum.py`, `px4_durum_sabit.py`, `nav_izleyici.py`, `fastdds_dinamik.xml` |
| Dünya / modeller | `gz_worlds/` (arena, QR, doku üreticileri), `gz_models_cam_hw/`, `gz_models_nocam/` |
| Kuru testler (sim gerekmez) | `*_test.py` |
| Ölçümler ve veri | `test/` (her klasörde `sonuc.md`/`tasarim.md`) |

Kuru testler (ROS ortamı kaynaklanmış olmalı): `python3 alan_tarama_test.py`, `qr_pass_test.py`, `ayrim_test.py`,
`lora_dalis_test.py`, `lora_tarama_zincir_test.py`, `yer_gorevi_test.py` (`lora_yaklasma_test.py` yalnız tablo üretir).

## Proje kuralları

- Yalnızca standart **Offboard + TrajectorySetpoint**; L1/TECS/APF/Pure Pursuit tabanlı özel denetleyiciler yasak.
- Çoklu araç: uav0 `/fmu` (SysID 1), uav1 `/px4_1/fmu` (SysID 2). PX4 çıkış topic'leri BEST_EFFORT+VOLATILE,
  komut yayıncıları TRANSIENT_LOCAL. Okuma topic'leri sürüm ekli (`vehicle_local_position_v1` vb.).
- **Simülasyona özgü PX4 parametreleri** (`mavlink_takeoff.py` → `SIM_ONLY_PARAMS`, `UCUS_DINAMIGI_PARAMS`)
  gerçek donanıma **taşınmaz**. Geri alma: `python3 mavlink_takeoff.py --uav 0 1 --revert-sim-params`.
- Simülasyon süreçleri kullanıcı onayı olmadan başlatılıp durdurulmaz.
- Gizli bilgiler (token, anahtar) depoya yazılmaz.

## Bilinen açık konular (devam noktası)

1. **uav1 yakın çekim videosu** (kalkış + dalış): `test/yerden_kalkis_2026-11-15/ucus/s2/calistir.sh` yeniden koşturulabilir
   (~9 dk); mevcut `uav1_yakin.mp4` bozuk, `kayit.py` düzeltildi.
2. **İniş boyuna taşması** +52…+78 m (flare): `FW_LND_FL_TIME` / `FW_LND_FLALT` / süzülme açısı ile daraltılabilir.
3. **Konum kestirimi anomalisi:** bir geçişte 9–9,9 m hata (PX4 pitch −7,4° iken); diğerleri 2–3,3 m. Neden bulunmadı. **(2026-11-20: büyük olasılıkla PX4 gz_bridge magnetometre eşlemesi — yöne bağlı ±10° yaw; yama ve ölçüm `test/aruco_2026-11-20/sonuc.md`.)**
4. **uav1 geçiş bacağında engel payı:** Hold geçişinde direğe ~10 m yaklaşma görüldü.
5. Hava hızı sıcaklık hatası (gz `AirSpeed` eklentisi 0 K gönderiyor) — geçici önlemler yalnız simülasyonda; gerçek düzeltme PX4 eklentisinde.
6. `px4_msgs` sürüm uyumsuzluğu (nav_state ham okumayla aşıldı); uav1 kamerası ve YOLO/tracker entegrasyonu yapılmadı.
7. **Simülatör kısıtı:** pist yönü 90° (gz yaw 0) zorunlu — `AdvancedLiftDrag` yalnız yaw 0 spawn'ında doğru çalışıyor.
8. GUI/pencere RTF darboğazı (WSLg yazılım render); doğrulamalar penceresiz yapıldı.

Yol haritası: `.ulg` analizi → tek İHA orbit → iki İHA orbit → iki kamera köprüsü → YOLO → tracker + Kalman →
tespitten uçuş komutuna kontrol katmanı. Gerçek donanım aşamasından önce madde 3 ve 4 ele alınmalı.

## Yeni gelenler için okuma sırası

1. Bu README → 2. `CLAUDE.md` ("Yerden kalkış senaryosu" ve "Simülasyon aşaması durumu" bölümleri) →
3. `v2_proje_gunlugu.md` (en üst kayıtlar) → 4. `test/yerden_kalkis_2026-11-15/tasarim.md` ve `test/tam_tarama_2026-11-12/sonuc.md`.
