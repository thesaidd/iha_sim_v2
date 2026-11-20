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

## Yığın

Ubuntu 22.04 (WSL2) · ROS 2 Humble · Gazebo Sim 8 (Harmonic) · PX4 SITL (v1.17.0-alpha1) ·
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
3. **Konum kestirimi anomalisi:** bir geçişte 9–9,9 m hata (PX4 pitch −7,4° iken); diğerleri 2–3,3 m. Neden bulunmadı.
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
