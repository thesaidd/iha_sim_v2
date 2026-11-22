# Tulpar test sonuçları
(2026-11-22: model "gercek ucak" adından Tulpar'a yeniden adlandırıldı; aşağıdaki A0–A5 kayıtları eski adla yazılmış olup dosya/betik adları güncellendi.)

## A0 — statik (2026-11-21) GEÇTİ
gz sdf -k Valid; 7 link / 6 eklem; kütle 2,3 kg; CG z 0,026 m; bbox 1,70 × 1,25 m; airspeed_link/air_speed sensörü eklendi.

## A1 — servo yönü (2026-11-21) GEÇTİ
Yöntem: boş dünya (test/bos.sdf, yalnız sunucu), `servo_testi.py` +0,3 / −0,3 rad, dynamic_pose'tan göreli dönüş.
Sonuç: 4 servo da komutla 0,300 rad döner. Hepsinde + komut = linkin yerel +y ekseni etrafında dönüş = **arka kenar (TE) yukarı** (aileron ve her iki ruddervator). (Not: ilk denemede sağ ruddervator ters çıktı → eksen düzeltildi; sol 'axis' = (0,cos40,−sin40), sağ = (0,cos40,+sin40).)
PX4 eşlemesi: ters-V (Λ) kuyrukta yaw işareti V'nin tersi olduğundan sol fiziksel panel (servo_2) = PX4 "Right V-Tail" (CA_SV_CS2_TYPE 8), sağ (servo_3) = "Left V-Tail" (TYPE 7). A4'te actuator_test ile doğrulanacak.
Aero işaretleri (model.sdf, direction=+1): + komut = TE yukarı → CL_ctrl −, Cem_ctrl + (burun yukarı); sol: CY −, Cen +; sağ: CY +, Cen −.

## A2 — motor itkisi (2026-11-21) GEÇTİ
Yöntem: yerçekimsiz boş dünya (bos_g0.sdf), `motor_testi2.py` komuttan sonraki 0,5 sn'de x(t) parabol uydurma, T = m·a (m = 2,3 kg).
İlk model motorConstant 2,2e-5 / maxRot 3000 ile tam gazda ~200 N verirdi (şablonun 1 kg'lık uçağına göre kalibre): düzeltildi → motorConstant 8,9e-6, maxRotVelocity 2000, airframe SIM_GZ_EC_MAX1 1500.
| ω (rad/s) | ölçülen itki | k·ω² |
|---|---|---|
| 600 | 3,3 N | 3,2 N |
| 1000 | 9,4 N | 8,9 N |
| 1500 | 19,5 N | 20,0 N |
İtki +x (ileri), yan/dikey sapma ihmal edilebilir (Δz −0,05 m). Tam gaz (1500 rad/s) ≈ 20 N ≈ gerçek X2820/13,6" tahmini. Not: gerçek itki-gaz eğrisi (bench) bilinmiyor.

## A3 — serbest uçuş (PX4'siz) (2026-11-21) GEÇTİ (boyuna), yanal yavaş spiral
Yöntem: `bos_h100.sdf` (100 m, yerçekimi açık, yalnız sunucu, başlangıçta duraklatılmış), `ucus_testi.py V0 ω süre adım`: unpause → 0,5 sn sonra ApplyLinkWrench darbesi (F = m·V0/0,004; `gz-sim-apply-link-wrench-system` dünyada gerekli) + sabit motor, servo = 0. Kayıt: `ucus_A3_1.txt`.
- Boyuna: V0 = 15 m/s, ω = 550 rad/s (≈ 2,7 N). Hız ~16–17 m/s'de dengelendi, irtifa 30 sn'de 100 → ~83–97 m (hafif süzülme), pitch ≈ −2…0° ve 2 sn içinde sönümlendi (osilasyon yok). Şiddetli ilk deneme (zayıf darbe, sıfıra yakın hızla bırakma) pitch 69°'ye çıkıp toparladı → sıfır hızda bırakma YASAK, fırlatma hızı şart.
- Yanal (komutsuz): yavaş spiral — roll 2° → 30° / 30 sn, yaw 0 → −175° (sağa dönüş). Osilasyonlu Dutch roll gözlenmedi (uyarılmadı). Katkı: motor reaksiyon torku (momentConstant 0,01, pusher, cw). PX4 kapalı çevrimde bastırması gerekir; A5'te ilk gözlenecek.
- Not: düz uçuşta pitch ≈ −2° iken irtifa korunuyor → eklentideki etkin CL0, CL0 etiketinden büyük; PX4 FW_PSP_OFF/trim buna göre ayarlanacak.

## A4 — PX4 SITL entegrasyonu (2026-11-21) GEÇTİ
- Airframe PX4'e **4030_gz_tulpar** adıyla eklendi (4009 r1_rover'a ait; çakışma). PX4'te değişen: `ROMFS/.../airframes/4030_gz_tulpar` (yeni) + `CMakeLists.txt` +1 satır (yedek `yedek/airframes_CMakeLists.txt.bak-2026-11-21`). `make px4_sitl_default` hatasız.
- Ortam: `tulpar_ortam.sh start|stop` (Agent + gz sunucusu + PX4 instance 0, penceresiz, PID ile kapatma). Model adı gz'de `tulpar_0`.
- Sensörler: imu, baro, mag, navsat, **air_speed** topic'leri akıyor; `Ready for takeoff` alındı.
- Bulgu 1: `FD_ESCS_EN` — gz_bridge ESC "armed" bayrağını yalnız motor hızı>0 iken kuruyor; motor kapalı (NaN) beklerken "ESC failure" failsafe arm'ı düşürüyor (el fırlatmada motor kalkışa kadar kapalı). `FD_ESCS_EN 0` airframe'e (SİMÜLASYON-ONLY, gerçek uçağa taşınmaz) eklendi.
- Bulgu 2: `CA_SV_CSn_TRQ_*` yaml varsayılanları yalnız arayüzde geçerli; V-kuyruk için açıkça verildi (aksi halde pitch/yaw servoları 0).
- Bulgu 3: yerde düşük gazla "Disarmed by landing" (test için gazı tutarak aşıldı; el fırlatmada beklenen davranış, A5'te launch-detect modu kullanılacak).
- PX4 → gz eşleme (Manual mod, `px4_servo_testi.py`), servo_N komutları (rad):
| girdi | servo_0 solAil | servo_1 sağAil | servo_2 solRud | servo_3 sağRud |
|---|---|---|---|---|
| roll sağ | −0,471 | +0,471 | 0 | 0 |
| roll sol | +0,471 | −0,471 | 0 | 0 |
| pitch yukarı | 0 | 0 | +0,471 | +0,471 |
| pitch aşağı | 0 | 0 | −0,471 | −0,471 |
| yaw sağ | 0 | 0 | −0,471 | +0,471 |
| yaw sol | 0 | 0 | +0,471 | −0,471 |
+ komut = arka kenar yukarı (A1). Beklenenle birebir: roll sağ → sol aileron TE aşağı/sağ yukarı; pitch yukarı → ruddervator TE yukarı; yaw sağ → sağ panel TE yukarı/sol panel TE aşağı.
- Motor: gaz %30 → 457 rad/s, %80 → 1202 rad/s (EC_MAX 1500).

## A5 — El fırlatma uçuşu (2026-11-21) GEÇTİ
Yöntem: `tulpar_ortam.sh start` + `test/tulpar_firlat.py 12 <süre>`: parametre yazma (geri okumalı) → Takeoff modu + arm (launch-detect bekler) → modeli yerden 1,5 m'ye ışınla, 10° burun yukarı → 0,3 sn ApplyLinkWrench darbesi (a = 40 m/s², hedef ≈ 12 m/s; gz.transport13 ile hassas zamanlama; CG ofseti için tork telafisi) → izleme. Kayıt: `A5_basarili_hold.ulg`.
Sonuç: "Takeoff detected" → motor 0,9 → 40 m'ye < 6 sn; ~40 m'de Hold (AUTO_LOITER) 2,5+ dk kararlı: irtifa 40,0 ± 0,3 m, hız 13–17 m/s, pitch 2–6°, roll ≈ +16° (sabit daire), failsafe yok.
Bulunan/çözülen hatalar (sırayla):
1. Float parametreler INT32 tipiyle yazılınca reddediliyordu (MIS_TAKEOFF_ALT 25'te kalıyordu) → tip tablosu.
2. **Ruddervator YAW işareti ters** (yaw sağ komutu → sola dönüş; PX4 yaw döngüsü pozitif geri besleme, ~1 sn periyotlu salınım büyüyüp devrildi) → CY/Cen işaretleri çevrildi. Elevatör (Cem) işareti ORİJİNALİ doğru: harness'te `pitch` gz konvansiyonunda (+ = burun AŞAĞI) okunmuştu; yanlışlıkla çevirip geri aldım (not: `ucus_testi.py` çıktısında pitch<0 = burun yukarı, gam>0 = tırmanış).
3. Kontrol yüzeyi katsayıları şablonun (1 kg RC Cessna) 2,5–4× fazlaydı → fiziksel tahminle ölçeklendi: aileron Cell ±0,0009/°, ruddervator Cem 0,0052/°, CL −0,0020/°, yaw Cen ∓0,0008/°, CY ±0,0019/° (yüzey başına). **Birim: derece başına** (servo komutu rad; eklenti çevirir — ölçümle uyumlu).
4. El fırlatma darbesi link orijininden uygulanınca CG ofseti (z 0,0265) yüzünden büyük burun-yukarı tork veriyordu → tork telafisi (`COMP=1,0` hafif fazla; 0,75 daha nötr).
5. PX4 kapalı çevrim ayarı: aileron etkinliği düşünce roll gain artırıldı: FW_RR_P 0,03→0,08, FW_RR_FF 0,05→0,12, FW_R_LIM 35 (roll ±60° salınımı → sabit 16°); FW_TKO_PITCH_MIN 10, FW_P_LIM_MAX 22, FW_PR_I 0,1, FW_RR_I 0,05 (rampa sonrası integral).
Tüm ayarlar `4030_gz_tulpar` ve `tulpar_firlat.py PARAMS` içinde; sim-özgü olanlar (FD_ESCS_EN, ASPD_DO_CHECKS, ASPD_SCALE_APPLY, NAV_DLL_ACT, EKF2_IMU_CTRL) gerçek uçağa taşınmaz.
Açık: fırlatma hızı gerçek eli temsil eden bir model değil (ışınlama + darbe); daire roll'u 16° (yarıçap ~?); motor itki eğrisi, CG, aero tahmin; manuel/gamepad kumanda denenmedi; kamera yok.

## Video (2026-11-21)
Kameralı varyant `tulpar` (arkadan takip + burun kamerası, 30 Hz, gz sensörü); 1428 kare/48 sn sim zamanı, `video/chase.mp4` + `video/burun.mp4`. El fırlatma → tırmanış → Hold dairesi (roll ~16°) kaydedildi. Burun kamerasında boş dünyada yalnız zemin/ufuk görünür (doku yok).

## Kuru test baz çizgisi (2026-11-22, Tulpar geçişi öncesi, advanced_plane zinciri)
| Test | Geçen doğrulama |
|---|---|
| alan_tarama_test.py | 47 |
| qr_pass_test.py | 23 |
| ayrim_test.py | 18 |
| yer_gorevi_test.py | 83 (çıktı geçici klasöre) |
| lora_tarama_zincir_test.py | 26 |
Her adımdan sonra aynı sayılar beklenir; kötüleşme = dur.

## Adım 1 — Yeniden adlandırma gercek ucak → Tulpar (2026-11-22) GEÇTİ
- `gz_models_gercek_ucak` → `gz_models_tulpar` (`git mv`); `nocam/tulpar`, `kam/tulpar` (model adı her varyantta `tulpar`, örnek adı `tulpar_0`); `tulpar_ortam.sh` (`VARIANT=nocam|kam`), `test/tulpar_firlat.py`; PX4 `4030_gz_tulpar` (`PX4_SIM_MODEL:=tulpar`), `CMakeLists.txt` satırı, eski airframe silindi (ROMFS + build), `px4_yamalar/4030_gz_tulpar` + `4030_airframes_CMakeLists.patch`.
- Ölçüt kanıtları: eski ad grep (`gercek_ucak|gercek_ortam|gercek_firlat`, `.git` hariç) = günlükteki 2 tarihsel satır dışında **0**; `gz sdf -k` nocam ve kam **Valid**; `make px4_sitl_default` hatasız (ROMFS yenilendi, `4030_gz_tulpar` build'de var); `tulpar_ortam.sh start` → `gz_bridge model: tulpar_0`, "Ready for takeoff".
- A5 tekrarı (`tulpar_firlat.py 12 45`, arena değil boş default dünya, ulg `2026-11-22/11_54_28`): fırlatma → 39,5 m'ye **6,5 s** (hedef < 10 s ✔); tırmanış tepesi 46,5 m (8 s), Hold'a oturma sonrası irtifa **40,0 ± 0,4 m** (hedef |hata| ≤ 1 m ✔), Hold **75 s** izlendi (≥ 60 s ✔), failsafe **0** ✔, roll 16–17° sabit daire (A5: 16°), yer hızı 20–80 s: ort **15,2** (14,4–16,0) m/s (A5: 13–17), hava hızı sensörü 10–19 arası gürültülü (A5 ile aynı). Karşılaştırma A5 (eski ad): 40,0 ± 0,3 m — fark ≤ 0,1 m.
- Not: kamerasız `nocam/tulpar` kullanıldı; `kam` varyantı yalnız `gz sdf -k` ile doğrulandı (uçuşta A5 öncesi video çekiminde kullanılmıştı).

## Adım 2 — Tulpar T10 (aşağı bakan kamera) (2026-11-22) GEÇTİ (kapsamı aşağıda)
- `uret_model.py`: `TULPAR_VARYANT=T10` → `T10/tulpar/model.sdf` (nocam üretimi sonunda otomatik). Kamera bloğu `gz_models_cam_hw/T_nadir_10/advanced_plane/model.sdf:127-139` ile aynı (hfov 0,3114, 1456×1088, clip 0,1–1000, 10 Hz, dik aşağı `0 1.5708 0`, sensör adı `camera`); poz base_link'te x = 0,30 m, z = 0 (gövde altı 1 cm aşağıda), görüntü üstü = burun. Kamera + lens kütlesi 0,12 kg (CG batarya konumuyla yeniden çözüldü: batarya 0,62 → 0,50 kg, x 0,280 → 0,276 m); nocam/kam dosyaları değişmedi (`git diff` boş).
- Doğrulama: `gz sdf -k` nocam/kam/T10 Valid; CG x = 0,000 m (çeyrek kord; ölçüt ≤ 5 mm ✔); kamera parametreleri referansla birebir (parametre diff boş).
- **camera_info fx = fy = 4637,814** (referans 4637,81; fark < %0,001 ✔; cx 728, cy 544).
- **Statik QR ölçümü** (`test/t10_statik_qr.py` → `gz_models_cam_hw/qr_irtifa_tarama.py` yeniden kullanıldı; statik sarmalayıcı `t10_statik_model.sdf`, sensör sarmalayıcı orijininde, aynı dünya `iha_arena_gercek`, aynı anda referans `kamera_test` ile koşuldu). Modül boyutu (px), teorik / referans / Tulpar T10:
| irtifa / yön | teorik | referans (kamera_test) | Tulpar T10 | fark |
|---|---|---|---|---|
| 40 m / 0° | 5,52 | 5,476 | 5,476 | 0,000 |
| 40 m / 45° | 5,52 | 5,539 | 5,539 | 0,000 |
| **60 m / 0°** | 3,68 | 3,690 | **3,690** | **0,000 (%0)** |
| 60 m / 45° | 3,68 | 3,662 | 3,662 | 0,000 |
| 80 m / 0° | 2,76 | 2,714 | 2,714 | 0,000 |
| 80 m / 45° | 2,76 | 2,719 | 2,719 | 0,000 |
60 m bulma/çözme: ham QRCodeDetector 100/100 (iki yönde). Ölçüt (fark ≤ %1, referans 3,67) ✔. Önceki kayıt (2026-10-30) 3,67 ile bugünkü referans 3,69 arasındaki ~%0,5 fark renderer/dünya (iha_arena_gercek) farkından, bu görevin dışı.
- **Kapsam/sınır (dürüst):** fark 0 olması beklenen sonuç (aynı sensör bloğu, aynı poz) — test **kamera bloğunu, optik ekseni (dik aşağı) ve gövdenin görüşü kapatmadığını** doğrular (Tulpar gövde/kanat/boom görselleri sahnede, 40 m karede QR engelsiz: `t10_statik_2026-11-22/tulpar_kare_040m.png`). **Doğrulanmayan:** modelin gerçek uçuşta eğimi (pitch ~4°) ve model içindeki 0,30 m ofseti (sarmalayıcı −0,30 ile telafi etti), yerdeki yakın kırpma (near 0,1 m; yerde kamera yüksekliği ölçülmedi) — Adım 5'te ölçülür.
- Beş kuru test (Adım 2 sonrası): alan_tarama **47**, qr_pass **23**, ayrim **18**, yer_gorevi **83**, lora_tarama_zincir **26** — baz çizgisiyle aynı ✔.

## Adım 3 — Araç adı merkezileştirme + `launch_all.sh UAV0=tulpar` (2026-11-22) GEÇTİ (kapsamı aşağıda)
**Değişenler:** yeni `arac_modelleri.py` (`UAV0_MODEL`/`UAV1_MODEL`, varsayılan `advanced_plane`; gz örnek adı, airframe, kamera topic'i, kamera kol mesafesi); uav0'a bağlı yerler buradan okuyor: `qr_online.py` (gerçek poz adı + kamera kolu), `qr_recorder.py`, `gz_gercek_poz.py`, `viz_trails.py`, `cam.sh`, `qr_analyze.py` (CAM_T_FLU) ve analiz araçları `mozaik.py`, `kapsama_analiz.py`, `aruco_degerlendir.py`, `tarama_video.py`. **Dokunulmadı:** `kamikaze_bridge.yaml`, `zincir_degerlendir.py` (uav1), `camera_bridge.yaml` (git'teki dosya olduğu gibi; Tulpar'da `/tmp/iha_camera_bridge.yaml` üretilir). `launch_all.sh`: `UAV0=tulpar` → uav0 için `PX4_SIM_MODEL=gz_tulpar`, `PX4_SYS_AUTOSTART=4030`, yalnız o örnek için `PX4_GZ_MODELS=gz_models_tulpar/{nocam|T10}` (uav1 aynı genel dizin); uav0 kalıcı parametreleri model değişince yedeklenip sıfırlanır (işaret dosyası `rootfs/0/.arac_modeli`).
**Önemli bulgu (parametre sızıntısı):** `tulpar_ortam.sh`/A5 testleri `rootfs/0/parameters.bson` içine yazmıştı (içinde `FW_LAUN_DETCN_ON=1` kalmıştı); `param set-default` kayıtlı değerleri ezmez → advanced_plane uav0'ı fırlatma algılaması açık başlayacaktı. Önce elle yedeklenip sıfırlandı (`yedek/rootfs0_params_2026-11-22/`), ardından sıfırlama launch_all.sh'a otomatik eklendi (model değişiminde yedekli sıfırlama).
**Ölçüt (a) UAV0 verilmezse birebir aynı — ölçüldü:**
- Taban = HEAD `launch_all.sh` ile `CAM=T10` başlatma; yeni script ile aynı komut: **gz topic 68 / 68 (diff 0 satır)**, **ROS topic 132 / 132 (diff 0 satır)**, `/fmu/out` ve `/px4_1/fmu/out` 27'şer topic, iki PX4 "başlangıç betiği tamamlandı", model adları `advanced_plane_0/1`.
- PX4 başlatma komut satırları (çalışan `bash -c` sarmalayıcıları) HEAD şablonunun açılımıyla aynı (`PX4_SIM_MODEL=gz_advanced_plane PX4_SYS_AUTOSTART=4008 …`, değişen yok).
- `python3 arac_modelleri.py yaml camera_bridge.yaml …` (varsayılan) → `camera_bridge.yaml` ile **bayt bayt aynı** (`cmp`).
**Ölçüt (b) kuru testler:** alan_tarama **47**, qr_pass **23**, ayrim **18**, yer_gorevi **83**, lora_tarama_zincir **26** — baz çizgisiyle aynı.
**Ölçüt (c) `UAV0=tulpar CAM=T10 LORA=1` (venv etkin), uçuşsuz:** PX4-0 `model: tulpar_0`, `SYS_AUTOSTART 0 → 4030`; PX4-1 `advanced_plane_1` aynı dünyada; gz topic `/world/default/model/tulpar_0/link/base_link/sensor/camera/image` (+camera_info) ve `advanced_plane_1` kamerası; köprü `tulpar_0` topic'ini bağladı; `/uav0/camera/image` **1456×1088, 9,8 Hz** (hedef ~10; duvar ve sim saati aynı, 147 kare/15 s); `camera_info` fx 4637,81 (qr_analyze ile aynı); `qr_online` çalışıyor ve değerlendirme satırlarında gerçek-poz alanları dolu (`mesafe_3b` 85,07 m, `dt_gt_ms` 0–8 ms) → gerçek poz örneği > 0, **sessiz kayıp yok**.
**Kapsam/sınır (dürüst):**
- **Canlı sınanan:** `launch_all.sh` (varsayılan ve `UAV0=tulpar`), `arac_modelleri.py`, `qr_online.py` (canlı), kamera köprüsü, `cam.sh`/`viz_trails.py`/`gz_gercek_poz.py`/`qr_recorder.py` **çalıştırılmadı** (yalnız `py_compile`/`bash -n`).
- **Yalnız derleme denetimi:** `mozaik.py`, `kapsama_analiz.py`, `aruco_degerlendir.py`, `tarama_video.py` (`bap_geo` yok → zaten içe aktarılamıyor; ArUco hattı bu ortamda atlanmış).
- **Uçuş yok:** Tulpar bu adımda yalnız yerde doğdu (z 0,5); fırlatma Adım 4. Tulpar kamera kolu (0,30; 0; 0) `arac_modelleri.py` içinde sabit (T10 modeliyle aynı).
- `qr_online.py` venv gerektirir (`source ~/venv_iha/bin/activate`; numpy<2). İlk denemede venv'siz başlatıldı → `ImportError: numpy.core.multiarray` (bilinen ortam şartı, bu değişiklikle ilgisiz); venv ile düzeldi.
- Not: `./launch_all.sh` çalıştıran komutun metninde `camera_bridge` gibi stop desenleri geçerse `pkill -f` çağıran kabuğu öldürür (bir kez yaşandı, exit 144).

## Adım 4 — El fırlatma modülü + arena dünyasında 100 m Hold (2026-11-22) GEÇTİ
**Değişenler:** yeni `el_firlatma.py` (darbe fiziği `hesapla()`, `Darbe` sınıfı: gz yayıncıları önceden kurulur, ışınlama, ApplyLinkWrench darbesi **sim saatiyle** ölçülür, CG z varyanta göre (nocam 0,0265 / T10 0,0244, çalışan modelden `gz topic -l` ile algılanır), Tulpar parametre tabloları `TULPAR_SIM_ONLY` ve `TULPAR_UCUS_PARAMS`); `mavlink_takeoff.py`: `--el-firlatma` (yalnız `--uav 0`, `UAV0_MODEL=tulpar` ister), `sim_params_for(uav_id)` ve `ucus_params_for(uav_id)` (modele göre tablo; Tulpar'a `RWTO_MAX_THR`/`FW_TKO_PITCH_MIN=20`/`FW_AIRSPD_TRIM=14` yazılmaz, `FD_ESCS_EN=0` yazılır), `el_firlat()`; yeni kuru test `el_firlatma_test.py` (28 doğrulama), `test/ulg_hold.py` (Hold analizi).
**Ölçüt (a) varsayılan çağrı aynı:** `el_firlatma_test.py` taban commit `ee97a24`'teki `mavlink_takeoff.py` ile `SIM_ONLY_PARAMS` (7 parametre) ve `UCUS_DINAMIGI_PARAMS` tablolarını **birebir** karşılaştırır; `UAV0_MODEL`/`UAV1_MODEL` yokken `sim_params_for(0|1)` = `SIM_ONLY_PARAMS`, `ucus_params_for` boş; `UAV0_MODEL=tulpar` iken uav1 hâlâ advanced_plane tablosunu alır. **Sınır:** bu karşılaştırma tablo/fonksiyon düzeyinde; `mavlink_takeoff.py --uav 0 1` canlı çalıştırılmadı (advanced_plane iki araçlı kalkış bu adımda uçulmadı). Kuru testler 47/23/18/83/26 baz çizgisiyle aynı.
**Ölçüt (b) arena dünyasında Tulpar → 100 m Hold** (`UAV0=tulpar CAM=T10 ./launch_all.sh` + `UAV0_MODEL=tulpar python3 mavlink_takeoff.py --uav 0 --el-firlatma --alt 100 --watch 60`; dünya `iha_arena`, uav1 `advanced_plane_1` yerde; kayıtlar `test/adim4_firlatma_2026-11-22/`, ulg commit'lenmez):
| ölçüt | hedef | ölçülen |
|---|---|---|
| 100 m'ye ulaşma (motor açılışından) | < 40 s | **15,6 s** (40 m 6,4 s, 80 m 12,5 s; tepe 105,8 m @17,1 s) |
| Hold yarıçapı | kaydedilir | **80,1 m** (std 0,2 m), süre 151 s |
| Hold yer hızı | kaydedilir | ort **15,2** (14,4–16,8) m/s |
| Hold irtifası | — | 99,3 ± 0,3 m |
| RTF (150 sn pencere, 1 sn'lik) | kaydedilir | ort **0,96**, min 0,77, max 1,00 (n=29) |
| failsafe | 0 | **yok** (`failsafe_flags` taraması boş, PX4 mesajı yok) |
| uav1'e uzaklık | ≥ 40 m | ≥ 182 m yatay (Hold merkezi uav1'e 262 m, yarıçap 80) ✔ |
- Darbe: hedef a = 40,0 m/s², sim süresi **0,300 s** (duvar 0,296 s, RTF ≈ 1 o an). IMU: gövde x ivmesi 20 m/s² üstüne çıkış **motor açılışından 0,436 s önce**, ilk dalga 0,296 s, tepe 41,6 m/s² → fırlatma algılaması darbenin ivmesiyle tetiklendi (eşik 20, 0,05 s + `FW_LAUN_MOT_DEL` 0,3 s + ~0,09 s gecikme); ışınlama kendi başına tetiklemedi.
- Parametre yazımı: Tulpar tablosu ilk bağlantıda yazıldı (`ASPD_*`, `NAV_DLL_ACT`, `EKF2_IMU_CTRL`, `COM_DISARM_LAND` değişti; geri kalanı zaten ayarlıydı). `ensure_param` float parametrelerde `0.05 → 0.0500000007` gibi tam eşitlik karşılaştırmasıyla gereksiz yeniden yazıyor (zararsız, günlük gürültüsü; ortak fonksiyon değiştirilmedi).
- Karşılaştırma: A5 (boş dünya, 40 m): 6,5 s'de 40 m; bugün arena + T10 modeli: 6,4 s'de 40 m (aynı); Hold yarıçapı 80 m PX4 varsayılanı; advanced_plane Hold (CLAUDE.md) yarıçap 80 m, hız ~15 m/s, RTF 0,967 — Tulpar RTF 0,96.
**Kapsam/sınır (dürüst):**
- Tek uçuş (1/1), tekrar yok; hava hızı sensörü (airspeed_validated) gürültülü (min −9,6, max 22,0 m/s; yer hızı güvenilir) — A5'teki aynı sim sorunu, SIM_ONLY önlemleri kapalı kontrolü bypass ediyor.
- Fırlatma = ışınlama + darbe; gerçek el hareketi değil. Darbe `uav0` doğu yönünde (yaw-0 kısıtı).
- `UAV0_MODEL=tulpar` ayrı kabukta elle verilmeli (launch_all.sh yalnız kendi alt süreçlerine verir); aksi halde `--el-firlatma` reddedilir (hata mesajı güncellendi).
- Hold merkezi spawn noktasının ~247 m doğusunda (80 m yarıçap); arena direkleri (r 140 m) ve QR hedefiyle ilişkisi Adım 5'te planlanır (uav0 Hold'u şimdilik serbest).

## Adım 5.1 — Tulpar el fırlatma → 100 m Hold, 3 dk+ (2026-11-22) GEÇTİ
Yöntem: Adım 4 ile aynı (`UAV0=tulpar CAM=T10 ./launch_all.sh`; `UAV0_MODEL=tulpar python3 mavlink_takeoff.py --uav 0 --el-firlatma --alt 100 --watch 180`; arena `iha_arena`, uav1 yerde); kayıtlar `test/adim5_1_2026-11-22/` (ulg commit'lenmez; analiz `test/ulg_hold.py`, genişletildi: dakika dakika döküm, pitch/roll, servo doygunluğu).
| ölçüt | hedef | ölçülen (1 uçuş) |
|---|---|---|
| Hold süresi | ≥ 3 dk (180 s) | **218 s** (izleme 180 s + kalp atışı süren kalan süre) |
| 100 m'ye ulaşma (motordan) | — | 15,6 s (40 m 6,2 s, 80 m 12,4 s, tepe 105,5 m @17,2 s) — Adım 4 ile aynı 15,6 s |
| Hold yarıçapı | kaydedilir | **80,2 m** (std 0,2); dakikalık 80,3 / 80,2 / 80,1 / 79,8 m |
| Hold yer hızı | kaydedilir | ort **15,2** (14,3–16,4) m/s; dakikalık 14,9 / 15,1 / 15,4 / 15,6 |
| Hold irtifası | — | 100,0 ± 0,4 m (dikey hız std 0,22 m/s) |
| RTF (240 s pencere, 1 sn'lik) | kaydedilir | ort **0,965**, min 0,770, max 1,000 (n=46) |
| failsafe | 0 | **yok** (bayrak taraması boş, PX4 mesajı yok) |
| servo doygunluğu (\|cmd\|>0,95) | — | 4 servo da **%0,0** |
| roll (Hold) | — | ort 16,3° (std 0,9; 12,6…19,0) |
| pitch (Hold) | — | ort 1,37° (std 1,53; −4,1…4,9) |
- Darbe: sim süresi 0,304 s (duvar 0,304 s); 6 tur dönüldü (saat yönü, NED).
- **Bulgu — yavaş trim sürüklenmesi:** Hold boyunca pitch dakikalık ortalaması +2,93° → +1,29° → +0,42° → −0,24°, gaz 0,353 → 0,362, yer hızı 14,9 → 15,6 m/s (roll ~16,0 → 16,8°). Yani seyir pitch'i Hold'a girişten sonraki ilk ~4 dakikada ~3° değişiyor; Adım 4 kaydında (3 dk) pitch ort 3,75° idi, bu uçuşta 1,37°. **Adım 5.2 için sonuç:** QR geçişinde pitch 0–4° arası ve zamana bağlı → kadraj kayması `60·tan(pitch)` 0–4,2 m; sabit "4°" varsayımı yerine ölçülen pitch kullanılmalı. Sürüklenme nedeni (TECS/pitch integratörü yakınsaması mı) incelenmedi.
- Karşılaştırma (advanced_plane değerleri günlükten, bu adımda advanced_plane uçulmadı): Hold yarıçap 80 m (aynı), yer hızı ~14,5–15,5 m/s (Tulpar 15,2), irtifa tutma ±~0,3 m (Tulpar ±0,4), RTF 0,967–0,985 (Tulpar 0,965) → pencereli değil, penceresiz koşuldu. Tulpar A5 (boş dünya, 40 m): roll ~16°, 13–17 m/s — aynı.
- Sınır: **1 uçuş**, tekrar yok; hava hızı sensörü (airspeed_validated) gürültülü (min −4,4, max 21,8 m/s; yer hızı güvenilir); 3 dk eşiği ölçüldü ama uzun uçuşta (15–25 dk) ivmeölçer sapması/`High Accelerometer Bias` (bilinen) denenmedi.

## Adım 5.2 — Tulpar 60 m QR geçişi, 2 geçiş tek uçuşta (2026-11-22) GEÇTİ
Yöntem: `UAV0=tulpar CAM=T10 LORA=1 ./launch_all.sh` (kamera köprüsü + `qr_online` canlı + `lora_relay`; arena `iha_arena`; uav1 yerde) → `UAV0_MODEL=tulpar python3 mavlink_takeoff.py --uav 0 --el-firlatma --alt 100` → `qr_watch.py` + `qr_pass.py --pass-id N --alt 60 --bias 0` (ψ 210°, 15 m/s, varsayılan `--end 150`) iki kez art arda (aynı uçuş); `gz_gercek_poz.py` (50 Hz gerçek poz). Analiz: `test/qr_gecis_analiz.py` (yeni; `degerlendirme.csv` + gerçek poz + ulg). Kayıtlar `test/adim5_2_2026-11-22/` (ulg ve tam gerçek-poz CSV'si depoda yok; poz ±8 sn pencereli).
| ölçüt | hedef | geçiş 1 | geçiş 2 |
|---|---|---|---|
| QR bulma / çözme (kadrajdaki kareler) | %100 | **8/8 – 8/8** | **6/6 – 6/6** |
| kadrajda süre (kare) | — | 0,9 s (8 kare) | 0,8 s (6 kare) |
| hat yanal sapması x [m] (s∈[−400,+150]) | — | ort −0,08, std 0,03, maks 0,17 | ort −0,14, std 0,04, maks 0,24 |
| hedef üstü yanal / dikey [m] | — | −0,06 / −0,17 | −0,18 / +0,16 |
| irtifa (kamera–QR 3B) | 60 ± 15 | 59,8–60,4 m | 60,1–60,6 m |
| konum hatası, gerçek-poz yöntemi | ≤ 0,1 m | **5,7 cm** (maks 6,6) | **6,4 cm** (maks 7,7) |
| konum hatası, PX4-tam (yatay) | kaydedilir | ort **0,55 m** (0,45–0,61) | ort **0,44 m** (0,41–0,48) |
| PX4 − gerçek yatay / dikey | — | 0,38 / 0,03 m | 0,39 / 0,04 m |
| pitch (burun yukarı +) gerçek / PX4 | ölçülür | +2,68° / +2,50° | +4,02° / +3,95° |
| **kadraj kayması (QR merkezi v=544'ten geçerken)** | = 60·tan(pitch) | ölçülen **−2,77 m** (beklenen 2,80) | ölçülen **−4,24 m** (beklenen 4,22) |
| QR merkezi kadrajda: s aralığı | — | −10,4…+4,6 m (15,1 m, 0,98 s) | −10,5…+2,7 m (13,2 m, 0,86 s) |
- Modül boyutu (qr_online yayınları, 14 adet): kenar 76,4–78,3 px → modül **3,64–3,73 px** (statik 60 m T10: 3,69 px, fark ≤ %1,4).
- Kadraj kayması işareti: burun yukarı pitch θ'da aşağı bakan kameranın bakış ekseni h·tanθ **öne** kayıyor (QR merkezi, uçak hedefin h·tanθ gerisindeyken görüntü ortasından geçer); ölçülen kayma beklenen 60·tan(pitch) ile ≤ 0,03 m uyumlu (iki pitch değerinde).
- Güvenlik/ortam: qr_watch RTF (5 sn etkin) 0,98–1,00 (geçiş 2'de 1,00), abort yok, ulg: nav_state yalnız {AUTO_LOITER, OFFBOARD, TAKEOFF}, failsafe bayrağı yok, PX4 uyarı mesajı yok, servo doygunluğu %0 (en büyük |cmd| 0,43); iki araç arası en küçük 3B 59,5 m (geçiş 1) / 60,1 m (geçiş 2) (eşik 40 m; uav1 yerde, dikey ayrım).
- Karşılaştırma (advanced_plane, günlük 2026-10-30/31; bu adımda advanced_plane uçulmadı): 1,6° pitch → kayma ≈ 1,7 m (Tulpar 2,8 / 4,2 m: pitch'e bağlı); QR 0,7–0,8 s / 8–9 kare kadrajda, kadrajdakilerin %100'ü bulundu/çözüldü (Tulpar 0,8–0,9 s, %100); yanal ort −0,25/−0,46/+0,23 m, std 0,05, maks 0,55 (Tulpar −0,08/−0,14, std 0,03–0,04, maks 0,17/0,24); konum hatası gerçek-poz ~6 cm (Tulpar 5,7–6,4 cm); PX4-tam 0,93–2,10 m (2026-10-31, EKF gecikme düzeltmesinden önce) (Tulpar 0,44–0,55 m — n=2, farklı EKF ayarları, doğrudan kıyaslanamaz).
- **Sınırlar (dürüst):** n = 2 geçiş, tek uçuş, aynı bias; pitch iki geçişte farklı (2,7° / 4,0°) ve Hold'daki trim sürüklenmesiyle (5.1) tutarlı. Kare kaybı: qr_online "köprü kaybı" toplam 295 / 4294 (%6,9; advanced_plane 2026-10-30: %2,4); geçiş 2'de 0,86 s'lik pencerede 6 kare (~8,6 beklenir) → ~2–3 kare köprü/DDS'te kayıp, tespit kadrajdaki işlenen karelerde %100. Tespit oranı yalnız *işlenen* kareler üzerinden. `kadrajda_gercek` qr_online'ın geometrik denetimi (tüm QR köşeleri kadrajda?) — kadraj penceresi analizi gerçek pozdan merkez-pikseliyle ayrıca hesaplandı. Yanlış poz denemesi: `gz_gercek_poz.py` ilk koşuda `UAV0_MODEL` verilmeden başlatıldı (yalnız uav1 kaydetti; atıldı, Hold'dayken doğru adla yeniden başlatıldı) — **ayrı kabukta başlatılan uav0 araçlarına `UAV0_MODEL=tulpar` verilmeli** (launch_all.sh yalnız kendi alt süreçlerine verir; `qr_online` bu yüzden doğru çalıştı).
- Kullanım tuzağı: `qr_watch`/`qr_pass`/`mavlink_takeoff` gibi stop desenlerindeki adlar bir komut metninde geçince, aynı kabukta çalışan `launch_all.sh stop` kabuğu öldürür; ayrıca `awk '/python3 gz_gercek_poz.py/'` gibi kendi komut metnini eşleyen filtreler kabuğa sinyal gönderir (bir kez yaşandı, exit 1).

## Adım 5.3 — Tulpar tam kutu taraması, 9/9 hat (2026-11-22) GEÇTİ (`--koridor-yok` ile; koridor denetimli koşu 5/9'da durdu)
Yöntem: `UAV0=tulpar CAM=T10 LORA=1 ./launch_all.sh` → el fırlatma → 100 m Hold → `alan_tarama.py --tam-tarama` (+ `qr_watch.py`); aynı uçuşta **iki koşu**; `gz_gercek_poz.py` (UAV0_MODEL=tulpar). Analiz `test/tarama_analiz.py` (yeni). Kayıtlar `test/adim5_3_2026-11-22/` (ulg yok; gerçek poz 2 Hz'e seyreltilmiş, aşağıdaki gerçek-poz sayıları 20 Hz'lik asıl kayıttan).
**Koşu 1 (varsayılan, koridor denetimi açık):** 5/9 hat, 359 s, ardından "TARAMA DURDU": kalan 4 hat (c = +40, +53,7, +67,5, +81,2) "hat kalkış koridorunu kesiyor ve koridor serbest değil". Neden: tarama planı doğu kalkış koridorunu (uav1 orijin pistinden kalkacak varsayımı) korumak için, öteki araç orijin pistindeyken (uav1 burada (−15,15) yerde, disarmed) bu hatları reddediyor — uav1 hiç kalkmayacak olsa bile. Tespit: hat 5'te (c = −55↑).
**Koşu 2 (`--koridor-yok`, uav1 yerde ve kalkış planı yok):** **9/9 hat, abort yok**, tarama **551 s** (plan tahmini 513 s), tüm hatlar uçuldu → çıkış Hold (−310,−135), `qr_watch` çıkış kodu 0.
| ölçüt | hedef | ölçülen |
|---|---|---|
| hat sayısı / abort | 9/9, 0 | **9/9**, abort **0** (koşu 2) |
| kutu girişi yanal hata medyanı (9 hat) | ≤ 1 m | **0,95 m** (ort 0,96, maks 1,54; hatlar: +0,01 −1,54 −1,18 −0,66 +1,18 +0,95 +0,93 −0,68 −1,49) |
| kutu içi hat |x|maks | — | maks 1,76 m (hat başına medyan 1,25) |
| GERÇEK hat–direk en yakın (gerçek poz, 20 Hz) | ≥ 37 m (advanced_plane 37,5) | **38,6 m** (plan 40; hat 1: 40,1, hat 2: 38,6, hat 5: 39,6, hat 6: 39,7, diğerleri 52–69 m); dönüşlerde 115 m |
| irtifa (hat fazı) | 60 | 60,01 ± 0,33 m (59,1–61,1) |
| tespit | kaydedilir | QR (−60,−60): koşu 1 hat 5 (c=−55↑) ilk yayın (−60,47; −60,05) hata 0,47 m; koşu 2 hat 9/9 (c=−55↓) (−60,11; −59,65) hata 0,37 m; kadrajdaki karelerde 6/6 ve 8/8 bulma+çözme; PX4-tam hata 0,46 / 0,31 m, gerçek-poz 11,9 / 5,6 cm |
| tarama süresi | kaydedilir | 551 s (koşu 2); koşu 1: 359 s (5 hat) |
| uav1'e en küçük 3B ayrım | ≥ 40 m | 65,0 m (koşu 1: 64,7 m; uav1 yerde, dikey ayrım ~60 m) |
| RTF (qr_watch, 5 sn etkin) | — | en düşük **0,996** (koşu 2), 0,994 (koşu 1) |
| gerçek kamera ayak izinden kapsama (300×300 m kutu) | plan %44,4 | **%46,9** (koşu 2, 9 hat; koşu 1, 5 hat: %26,2) |
| ulg | failsafe 0 | failsafe bayrağı **yok**; nav_state {4,14,17} (RTL/LAND yok); servo |cmd| maks 0,57/0,57/0,87/**1,00** (servo3 %0,03 örnekte doygun) |
- **Dönüş yayları (8, R 85 m):** yanal sapma |x|maks ort **16,0 m** (maks 16,5), çıkışta |x_son| ort 9,4 m (maks 15,5); planlanan dönüş–direk 114 m. Advanced_plane (günlük 2026-11-11): 85 m yaylarda ort 11 m, en çok 19 m → Tulpar ortalaması daha **büyük** (yay dışına kayma), maksimum benzer; hat girişleri etkilenmedi (uzatma 100 m yeterli).
- **Roll/pitch (tarama):** roll −38,9…+38,5° (|roll| ort 8,4°; `FW_R_LIM` 35'i yay girişlerinde kısa süre aşıyor), pitch ort 2,88° (std 1,28; −1,9…8,8°).
- Karşılaştırma (advanced_plane, günlük 2026-11-12; bu adımda advanced_plane uçulmadı): 9/9 hat 646/649 s (Tulpar 551 s; hız ~15 m/s aynı, süre farkı hat/yay sırası ve uçuş koşulu farkı olabilir, incelenmedi), giriş yanal hata medyanı 1,0 m (Tulpar 0,95), hat–direk gerçek 37,5 m (Tulpar 38,6), kapsama plan %44.
- **Sınırlar (dürüst):** 9/9 yalnız `--koridor-yok` ile (koridor denetimi bilinçli kapatıldı; gerçek zincirde uav1 uzak üste (IHA_UAV1_SPAWN_NE 200,−950) olduğundan zaten bu bayrak gerekir); tek uçuş, tek tam koşu (koşu 1 kısmi); QR yalnız c=−55 hattında görüldü (diğer hatlar şerit dışı); kapsama kendi hesabımız (kamera ayak izi, yer z=0, 1 m ızgara) — `kapsama_analiz.py` `bap_geo` yok → çalıştırılamadı; yay sapması advanced_plane'den büyük, nedeni incelenmedi; `servo3` tek kısa doygunluk (ulg örneği).

## Adım 5.4 — Tulpar zinciri: tarama → tespit → LoRa → uav1 yerden kalkış → dalış → iniş (2026-11-22) ZİNCİR TAMAMLANDI (dalış gerçek 3B 20,21 m: hedef bandının 0,21 m üstünde)
Yöntem: `UAV0=tulpar CAM=T10 LORA=1 UAV1_POSE=-950,200,0.5,0,0,0 IHA_UAV1_SPAWN_NE=200,-950 ./launch_all.sh` (dünya iha_arena; **ROS ortamı launch_all'dan ÖNCE kaynaklanmamalı** — kaynaklanınca gz sunucusu başlamıyor, PX4 "Waiting for Gazebo world" ile zaman aşımına uğruyor, 2 denemede yaşandı) → uav0 `mavlink_takeoff.py --el-firlatma --alt 100` → `lora_yaklasma.py --uav 1 --profil dalis --alcalma-siniri 4.0 --baslangic yer --kaynak-fark-n 200 --kaynak-fark-e -950 --pist-yonu 90 --guvenli-irtifa 120` → `alan_tarama.py --ilk-hat -40 --koridor-yok`. Betikler ve kayıtlar `test/adim5_4_2026-11-22/` (ulg yok; gerçek poz 2 Hz'e seyreltilmiş). Analiz `test/zincir_analiz.py` (yeni). **Tek uçuş, tek zincir.**
| ölçüt | hedef / referans | ölçülen |
|---|---|---|
| zincir tamam (abortsuz) | evet | **evet** — tespit → röle → kalkış → dalış → tırmanış → Hold → dönüş → iniş → disarm, abort yok |
| dalış durması, gerçek 3B (gz) | 19,8–20,0 m (ref. 19,74–19,98) | **20,21 m** (yatay 12,49, irtifa 15,91) — bandın 0,21 m ÜSTÜNDE; node'un PX4 tahminiyle 19,87 m (yatay 12,62, irtifa 15,35), durma nedeni 3B mesafe |
| uçuş yolunun hedefe en yakın 3B'si | — | 14,52 m (yatay 0,78 m): durmadan sonra tırmanış hedefin tam üstünden 14,5 m'de geçiyor; dip irtifa 14,2 m (node) / 14,4 m (gz) |
| dalış açısı / profil izleme | plan 8,19° | ölçülen 6,8°; hat girişi yanal −0,06 m, irtifa 38,3 m (hedef 38,3); dalışta yanal ≤ 0,1 m |
| konum kestirimi (zincirin kullandığı ilk yayın) | PX4 yöntemiyle ≤ 3 m | (−60,40; −59,70) → **0,50 m** (gerçek (−60,−60)); yalnız ilk yayın hesaplandı, tüm yayınların medyanı çıkarılmadı |
| tespit | — | hat 3/9 (c=−70↑), tarama 189 s, tespit 163,9. sn; uav0 Hold'u tespitten 37 sn sonra oturdu |
| tespit → uav1 kalkış komutu / harekete geçiş / tırmanış sonu (Hold) | ref. 42,5–45,0 / — / — | 37,6 s / 49,1 s / 141,1 s; tespit → iniş algılama 330 s (node toplam 330,2 s) |
| uav1 kalkış | ref. teker kesme 6,4–7,1 s | arm 3,8 s, teker kesme 8,42 s (komuttan), yer koşusu 26,0 m, 25,2 m/s |
| iki araç 3B ayrım (ikisi havada) | ≥ 40 m | en küçük 300 m (dalış anında 319 m) |
| uav1 / uav0 en yakın direk (yatay) | ≥ 37 m | uav1 47,2 m; uav0 38,6 m |
| iniş | ref. yanal ≤ 0,5 m, boyuna +52…+78 m | yanal −0,2 m, boyuna **+67,2 m**, disarm 2,1 s; son konum üsse 17 m |
| ulg failsafe | 0 | uav0 0/1476, uav1 0/1402 örnek; nav_state uav0 {4,14,17}, uav1 {3,4,14,17}; accel_bias 0 (EKF2_IMU_CTRL=5) |
- **Karşılaştırma (advanced_plane, 2026-11-16 yerden kalkış):** dalış durması 3B 19,74–19,98 m, dip 13,4–13,8 m, açı 6,8°; burada 3B bandın hemen üstünde (gz 20,21 m), dip 14,2 m, açı 6,8°. Fark Tulpar'dan değil uav1 tarafının tek örneğinden de gelebilir; n=1.
- **Sınırlar (dürüst):** tek uçuş/zincir, istatistik yok. RTF uçuş boyunca sürekli kaydedilmedi (`qr_watch` çalıştırılmadı): yalnız kalkıştan sonra 0,999 ve iniş sonrası 1,000 ölçüldü. `--koridor-yok` kullanıldı (uav1 uzak üste olduğu halde planlayıcı koridoru orijin pistine göre koruyor). PX4 EKF xy reset sayacı uav0'da 4, uav1'de 5 (başlangıçtaki değeri ayrıca doğrulanmadı). uav1'in dalıştan sonraki yolu hedefin hemen üstünden 14,5 m'de geçiyor (önceki uçuşlarda aynı ölçü çıkarılmadı). iz2.log boş: iz.csv yine yazıldı (PYTHONPATH gerekir).
