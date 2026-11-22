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
