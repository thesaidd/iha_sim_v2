# gercek_ucak test sonuçları

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
- Airframe PX4'e **4030_gz_gercek_ucak** adıyla eklendi (4009 r1_rover'a ait; çakışma). PX4'te değişen: `ROMFS/.../airframes/4030_gz_gercek_ucak` (yeni) + `CMakeLists.txt` +1 satır (yedek `yedek/airframes_CMakeLists.txt.bak-2026-11-21`). `make px4_sitl_default` hatasız.
- Ortam: `gercek_ortam.sh start|stop` (Agent + gz sunucusu + PX4 instance 0, penceresiz, PID ile kapatma). Model adı gz'de `gercek_ucak_0`.
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
Yöntem: `gercek_ortam.sh start` + `test/gercek_firlat.py 12 <süre>`: parametre yazma (geri okumalı) → Takeoff modu + arm (launch-detect bekler) → modeli yerden 1,5 m'ye ışınla, 10° burun yukarı → 0,3 sn ApplyLinkWrench darbesi (a = 40 m/s², hedef ≈ 12 m/s; gz.transport13 ile hassas zamanlama; CG ofseti için tork telafisi) → izleme. Kayıt: `A5_basarili_hold.ulg`.
Sonuç: "Takeoff detected" → motor 0,9 → 40 m'ye < 6 sn; ~40 m'de Hold (AUTO_LOITER) 2,5+ dk kararlı: irtifa 40,0 ± 0,3 m, hız 13–17 m/s, pitch 2–6°, roll ≈ +16° (sabit daire), failsafe yok.
Bulunan/çözülen hatalar (sırayla):
1. Float parametreler INT32 tipiyle yazılınca reddediliyordu (MIS_TAKEOFF_ALT 25'te kalıyordu) → tip tablosu.
2. **Ruddervator YAW işareti ters** (yaw sağ komutu → sola dönüş; PX4 yaw döngüsü pozitif geri besleme, ~1 sn periyotlu salınım büyüyüp devrildi) → CY/Cen işaretleri çevrildi. Elevatör (Cem) işareti ORİJİNALİ doğru: harness'te `pitch` gz konvansiyonunda (+ = burun AŞAĞI) okunmuştu; yanlışlıkla çevirip geri aldım (not: `ucus_testi.py` çıktısında pitch<0 = burun yukarı, gam>0 = tırmanış).
3. Kontrol yüzeyi katsayıları şablonun (1 kg RC Cessna) 2,5–4× fazlaydı → fiziksel tahminle ölçeklendi: aileron Cell ±0,0009/°, ruddervator Cem 0,0052/°, CL −0,0020/°, yaw Cen ∓0,0008/°, CY ±0,0019/° (yüzey başına). **Birim: derece başına** (servo komutu rad; eklenti çevirir — ölçümle uyumlu).
4. El fırlatma darbesi link orijininden uygulanınca CG ofseti (z 0,0265) yüzünden büyük burun-yukarı tork veriyordu → tork telafisi (`COMP=1,0` hafif fazla; 0,75 daha nötr).
5. PX4 kapalı çevrim ayarı: aileron etkinliği düşünce roll gain artırıldı: FW_RR_P 0,03→0,08, FW_RR_FF 0,05→0,12, FW_R_LIM 35 (roll ±60° salınımı → sabit 16°); FW_TKO_PITCH_MIN 10, FW_P_LIM_MAX 22, FW_PR_I 0,1, FW_RR_I 0,05 (rampa sonrası integral).
Tüm ayarlar `4030_gz_gercek_ucak` ve `gercek_firlat.py PARAMS` içinde; sim-özgü olanlar (FD_ESCS_EN, ASPD_DO_CHECKS, ASPD_SCALE_APPLY, NAV_DLL_ACT, EKF2_IMU_CTRL) gerçek uçağa taşınmaz.
Açık: fırlatma hızı gerçek eli temsil eden bir model değil (ışınlama + darbe); daire roll'u 16° (yarıçap ~?); motor itki eğrisi, CG, aero tahmin; manuel/gamepad kumanda denenmedi; kamera yok.

## Video (2026-11-21)
Kameralı varyant `gercek_ucak_kam` (arkadan takip + burun kamerası, 30 Hz, gz sensörü); 1428 kare/48 sn sim zamanı, `video/chase.mp4` + `video/burun.mp4`. El fırlatma → tırmanış → Hold dairesi (roll ~16°) kaydedildi. Burun kamerasında boş dünyada yalnız zemin/ufuk görünür (doku yok).
