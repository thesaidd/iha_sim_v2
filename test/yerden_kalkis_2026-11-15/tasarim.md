# uav1 "yerden kalkış" senaryosu — tasarım (Aşama 0, 2026-11-15)

Kod: `yer_gorevi.py` (yeni; plan + kalkış + dönüş + iniş), `lora_yaklasma.py --baslangic yer|hava` (varsayılan `yer`;
`hava` = eski davranış, değişmedi), kuru test `yer_gorevi_test.py` (66 doğrulama). Yardımcı değişiklikler:
`launch_all.sh` `UAV1_POSE`, `ayrim.py` `IHA_UAV1_SPAWN_NE`.

## 1. PX4 v1.17 kaynak bulguları
| Konu | Kaynak | Bulgu → tasarım kararı |
|---|---|---|
| Pist yönü | `fw_mode_manager/FixedWingModeManager.cpp` `control_auto_takeoff` | Kalkış hattı = arm anındaki yaw (`_launch_current_yaw`), başlangıç noktasından. → Pist yönü `UAV1_POSE` yaw'ı ile verilir; düğüm başlarken yaw ↔ `--pist-yonu` farkı > 10° ise başlamaz. |
| Kalkış irtifası | aynı + `navigator/takeoff.cpp` | Açıklık irtifası = `MIS_TAKEOFF_ALT`; CLIMBOUT'ta gaz `RWTO_MAX_THR`, pitch ≥ `FW_TKO_PITCH_MIN`. İrtifa geçilince navigator **o noktada** Hold çemberine döner (`GO_TO_LOITER`). → `MIS_TAKEOFF_ALT` = 25 m (`--kalkis-irtifa`); OFFBOARD bu geçişte devralınır, 38,3 m'lik hat girişine TECS ile tırmanılır (düşük irtifada devralma = daha az hız / irtifa taşması). |
| Erken arm | `commander` `COM_DISARM_PRFLT` (10 sn) | Arm edip kalkmayan araç disarm olur. → Arm, kalkış komutuyla birlikte (`mavlink_takeoff.take_off`: NAV_TAKEOFF + ARM, yeniden denemeli — yeniden yazılmadı). |
| Pist hizalı iniş | `FixedWingModeManager.cpp` mod seçimi (satır ~430) | `SETPOINT_TYPE_LAND` + **önceki yol noktası geçerli** → `AUTO_LANDING_STRAIGHT`; "yalnız görevde mümkün". Land modu ve RTL (RTL_TYPE 0) `AUTO_LANDING_CIRCULAR` → pist hizası yok. → **Görev iniş deseni** seçildi. |
| İniş deseni | `navigator/MissionFeasibility/FeasibilityChecker.cpp` `checkFixedWingLandApproach`, `mission_block.cpp` (teğet çıkış) | LAND'den önceki öğe WAYPOINT ya da LOITER_TO_ALT olmalı; süzülme ≤ `FW_LND_ANG` (airframe 4008: 8°). LOITER_TO_ALT + `loiter_exit_xtrack` → çıkışta setpoint teğet noktaya çekilir, yaklaşma hattı teğet → iniş noktası. → Görev: `LOITER_TO_ALT` (R 80, 25 m, yön şartı + teğet çıkış) → `LAND`. `DO_LAND_START` gerekmez. Süzülme 25 / 350 m = 4,1°. |
| Flare / arazi | `getLandingTerrainAltitudeEstimate` | `FW_LND_USETER` > 0 iken mesafe sensörü geçerli ölçüm vermezse `FW_LND_ABORT` (3) inişi iptal eder. → `FW_LND_USETER = 0` (sim zemini düz, iniş noktası irtifası kesin). |
| Disarm | `COM_DISARM_LAND` (2 sn) | Yer algılanınca otomatik. Düğüm 20 sn'de olmazsa DISARM komutu gönderir. |

**Parametre listesi `yer_gorevi.INIS_PARAMS`** (yalnız uav1, yalnız simülasyon senaryosu; `SIM_ONLY_PARAMS` tarzı, ayrı liste):
`FW_LND_USETER` 0 (geri alma 1), `MIS_TKO_LAND_REQ` 0 (varsayılan; açıkça sabitlendi). Düğüm ayrıca `SIM_ONLY_PARAMS`
ve `UCUS_DINAMIGI_PARAMS[1]` (ayar) değerlerini uav1'e kendisi yazar; `MIS_TAKEOFF_ALT` `take_off` içinde.

## 2. Üs seçimi
Kısıtlar: doğu = uav0 kalkış hattı + kalkış Hold'u; kuzey / güney = tarama dönüş yayları ve çıkış Hold'ları
((±310, c ± 80), 60 m). Aday üsler arena merkezinden kerteriz × uzaklık; pist arena merkezine bakar. Her aday için
9 tarama hattı × 8 kuzey konumu × 2 çıkış Hold'u = 144 durumda `yer_plani` çalıştırıldı (direk payı 45 m, uav0 Hold'una
3B ≥ 110 m, ±20° sapma, dönüş rotası + iniş deseni dahil).

| Üs (uav0 N, E) | Pist | Plan var | Not |
|---|---|---|---|
| (0, −650) batı | 90° | 38 / 144 (%26) | B direği tam hat üstünde |
| (0, −750) | 90° | 46 (%32) | |
| (0, −850) | 90° | 51 (%35) | |
| (−287, −693) BGB | 67,5° | 36 (%25) | |
| (287, −693) BKB | 112,5° | 65 (%45) | |
| (325, −785) BKB | 112,5° | 74 (%51) | gerçek QR için sapma −20° (sınırda) |
| **(207, −773)** kerteriz 285°, 800 m | 105° | **72 (%50)** | **gerçek QR (−60, −60) dört çıkış Hold'unda da uygun, sapma −15°** |
| (460…600, −460…−600) KB | 135° | 68–70 (%47–49) | gerçek QR için plan yok |
| (0, +650…850) doğu | 270° | 63–72 (%44–50) | uav0 kalkış hattı / Hold'u — elendi |
| (±650…850, 0) kuzey / güney | 0 / 180° | 0 | tarama dönüşleri + çıkış Hold'ları |

**Seçilen üs: uav0 NED (200, −775), pist yönü 105°** (yuvarlanmış; 65 / 144 = %45, gerçek QR 4 / 4).
`UAV1_POSE="-775,200,0.5,0,0,-0.2618"`, `IHA_UAV1_SPAWN_NE="200,-775"`, `--kaynak-fark-n 200 --kaynak-fark-e -775 --pist-yonu 105`.
Erişilemeyen durumların nedeni direkler (kutunun içinde, boy 140 m): hat ya da ~540 m'lik tırmanış uzantısı bir direğin
45 m'sine giriyor ve ±20° sapma yetmiyor. Üsten kutu merkezine 800 m; en yakın kutu köşesine ~640 m.

## 3. Plan algoritması (`yer_gorevi.yer_plani`, kalkıştan ÖNCE)
1. Kalkış sonu noktası P = üs + `kalkis-mesafe` (120 m, uçuşta ölçülecek) · pist yönü; kerteriz = P → hedef.
2. Adaylar: kerteriz + {0, ±5, ±10, ±15, ±20}°. Her aday için sırayla:
   - **Katılma:** kaba kinematik öngörü (en yakın hat noktasını izleme, yanal ivme 3,5 m/s², hız 22 → 15 m/s) → uçak
     alçalma başından en az yarım düz-giriş (37,5 m) önce hatta oturmalı (|x| ≤ 5 m, rota ≤ 5°).
   - **Engeller:** katılma izi (pay 45 + 25 m iz payı), hat A → S (profil irtifası), tırmanış S → C (tahmini ~540 m —
     istenen 320 m'den uzun, tamamı denetlenir), C'deki Hold çemberi (güvenli mod için).
   - **uav0:** çıkış Hold çemberine (yarıçap 80 m, irtifa ± 10 m) en kötü durum 3B ≥ 110 m — `ayrim.bacak_ayrimi`
     ile, örnek örnek; C çemberi için `ayrim.cember_ayrimi` (40 m).
   - **Dönüş:** C → iniş orbit merkezi, güvenli irtifada, düz bacaklar (0, 1 ya da 2 ara nokta); her bacak pay 45 + 25 m,
     her köşe dönüş diski (80 / 150 m), orbit çevresi 250 m (görev devri) ile denetlenir.
   - Pist + kalkış uzantısı + iniş hattı yerde tüm engellere karşı.
3. Uygun yön yoksa **NO-GO: uav1 arm edilmez**, neden loglanır. İniş görevi PX4'e yüklenemezse de NO-GO.
4. Denetimsiz bacak yok; `qr_pass`'teki "3 denemeden sonra denetimsiz git" geri dönüşü bu kodda yok.

## 4. Akış ve güvenli modlar
| Aşama | Yürüten | Başarısızlıkta |
|---|---|---|
| Yerde bekleme (disarmed) | düğüm + `mavlink_takeoff.Vehicle` kalp atışı | beklenmedik arm → durur |
| uav0 Hold'unun ölçülmesi → plan → görev yükleme | `Uav0Izleyici`, `yer_plani`, `gorev_yukle` | NO-GO, kalkış yok |
| Arm + pist kalkışı | `mavlink_takeoff.take_off` | teker kesilmezse DISARM; kesilip irtifaya çıkamazsa Hold |
| OFFBOARD + katılma + hat + durma + tırmanış | `lora_yaklasma._dalis_akisi` (mevcut kod; yalnız giriş kapısı farklı) | mevcut kapılar (RTF, uav0, irtifa tabanı, hat dışı) → tırmanış + Hold |
| Dönüş (OFFBOARD) | `yer_gorevi.donus_inis` | Hold |
| Görev inişi | PX4 (`AUTO_MISSION`) | görev moduna geçilemezse / PX4 inişi bırakırsa / zaman aşımı → Hold |

## 5. Yorum gerektiren noktalar
- "uav0 çıkış Hold'una ≥ 110 m (ayrim.py ile)": Hold **çemberine** (yarıçap 80 m) en kötü durum 3B ≥ 110 m olarak
  uygulandı (yatay boşluk + irtifa bandı ± 10 m). Merkeze 110 m okunsaydı çembere 30 m'ye izin verirdi.
- "320 m tırmanış uzantısı": gerçek tırmanış ~480–540 m sürüyor; tamamı denetleniyor.
- İniş yönü = kalkış yönü, desen üssün arenadan uzak tarafında (arena tarafındaki ilk tasarımda orbit B direğine
  yaklaştığı için hiçbir plan çıkmıyordu).
- Kuru testteki araç modeli kabadır; kalkış mesafesi / hızı ve katılma davranışı Aşama 1–2'de ölçülüp plana işlenecek.

---

# KÖK NEDEN, DÜZELTMELER ve SONUÇLAR (2026-11-16)

## 1. Aşama 1 kalkış arızasının kök nedeni: spawn yaw ≠ 0 (simülatör aerodinamiği)
İlk denemede (a1, üs (200, −775), pist 105°) uav1 arm oldu, pistte 13 m/s'ye çıktı, sonra 7–8 m/s'ye düşüp
40 sn yuvarlandı, hiç havalanmadı. Teşhis sırası:

| Adım | Bulgu |
|---|---|
| ULG A (başarısız) ↔ B (başarılı referans, 2026-11-13) | **1105 parametrenin tamamı aynı**; gaz iki koşuda da 0,85 (RWTO_MAX_THR); runway state ikisinde de CLIMBOUT'a geçti. A'da elevatör 5,5 sn'den sonra **1,00'de doygun**, pitch −1,3…+0,3°'de kaldı; B'de elevatör 0,58 ile 4,3 sn'de rotasyon, pitch 31°. |
| Kod karşılaştırması (`yer_gorevi` ↔ `mavlink_takeoff`) | Kalkış mantığı yeniden yazılmamış (`mt.take_off` çağrılıyor); parametre kümesi aynı. Tek fark: `MIS_TAKEOFF_ALT` 25 (vs 120), ön yüklenmiş iniş görevi, `FW_LND_USETER`/`MIS_TKO_LAND_REQ`. Hiçbiri yer koşusu fiziğini etkilemez. |
| Zemin | `iha_arena_v2.sdf` zemini TEK sonsuz `plane` (varsayılan sürtünme); görsel karolar yalnız ±350 m — fizik her yerde aynı. Uçak üste normal oturdu (gz z = 0,125, `landed` 1). |
| **Ayırıcı deneme (t1)** | Aynı koşuda **değiştirilmemiş `mavlink_takeoff.py`** ile uav0 (spawn yaw 0) normal kalktı, uav1 (yeni üs, spawn yaw −0,2618 = pist 105°) yine kalkamadı → sorun yer_gorevi'nin sırasında değil. |
| **Kesin kanıt (groundtruth)** | uav1: gövde yönü (gt yaw) 105,0°, **iz yönü 90,3° (tam doğu)**, yanal kayma sabit −15°, **gt pitch 0,000°** (elevatör doygunken bile). uav0: yaw 90°, iz 89,8°, yanal kayma ≈ 0, pitch 31°. Uçak, yaw'ından bağımsız olarak dünya X (doğu) ekseni boyunca kayıyor ve hiç yunuslamıyor. |

**Sonuç:** `advanced_plane` modelinin `AdvancedLiftDrag` aerodinamiği (`<forward>1 0 0</forward>`, `<upward>0 0 1</upward>`)
pratikte yalnız **gz yaw 0** (NED 90°, doğu) spawn'ında doğru çalışıyor; başka yaw'da kuvvetler gövdeyle dönmüyor.
Projede bugüne kadarki tüm başarılı kalkışlar yaw 0 ile yapılmıştı; yaw ≠ 0 hiç denenmemişti. PX4 ağacına ve
model dosyasına dokunmak yasak olduğundan **tasarımla kaçınıldı: pist yönü 90° zorunlu** (`yer_gorevi.sim_yaw_uygun`,
düğümde `--yaw-denetimi 1` ile kalkıştan önce reddedilir). Gerçek donanımda böyle bir kısıt yoktur.

## 2. Yeni üs: (N 200, E −950), pist yönü 90°
Pist 90° kısıtıyla üs taraması yeniden yapıldı (8 kuzey × 4 doğu konumu × 144 hedef/çıkış-Hold durumu):

| Üs (uav0 NED) | Plan var | Gerçek QR (4 çıkış Hold'u) |
|---|---|---|
| (250, −750) | 74/144 (%51) | −20° (sapma sınırında) |
| (200, −850) | 69/144 (%48) | −15° |
| **(200, −950)** | **68/144 (%47)** | **−10° (pay var)** |
| (100, −950) | 57/144 (%40) | −5° |

Seçilen **(200, −950)**: kapsama en iyilerden biri ve gerçek hedefe ±20° sınırına 10° pay bırakıyor.
`UAV1_POSE="-950,200,0.5,0,0,0"`, `IHA_UAV1_SPAWN_NE="200,-950"`, `--kaynak-fark-n 200 --kaynak-fark-e -950 --pist-yonu 90`.

## 3. Diğer düzeltmeler
| Sorun (kanıt) | Düzeltme |
|---|---|
| **Kalkış başarısızlığında güvenli mod yoktu**; a1'de PX4 "Takeoff detected" (yerde, 11,7 m/s) dediği için normal DISARM "not landed" diye reddedildi ve araç 40 sn yuvarlandı | `yer_gorevi.guvenli_dur()`: tetikler = 15 sn'de 3 m irtifa kazanılmaması (`--kalkis-irtifa-asimi`, `--kalkis-irtifa-esik`), pist sonu aşımı (`--pist-boyu` 400 m), teker kesme/irtifa zaman aşımı. Davranış: **yerdeyse** (irtifa < 1 m **ve** \|vz\| < 0,5, ölçümle doğrulanır) normal DISARM; reddedilirse **ZORLA DISARM (param2 21196)** → motor kesilir, araç yerde durur. **Havadaysa zorla disarm ASLA**, Hold'a geçilir. Havalanma yalnız irtifa + dikey hızla doğrulanır ("Takeoff detected"e güvenilmez) |
| **`MIS_TAKEOFF_ALT` yarışı:** parametre yazıldıktan 0,2 sn sonra NAV_TAKEOFF verilince navigator eski değeri okudu ("Using default takeoff altitude: 120.0 m") | `kalkis_komutu()` irtifayı önce yazar, `--param-bekleme` (1,5 sn) bekler, **geri okuyup doğrular**, sonra `mt.take_off` çağrılır (kanıtlanmış sıra değişmedi). Uçuşlarda artık "Using default takeoff altitude: 25.0 m" |
| **İniş pist ekseninin 68–81 m kuzeyine düştü** (a3, a4). ULG: PX4 görev dizini (`mission_result.seq_current`) `dataman`'da KALICI; önceki uçuş 1. öğede (LAND) bittiği için yeni görev yüklenince dizin sıfırlanmadı → AUTO_MISSION doğrudan LAND'den başladı, yaklaşma yol noktası olmadığı için PX4 **DAİRESEL** iniş yaptı (iz: iniş noktası çevresinde R 80 m çember) | `gorev_basa_al()`: yüklemeden sonra `MISSION_SET_CURRENT(0)` + `MISSION_CURRENT` ile **doğrulama** (olmazsa yükleme başarısız → kalkış yok). Ek olarak dönüşün son bacağında `--inis-desen-irtifa` (60 m) irtifasına inilir (kanıtlanmış a2 koşusuyla aynı devir koşulu; plan bu bacağı o irtifada denetler) |
| `launch_all.sh stop` deseni | `yer_gorevi\.py` eklendi (pkill tuzağı: kapatma/başlatma komutunun METNİNDE bu adlar geçmemeli) |

Kuru test `yer_gorevi_test.py`: 66 → **83 doğrulama** (yeni: güvenli durdurma tetikleri, zorla disarm yalnız yerde,
havada disarm yok, pist sonu kapısı, MIS_TAKEOFF_ALT doğrulaması, takılı görev dizini).

## 4. Uçuş sonuçları (2026-11-16)
Tüm koşular: üs (200, −950), pist 90°, `CAM=T10 WORLD=iha_arena_v2`; a5/s1/s2 düzeltmelerin tamamıyla.

| Koşu | İçerik | Sonuç |
|---|---|---|
| a1 (11-15) | Aşama 1, eski üs/pist 105° | **BAŞARISIZ** — kalkamadı (kök neden yukarıda) |
| t1 | Ayırıcı teşhis (uav0 + uav1, değiştirilmemiş kalkış betiği) | uav0 ✔ / uav1 ✘ → yaw kısıtı kanıtlandı |
| **a2** | Aşama 1: kalkış → dönüş → iniş | ✔ ilk denemede |
| a3 | Aşama 2: bilinen hedef → dalış → iniş | Dalış ✔ (3B 19,98 m), **iniş dairesel** (yanal −67,7 m) |
| a4 | Aşama 2 (yalnız irtifa düzeltmesi) | Dalış ✔, iniş yine dairesel (yanal −81,3 m) → asıl neden görev dizini |
| **a5** | Aşama 2 (görev dizini + irtifa düzeltmesi) | ✔ **iniş yanal −0,0 m** |
| **s1** | **Aşama 3: tam zincir + geniş video** | ✔ ilk denemede |
| **s2** | Aşama 3 tekrar (uav1 yakın çekim) | ✔ zincir tamam; **video bozuk** (kaydedici hatası, §5) |

### Kalkış (5 koşu)
| Ölçüt | a2 | a3 | a5 | s1 | s2 |
|---|---|---|---|---|---|
| Komut → teker kesme [s] | 6,44 | 6,36 | 6,36 | 6,89 | 7,13 |
| Yer koşusu [m] | 27,0 | 26,3 | 26,4 | 25,5 | 26,0 |
| Teker kesme yer hızı [m/s] | 18,7 | 18,5 | 18,6 | 18,0 | 18,4 |
| 25 m irtifa [s] / yatay [m] | 8,3 / 72 | 8,3 / 72 | 8,3 / 74 | 9,3 / 71 | 9,5 / 73 |
ULG (a2, groundtruth): teker kesme arm'dan 4,46 s, koşu 21,1 m, 16,3 m/s; 25 m'ye 6,76 s / 75,3 m; koşuda yanal sapma 0,02 m; en yüksek pitch 30,4°.

### Yaklaşma hattı ve dalış (Aşama 2–3)
| Ölçüt | a3 | a5 | s1 | s2 |
|---|---|---|---|---|
| Hat girişi yanal [m] | +0,13 | +0,07 | +0,11 | +0,13 |
| Hat girişi irtifa [m] (hedef 38,3) | 37,7 | 37,4 | 37,4 | 37,5 |
| Hat girişi rota hatası [°] | −0,4 | −0,3 | 0,0 | −0,2 |
| Dalışta en büyük yanal [m] | 0,6 | 0,6 | 0,4 | 0,6 |
| **Durma 3B [m]** (hedef 20) | 19,98 | 19,88 | 19,92 | 19,74 |
| Durma irtifası [m] | 14,8 | 14,4 | 14,6 | 14,5 |
| Ölçülen dalış açısı [°] | 6,8 | 6,8 | 6,8 | 6,8 |
| Dip irtifa [m] | 13,8 | 13,4 | 13,5 | 13,5 |
| Direklere en küçük plan açıklığı [m] | ≥ 45 (direk_B 45,8) | ← | ← | ← |

### İniş
| Ölçüt | a2 | a3 ✘ | a5 | s1 | s2 |
|---|---|---|---|---|---|
| Teker koyma yanal sapma [m] | +0,1 | −67,7 | **−0,0** | **+0,3** | **−0,5** |
| Boyuna sapma (Td'ye göre) [m] | +63,8 | +52,0 | +58,5 | +70,9 | +78,0 |
| Teker koyma dikey hızı [m/s] | 0,38 (ulg) | — | 0,15 | 0,12 | 0,20 |
| Teker koyma yer hızı [m/s] | 11,4 (ulg) | — | 0,3* | 0,2* | 0,2* |
| Sekme / devrilme | yok / yok (ulg: maks 0,15 m, roll < 1°) | — | yok | yok | yok |
| Rulo [m] | 12,2 (ulg) | — | — | — | — |
| Disarm [s] | 2,0 | 2,0 | 2,1 | 2,6 | 2,7 |
| Üsten son uzaklık [m] | 13,9 | 67,8 | 8,5 | 21,0 | 28,1 |
\* düğüm "yer algılandı" anını PX4 land_detected ile alır (teker koymadan ~4 s sonra, araç durmuş olur); gerçek
teker koyma hızları ULG'den (a2) alınmıştır. Boyuna +52…+78 m: flare süzülmesi iniş noktasını aşıyor (sistematik).

### Zincir (Aşama 3)
| Ölçüt | s1 (geniş video) | s2 (yakın çekim) |
|---|---|---|
| Tarama | 5/9 hat, 438 s | 1/1 hat (`--hat-sec -55`), 64 s |
| QR tespiti (gerçek) | (−61,1; −56,9) | (−62,5; −61,6) |
| Tespit → uav0 Hold ölçümü [s] | 41,7 | 43,2 |
| **Tespit → kalkış komutu [s]** | 42,5 | 45,0 |
| Kalkış → hat girişi [s] | 64 | 65 |
| Tespit → iniş (toplam görev) [s] | 376,7 | 385,8 |
| İki araç arası en küçük 3B [m] | 785 | 897 |
| RTF (yerde / pencereli uçuş / son) | 0,993 / 0,938 / 0,806 | 0,994 / 0,807 / 0,937 |

## 5. Video
| Dosya | İçerik |
|---|---|
| `ucus/s1/zincir.mp4` | **15,3 dk (9197 kare, 10 fps, 898×558)** — tam zincir, sabit dış kamera (ENU −500, −1150, 620 → bakış (−480, 60, 0)); üst şeritte video zamanı, duvar saati ve AŞAMA etiketi |
| `ucus/s1/zincir.csv` | aşama etiketlerinin video zamanları |
| `ucus/s1/kilit_anlar.jpg` | 8 kilit kare |
| `ucus/s2/uav1_yakin.mp4` | **BOZUK** — takip kameralı kayıtta pencere boyutu değişti, `cv2.VideoWriter` farklı boyutlu kareleri sessizce yok saydı: 5791 kare yazıldı, dosyada 63 kare kaldı. `kayit.py` düzeltildi (ilk karenin boyutuna ölçekleme + uyarı), ama Aşama 3 uçuş bütçesi (2/2) dolduğu için yeniden çekilmedi |

Kilit anlar (`zincir.mp4` video saniyesi, `zincir.csv`'den): **5** uav1 üste hazır (yerde, disarmed) · **38** plan ·
**66** uav0 taramaya çıkıyor · **125** uav0 OFFBOARD (tarama hattı) · **531 QR TESPİTİ** · **558** uav0 tarama bitti,
çıkış Hold'u · **574 uav1 KALKIŞ KOMUTU (röle)** · **581** teker kesme · **583** kalkış irtifası · **586** uav1 OFFBOARD,
hatta katılma · **639** yaklaşma hattı (A) · **657 DALIŞ: 3B 19,9 m'de durdu** · **694** üsse dönüş · **824** iniş deseni
(PX4 görevi) · **906 teker koyma** · **908** disarm.
Not: etiket eşlemesi "OFFBOARD ✔" satırını hem uav0 tarama hem uav1 logunda görüyor; 125 s'deki etiket uav0'a aittir.
