# Tam 300×300 m tarama + zincir uçuşu — 2026-11-12 22:25–22:53 (1/3 uçuş denemesi)

Tek uçuş (23,8 dk havada, abort / failsafe yok), `CAM=T10 LORA=1 WORLD=iha_arena_v2`, kalkış `--ucus-dinamigi`.
Direk payı 40 m, giriş payı 100 m, irtifa 60 m, 15 m/s. Gerçek QR (−60, −60). Ham veri: `ucus/`.

## Koşular
| Koşu | Ne | Sonuç |
|---|---|---|
| u1 (22:28–22:35) | Tam plan, QR hattı sonda (`--ilk-hat -40`) + uav1 dalış düğümü | 3. hatta (−70↑) tespit → tarama 3/9 hatta durdu (224 sn) → **zincir 1 tamam** |
| u2 (22:36–22:48) | Tam plan, tespit taramayı kesmeyecek biçimde (`--icerik` eşleşmez) | **9/9 hat, 646 sn** (plan 513), tespit yalnız −55↓ hattında (9 kare) |
| u3 (22:48–22:53) | Yalnız −55↑ hattı + uav1 dalış düğümü | tespit → **zincir 2 tamam** |

## Madde 9 ölçütleri (bu uçuşun verisi)
| Ölçüt | Ölçüm | Değer | Örnek |
|---|---|---|---|
| **Hedef tespit doğruluğu** | QR gerçekten kadrajdayken bulma / çözme (kare) | 28/28 = %100 / %100 | 28 kare, 3 geçiş |
| | Yanlış tespit (kadraj dışı bulma, yanlış içerik) | 0 | 13 855 kare |
| | Geçiş bazında: hedef şeridi (−55, yanal 5 m) | 2/2 geçişte tespit | |
| | Şerit dışı komşu hat (−70, yanal 10 m) | 1/2 geçişte kadraja girdi ve tespit edildi (↑ evet, ↓ hayır) | |
| | Diğer 7 hat (yanal ≥ 20 m) | 0 tespit (beklenen) | |
| **Konum hatası** (kestirim − gerçek, yatay) | İlk yayın (zincirin kullandığı) | 2,38 / 2,40 / 3,07 m | 3 geçiş |
| | Tüm yayınlar | ort 2,74, medyan 2,57, en çok 3,83 m | 28 |
| | Aynı kareler, gerçek pozla (yöntem hatası) | ort 0,057 m | 28 |
| | PX4 yatay konum hatası (uav0, tüm uçuş, ulg) | ort 1,41, p95 1,89, en çok 3,40 m | |
| **Aktarım gecikmesi** | Kare damgası → `/hedef_konum` yayını | ort 77, en çok 89 ms | 28 |
| | Röle (`/hedef_konum` → `/hedef_konum_role`) | ort 49,3, 32,6–70,0 ms | 28 |
| | Kare → uav1'e varış (toplam) | ≈ 126 ms ort | |
| **Aktarım bütünlüğü** | Röle iletilen / alınan (kayıp modeli %3) | 28/28, kayıp %0 | 28 |
| | Zinciri başlatan ilk mesajın uav1'e ulaşması | 2/2 | |
| **Dalış hassasiyeti** (hedef: 3B 20 m, irtifa 15 m) | Gerçek hedefe en yakın 3B (groundtruth) | 25,6 / 23,2 m (hedeften +5,6 / +3,2 m) | 2 dalış |
| | O andaki yatay mesafe / irtifa | 8,5 m / 24,2 m; 6,2 m / 22,4 m | |
| | Dip irtifa (gerçek) | 22,9 / 21,4 m (taban 8 m) | |
| | Alçalma açısı (gerçek) | 6,9° / 7,3° (plan 12°) | |
| | 3B 20 m tetiklemesi | **0/2** — ikisi de "hedef geçildi" yedeğiyle durdu | |
| | Tırmanış + Hold | 115 m, AUTO_LOITER 2/2 | |
| **Genel görev başarısı** | Zincir (tespit → röle → plan → dalış → tırmanış → Hold), abortsuz | 2/2 | |
| | 3B 20 m ölçütü dahil sayılırsa | 0/2 | |
| | Tarama koşusu abortsuz | 3/3 (tam kutu 1/1) | |
| | Tespit → uav1 Hold süresi | 156 / 162 sn | |
| | Tespit → uav1 harekete geçiş | 101 / 110 sn (uav0 Hold'unun oturması 54 sn) | |

## Tarama (u2, tam kutu)
- 9 hat sırası −40↓ +40↑ −70↓ +67↑ −100↓ +81↑ −85↓ +54↑ −55↓; 1 stadyum + 7 ampul.
- Kutu girişinde yanal hata (m): 6,65 / 0,42 / 2,86 / 3,14 / 0,64 / 0,66 / 0,43 / 3,44 / 1,86 → 9 hattın 5'i > 1 m. Kutu içi en büyük 6,65 m (ilk hat; Hold'dan devralma x = +5,7 m), diğerleri ≤ 3,5 m.
- Dönüş yayı yanal hatası en çok 15,9–18,2 m; 100 m giriş payının sonunda hâlâ 6–16 m olabiliyor.
- Hat–direk gerçek en yakın: u2'de 39,3 m, u1'de 37,5 m (+40 hattı; plan 40,0).
- **Direk payı 20 m ölçütü (her hatta giriş ≤ 1 m) SAĞLANMADI → pay 40 m'de kalır.**

## Ayrım, RTF, sağlık
- İki araç en küçük 3B: taşıma sonrası 115 m; taramada ≥ 232 m, tam kutuda ≥ 321 m. Kalkış Hold'larında (taşıma öncesi) 66 m (bilinen açık).
- uav1–direk ≥ 41,9 m (dalış 1 çıkışı, irtifa 29 m).
- Etkin RTF: yerde 1,000; Hold 0,93; tarama/zincir 0,89–0,98.
- PX4 (ulg): failsafe 0, RTL/LAND yok, ivmeölçer uyarısı yok, zaman damgası boşluğu yok. `estimator_sensor_bias` accel alanları 0 (EKF2_IMU_CTRL=5; bias kestirilmiyor).
- uav1 PX4 yatay konum hatası ort 1,69, p95 2,80, en çok 5,33 m.

## Açıklar
1. Dalış 20 m'ye inmiyor: gerçek açı ~7° (plan 12°), hedef üstünde irtifa 22–24 m ve yanal 5–8 m sapma (önceki uçuşta 19,9 m / 2,0 m). Örnek 2; neden incelenmedi.
2. Tespit şeridi dışındaki hat (yanal 10 m) yöne göre bazen tespit veriyor → tam taramada tespit sırası plandan önce gelebilir (u1'de 3. hat).
3. Konum hatası 2,4–3,8 m; PX4 konum hatasından (1,1–1,4 m) büyük, yöntem hatası 6 cm → kalan kısım yönelim/zaman eşleşmesi olabilir, incelenmedi.
4. Tam tarama 646 sn (plan 513): hizalama + yay sonrası toparlanma.

---

# GENİŞLETİLMİŞ madde 9 tablosu — 4 uçuş, 6 zincir, 7 tespit geçişi (2026-11-13)

Yukarıdaki tablo ilk uçuşa (2026-11-12 22:25) aittir ve olduğu gibi bırakıldı. Bu bölüm sonraki üç uçuşu ekler.

| Uçuş | Zincir | Dalış hattı | Veri |
|---|---|---|---|
| 11-12 22:25 | Z1, Z2 | eski (12° hat) | `test/tam_tarama_2026-11-12/ucus/` |
| 11-12 23:22 | Z3, Z4 | düzeltilmiş (8,19°) | `test/dalis_duzeltme_2026-11-12/ucus/` |
| 11-13 00:06 (tur 1) | Z5 | düzeltilmiş | `test/sim_kapanis_2026-11-13/tur1/` |
| 11-13 03:13 (tur 2) | Z6 | düzeltilmiş | `test/sim_kapanis_2026-11-13/tur2/` |

Tur 2 tam görevdir: 9/9 hat (649 sn), QR son hatta bulundu, zincir tamamlandı. Tur 1'de QR 3. hatta (−70↑) bulundu; zincirden sonra oturum kesintisi nedeniyle araçlar ~3 saat Hold'da kaldı ve tam kutu o turda uçulmadı.

## Ölçütler
| Ölçüt | Ölçüm | Ort | Medyan | En kötü | n |
|---|---|---|---|---|---|
| **Tespit doğruluğu** | QR kadrajdayken bulma (kare) | 53/54 = %98,1 | — | 1 kaçan kare | 54 kare, 7 geçiş |
| | QR kadrajdayken çözme (kare) | 53/54 = %98,1 | — | — | 54 |
| | Kadraj kenarında bulma, çözülemeyen (boş içerikle yayınlanan) | 5 kare | — | — | 4 uçuş |
| | Yanlış içerik / QR dışı yerde tespit | 0 | — | — | ~190 bin kare |
| | Hedef şeridinden (−55) geçişte tespit | 4/4 | — | — | 4 geçiş |
| | Şerit dışı komşu hat (−70, yanal ~10 m) | 3/5 geçişte kadraja girdi ve bulundu | — | — | 5 geçiş |
| **Konum hatası, PX4-tam** (yatay) | Geçişin ilk yayını (zincirin kullandığı) | 3,68 m | 2,67 m | **9,93 m** | 7 |
| | aynı, 9 m anomalisi hariç | 2,64 m | 2,54 m | 3,25 m | 6 |
| | Tüm yayınlar | 3,89 m | 3,12 m | 9,93 m | 58 |
| | aynı, 5 m üstü hariç | 2,90 m | 2,94 m | 4,67 m | 49 |
| **Konum hatası, gerçek-poz** | Aynı kareler, groundtruth pozla | 0,06 m | 0,06 m | 0,12 m | 53 |
| | PX4 yatay konum hatası (uav0, uçuş ort) | 1,38–1,43 m | — | p95 1,9–2,2; en çok 3,6 m | 4 uçuş |
| **Aktarım gecikmesi** | Kare damgası → `/hedef_konum` yayını | 76,8 ms | 73,7 ms | 114,2 ms | 58 |
| | Röle | 49,8 ms | — | 70,0 ms (en az 30,0) | 62 |
| **Aktarım bütünlüğü** | Röle iletilen / alınan (kayıp modeli %3) | 62/62, kayıp %0 | — | — | 62 |
| | Zinciri başlatan mesajın uav1'e ulaşması | 6/6 | — | — | 6 |
| **Dalış, düzeltilmiş hat** | Durma nedeni = 3B durma mesafesi | 4/4 | — | — | 4 |
| | Durma anı 3B (kestirilen hedefe; hedef 20) | 19,86 m | 19,86 m | 19,90 m | 4 |
| | Durma anı irtifa (tasarım 15) | 16,6 m | 16,0 m | 19,1 m (Z5) | 4 |
| | Gerçek hedefe en yakın geçiş, 3B | 17,2 m | 16,3 m | 20,7 m / en yakın 15,5 m | 4 |
| | O andaki yatay ıskalama | 5,7 m | 6,7 m | 7,8 m | 4 |
| | Dip irtifa (gerçek; taban 8 m) | 15,9 m | 15,1 m | en düşük 14,7 m | 4 |
| | Gerçek alçalma açısı | 6,4° | 6,5° | 5,7° | 4 |
| **Dalış, eski 12° hat** | Durma nedeni = 3B | 0/2 ("hedef geçildi") | — | — | 2 |
| | Durma anı 3B | 24,3 m | — | 25,1 m | 2 |
| | Dip irtifa / gerçek açı | 22,2 m / 7,1° | — | — | 2 |
| **Genel görev başarısı** | Zincir abortsuz, uav1 AUTO_LOITER ile bitti | 6/6 | — | — | 6 |
| | 3B 20 m durma ölçütü dahil | 4/6 (düzeltilmiş hatla 4/4) | — | — | 6 |
| | Tarama koşusu abortsuz | 7/7 (tam 9/9 kutu: 2) | — | — | 7 |
| | Tespit → uav1 Hold | 184 sn | 184 sn | 218 sn | 6 |
| | aynı, düzeltilmiş hat | 197 sn | 207 sn | 218 sn | 4 |
| | Failsafe / RTL / OFFBOARD kaybı | 0 | — | — | 4 uçuş |

## Zincir başına
| | Z1 | Z2 | Z3 | Z4 | Z5 | Z6 |
|---|---|---|---|---|---|---|
| Tespit hattı (yanal) | −70↑ (9,4 m) | −55↑ | −70↑ | −55↓ | −70↑ (9,7 m) | −55↑ (tam taramanın son hattı) |
| Kadrajda / bulunan / çözülen kare | 10/10/10 | 9/9/9 | 13 kadrajda, 12 bulunan (Z3 + Z4 birlikte) | ← | 4/4/4 | 9/9/9 |
| İlk yayın konum hatası (m) | 2,38 | 3,07 | 2,07 | **9,93** | 3,25 | 2,67 |
| Gerçek-poz hatası (m) | 0,06 | 0,06 | 0,05 | 0,06 | 0,06 | 0,06 |
| Röle gecikmesi ort (ms) | 49,3 | ← | 49,0 | ← | 47,5 | 54,3 |
| Durma nedeni | hedef geçildi | hedef geçildi | 3B | 3B | 3B | 3B |
| Durma 3B / irtifa (m) | 25,1 / 24,5 | 23,4 / 22,1 | 19,85 / 16,2 | 19,87 / 15,5 | 19,9 / 19,1 | 19,8 / 15,7 |
| Gerçek en yakın 3B (m) | 25,6 | 23,2 | 15,5 | 16,4 | 20,7 | 16,1 |
| Dip (m, gerçek) | 22,9 | 21,4 | 15,4 | 14,7 | 18,7 | 14,8 |
| Gerçek alçalma açısı | 6,9° | 7,3° | 6,5° | 6,5° | 5,7° | 6,7° |
| Tespit → uav1 Hold (sn) | 156 | 162 | 218 | 205 | 156 | 208 |

## Tarama (tam kutu, iki koşu: 11-12 u2 ve tur 2)
- Süre 646 / 649 sn (plan 513). Kutu girişinde |yanal hata|: ort 1,75, medyan 1,02, en kötü 6,65 m (n = 18 hat); tur 2'de en kötü 2,73 m.
- Hat–direk gerçek en yakın 37,1–37,9 m (plan 40). Dönüş yay hatası en çok 16–21 m.

## İşaretlenen veri noktaları (araştırılmadı)
1. **Konum hatası > 5 m:** yeni iki turda yok (en çok 4,67 m). Tek örnek hâlâ Z4 (9,0–9,9 m, 9 kare). Karşılaştırma için: Z5 ve Z6 tespit anında PX4 − groundtruth yaw farkı −10,0° / −7,9° (roll −0,2° / −3,5°, pitch +0,3° / −0,1°); Z6'da hata doğu yönünde sistematik (+3,7 m).
2. **Z5 dalışı sınırda:** durma anında irtifa 19,1 m (tasarım 15), gerçek en yakın 20,7 m, alçalma 1,34 m/s, pitch min −15,6°. Devralma giriş irtifasının altında (35,9 m; giriş 38,3) ve hızlı oldu (hizalama 47 sn; diğerlerinde 101–108). 3B yine de tetiklendi.
3. **Z6'da uav1 G direğine yatay ~10 m yaklaştı** (118 m irtifada, direk boyu 140 m; Hold modunda hizalanma çemberine geçiş bacağı; iz kaydı 11,0 m, groundtruth 10,2 m). Planın açıklık denetimi bu bacağı uygun bulmuştu.
4. **Tur 1'de yardımcı düğümler çift çalıştı** (`qr_online`, röle, kamera köprüsü ikişer süreç; yarıda kesilen ilk başlatmadan). Röle sayacı 9 (5 + 4 yinelenen); tablo yalnız tek sürecin kayıtlarını (5 yayın) sayar. Zincire etkisi görülmedi.
5. Tur 1'de ~3 saat Hold sonrası "Compass 0 fault / Land now" uyarısı (uav1 ~17. dk, uav0 ~67. dk sim zamanı); zincir bundan önce bitmişti.
