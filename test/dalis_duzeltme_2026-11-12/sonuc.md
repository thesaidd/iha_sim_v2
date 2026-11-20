# Dalış hassasiyeti: teşhis, düzeltme, doğrulama — 2026-11-12 (1/2 uçuş denemesi)

## Teşhis (uçuşsuz; 5 dalışın düğüm logu + ulg karşılaştırması)
- `durma_karari()` sırası: 3B ≤ 20 m → irtifa tabanı → `s ≥ 0` ("hedef geçildi"). Yedek erken gelmiyor; uçak hedef hizasını 3B > 20 m ile geçince geliyor.
- 12° hat hiçbir dalışta izlenmedi: komut 3,2–3,4 m/s alçalıyor, uçak beş dalışta ort 2,06–2,35 m/s alçaldı (PX4 sink sınırı 4,0 / 5,0 bağlamıyor). Uçak profilin 6–10 m üstünde kalıyor.

| | D1 11-10 | D2 11-11 | D3 11-11 | D4 11-12 | D5 11-12 |
|---|---|---|---|---|---|
| Sonuç (3B durma) | 19,9 ✔ | 19,9 ✔ | 20,0 ✔ | 25,1 ✘ | 23,4 ✘ |
| Devralmada dikey hız (m/s) | −0,56 | −0,34 | −0,64 | +0,87 | +0,17 |
| Devralmadan önceki 10 sn irtifa değişimi (m) | −9,4 | −9,4 | −20,6 | −1,5 | +1,5 |
| Devralma → alçalma başlangıcı (sn) | 2,0 | 3,4 | 2,9 | 4,4 | 4,2 |
| Alçalma hızı ort (m/s) | 2,09 | 2,35 | 2,32 | 2,06 | 2,18 |
| CAS alçalma ort (m/s) | 15,7 | 15,8 | 15,4 | 13,9 | 14,6 |
| Dip irtifa (m) | 17,0 | 17,3 | 19,0 | 22,7 | 21,1 |

- "İyi" üç dalış da 3B'yi hedefin tam üstünde 17,3–19,9 m irtifayla kıl payı tetikledi. Kötü ikisi, çemberdeki alçalmanın taşma / toparlanma evresinde (seviyede ya da tırmanırken, yavaş) devraldı: alçalma ~1,5 sn geç ve ~3 m yüksekten başladı.

## Düzeltme (`lora_yaklasma.py`)
1. `--suzulme-siniri` (varsayılan 2,4 m/s): açı sınırı `min(alcalma-siniri, suzulme-siniri) × pay`. İstenen açı 12° kalır; etkin hat 8,19° (2,16 m/s), giriş irtifası 49,4 → 38,3 m. Durma noktası (3B 20 m @ 15 m) aynı.
2. `devralma_kosulu(dikey_hiz=)`: tırmanırken (> +0,5 m/s) ya da girişin 2 m üstünde olup alçalmıyorken devralınmaz ("dikey").
- `durma_karari`, taban (8 m), abort ve ayrım kuralları değişmedi.
- Kuru test: `lora_dalis_test.py` 53 → 66 assert ✔ (ölçülen alçalma yeteneğiyle eski hat ıskalıyor, yeni hat 3B'de duruyor; beş gerçek devralma durumu); diğer takımlar ✔ (18 / 23 / 43 / 26).

## Doğrulama uçuşu (23:22–23:38, 14,9 dk, `CAM=T10 LORA=1 WORLD=iha_arena_v2`, `--ucus-dinamigi`)
| | Zincir 1 (tam plan, 3. hatta tespit) | Zincir 2 (−55 hattı) | Önceki uçuş |
|---|---|---|---|
| Durma nedeni | durma-mesafesi (3b) | durma-mesafesi (3b) | hedef geçildi ×2 |
| Durma anı 3B (kestirilen hedefe) | 19,85 m | 19,87 m | 25,1 / 23,4 |
| Durma anı irtifa (tasarım 15) | 16,2 m | 15,5 m | 24,5 / 22,1 |
| Hat üstünde yanal (en çok) | 2,2 m | 1,5 m | 6,2 / 7,8 |
| Profil izleme (irtifa − profil, son 150 m) | −1,8…+1,3 m | −2,0…+0,5 m | +6…+10 m |
| Dip irtifa (gerçek) | 15,4 m | 14,7 m | 22,9 / 21,4 |
| Gerçek hedefe en yakın geçiş, 3B (gerçek) | 15,5 m (yatay 1,7) | 16,4 m (yatay 7,3) | 25,6 / 23,2 |
| Alçalma hızı ort / gerçek açı | 1,75 m/s / 6,5° | 1,78 m/s / 6,5° | 2,06–2,18 / 6,9–7,3° |
| Pitch min | −5,8° | −6,4° | −16,7 / −13,9° |
| Tırmanış + Hold | 115 m, AUTO_LOITER | 115 m, AUTO_LOITER | aynı |
| Tespit → uav1 Hold | 218 sn | 205 sn | 156 / 162 |

- Durma ölçütü kestirilen hedefe göre işler; uçak durma kararından sonra tırmanışa geçerken hedefin üstünden geçtiği için en yakın geçiş 20 m'nin altındadır (tasarım: dip tahmini 11,8 m, taban 8 m).
- Zincir süresi ~55 sn uzadı: çemberde 119 → 38 m alçalma (hizalama 106–108 sn; önce 46–55). "Dikey" kapısı bu uçuşta bekletmedi.
- Röle 15/15, gecikme ort 49 ms. Failsafe yok, zaman boşluğu yok. RTF 0,90–1,00.
- uav1–direk en yakın 38,7 m (geçişte, irtifa 125 m; plan payı 40). uav0 hat–direk 37,9 m.

## Yeni bulgu (sıradaki iş için)
Zincir 2'de hedef konum kestirimi **9,0–9,9 m** hatalı (zincir 1: 2,0–2,2 m); hatanın tamamına yakını hat yönünde (kuzey +9,0…+9,4 m). Aynı karelerde gerçek pozla hata 0,06 m, PX4 yatay konum hatası 1,1–1,3 m, EKF reset yok. O geçişte PX4 pitch kestirimi −7,4° (zincir 1 geçişinde +1,1°); 60 m × tan 8,5° ≈ 9,0 m. Neden doğrulanmadı.
