# QR tespit → LoRa röle → uav1 devralma zinciri — sabit kamera belgeleme kaydı (2026-11-01)

**Önemli:** Bu bir **arama görevi değildir.** uav0'a QR hedefinin konumu (−60, −60) ve geçiş hattı
(ψ 210°, 60 m, 15 m/s) **önceden verilmiştir** (`qr_pass.py --pass-id 4 --alt 60 --bias 0`). Kayıt, kontrollü
bir geçiş testinde zincirin (tespit → röle → devralma → durma → tırmanış → Hold) uçtan uca çalışmasını belgeler.

## Dosyalar
| Dosya | İçerik |
|---|---|
| `qr_belgeleme.mp4` | 130,1 sn, 898×512, 10 fps (mp4v), 7,1 MB. Gerçek zamanlı oynatma (duvar saati) |
| `qr_belgeleme.csv` | Her video karesi: `video_s, duvar saati, aşama etiketi` |
| `kilit_anlar.jpg` | 8 kare: 5 / 45 / 80 / 100 / 111 / 114 / 119 / 128 sn |

## Kayıt yöntemi
- Gazebo penceresi (`gz sim -g`) çalışan sunucuya bağlandı, yapılandırma `~/iha_sim_v2/gui_kayit.config`
  (kullanıcı `~/.gz/sim/8/gui.config` kopyası: pencere 900×560, yan paneller çıkarıldı, VideoRecorder eklendi).
- **Gazebo VideoRecorder kullanılamadı:** eklenti yüklendi ama bu sürümde (gz-sim 8) yalnızca arayüz düğmesiyle
  tetikleniyor, `/gui/record_video` servisi sunulmuyor; ffmpeg / xdotool yok (apt yasak).
- **Kullanılan:** `/gui/screenshot` servisine periyodik istek (2 paralel `gz service` çağrısı, gerçekleşen 2,84 Hz),
  kareler OpenCV ile 10 fps gerçek zamanlı mp4'e (kare tekrarı ile) yazıldı, PNG'ler silindi. Üst şeritte video
  zamanı, duvar saati ve zincir aşaması (logdan otomatik).
- **Sabit kamera** (tek seferlik `/gui/move_to/pose`, takip yok): konum dünya ENU (212, −148, 190 m), bakış
  (−30, −8, 70 m) — hedef geçiş hattına dik, doğu-güneydoğudan. uav0 sağdan sola geçer; uav1'in Hold/yaklaşma
  bölgesi ve QR hedefi aynı kadrajda. İşaretçiler (`viz_trails.py`): uav0 kırmızı, uav1 mavi küre (8 m) + 60 sn iz
  + zemine dikey çizgi.

## Zaman çizelgesi (video t0 ≈ 10:58:20,0 duvar saati; video zamanları ±1 sn)
| Olay | Duvar saati | Video [sn] | Kaynak satırı |
|---|---|---|---|
| Kayıt başı (uav0 H'de 100 m, kadraj dışında ~577 m; uav1 Hold) | 10:58:20 | 0 | — |
| Hizalanma başı (DO_REPOSITION → H, 60 m) | 10:58:27,2 | ~7 | qr_pass |
| uav0 hatta, OFFBOARD (s = −681 m, 36 sn hizalama) | 10:59:03,6 | ~44 | qr_pass |
| uav0 kadraja girer (sağ kenar) | ~10:59:38 | ~78 | video |
| **QR tespiti** (ilk yayın, (−59,2, −62,3), HEDEF-01) | 11:00:10,9 | ~111 | qr_online |
| Röle → uav1 hedef aldı, plan (yatay 82 m) | 11:00:10,95 | ~111 | lora_yaklasma |
| **uav1 devraldı (OFFBOARD)** — tespitten 1,03 sn | 11:00:12,0 | ~112 | lora_yaklasma |
| **uav1 durdu** — yatay 19,77 m, 3B 107,9 m, irtifa 106,1 m | 11:00:18,7 | ~119 | lora_yaklasma |
| **Tırmanış tamam** — 115,0 m (3,5 sn) | 11:00:22,3 | ~122 | lora_yaklasma |
| uav1 Hold (AUTO_LOITER) ✔ — toplam manevra 11,5 sn | 11:00:22,4 | ~122 | lora_yaklasma |
| uav0 geçiş sonu, Hold (AUTO_LOITER) ✔ | 11:00:25 | ~125 | qr_pass |
| Kayıt sonu (uav1 Hold + 8 sn) | 11:00:30,4 | 130,1 | — |

Aşama etiketleri kare tüketildiği anda basıldığı için gerçek olaydan en fazla ~1 sn önde/geride olabilir;
kesin zamanlar yukarıdaki duvar saatleridir. 898 px genişlikte uzun etiketlerin sağ ucu kesiliyor.

## Sonuçlar ve önceki koşularla karşılaştırma
| | Deneme 2 (09:58) | Gösterim (10:39) | **Belgeleme (11:00)** |
|---|---|---|---|
| Tespit → devralma | 1,07 sn | 1,09 sn | **1,03 sn** |
| Yatay durma | 19,9 m | 19,86 m | **19,77 m** |
| Tırmanış sonrası irtifa | 115,0 m | 115,0 m | **115,0 m** |
| Başlangıç yatay mesafe | 410 m | 110 m | 82 m |
| Toplam manevra | 54,9 sn | 19,2 sn | 11,5 sn |

Toplam manevra süresi başlangıç mesafesine bağlı (uav1 bu koşuda hedefe 82 m yakın Hold'daydı); devralma gecikmesi,
durma mesafesi ve tırmanış irtifası üç koşuda tutarlı. Abort yok, latch-and-wait yok, High Accelerometer Bias 0×.
Röle 16/16 (oturum toplamı), kayıp 0, ort. gecikme 47 ms. uav0 hedef üstü yanal −0,73 m; hattaki yanal sapma bu
koşuda daha büyük (std 3,9 m, maks 16 m — devralma x = +9,1 m ile başladı; önceki geçişlerde std ≤ 0,5 m).

**Etkin RTF (15 sn, kayıt açık):** 0,828 → 0,841 → 0,820 → 0,801 → 0,751 → 0,699 → 0,696 → 0,688
(uav0 arena merkezine yaklaşırken düşüş; 0,5 abort eşiğinden uzak).
