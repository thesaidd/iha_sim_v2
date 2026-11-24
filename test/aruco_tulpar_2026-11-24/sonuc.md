# Tulpar + BAP kamerası (W10) + ArUco zinciri ve gerçekçilik bayrakları — 2026-11-24

Ürünler: uav0 = **Tulpar** (el fırlatma, PX4 4030) + RPi Global Shutter (IMX296, 1456×1088) + 6 mm CS lens, aşağı bakan
(`gz_models_tulpar/W10`, T10 ile aynı kütle ve konum: x = 0,30 m); 2×2 m DICT_4X4_50 id 0 (çerçevesiz kenar 2 m —
kullanıcı teyidi, dünya ve bapp `marker_size` zaten 2 m); 100 m arama irtifası; LoRa = Waveshare SX1262 868 MHz HAT
(simde `lora_relay.py --mod paket`, **duty %1**). uav1 = advanced_plane (W_nadir_10 + kamikaze kamerası), uzak üste yerde.

## Yeni / değişen

| Parça | Dosya |
|---|---|
| Tulpar W10 varyantı (üretici `TULPAR_VARYANT=W10`; nocam/T10/kam çıktıları bayt bayt aynı kaldı) | `gz_models_tulpar/uret_model.py`, `gz_models_tulpar/W10/` |
| `UAV0=tulpar CAM=W10 ARUCO=1`: kamera kolu `--t-body-cam 0.30 0 0`, Tulpar β₀ 4,4° (F1 ölçümü) | `launch_all.sh` |
| El fırlatma CG tablosu W10 (= T10) | `el_firlatma.py`, `el_firlatma_test.py` (29 ✔) |
| Uçtan uca zincir: el fırlatma → 100 m → B1 taraması (`--serit-kamera … --kenar-px 15 --daire 100 --uzatma 60`) → bapp → LoRa → uav1 | `zincir.sh` |
| LoRa paket modu varsayılan duty %10 → **%1** (868,0–868,6 MHz ETSI g1) | `lora_relay.py` |
| Gerçekçilik (varsayılan KAPALI): `GERCEKCI=1` + `GERCEKCI_KAMERA/GNSS/MAG/RUZGAR` | `launch_all.sh`, `gs_kamera.py`, `gercekci_model.py`, `kalibrasyon/w_nadir_10_gercekci.json` |
| Statik köşe gürültüsü ölçümü | `gercekci_olcum.py`, `gercekci_olcum.json` |
| PX4 − gerçek poz (uçuş boyu), rüzgâr | `poz_hatasi.py` |
| H1/H2 çevrimdışı karşılaştırma (bapp `Geolocator`, kareler.csv) | `h1_h2.py` |
| H2 geçiş uçuşu (+ `h2/`, donan ilk deneme `h2a_gpu_dondu/`) | `h2.sh` |
| bapp kurulum notu (ekip arkadaşı: `bap_geo` yoksa analiz araçları çalışmıyordu) | `README.md` |

PX4'e Tulpar airframe'i bu makinede de kuruldu (`px4_yamalar/README.md`: `4030_gz_tulpar` + CMakeLists yaması,
`make px4_sitl_default`); kurulmadan `UAV0=tulpar` PX4-0'ı başlatamıyor ("no autostart file found").

**Varsayılan yollar değişmedi:** gerçekçilik bayrakları verilmezse `launch_all.sh`'in ürettiği değerler (advanced_plane
`--t-body-cam 0.07 0 0.1`, `--heading-beta0-deg 2.7`, sim kalibrasyonu, GS_ARGS boş, dünya/model kopyası yok) eskisiyle
aynı (değişken açılımı denetlendi); kuru testler baz çizgisiyle aynı: alan_tarama 47, qr_pass 23, ayrim 18, yer_gorevi 83,
lora_tarama_zincir 26, lora_dalis ✔, kamera_geometri 20, gs_kamera 6/6 (3 yeni geometri testi), el_firlatma 29.
Tulpar W10 başlatması: gz topic 69, `/uav0/camera/camera_info_gs` fx **1739,14** (W_nadir_10: 1739,13), marker kenarı
uçuşta 33–35 px (teori 100 m'de 34,8).

## Gerçekçilik bayrakları — ne ekliyor, nasıl ölçüldü

Tüm değerler **VARSAYIM**; GS kamerada / gerçek uçuşta ölçülüp güncellenecek (aşağıda "Pi'ye geçince").

| Bayrak | Model | Değer (varsayım) | Ölçülen etki (gz gerçeğine karşı) |
|---|---|---|---|
| `GERCEKCI_KAMERA` | `gs_kamera --gercekci`: Brown–Conrady fıçı distorsiyonu (tek remap), boresight kayması, Gauss bulanıklık, shot/read | k1 −0,15, k2 0,06 (köşede ≈ −%3,5); boresight rx 0,5°, ry 0,5°, rz 0,3° (bapp'e verilmez); σ_blur 1,0 px; shot 8 / read 3 DN | statik 100 m: köşe titreşimi 0,003 → **0,22 px** (laptop 0,30–0,35), içe bias 0,41 → 0,55 px/köşe, tespit 96/96 → 86/96; uçuşta ray-cast **gerçek pozla** 0,05 → 1,21 m (= boresight 0,71° × 100 m) |
| `GERCEKCI_GNSS` | gz NavSat `<noise>`: sabit bias (işaret rastgele), Gauss–Markov gezinme, beyaz | yatay 1,0 ± 0,5 m, gezinme 0,7 m / 300 s, beyaz 0,3 m; dikey 1,5× | F2: PX4 mutlak yatay hata medyan **2,05 m** (F1 0,55), irtifa +1,61 m (F1 −0,17) |
| `GERCEKCI_MAG` | gz magnetometre sert demir kalıntısı (eksen başına, işaret rastgele) | x/y 8 mG, z 5 mG | **etkisi küçük**: F2 uçuş boyu yaw hatası medyan 0,60°, %95 3,0° — nominal F1b 0,48 / 3,07°, F1 0,46 / 1,23° (EKF mag biasını uçuşta öğreniyor; yerde uav1 yönü 93° ↔ gerçek 90°) |
| `GERCEKCI_RUZGAR` | dünya `<wind>` (WindEffects eklentisi YOK) | ENU (−4, 0): doğudan 4 m/s | yer hızı p05–p95 13,4–18,1 m/s (F1 14,7–15,9) |

Bulunan simülatör özellikleri (ölçümle):
1. **NavSat yatay gürültüsü DERECE cinsinden** (gz-sensors 8): `bias_mean 50` enlem ve boylamı **+50°** kaydırdı, dikey
   75 m (1,5 × 50) → dikey metre. Aynı sabit bias enlem ve boylama birlikte eklenir → doğu kayması kuzeyin cos(47,4°) =
   0,68 katı (bias kuzeydoğu/güneybatı köşegeninde). `gercekci_model.py` metreyi 1/111 320 ile dereceye çevirir.
2. **Magnetometre gürültüsü çıktı biriminde (gauss)**; eksen başına işaret rastgele (statik deneme: 0,05 → −0,044 / −0,066 /
   −0,042). gz_bridge magnetometre yaması (2026-11-20) sensör çerçevesindeki biası doğru taşıyor (biası gerçek pozla
   döndürüyor, büyüklük korunur).
3. **gz LiftDrag rüzgârı yerdeki uçağa da uygular**: ilk deneme (F2a, rüzgâr ENU (3, 3) — güneybatıdan yan rüzgâr) uav1'i
   yerde −14,6°, Tulpar'ı −7,2° döndürdü (gerçek poz); uav1'in "araç yönü pist yönünden > 10° farklı" denetimi kalkışı
   reddetti (zincir yok; kayıt `f2a_yanruzgar/`). Varsayılan rüzgâr pist eksenine paralel yapıldı (doğudan, karşıdan).
4. **gz AirSpeed eklentisi rüzgârı görmüyor**: F2'de PX4 TAS ≈ yer hızı (doğuya 14,07 / 14,01; batıya 17,89 / 17,53 m/s)
   → PX4 rüzgâr kestirimi ~0,3 m/s (gerçek 4), bapp yaw izleyicisi (`heading.csv`) 92 satır `bad`, heading_check
   Γ 16°. Aerodinamik rüzgârı görüyor ama hava hızı sensörü görmüyor → **rüzgâr yalnız kısmen gerçekçi**; yaw
   izleyicisinin rüzgârlı sonuçları simde anlamsız.
5. Kamera kare hızı yazılım render'ında (llvmpipe) iki kamerayla (uav0 + kamikaze) **~2,4 Hz** (F1); `GPU=1` (D3D12, RTX
   4050) ile **~7,8 Hz** (F1b, F2). 2026-11-20 koşuları ~9 Hz idi. F1 bu yüzden az kareyle (12 tespitli) koştu; F1b GPU'yla
   yinelendi.
6. `gz_gercek_poz.py` damgası ile PX4 saati arasında GPU'lu koşularda −60…−80 ms kayma (yaw RMS'sinden kestirildi; F1'de
   +20 ms). Düzeltilmezse Hold dönüşünde (~11°/s) ~0,8° sahte sabit yaw hatası çıkıyor — `poz_hatasi.py` kaymayı kestirip
   uyguluyor (konum için ayrı kestirim: 0 ± 10 ms). aruco_degerlendir'in tespitli kare değerleri (kısa düz hat) bundan
   az etkilenir.

## Uçuşlar

| | F1 nominal (CPU render) | F1b nominal (GPU) | F2 gerçekçi (GPU) |
|---|---|---|---|
| Tulpar el fırlatma → 100 m Hold, tarama | 3/3 hat, 206 s, tespit 3. hatta | 3/3 hat, 175 s, tespit 3. hatta | 3/3 hat, 172 s, tespit 3. hatta |
| Tespit (beklenen karede) / sahte | 12/12 (%100) / 0 | 33/34 (%97,1) / 0 | 21/23 (%91,3) / 0 |
| Kare başına ray-cast (PX4 pozu / gerçek poz) | 0,83 / 0,05 m | 0,84 / 0,05 m | 1,66 / 1,21 m |
| Kare başına PnP (PX4 / gerçek) | 0,92 / 0,69 m | 1,04 / 0,67 m | 1,42 / 1,98 m |
| PX4 poz hatası (tespitli kareler): yatay, D, yaw | 0,34 m, 0,06 m, 0,07° | 0,14 m, 0,02 m, 0,75° | 1,77 m, 1,57 m, 0,17° |
| PX4 uçuş boyu: yaw medyan/%95, mutlak yatay medyan, irtifa ort. | 0,46 / 1,23°, 0,52 m, −0,16 m | 0,48 / 3,07°, 0,51 m, +0,01 m | 0,60 / 3,00°, 2,05 m, +1,61 m |
| Son LoRa raporu: hata / σ95 (n_obs) | 0,87 / 1,89 m (12) | 0,86 / 1,72 m (33) | 1,65 / 4,74 m (21) |
| Raporların σ95 içinde kalan payı | %100 | %100 | %100 |
| LoRa: alınan / gönderilen / kayıp / ezilen; gecikme | 440 / 64 / 0 / 375; 66,8–79,7 ms | 376 / 54 / 1 / 320; 66,8–83,4 ms | 359 / 50 / 3 / 305; 69,9–81,5 ms |
| Tespit → uav1 kalkış komutu | 37,5 s | 33,8 s | 32,3 s |
| uav1 dalış durması (kendi kestirimi) | 3B 19,99 m | 3B 19,89 m | 3B 19,7 m |
| **uav1 GERÇEK en yakın yatay geçiş** (o anki irtifa) | **0,63 m** (14,3 m) | **0,90 m** (14,1 m) | **1,75 m** (≈12 m) |
| Kamikaze okuması (tespitli kare, ilk okuma gerçek 3B, kestirim hatası) | 23, 76,7 m, 0,71 m | 21, 68,0 m, 0,38 m | 17, 48,6 m, 1,96 m |
| uav1 iniş | indi | indi | indi (üsse 22 m) |

Notlar:
- LoRa duty %1: 24 + 4 bayt SF7/125 kHz ToA 66,8 ms → paketten sonra kanal 6,6 s kapalı; uçuş boyunca ~6,7 s'de bir rapor
  (F1: 64 rapor / ~12 dk). Kalkış tetiği ilk rapordur → etkisi yok (tespit → kalkış 32–38 s, çoğu uav1 hazırlığı). Dalış
  profili (`lora_yaklasma --profil dalis`) rapor yaşına bakmıyor; `--profil duz`'ün "2 × 3 s rapor gelmezse dur" kuralı
  duty %1'de **tetiklenir** (6,7 s > 6 s) — o profil kullanılacaksa `--zaman-asimi ≥ 4` verilmeli.
- F2'de ray-cast hatası (1,66 m) boresight (1,2 m, gerçek pozla) + GNSS + irtifa biasının bileşkesi; PnP (1,42 m) burada
  daha iyi ama gerçek pozla 1,98 m — iki hata kısmen birbirini götürüyor (n = 1, genelleme değil).
- σ95 dürüst: F2'de bapp gerçek uçak varsayılanları (GNSS 1,5 m, eğim 0,5°, yaw 2°, HAG 1 m) kullanıldı (gerçekçi modda
  `SIM_BIAS_ARGS` yalnız β₀); raporların hepsi σ95 içinde, hata/σ95 0,35.

## H1 (yöntem) — çevrimdışı, aynı kareler (`h1_h2.py`, bias tabanı yok)

| koşu | n kare | ray-cast son | PnP son | füzyon son |
|---|---|---|---|---|
| F1 | 12 | 0,87 | 0,92 | 0,89 |
| F1b | 33 | 0,86 | 1,01 | 0,94 |
| F2 (gerçekçi) | 21 | 1,65 | 1,35 | 1,49 |
| H2 (gerçekçi, 8 geçiş + Hold) | 247 | 2,70 | 2,75 | 2,73 |

H1 bu verilerde **desteklenmiyor**: PnP ile füzyon ray-cast'ten belirgin iyi değil (fark ≤ 0,3 m, iki yönde de); hata
PX4 pozundaki ortak biaslardan (GNSS, boresight, yaw) geliyor ve iki yöntem de bunlara muhtaç. bapp kararı (füzyon yalnız
ray-cast, PnP tanı) değişmez.

## H2 (geçiş deseni) — `h2.sh`, GERCEKCI=1, CPU render (tek kamera, ~7 Hz)

Tulpar 100 m'de hedef üstünden `qr_pass.py` geçişleri: ψ 210° ×3, 30°, 90°, 270° (6/6 tamam, abort yok). Her geçiş
döngüsünde marker iki kez görülüyor (H hizalama bacağı + geçiş) → düz uçuşta (|roll| ≤ 10°) **8 geçiş grubu**, 229 kare;
yer yönleri (PX4 yaw, rüzgâr yengeci dahil) 198°, 53°, 199°, 197°, 33°, 229°, 87°, 262°. Her küme bapp ray-cast füzyonuyla
(bias tabanı yok) tek tahmine indirildi; hata gerçek hedefe yatay.

| geçiş grubu (yön) | 198° | 53° | 199° | 197° | 33° | 229° | 87° | 262° |
|---|---|---|---|---|---|---|---|---|
| tek grup ray-cast hatası [m] | 5,72 | 1,34 | 4,58 | 4,13 | 0,55 | 2,92 | 1,30 | 1,90 |

| küme | aynı yön (yayılım < 30°) | farklı yön (yayılım ≥ 90°) |
|---|---|---|
| 2 geçiş | medyan **4,64 m** (n = 4 küme) | medyan **2,59 m** (n = 15) |
| 3 geçiş | 4,80 m (n = 1) | medyan **2,61 m** (n = 45) |
| 4 geçiş (en dar / en geniş yayılım) | 4,38 m (16°) | 1,68 m (119°) |

- Tüm uçuş (Hold dönüşleri dahil 244 gözlem) son rapor **2,70 m**, σ95 4,59 m; raporların %95'i σ95 içinde.
  aruco_degerlendir'in saydığı "1 sahte tespit" gerçek marker: t 201,0 s, kenar 30 px, ray-cast gerçek pozla 1,38 m — gerçek
  izdüşüm boresight'ı bilmediği için marker'ı kadraj dışı sanıyor (kadraj kenarı). Yanlış id yok.
- PX4 bu uçuşta: yaw medyan 1,63°, %95 4,1° (ort. −1,5°, yöne bağlı −2,3…+0,3°), mutlak yatay 1,87 m (ort. N +1,51,
  E +1,08), irtifa −0,54 m. Aynı yönlü geçişler aynı yaw/boresight hatasını tekrarlıyor; farklı yönler kısmen söndürüyor.
- **Sonuç: H2 simde destekleniyor** (farklı yönlü kümeler aynı yönlülerin ~%55'i). Sınırlar: tek uçuş, kümeler aynı
  GNSS/mag/boresight örneğini paylaşıyor (bağımsız değil; tek bir hata gerçekleşmesi), "aynı yön" yalnız 198° civarı;
  GNSS biası (~1,9 m) yönle sönmediği için farklı yön kümesinin tabanı ~1,7–2,6 m.
- İlk deneme (`h2a_gpu_dondu/`, `GPU=1`): gz sunucusu 4. geçişte dondu (stats yok, kare/poz sayaçları durdu, dmesg'de
  dxg ioctl hataları) → elle durduruldu; ilk 3 geçişin verisi tutarlı (aynı yön 2 küme 2,09 m, farklı yön 1,57 m).

## Simin hâlâ iyimser / gerçekçi olmayan yanları

- Kamera kalibrasyonu kusursuz varsayıldı (bapp'e uygulanan distorsiyonun tam kendisi verildi); gerçek ChArUco kalıntısı yok.
- Köşe titreşimi 0,22 px < laptop 0,33 px; GS + gerçek ışık / MTF / gölge / basılı marker kontrastı yok; zemin düz.
- Rüzgâr hava hızı sensörüne yansımıyor (madde 4), türbülans / esinti yok; yan rüzgâr yerdeki uçakları döndürüyor (madde 3).
- GNSS biası iki araçta bağımsız (gz her sensörde işareti ayrı çekiyor): gerçekte aynı uydular / iyonosfer → ortak kısım
  söner; sim bu açıdan **kötümser**.
- IMU / baro / hava hızı biası, GNSS kesintisi, çok yol yok. Gerçekçi zincir tek koşu (F2, n = 1), nominal n = 2 (F1,
  F1b); H2 tek uçuş — dağılım için tekrar gerekir.
- Kare damgası = PX4 saati (`UXRCE_DDS_SYNCT=0`); gerçek uçakta saat eşlemesi ayrı hata kaynağı.

## Pi / GS kameraya geçince simden gelen girdiler (ölçülüp yerine yazılacak)

| Sim varsayımı | Nerede | Gerçekte ölçülecek |
|---|---|---|
| distorsiyon k1 −0,15, k2 0,06 | `gs_kamera.GERCEKCI_DISTORSIYON`, `kalibrasyon/w_nadir_10_gercekci.json` | GS + 6 mm ChArUco kalibrasyonu (bapp `scripts/camera/calibrate.py`) → aynı JSON biçimi |
| boresight 0,5 / 0,5 / 0,3° | `GERCEKCI_BORESIGHT_DEG` | yerde bilinen noktalar + PX4 attitude (bapp hazırlık listesi #12) |
| bulanıklık σ 1 px, shot 8 / read 3 DN → köşe σ 0,22 px | `GERCEKCI_BULANIKLIK/SHOT/READ` | `live_aruco.py --jitter` (GS, 100–300 µs pozlama); hedef: simde aynı σ'yı veren ayar |
| içe köşe biası 0,55 px | gs_kamera zinciri | basılı marker bias ölçümü → bapp `--inward-bias-px` |
| GNSS 1 ± 0,5 m bias, 0,7 m / 300 s gezinme | `gercekci_model.py` | RTK'lı / jeodezik ölçülmüş noktada GNSS kaydı (u-blox modeli belli olunca) |
| mag 8 mG kalıntı | `gercekci_model.py --mag-bias-*` | QGC mag kalibrasyonu sonrası `heading_check.py --ulog` |
| Tulpar β₀ 4,4° (sim gövdesi) | `TULPAR_BETA0` | gerçek Tulpar'ın düz uçuş yan kayması (`heading_check.py`) |
| LoRa SF7/125 kHz, duty %1, kayıp %3 | `lora_relay.py` | HAT'in hava hızı ayarına karşılık gelen SF/BW, gerçek ToA ve PER (sahada) |
| kamera 10 Hz | W10 modeli | Pi 5 tespit FPS'i (`bench_detect.py`) |
