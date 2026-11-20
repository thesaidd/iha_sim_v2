# Simüle kamera (W_nadir_10) ChArUco iç kalibrasyonu

`gz_models_cam_hw/W_nadir_10/advanced_plane/model.sdf` içindeki `camera` sensörü (1456×1088,
`horizontal_fov` 0,79287, Gauss gürültü σ 0,012, Brown k1 −0,08, k2 0,02) gerçek kamera gibi
kalibre edildi: dokulu ChArUco tahtası, 48 farklı poz, renk hattı `gs_kamera.isle` (RGGB mozaik +
demozaik), bapp kalibrasyon aracı (`scripts/camera/calibrate.py` → `bap_geo.calibration.calibrate`).

## Ana sonuç

**gz Harmonic (gz-sim 8, ogre2) SDF `<distortion>` bloğunu render'a UYGULAMIYOR.** Görüntü
distorsiyonsuz pinhole. Sunucu logu bunu söylüyor:

```
[Wrn] [ImageBrownDistortionModel.cc:112] ImageBrownDistortionModel is not supported in ogre2
[Wrn] [RenderPassSystem.cc:54] RenderPass of typeid '...DistortionPass' is not registered
```

Kalibrasyon da aynısını gösteriyor: bulunan D'nin görüntüdeki etkisi en çok 0,7 px. SDF'teki
katsayılar uygulansaydı köşede ~18,5 px olurdu. **Ama `camera_info` yine D = [−0,08, 0,02, 0, 0, 0]
yayınlıyor.** Simde bu D ile undistort yapan her tüketici (geolocator, PnP) görüntü kenarında
~19 px'e varan sistematik hata alır. Simde `w_nadir_10_sim.json` kullanılmalı (ya da K nominal ve D = 0).

## Sonuçlar

Kalibrasyon: 40 eğitim karesinden ≥ 12 köşeli 39'u kullanıldı (calibrate.py `AUTO_MIN_POINTS`).
Köşeler 8×6 görüntü hücresinin 48'ini de dolduruyor; köşe hücreleri dahil.

| | fx | fy | cx | cy | k1 | k2 | p1 | p2 | k3 |
|---|---|---|---|---|---|---|---|---|---|
| kalibre (`w_nadir_10_sim.json`) | 1738,95 | 1738,93 | 727,74 | 542,73 | −0,0020 | 0,0319 | −8e−5 | 2e−5 | −0,1225 |
| nominal (SDF / camera_info) | 1739,13 | 1739,13 | 728 | 544 | −0,08 | 0,02 | 0 | 0 | 0 |

k2 ile k3 birbirini dengeliyor (ilintili); D'nin toplam etkisi ≤ 0,7 px. Yatay FOV 45,42°.

Yeniden yansıtma RMS (px):

| | kalibre | nominal (D SDF'teki) | pinhole (nominal K, D = 0) |
|---|---|---|---|
| eğitim (39 görünüm, calibrateCamera) | 0,066 | – | – |
| test, 8 ayrılmış görünüm (görünüm başına solvePnP, toplu) | **0,051** | 0,384 | 0,052 |

Test görünümleri tek tek (RMS px):

| dosya | köşe | kalibre | nominal | pinhole |
|---|---|---|---|---|
| view_005 | 17 | 0,052 | 0,551 | 0,052 |
| view_011 | 24 | 0,040 | 0,236 | 0,040 |
| view_017 | 10 | 0,040 | 0,207 | 0,041 |
| view_023 | 24 | 0,047 | 0,491 | 0,048 |
| view_029 | 24 | 0,069 | 0,220 | 0,071 |
| view_035 | 17 | 0,042 | 0,359 | 0,044 |
| view_041 | 24 | 0,047 | 0,250 | 0,050 |
| view_047 | 24 | 0,055 | 0,525 | 0,053 |

Modeller arası görüntü farkı. Yöntem: 57×43 piksel ızgarası kalibre modelle 5 m derinliğe geri
yansıtılıp diğer modelle yeniden yansıtıldı.

| karşılaştırma | en çok | görüntü köşesi | sol kenar ortası | merkez |
|---|---|---|---|---|
| kalibre ↔ nominal | **18,8 px** | 18,8 | 10,0 | 1,3 |
| kalibre ↔ pinhole (nominal K, D = 0) | 1,9 | 1,0 | 1,3 | 1,3 |
| kalibre D'nin kendi etkisi (aynı K) | 0,7 | 0,55 | 0,05 | 0 |
| başvuru: nominal D'nin beklenen etkisi | 18,5 | | | |

Test görünümlerinde nominal modelin RMS'i yalnız 0,2–0,55 px çıkıyor, çünkü PnP pozu hatanın
çoğunu soğuruyor ve tahta görüntünün yalnız bir kısmını kaplıyor. Gerçek model farkını haritadaki
18,8 px gösteriyor. Kalibre ↔ pinhole farkının ~1 px'i asal nokta kaymasından geliyor
(cx 727,7 / cy 542,7, camera_info'da 728 / 544). OpenCV piksel-merkez kuralında beklenen değer
(W−1)/2 = 727,5, (H−1)/2 = 543,5.

Taze kare denetimi: 48 görünümün hepsinde PnP'den bulunan kamera–tahta uzaklığı planlanan pozla
≤ 1,6 mm farkla örtüşüyor. Bayat (önceki poza ait) kare yok, tahta ölçeği doğru.

Not: köşe gürültüsü simde çok düşük (RMS ~0,05–0,07 px). Gerçek kamerada σ_px ≈ 0,33 bekleniyor.
gz Gauss gürültüsü (σ 0,012 ≈ 3 DN) ve demozaik, keskin ve bulanıklıksız render'da köşeyi az
bozuyor.

## Dosyalar

- `make_kalib_world.py` → `tahta.png` + `kalib.sdf`. Tahta bapp tanımında: `bap_geo.calibration`,
  7×5 kare, DICT_5X5_100, marker/kare = 26/35. Sim ölçeği: kare 0,30 m (tahta 2,1×1,5 m), çevresinde
  0,15 m beyaz kenar. Kamera sensör bloğu W_nadir_10'dan aynen kopyalanır; yalnız `<pose>` 0 ve
  `<update_rate>` 5 olur. Dünyada Physics, UserCommands, SceneBroadcaster ve Sensors (ogre2) var.
- `kalib_yakala.py`: 48 poz (uzaklık 2,6–6,5 m, eğim 0–40°, rastgele optik eksen dönüşü), sırayla
  dört tür: tahta merkezde; tahta merkezi 5×4 ızgarada; tahtanın dış köşesi görüntü kenarı ya da
  köşesine 8–40 px yakın (iki tür). `set_pose` için `gz service` CLI kullanılır. Kare `set_pose`
  yanıtından sonraki 2. karedir. Her kareye `gs_kamera.isle` uygulanır. Her 6. poz `kareler/test/`,
  kalanlar `kareler/egitim/`. Pozlar `kareler/pozlar.json`'a yazılır.
- `w_nadir_10_sim.json`: `CameraModel.save_json` biçimi (+ rms_px, n_views, hfov_deg, source).
- `kalib_degerlendir.py` → `degerlendirme.json`: test RMS, model farkı, kapsama, taze kare denetimi.

Tahta ölçeği iç parametreleri etkilemez. calibrate.py varsayılan 35 mm tahtayla çalıştı.
Değerlendirme betiği aynı kareleri 0,30 m'lik tahtayla yeniden kalibre ediyor: K farkı < 2e−4 px,
D farkı < 3e−5.

## Yeniden üretme

```bash
cd ~/sim/iha_sim_v2/kalibrasyon
~/bapp/.venv-ros/bin/python make_kalib_world.py
export GZ_CONFIG_PATH=/usr/share/gz:$GZ_CONFIG_PATH GZ_PARTITION=kalib GZ_SIM_RESOURCE_PATH=$PWD
gz sim -s -r kalib.sdf &          # başsız tek sunucu (~0,55 GB RSS); PID'i not et
~/bapp/.venv-ros/bin/python kalib_yakala.py          # ~1 dk, 48 kare
kill <gz PID>
cd ~/bapp && .venv-ros/bin/python scripts/camera/calibrate.py --headless \
    --source "images:$HOME/sim/iha_sim_v2/kalibrasyon/kareler/egitim/*.png" \
    --out ~/sim/iha_sim_v2/kalibrasyon/w_nadir_10_sim.json
cd ~/sim/iha_sim_v2/kalibrasyon && ~/bapp/.venv-ros/bin/python kalib_degerlendir.py
```

`GZ_PARTITION=kalib`, aynı anda çalışan başka bir simülasyonla karışmayı önler.

## Karşılaşılan sorunlar

- OpenCV 5.0'da `CharucoBoard.generateImage` bazı boyutlarda (200 px/kare, `marginSize` > 0) ROI
  hatası veriyor. Çözüm: 240 px/kare, kenarsız üretim, beyaz kenar `copyMakeBorder` ile eklendi.
- `gz service` CLI ara sıra servis keşfinde zaman aşımına düşüyor. `set_pose` üç kez denenir.
- Tahtanın iç köşesi doğrudan görüntü kenarına konunca dış sıradaki marker'lar kesiliyor ve o köşe
  tespit edilemiyor, köşe hücreleri boş kalıyordu. Çözüm: tahtanın dış köşesi kenara dayanır ve
  tahta tümüyle görüntüde kalacak optik eksen dönüşü seçilir.
