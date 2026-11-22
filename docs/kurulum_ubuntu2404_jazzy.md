# Kurulum: Ubuntu 24.04 (WSL2) + ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17.0

Durum (2026-11-21): kurulum + kuru testler yapıldı. **Simülasyon/uçuş doğrulaması (launch_all, kalkış, QR geçişi) henüz yapılmadı** —
eski Humble sonuçları Jazzy'de tekrarlanana kadar "kanıtlanmış" sayılmaz.

## Sürümler
| Bileşen | Sürüm |
|---|---|
| Ubuntu | 24.04.5 (WSL2, kernel 6.18) |
| ROS 2 | Jazzy (`ros-jazzy-desktop`, `ros-dev-tools`, `ros-jazzy-ros-gz` 1.0.25) |
| Gazebo | `gz-harmonic` 1.0.0, `gz sim` 8.15.0 |
| PX4 | **v1.17.0** (etiket). Eski ortam: `v1.17.0-alpha1-959-g2ca42e5252` (alpha1'den 959 commit sonrası) |
| Micro-XRCE-DDS-Agent | **v2.4.3** (PX4 v1.17 istemcisi v2.x; v3.x uyumsuz) |
| px4_msgs | `v1.17.0` etiketi (msg tanımları PX4 `msg/` + `msg/versioned/` ile içerik olarak birebir aynı) |
| OpenCV | 4.10.0.84 contrib (pip, venv), numpy 1.26.4 |

## Adımlar
1. Sistem + araçlar: `sudo apt update && sudo apt upgrade`; `git build-essential cmake python3-pip python3-venv gh`.
2. ROS apt kaynağı (`ros2-apt-source` .deb) ve OSRF Gazebo kaynağı; `sudo apt install ros-jazzy-desktop ros-dev-tools gz-harmonic ros-jazzy-ros-gz`.
3. Agent: `git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent && cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j12 && sudo make install && sudo ldconfig`.
4. PX4: `git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot && cd ~/PX4-Autopilot && git checkout v1.17.0 && git submodule update --init --recursive && bash Tools/setup/ubuntu.sh`.
5. Yamalar (bkz. `px4_yamalar/README.md`): `git apply px4_yamalar/gz_bridge_magnetometre.patch` (PX4 kökünde); `cd Tools/simulation/gz && git apply ../../../../iha_sim_v2/px4_yamalar/gz_advanced_plane_el_degisiklik.patch` (isteğe bağlı).
6. Derleme: `cd ~/PX4-Autopilot && GZ_CONFIG_PATH=/usr/share/gz make px4_sitl` (1141 adım, hatasız).
7. px4_msgs: `mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && git clone https://github.com/PX4/px4_msgs.git && cd px4_msgs && git checkout v1.17.0`; `cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select px4_msgs` (~2,5 dk).
8. Python: `python3 -m venv --system-site-packages ~/venv_iha && source ~/venv_iha/bin/activate && pip install "numpy<2" "opencv-contrib-python==4.10.0.84" pymavlink`.
   Betikler düz `python3` çağırdığı için **`launch_all.sh` ve kuru testlerden önce venv'i etkinleştir**, sonra `source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash`.

## Sürprizler
- **Agent v2.4.2 derlenmiyor:** içindeki Fast-DDS `2.12.x` dalı uzak depoda artık yok → v2.4.3 (PX4 dokümanındaki sürüm) kullan.
- **`git checkout v1.17.0` artık submodule dizinleri bırakıyor** (`rl_tools`, `ulog_cpp`, `mc_raptor`, `Micro-XRCE-DDS-Client-v3` vb.; v1.17.0 `Micro-XRCE-DDS-Client` v2 kullanıyor). `git clean -fd` ile bu dizinler silindi; yeniden klonlayıp doğrudan etiketle başlamak daha temiz.
- **numpy çatışması:** `~/.local` altında numpy 2.5.5 var (PX4 `ubuntu.sh`/pip), apt'in cv2 4.6'sı ve ROS `cv_bridge` numpy 1.x ister. Apt OpenCV 4.6 ayrıca `QRCodeDetectorAruco` içermez. Çözüm: venv + numpy<2 + opencv-contrib 4.10. Sistem pandas (3.1.1) numpy≥2 ister; depodaki betikler pandas kullanmıyor (`grep` ile bakıldı), venv dışında etkilenmez.
- **gz sürüm kontrolü:** ROS source edilince PATH'te `/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz` öne geçiyor; `gz sim --versions` ile hem `GZ_CONFIG_PATH=/usr/share/gz` ile hem onsuz 8.15.0 verdi (fark yok).
- `sudo` şifre istediği için apt adımları kullanıcının terminalinde çalıştırıldı.

## Kuru test sonuçları (Jazzy, venv; hepsi geçti)
alan_tarama 47 · ayrim 18 · qr_pass 23 · lora_tarama_zincir 26 · yer_gorevi 83 · kamera_geometri 20 · lora_dalis 66 assert · gs_kamera 3 test. Toplam 283 assert + 3 test.
Not: görev metnindeki "eski ortamda 220" ile birebir örtüşmüyor; alan_tarama CLAUDE.md'de 43, burada 47 (depo sonradan değişmiş olabilir).
Testler `test/lora_dalis_2026-11-21/` ve `test/yer_gorevi_2026-11-21/` altına çıktı yazar (izlenmeyen dosyalar).

## Simülasyon doğrulaması (2026-11-21, `CAM=T10 LORA=1 WORLD=iha_arena_v2`, GUI=1 GPU=1)
- 2 PX4 ayakta, her araçta 27 `/fmu/out` topic'i, kamera 9,3 Hz, odometri 100 Hz, RTF 1,0, loglarda hata yok (gz.log'da yalnız aşağıdaki eklenti hatası).
- `vehicle_status` normal abonelikle okunuyor: topic **`/fmu/out/vehicle_status_v1`** (MESSAGE_VERSION 1). Betiklerdeki ham nav_state geçici çözümü artık gerekmeyebilir (kod değiştirilmedi).
- Kalkış (`mavlink_takeoff.py --uav 0 1 --ucus-dinamigi`) başarılı: 100 / 120 m Hold, hava hızı geçerli (~14–15 m/s). Kalkışta iki araç arası en küçük 3B 20,7 m (bilinen açık sorun).
- QR geçişi: ilk deneme `ayrim.py` tarafından reddedildi (uav1 Hold'u komşu, 3B 9–10 m < 40 m). uav1 N350/E450'ye taşınınca geçiş abortsuz bitti: hedef üstü yanal 0,01 m, dikey +1,7 m, 14,0 m/s, AUTO_LOITER.
- **QR TESPİTİ BAŞARISIZ (0 / 7 kadrajdaki kare):** `iha_arena_v2.sdf` ve `iha_arena.sdf` içinde `qr_hedef01.png` / `zemin_doku.png` için **mutlak yol** `/home/yusuf22_04/iha_sim_v2/gz_worlds/...` var; bu makinede yok → doku yüklenmiyor. Çözüm adayı: dünyayı `make_iha_arena.py --zemin dokulu` ile yeniden üretmek (kullanıcı kararı bekliyor, YAPILMADI). Tespit/konum karşılaştırması bu yüzden henüz yapılamadı.
- `gz.log`: `libAirSpeedPlugin.so` bulunamıyor — depodaki modeller (`gz_models_nocam`, `gz_models_cam_hw/*`) eski PX4'ün `px4::AirSpeed` eklentisini istiyor, v1.17.0'da yok. Uçuşta hava hızı geçerli göründü; eski sıcaklık hatasının (0 K) durumu ayrıca ölçülmedi.

## Dünya düzeltmesi ve gerçekçi dünya (2026-11-21)
- Doku yolu hatası giderildi: `make_iha_arena.py` varsayılanı göreli yol yazıyor; `iha_arena.sdf` ve `iha_arena_v2.sdf` yeniden üretildi (yalnız doku yolu satırları değişti).
- **Yeni: `WORLD=iha_arena_gercek`** (`python3 gz_worlds/make_iha_arena.py --gercekci`): `make_zemin_gercek.py` ile 6144² tuval (11,4 cm/px, 4 karo JPEG, 8,4 MB) — tarla parselleri, ekin sıraları, toprak yollar, ağaç şeritleri + gölge; QR çevresi (R 25 m) ve doğma noktası (R 40 m) sade çayır. Halka, ızgara, merkez diski yok; 8 km çevre zemini, gök mavisi arka plan, sönük bina renkleri, beyaz/kırmızı direkler. Engel konum/boyutları değişmedi. `launch_all.sh` `iha_arena_gercek`'i kabul ediyor. v1/v2 dünyaları taban çizgisi olarak duruyor.
- Doğrulama (iha_arena_gercek, GUI=1 GPU=1, RTF ~1,0): QR geçişi hedef üstü yanal −0,13 m, dikey +0,28 m; QR kadrajdaki **9/9 kare bulundu + çözüldü + yayınlandı**, röle 9/9, konum hatası PX4 ile ~0,25–0,30 m (n = 1 geçiş, 9 kare; eski ortam n = 7: medyan 2,7 m — örnek küçük, kesin karşılaştırma değil).

## Henüz yapılmadı
Dokulu dünya (v2) ile aynı koşunun taban çizgisi karşılaştırması, `~/bapp` / ArUco hattı (erişim yok), uzun uçuş kararlılığı, PX4 v1.17.0'da `libAirSpeedPlugin` kalıntısının temizlenmesi.
