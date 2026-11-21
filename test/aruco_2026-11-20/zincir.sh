#!/bin/bash
# Uçtan uca ArUco zinciri (2026-11-20): uav0 100 m'de R = 100 m halka içinde çim biçme taraması (6 mm kamera, hat seçimi YOK — hedef konumu
# planlamada kullanılmaz) → bapp aruco_geolocator (2 m DICT_4X4_50 id 0) → 24 bayt TargetReport → lora_relay
# --mod paket (LoRa ToA/duty/kayıp) → uav1 YERDEN kalkış (üs NED 200, −950), raporun enlem/boylamını KENDİ EKF
# origin'ine çevirir (--girdi paket) → dalış (ileri kamerayla marker OKUMA), 3B 20 m'de duruş → tırmanış → üsse dönüş → iniş.
# Kullanım: bash zincir.sh <çıktı_klasörü>          (≈ 20 dk, penceresiz)
#           GOR=1 bash zincir.sh <çıktı_klasörü>    görsel: Gazebo penceresi (GPU, uçak izleri, kamera önce uav0'ı
#                                                    sonra uav1'i izler) + uav0 kamera penceresi (canli_kamera.py);
#                                                    sonunda simülasyon KAPATILMAZ (./launch_all.sh stop ile kapat)
set -u
SIM=~/sim/iha_sim_v2
D=${1:?çıktı klasörü}; mkdir -p "$D"; cd "$D"
GOR=${GOR:-0}
ROS="source /opt/ros/jazzy/setup.bash && source $HOME/ros2_ws/install/setup.bash"
PY=$HOME/bapp/.venv-ros/bin/python
export UAV1_POSE="-950,200,0.5,0,0,0"          # Gazebo ENU, yaw 0 → NED 90° (advanced_plane kısıtı)
export IHA_UAV1_SPAWN_NE="200,-950"            # ayrim.py: uav1 doğma noktası (uav0 NED) — yalnız ayrım denetimi
US_ARGS="--kaynak-fark-n 200 --kaynak-fark-e -950 --pist-yonu 90"   # paket modunda hedef zincirinde KULLANILMAZ
rm -f sonuc.jsonl tarama_sonuc.jsonl tarama.done tarama.abort abort olaylar.log
# Önceki koşunun /tmp çıktıları: sonuç dosyası "uav1 bitti", eski log "kalkış komutu" sanılıyordu (2026-11-20).
rm -f /tmp/lora_yaklasma_sonuc.jsonl /tmp/lora_yaklasma.log /tmp/lora_yaklasma.log.1
GORSEL_ENV=""
[[ $GOR == 1 ]] && GORSEL_ENV="GUI=1 VIZ=1 GPU=1"

(cd $SIM && ./launch_all.sh stop) > kapatma0.log 2>&1
(cd $SIM && env CAM=W10 ARUCO=1 YAKLASMA=1 $GORSEL_ENV \
    YAKLASMA_ARGS="--profil dalis --alcalma-siniri 4.0 --baslangic yer $US_ARGS --guvenli-irtifa 120 \
                   --bekleme-asimi 1500 --abort-file $D/abort" ./launch_all.sh) > baslatma.log 2>&1
setsid bash -c "source /opt/ros/jazzy/setup.bash; cd $SIM && exec python3 -u gz_gercek_poz.py --out /tmp/aruco_log/gercek_poz.csv" \
    > gercek.log 2>&1 < /dev/null &
if [[ $GOR == 1 ]]; then
    export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=${GPU_ADI:-NVIDIA}
    setsid bash -c "$ROS && cd $SIM && exec $PY -u canli_kamera.py --uav 0 --gercek-ne -60 -60" > canli_kamera.log 2>&1 < /dev/null &
    setsid bash -c "$ROS && cd $SIM && exec $PY -u canli_kamera.py --uav 1" > canli_kamera_uav1.log 2>&1 < /dev/null &
    # pencere hazır olunca uav0'ı izle; uav1 kalkış komutu alınca uav1'e geç (yalnız arayüz kamerası)
    setsid bash -c "export GZ_CONFIG_PATH=/usr/share/gz; cd $SIM
        for i in \$(seq 1 60); do ./cam.sh uav0 > /dev/null 2>&1 && break; sleep 2; done
        until grep -q 'KALKIŞ KOMUTU' /tmp/lora_yaklasma.log 2>/dev/null; do sleep 2; done
        ./cam.sh uav1" > kamera_takip.log 2>&1 < /dev/null &
fi
setsid bash -c "$ROS && cd $SIM && exec python3 -u mavlink_takeoff.py --uav 0 --watch 3000" > kalkis0.log 2>&1 < /dev/null &
for i in $(seq 1 60); do grep -q "Turlama başladı" kalkis0.log 2>/dev/null && break; sleep 5; done
echo "$(date +%T) uav0 Hold" >> olaylar.log
setsid bash -c "$ROS && cd $SIM && exec python3 -u alan_tarama.py --alt 100 --serit 70 --hat-araligi 70 --daire 100 --icerik ARUCO-0 --koridor-yok \
    --iz $D/tarama_iz.csv --done-file $D/tarama.done --abort-file $D/tarama.abort --result $D/tarama_sonuc.jsonl" \
    > tarama.log 2>&1 < /dev/null &
for i in $(seq 1 300); do [ -s /tmp/lora_yaklasma_sonuc.jsonl ] && break; sleep 5; done
echo "$(date +%T) uav1 sonuç" >> olaylar.log
sleep 5
cp -r /tmp/aruco_log /tmp/aruco_log_uav1 . ; cp /tmp/lora_yaklasma_sonuc.jsonl sonuc.jsonl 2>/dev/null
cp /tmp/{aruco_geolocator,aruco_geolocator_uav1,gs_kamera_uav1,lora_relay,lora_yaklasma,gs_kamera,px4_0,px4_1}.log . 2>/dev/null
kill $(pgrep -f "[g]z_gercek_poz.py") 2>/dev/null
(cd $SIM && $PY zincir_degerlendir.py "$D" > "$D/zincir_degerlendirme.txt" 2>&1)
if [[ $GOR == 1 ]]; then
    echo "$(date +%T) bitti (görsel mod: simülasyon açık bırakıldı)" >> olaylar.log
else
    (cd $SIM && ./launch_all.sh stop) > kapatma.log 2>&1
    echo "$(date +%T) bitti" >> olaylar.log
fi
