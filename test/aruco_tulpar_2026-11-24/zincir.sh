#!/bin/bash
# Tulpar + BAP ArUco zinciri (2026-11-24): uav0 = Tulpar (W10: RPi GS + 6 mm, aşağı bakan) el fırlatma → 100 m Hold →
# R = 100 m halka içinde kameradan şeritli tarama (B1 argümanları: --serit-kamera … --kenar-px 15 --uzatma 60; hat seçimi
# YOK) → bapp aruco_geolocator (2 m DICT_4X4_50 id 0) → 24 bayt TargetReport → lora_relay --mod paket (868 MHz g1: duty %1)
# → uav1 (advanced_plane, W_nadir_10 + kamikaze kamerası) YERDEN kalkış (üs NED 200, −950) → dalış (3B 20 m) → dönüş → iniş.
# test/aruco_2026-11-20/zincir.sh ile aynı akış; farklar: UAV0=tulpar, el fırlatma, B1 tarama, değerlendirmeler.
# Kullanım: bash zincir.sh <çıktı_klasörü>                  (≈ 15–20 dk, penceresiz)
#           GERCEKCI=1 bash zincir.sh <çıktı>               gerçekçilik bayrakları açık (launch_all.sh başı, GERCEKCI_*)
#           TARAMA_EK="--tam-tarama" ...                    ek alan_tarama argümanları
set -u
SIM=~/sim/iha_sim_v2
D=${1:?çıktı klasörü}; mkdir -p "$D"; D=$(cd "$D" && pwd); cd "$D"
ROS="source /opt/ros/jazzy/setup.bash && source $HOME/ros2_ws/install/setup.bash"
PY=$HOME/bapp/.venv-ros/bin/python
export UAV0=tulpar UAV0_MODEL=tulpar
export UAV1_POSE="-950,200,0.5,0,0,0"          # Gazebo ENU, yaw 0 → NED 90° (advanced_plane kısıtı)
export IHA_UAV1_SPAWN_NE="200,-950"            # ayrim.py: uav1 doğma noktası (uav0 NED)
US_ARGS="--kaynak-fark-n 200 --kaynak-fark-e -950 --pist-yonu 90"   # paket modunda hedef zincirinde KULLANILMAZ
KAMERA_JSON=kalibrasyon/w_nadir_10_sim.json
[[ ${GERCEKCI:-0} == 1 ]] && KAMERA_JSON=kalibrasyon/w_nadir_10_gercekci.json   # tarama şeridi distorsiyonlu modelden
rm -f sonuc.jsonl tarama_sonuc.jsonl tarama.done tarama.abort abort olaylar.log
rm -f /tmp/lora_yaklasma_sonuc.jsonl /tmp/lora_yaklasma.log /tmp/lora_yaklasma.log.1

(cd $SIM && ./launch_all.sh stop) > kapatma0.log 2>&1
(cd $SIM && env CAM=W10 ARUCO=1 YAKLASMA=1 \
    YAKLASMA_ARGS="--profil dalis --alcalma-siniri 4.0 --baslangic yer $US_ARGS --guvenli-irtifa 120 \
                   --bekleme-asimi 1500 --abort-file $D/abort" ./launch_all.sh) > baslatma.log 2>&1
if ! grep -q "PX4-0: başlangıç betiği tamamlandı" baslatma.log || ! grep -q "PX4-1: başlangıç betiği tamamlandı" baslatma.log; then
    echo "BASLATMA BASARISIZ" > DURUM; (cd $SIM && ./launch_all.sh stop) > kapatma.log 2>&1; exit 1
fi
echo "$(date +%T) başlatıldı" >> olaylar.log
setsid bash -c "source /opt/ros/jazzy/setup.bash; cd $SIM && exec python3 -u gz_gercek_poz.py --out /tmp/aruco_log/gercek_poz.csv" \
    > gercek.log 2>&1 < /dev/null &
setsid bash -c "$ROS && cd $SIM && exec python3 -u mavlink_takeoff.py --uav 0 --el-firlatma --alt 100 --watch 3000" \
    > kalkis0.log 2>&1 < /dev/null &
for i in $(seq 1 60); do grep -q "Turlama başladı" kalkis0.log 2>/dev/null && break; sleep 5; done
echo "$(date +%T) uav0 Hold" >> olaylar.log
sleep 20
setsid bash -c "$ROS && cd $SIM && exec python3 -u alan_tarama.py --alt 100 --serit-kamera $KAMERA_JSON --kenar-px 15 \
    --daire 100 --uzatma 60 --icerik ARUCO-0 --koridor-yok ${TARAMA_EK:-} \
    --iz $D/tarama_iz.csv --done-file $D/tarama.done --abort-file $D/tarama.abort --result $D/tarama_sonuc.jsonl" \
    > tarama.log 2>&1 < /dev/null &
for i in $(seq 1 300); do [ -s /tmp/lora_yaklasma_sonuc.jsonl ] && break; [ -f tarama.abort ] && break; sleep 5; done
echo "$(date +%T) uav1 sonuç" >> olaylar.log
sleep 5
rm -rf aruco_log aruco_log_uav1; cp -r /tmp/aruco_log /tmp/aruco_log_uav1 . ; cp /tmp/lora_yaklasma_sonuc.jsonl sonuc.jsonl 2>/dev/null
cp /tmp/{aruco_geolocator,aruco_geolocator_uav1,gs_kamera,gs_kamera_uav1,lora_relay,lora_yaklasma,px4_0,px4_1}.log . 2>/dev/null
kill $(pgrep -f "[g]z_gercek_poz.py") 2>/dev/null
(cd $SIM && ./launch_all.sh stop) > kapatma.log 2>&1
R=$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs
cp "$(ls -t $R/0/log/*/*.ulg | head -1)" uav0.ulg 2>/dev/null
cp "$(ls -t $R/1/log/*/*.ulg | head -1)" uav1.ulg 2>/dev/null
cd $SIM
$PY zincir_degerlendir.py "$D" > "$D/zincir_degerlendirme.txt" 2>&1
$PY aruco_degerlendir.py --log $D/aruco_log --md $D/aruco_degerlendirme.md > $D/aruco_degerlendir.txt 2>&1
$PY kapsama_analiz.py --log $D/aruco_log --tarama-log $D/tarama.log --daire 100 --etiket "$(basename $D)" \
    --png $D/kapsama.png --json $D/kapsama.json > $D/kapsama.txt 2>&1
[ -s $D/sonuc.jsonl ] && echo TAMAM > $D/DURUM || echo "SONUC YOK" > $D/DURUM
echo "$(date +%T) bitti" >> $D/olaylar.log
