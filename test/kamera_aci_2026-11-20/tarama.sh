#!/bin/bash
# Yalnız keşif taraması (uav0) + kamera kaydı + kapsama/mozaik değerlendirmesi (2026-11-20). uav1 uzak üste YERDE kalır
# (zincir.sh ile aynı spawn; dalış / röle tüketicisi yok). Tespit taramayı kesmez (--tam-tarama): alanın tamamı uçulur.
# Kullanım: TARAMA_ARGS="..." bash tarama.sh <çıktı_klasörü>          (~8–10 dk, penceresiz)
#           GOR=1 ... bash tarama.sh <çıktı>   görsel: Gazebo penceresi (GPU, iz) + uav0 kamera penceresi; sim açık kalır
#           KARE_ARALIK=0.5 → kare kaydı aralığı [s] (varsayılan 1,0; mozaik + tekrar videosu için)
set -u
SIM=~/sim/iha_sim_v2
D=${1:?çıktı klasörü}; mkdir -p "$D"; D=$(cd "$D" && pwd); cd "$D"
GOR=${GOR:-0}
TARAMA_ARGS=${TARAMA_ARGS:?TARAMA_ARGS gerekli (alan_tarama.py plan argümanları)}
ROS="source /opt/ros/jazzy/setup.bash && source $HOME/ros2_ws/install/setup.bash"
PY=$HOME/bapp/.venv-ros/bin/python
export UAV1_POSE="-950,200,0.5,0,0,0"          # zincir.sh ile aynı: uav1 uzak üste (yerde)
export IHA_UAV1_SPAWN_NE="200,-950"
rm -rf kareler; rm -f tarama_sonuc.jsonl tarama.done tarama.abort olaylar.log
GORSEL_ENV=""
[[ $GOR == 1 ]] && GORSEL_ENV="GUI=1 VIZ=1 GPU=1"

(cd $SIM && ./launch_all.sh stop) > kapatma0.log 2>&1
(cd $SIM && env CAM=W10 ARUCO=1 $GORSEL_ENV ./launch_all.sh) > baslatma.log 2>&1
setsid bash -c "source /opt/ros/jazzy/setup.bash; cd $SIM && exec python3 -u gz_gercek_poz.py --out /tmp/aruco_log/gercek_poz.csv" \
    > gercek.log 2>&1 < /dev/null &
setsid bash -c "$ROS && cd $SIM && exec $PY -u kare_kaydedici.py --dizin $D/kareler --aralik ${KARE_ARALIK:-1.0}" \
    > kare_kaydedici.log 2>&1 < /dev/null &
if [[ $GOR == 1 ]]; then
    export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=${GPU_ADI:-NVIDIA}
    setsid bash -c "$ROS && cd $SIM && exec $PY -u canli_kamera.py --uav 0 --gercek-ne -60 -60" > canli_kamera.log 2>&1 < /dev/null &
    setsid bash -c "export GZ_CONFIG_PATH=/usr/share/gz; cd $SIM
        for i in \$(seq 1 60); do ./cam.sh uav0 > /dev/null 2>&1 && break; sleep 2; done" > kamera_takip.log 2>&1 < /dev/null &
fi
setsid bash -c "$ROS && cd $SIM && exec python3 -u mavlink_takeoff.py --uav 0 --watch 3000" > kalkis0.log 2>&1 < /dev/null &
for i in $(seq 1 60); do grep -q "Turlama başladı" kalkis0.log 2>/dev/null && break; sleep 5; done
echo "$(date +%T) uav0 Hold" >> olaylar.log
bash -c "$ROS && cd $SIM && exec python3 -u alan_tarama.py $TARAMA_ARGS --tam-tarama --koridor-yok \
    --iz $D/tarama_iz.csv --done-file $D/tarama.done --abort-file $D/tarama.abort --result $D/tarama_sonuc.jsonl" \
    > tarama.log 2>&1 < /dev/null
echo "$(date +%T) tarama bitti (rc $?)" >> olaylar.log
sleep 8
kill $(pgrep -f "[k]are_kaydedici.py") $(pgrep -f "[g]z_gercek_poz.py") 2>/dev/null
sleep 1
rm -rf aruco_log; cp -r /tmp/aruco_log . ; cp /tmp/{aruco_geolocator,gs_kamera,px4_0}.log . 2>/dev/null
cd $SIM
$PY kapsama_analiz.py --log $D/aruco_log --tarama-log $D/tarama.log --daire 100 --etiket "$(basename $D)" \
    --png $D/kapsama.png --json $D/kapsama.json > $D/kapsama.txt 2>&1
$PY mozaik.py --kareler $D/kareler --log $D/aruco_log --tarama-log $D/tarama.log --cikis $D/mozaik > $D/mozaik.txt 2>&1
$PY aruco_degerlendir.py --log $D/aruco_log --md $D/aruco_degerlendirme.md > $D/aruco_degerlendir.txt 2>&1
if [[ $GOR == 1 ]]; then
    echo "$(date +%T) bitti (görsel mod: simülasyon açık bırakıldı)" >> $D/olaylar.log
else
    (cd $SIM && ./launch_all.sh stop) > $D/kapatma.log 2>&1
    echo "$(date +%T) bitti" >> $D/olaylar.log
fi
