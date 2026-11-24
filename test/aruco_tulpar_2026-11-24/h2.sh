#!/bin/bash
# H2 deneyi (2026-11-24): Tulpar (W10) 100 m'de ArUco hedefi (−60, −60) üstünden düz geçişler, qr_pass.py ile (marker
# konumu planda BİLİNİYOR — bu tespit değil, yinelenen gözlem deneyi). uav1 uzak üste yerde (dalış yok).
# Geçiş sırası GECISLER (varsayılan "210 210 210 30 90 270": 3 tek yön + 3 farklı yön); bapp düğümü tüm kareleri kaydeder,
# h1_h2.py kareleri geçişlere bölüp "tek yön" ve "çok yön" kümelerini ayrı ayrı birleştirir.
# Kullanım: [GERCEKCI=1] [GPU=1] bash h2.sh <çıktı_klasörü>
set -u
SIM=~/sim/iha_sim_v2
D=${1:?çıktı klasörü}; mkdir -p "$D"; D=$(cd "$D" && pwd); cd "$D"
ROS="source /opt/ros/jazzy/setup.bash && source $HOME/ros2_ws/install/setup.bash"
PY=$HOME/bapp/.venv-ros/bin/python
export UAV0=tulpar UAV0_MODEL=tulpar
export UAV1_POSE="-950,200,0.5,0,0,0" IHA_UAV1_SPAWN_NE="200,-950"
GECISLER=${GECISLER:-210 210 210 30 90 270}
rm -f olaylar.log gecis.jsonl
(cd $SIM && ./launch_all.sh stop) > kapatma0.log 2>&1
(cd $SIM && env CAM=W10 ARUCO=1 ./launch_all.sh) > baslatma.log 2>&1
grep -q "PX4-0: başlangıç betiği tamamlandı" baslatma.log || { echo "BASLATMA BASARISIZ" > DURUM; (cd $SIM && ./launch_all.sh stop) > kapatma.log 2>&1; exit 1; }
setsid bash -c "source /opt/ros/jazzy/setup.bash; cd $SIM && exec python3 -u gz_gercek_poz.py --out /tmp/aruco_log/gercek_poz.csv" \
    > gercek.log 2>&1 < /dev/null &
setsid bash -c "$ROS && cd $SIM && exec python3 -u mavlink_takeoff.py --uav 0 --el-firlatma --alt 100 --watch 3000" \
    > kalkis0.log 2>&1 < /dev/null &
for i in $(seq 1 60); do grep -q "Turlama başladı" kalkis0.log 2>/dev/null && break; sleep 5; done
echo "$(date +%T) uav0 Hold" >> olaylar.log
sleep 15
n=0
for psi in $GECISLER; do
    n=$((n + 1))
    rm -f gecis.done gecis.abort
    bash -c "$ROS && cd $SIM && exec python3 -u qr_pass.py --pass-id $n --heading $psi --alt 100 \
        --done-file $D/gecis.done --abort-file $D/gecis.abort --result $D/gecis.jsonl" > gecis_$n.log 2>&1 < /dev/null
    echo "$(date +%T) geçiş $n ψ $psi → çıkış $?" >> olaylar.log
    sleep 5
done
sleep 5
rm -rf aruco_log; cp -r /tmp/aruco_log . ; cp /tmp/{aruco_geolocator,gs_kamera,px4_0}.log . 2>/dev/null
kill $(pgrep -f "[g]z_gercek_poz.py") 2>/dev/null
cp aruco_log/gercek_poz.csv . 2>/dev/null
(cd $SIM && ./launch_all.sh stop) > kapatma.log 2>&1
R=$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs
cp "$(ls -t $R/0/log/*/*.ulg | head -1)" uav0.ulg 2>/dev/null
cd $SIM
$PY aruco_degerlendir.py --log $D/aruco_log --md $D/aruco_degerlendirme.md > $D/aruco_degerlendir.txt 2>&1
echo TAMAM > $D/DURUM
echo "$(date +%T) bitti" >> $D/olaylar.log
