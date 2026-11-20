#!/bin/bash
# Kamera sızıntısı denemesi: temiz başlat (kalkış yok), sunucuyu izle, kapat.
# Kullanım: cam_leak_trial.sh <etiket> <PX4_GZ_MODELS klasörü> <ek gz sim argümanı|-> <süre_sn> <aralık_sn>
label=$1; models=$2; gzarg=$3; dur=$4; step=$5
HERE=$(dirname "$(readlink -f "$0")")
PX4_DIR=~/PX4-Autopilot; PX4_BUILD=$PX4_DIR/build/px4_sitl_default
[[ $gzarg == - ]] && gzarg=""

~/iha_sim_v2/launch_all.sh stop > /dev/null
echo "=== $label | modeller: $models | gz ek argüman: '${gzarg:-yok}' | $(date +%T) ==="
echo "free -m (başlangıç): $(free -m | awk '/^Mem:/{print "kullanılan", $3, "MB, kullanılabilir", $7, "MB"}')"

cd "$PX4_BUILD/rootfs" && source ./gz_env.sh
export PX4_GZ_MODELS=$models
setsid gz sim --verbose=1 -r -s $gzarg "$PX4_GZ_WORLDS/default.sdf" < /dev/null > /tmp/cam_leak_gz_$label.log 2>&1 &
for _ in {1..30}; do gz topic -l 2>/dev/null | grep -q "^/world/default/clock" && break; sleep 1; done
setsid MicroXRCEAgent udp4 -p 8888 < /dev/null > /dev/null 2>&1 &
poses=("0,0,0.5,0,0,0" "-15,15,0.5,0,0,0")
for i in 0 1; do
    setsid env PX4_SIM_MODEL=gz_advanced_plane PX4_SYS_AUTOSTART=4008 PX4_GZ_WORLD=default PX4_GZ_NO_FOLLOW=1 \
        PX4_GZ_MODEL_POSE=${poses[$i]} "$PX4_BUILD/bin/px4" -d -i $i < /dev/null > /tmp/cam_leak_px4_${i}.log 2>&1 &
    for _ in {1..60}; do grep -q "Startup script returned successfully" /tmp/cam_leak_px4_${i}.log && break; sleep 1; done
done
cd "$HERE" && python3 -u cam_leak_monitor.py "$dur" "$step" 4096
~/iha_sim_v2/launch_all.sh stop > /dev/null
sleep 3
echo "free -m (bitiş): $(free -m | awk '/^Mem:/{print "kullanılan", $3, "MB, kullanılabilir", $7, "MB"}'); kalan süreç: $(ps -eo comm | grep -cE '^(px4|ruby|MicroXRCEAgent)$')"
