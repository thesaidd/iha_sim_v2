#!/usr/bin/env bash
# gercek_ucak için tek-araç PX4 SITL ortamı (launch_all.sh'tan bağımsız).
#   ./gercek_ortam.sh start   -> Agent + gz sunucusu (pencere yok) + PX4 (instance 0)
#   ./gercek_ortam.sh stop    -> yalnız bu betiğin başlattığı süreç gruplarını PID ile kapatır
set +u
D=$(cd "$(dirname "$0")" && pwd)
PX4_DIR=$HOME/PX4-Autopilot; PX4_BUILD=$PX4_DIR/build/px4_sitl_default
PGID_FILE=/tmp/gercek_ortam.pgids
WORLD_FILE=${WORLD_FILE:-$PX4_DIR/Tools/simulation/gz/worlds/default.sdf}
POSE=${POSE:-0,0,0.35,0,0,0}
start_bg() { setsid bash -c "$1" < /dev/null > /dev/null 2>&1 & echo $! >> "$PGID_FILE"; }
case "${1:-}" in
start)
  : > "$PGID_FILE"
  source /opt/ros/jazzy/setup.bash 2>/dev/null || true
  export PX4_GZ_MODELS=$D
  export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-}:$D:$PX4_DIR/Tools/simulation/gz/worlds
  export GZ_SIM_SERVER_CONFIG_PATH=$HOME/iha_sim_v2/server_nogst.config
  start_bg "MicroXRCEAgent udp4 -p 8888 > /tmp/gercek_dds.log 2>&1"
  start_bg "gz sim --verbose=1 -r -s '$WORLD_FILE' > /tmp/gercek_gz.log 2>&1"
  for _ in {1..30}; do gz topic -l 2>/dev/null | grep -q "^/world/default/clock" && break; sleep 1; done
  start_bg "cd $PX4_DIR && env PX4_PARAM_EKF2_GPS_DELAY=0 PX4_PARAM_EKF2_DECL_TYPE=3 PX4_SIM_MODEL=gz_${MODEL_NAME:-gercek_ucak} PX4_SYS_AUTOSTART=4030 PX4_GZ_WORLD=default PX4_GZ_MODEL_POSE='$POSE' PX4_GZ_NO_FOLLOW=1 $PX4_BUILD/bin/px4 -d -i 0 > /tmp/gercek_px4.log 2>&1"
  for _ in {1..60}; do grep -q "Startup script returned successfully" /tmp/gercek_px4.log 2>/dev/null && { echo "PX4 hazır"; exit 0; }; sleep 1; done
  echo "UYARI: PX4 60 sn içinde başlamadı (bkz. /tmp/gercek_px4.log)"; exit 1 ;;
stop)
  [[ -f $PGID_FILE ]] || { echo "pgid dosyası yok"; exit 0; }
  for p in $(tac "$PGID_FILE"); do kill -TERM -- -"$p" 2>/dev/null; done
  sleep 4
  for p in $(cat "$PGID_FILE"); do kill -KILL -- -"$p" 2>/dev/null; done
  : > "$PGID_FILE"; echo "kapatıldı" ;;
*) echo "kullanım: $0 start|stop"; exit 2 ;;
esac
