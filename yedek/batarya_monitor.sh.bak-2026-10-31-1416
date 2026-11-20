#!/bin/bash
# Post-geçiş Hold izleyici: batarya %, irtifa/mod 60 sn'de bir loglanır.
# Kendi kapatır: batarya < %30, anormallik (failsafe/RTL/iniş/disarm), ya da 20 dk.
# "mavlink.log" adı launch_all stop'un pkill desenine (mavlink_takeoff.py) uymaz → güvenli.
P=/home/yusuf22_04/iha_sim_v2/test/qr_pencere_2026-10-31
LOG=$P/batarya.log
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source ~/ros2_ws/install/setup.bash >/dev/null 2>&1
start=$(date +%s)
mav_base=$(wc -l < "$P/mavlink.log" 2>/dev/null || echo 0)
: > "$LOG"
kapat() {
  echo "$(date +%H:%M:%S)  KAPATILIYOR: $1" | tee -a "$LOG"
  cd /home/yusuf22_04/iha_sim_v2 && ./launch_all.sh stop >> "$LOG" 2>&1
  echo "$(date +%H:%M:%S)  kapandı ($1)"
  exit 0
}
while true; do
  rem=$(timeout 12 ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile \
        /fmu/out/battery_status_v1 px4_msgs/msg/BatteryStatus 2>/dev/null \
        | awk -F': ' '/^remaining:/{print $2; exit}')
  mav=$(tail -1 "$P/mavlink.log" 2>/dev/null | sed 's/\t/ /g')
  t=$(( $(date +%s) - start ))
  pct=$(awk -v r="$rem" 'BEGIN{if(r=="")print "NA"; else printf "%.0f", r*100}')
  echo "$(date +%H:%M:%S)  t=${t}s  batarya=${pct}%  | $mav" | tee -a "$LOG"
  # anormallik: izleyici başladıktan sonraki yeni STATUSTEXT satırları
  yeni=$(tail -n +$((mav_base+1)) "$P/mavlink.log" 2>/dev/null)
  if echo "$yeni" | grep -qiE 'failsafe|return to launch|RTL:|land(ing|ed)? (now|detected|mode)|critical|emergency'; then
    kapat "anormallik (STATUSTEXT)"
  fi
  if echo "$mav" | grep -qiE 'DISARMED|LANDED'; then
    kapat "disarm/iniş"
  fi
  if [ "$pct" != "NA" ] && [ "$pct" -lt 30 ]; then
    kapat "batarya %$pct < %30"
  fi
  if [ "$t" -ge 1200 ]; then
    kapat "20 dk doldu"
  fi
  sleep 60
done
