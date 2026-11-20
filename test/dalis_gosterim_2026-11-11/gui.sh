#!/bin/bash
# Çalışan sunucuya pencere + işaretçi bağlar (sunucunun ortamıyla), sabit kamerayı kurar.
SP=$(pgrep -f "gz sim.*-s" | head -1)
[ -z "$SP" ] && { echo "sunucu yok"; exit 1; }
while IFS= read -r -d '' kv; do case "$kv" in GZ_*|PX4_GZ_*|IGN_*) export "$kv";; esac; done < /proc/$SP/environ
setsid bash -c "exec gz sim -g --gui-config $HOME/iha_sim_v2/gui_kayit.config > /tmp/gz_gui.log 2>&1" < /dev/null &
sleep 12
G=$(pgrep -f "gz sim -g" | head -1); echo "pencere PID $G"; [ -n "$G" ] && echo $G >> /tmp/iha_sim.pgids
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
setsid bash -c "cd $HOME/iha_sim_v2 && exec python3 -u viz_trails.py --world default > /tmp/viz_trails.log 2>&1" < /dev/null &
sleep 3
V=$(pgrep -f "viz_trails" | head -1); echo "işaretçi PID $V"; [ -n "$V" ] && echo $V >> /tmp/iha_sim.pgids
gz service -s /gui/track --reqtype gz.msgs.CameraTrack --reptype gz.msgs.Boolean --timeout 3000 --req 'track_mode: NONE' 2>&1 | head -2
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: ""' 2>&1 | head -2
POSE=$(python3 - <<'PY'
import math
(cx, cy, cz), (tx, ty, tz) = (560.0, 0.0, 300.0), (-60.0, 0.0, 50.0)
yaw = math.atan2(ty - cy, tx - cx); pitch = math.atan2(cz - tz, math.hypot(tx - cx, ty - cy))
qw, qx = math.cos(pitch / 2) * math.cos(yaw / 2), -math.sin(pitch / 2) * math.sin(yaw / 2)
qy, qz = math.sin(pitch / 2) * math.cos(yaw / 2), math.cos(pitch / 2) * math.sin(yaw / 2)
print(f"pose: {{position: {{x: {cx}, y: {cy}, z: {cz}}}, orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}")
PY
)
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req "$POSE" 2>&1 | head -2
echo "kamera: $POSE"
