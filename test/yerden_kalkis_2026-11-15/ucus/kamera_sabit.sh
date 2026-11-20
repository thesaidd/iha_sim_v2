#!/bin/bash
# Sabit dış kamera (ENU): $1 x, $2 y, $3 z, hedef $4 x, $5 y, $6 z
python3 - "$@" <<'PY' > /tmp/kam_req.txt
import math, sys
cx, cy, cz, tx, ty, tz = (float(v) for v in sys.argv[1:7])
yaw = math.atan2(ty - cy, tx - cx)
pitch = math.atan2(cz - tz, math.hypot(tx - cx, ty - cy))
qw, qx = math.cos(pitch / 2) * math.cos(yaw / 2), -math.sin(pitch / 2) * math.sin(yaw / 2)
qy, qz = math.sin(pitch / 2) * math.cos(yaw / 2), math.cos(pitch / 2) * math.sin(yaw / 2)
print(f"pose: {{position: {{x: {cx}, y: {cy}, z: {cz}}}, orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}")
PY
gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: NONE' > /dev/null 2>&1
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 --req 'data: ""' > /dev/null 2>&1
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req "$(cat /tmp/kam_req.txt)"
