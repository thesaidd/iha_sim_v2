#!/bin/bash
# Gazebo penceresinin kamerası (yalnızca arayüz; simülasyonu ve araçları etkilemez).
#
# Kullanım:
#   ~/iha_sim_v2/cam.sh uav0    → kamerayı advanced_plane_0'ın arkasına, biraz yukarısına kilitle
#   ~/iha_sim_v2/cam.sh uav1    → advanced_plane_1 için aynısı
#   ~/iha_sim_v2/cam.sh genel   → takibi bırak, başlangıç kamerasına dön (güneybatı, 300 m)
#
# Servisler (arayüzün CameraTracking / MoveTo eklentileri):
#   /gui/follow/offset  gz.msgs.Vector3d  → takip ofseti, uçağın gövde çerçevesinde (x ileri, z yukarı)
#   /gui/follow         gz.msgs.StringMsg → takip edilecek model adı; boş = takibi bırak
#   /gui/move_to/pose   gz.msgs.GUICamera → kamera pozu (dünya çerçevesi, ENU)
# Genel bakış pozu gz_worlds/make_iha_arena.py'deki CAMERA_POSE/CAMERA_TARGET'ten hesaplanır.

BEHIND=25   # m, uçağın arkasında
ABOVE=8     # m, uçağın yukarısında
HERE=$(dirname "$(readlink -f "$0")")

call() {   # call <servis> <istek tipi> <istek>
    gz service -s "$1" --reqtype "$2" --reptype gz.msgs.Boolean --timeout 3000 --req "$3" 2>&1 \
        | grep -q "data: true" || { echo "HATA: $1 başarısız (pencere açık mı?)"; exit 1; }
}

case "$1" in
    uav0|uav1)
        m_var="UAV${1#uav}_MODEL"; model="${!m_var:-advanced_plane}_${1#uav}"   # arac_modelleri.py ile aynı kural
        call /gui/follow/offset gz.msgs.Vector3d "x: -$BEHIND, y: 0, z: $ABOVE"
        call /gui/follow gz.msgs.StringMsg "data: \"$model\""
        echo "kamera: $model takibi (arkada $BEHIND m, yukarıda $ABOVE m)"
        ;;
    genel)
        call /gui/follow gz.msgs.StringMsg 'data: ""'
        pose=$(python3 - "$HERE/gz_worlds" <<'EOF'
import math, sys
sys.path.insert(0, sys.argv[1])
from make_iha_arena import CAMERA_POSE, CAMERA_TARGET
(cx, cy, cz), (tx, ty, tz) = CAMERA_POSE, CAMERA_TARGET
yaw = math.atan2(ty - cy, tx - cx)
pitch = math.atan2(cz - tz, math.hypot(tx - cx, ty - cy))
qw, qx = math.cos(pitch / 2) * math.cos(yaw / 2), -math.sin(pitch / 2) * math.sin(yaw / 2)
qy, qz = math.sin(pitch / 2) * math.cos(yaw / 2), math.cos(pitch / 2) * math.sin(yaw / 2)
print(f"pose: {{position: {{x: {cx}, y: {cy}, z: {cz}}}, orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}}")
EOF
)
        call /gui/move_to/pose gz.msgs.GUICamera "$pose"
        echo "kamera: genel bakış ($pose)"
        ;;
    *)
        echo "Kullanım: $0 uav0|uav1|genel"
        exit 2
        ;;
esac
