#!/bin/bash
# Tüm süreçleri arka planda başlatır, logları /tmp/ altına yazar
#
# Kullanım:
#   ./launch_all.sh          → kamerasız model ($SIM_DIR/gz_models_nocam), varsayılan
#   CAM=1 ./launch_all.sh    → orijinal kameralı model (PX4-Autopilot/Tools/simulation/gz/models);
#                              varsayılan kapalı. Sunucu GstCameraSystem'siz server.config kopyasıyla
#                              açılır ($SIM_DIR/server_nogst.config, GZ_SIM_SERVER_CONFIG_PATH):
#                              2026-10-30 ölçümü sızıntı yok (527 → 529 MB / 2 dk), RTF 1,000, kamera ~16,7 Hz.
#                              Kamera görüntüsü yalnızca gz topic'inde; GStreamer UDP video (QGC 5600) yok.
#                              Kullanım: CAM=1 ./launch_all.sh  (GUI=1 / WORLD=... ile birleştirilebilir)
#   CAM=T ./launch_all.sh    → gerçek kamera modeli: RPi Global Shutter + 16 mm telefoto (1456×1088,
#                              horizontal_fov 0,3114 rad), uçağın altından DİK AŞAĞI bakar
#                              ($SIM_DIR/gz_models_cam_hw/T_nadir). server_nogst.config ile ve her zaman
#                              WORLD=iha_arena (QR hedefi (−60, −60)) açılır; başka WORLD yok sayılır.
#                              GUI=1 ile birleştirilebilir. ROS köprüsü ayrı: camera_bridge.yaml (CLAUDE.md).
#   CAM=T15 ./launch_all.sh  → CAM=T ile aynı, kamera kare hızı 15 Hz (gz_models_cam_hw/T_nadir_15;
#                              T_nadir'den tek fark update_rate 30 → 15). QR geçiş testi için.
#   CAM=T10 ./launch_all.sh  → aynı, 10 Hz (gz_models_cam_hw/T_nadir_10). 15 Hz uçuşta RTF 0,69'a düştü
#                              (2026-10-30), QR geçişleri bununla yapıldı.
#   GUI=1 ./launch_all.sh    → Gazebo penceresini de aç (gz sim -g, WSLg); varsayılan kapalı.
#                              Bu WSL'de yazılımla işleniyor (llvmpipe, ~4 çekirdek, ~670 MB);
#                              2026-10-29 ölçümü: RTF 0,999, sunucu belleği sabit. CAM=1 ile birleştirilebilir.
#   WORLD=iha_arena_v2 ./launch_all.sh → aynı arena, çim/toprak dokulu zemin (gz_worlds/iha_arena_v2.sdf);
#                              CAM=T* ile de seçilebilir: CAM=T10 WORLD=iha_arena_v2 (WORLD verilmezse iha_arena).
#   WORLD=iha_arena_gercek ./launch_all.sh → ortofoto benzeri zemin, halka/ızgara yok, gök mavisi (make_iha_arena.py --gercekci; 2026-11-21);
#                              CAM=T* ile de seçilebilir.
#   WORLD=iha_arena ./launch_all.sh → $SIM_DIR/gz_worlds/iha_arena.sdf (halka, direkler, ızgara);
#                              varsayılan PX4 default.sdf. Dünya adı SDF'ten okunur ("default" kalır),
#                              iki PX4 örneği ve ros_gz aynı adı kullanır. Bu dünyada PX4'ün kamera
#                              takibi kapalı (PX4_GZ_NO_FOLLOW=1), pencere dünyanın genel bakış
#                              kamerasıyla açılır. Dünyayı yeniden üretmek: gz_worlds/make_iha_arena.py
#   GUI=1 VIZ=1 ./launch_all.sh → penceredeki uçak işaretçilerini de başlat (viz_trails.py: küre, 60 sn iz,
#                              zemine dikey çizgi; uav0 kırmızı, uav1 mavi; 5 Hz, yalnızca görsel).
#                              Varsayılan kapalı; GUI=1 olmadan çalışmaz (işaretçileri pencere çizer).
#                              Kamera: $SIM_DIR/cam.sh uav0|uav1|genel
#   LORA=1 CAM=T10 ./launch_all.sh → ek olarak kamera köprüsü + qr_online (/hedef_konum) + lora_relay
#                              (/hedef_konum_role; LoRa taklidi, varsayılan gecikme ~50 ms, kayıp %3) başlatır.
#                              CAM=T/T10/T15 gerekir; opt-in, varsayılan kapalı. Röle parametreleri LORA_ARGS
#                              ile: LORA=1 LORA_ARGS="--gecikme-ort 60 --kayip-orani 5" CAM=T10 ./launch_all.sh
#   YAKLASMA=1 (LORA=1 gerektirir) → uav1 yaklaşma düğümü (lora_yaklasma.py) de başlar: uav1 120 m Hold'da
#                              olunca /hedef_konum_role'ye göre hedefe yönelir, standoff'ta durur, yükselir, Hold.
#                              Parametreler YAKLASMA_ARGS ile. Opt-in, varsayılan kapalı.
#   CAM=W10 ./launch_all.sh  → BAP gerçek kamerası: RPi GS (IMX296) + 6 mm CS lens (hfov 45,4°), 1456×1088, 10 Hz,
#                              dik aşağı, Gauss gürültü + Brown distorsiyonu (gz_models_cam_hw/W_nadir_10, PX4 v1.17
#                              advanced_plane + kamera bloğu). WORLD verilmezse iha_arena_aruco (2 m ArUco, (−60,−60)).
#   ARUCO=1 CAM=W10 ./launch_all.sh → kamera köprüsü + gs_kamera.py (renkli sensör/Bayer taklidi) + bapp
#                              aruco_geolocator (BAPP, varsayılan ~/bapp; /bap/hedef_rapor 24 bayt + /hedef_konum JSON)
#                              + lora_relay.py --mod paket (/bap/hedef_rapor_role; LoRa ToA/duty/kayıp). YAKLASMA=1 ile
#                              uav1 lora_yaklasma.py --girdi paket (enlem/boylam → kendi EKF origin'i). PX4'e
#                              UXRCE_DDS_SYNCT=0 verilir (damgalar sim saatinde kalsın; kare damgasıyla aynı saat).
#                              Ek argümanlar: ARUCO_ARGS (geolocator), LORA_ARGS (röle), YAKLASMA_ARGS.
#   GPU=1 ./launch_all.sh    → WSLg'de Windows GPU'su (Mesa D3D12; GPU_ADI=NVIDIA|AMD); varsayılan yazılım render.
#   Birleştirilebilir:  GUI=1 VIZ=1 WORLD=iha_arena ./launch_all.sh
#   ./launch_all.sh stop     → hepsini durdur
#
# Not: Orijinal server.config (GstCameraSystem yüklü) ile kameralı model ~1,4 GB/dk bellek sızdırıyor
#      (2026-10-29); CAM=1 bu yüzden kopya yapılandırmayı kullanır.
#      Konum değişkeni PX4_GZ_MODEL_POSE'dur; PX4_GZ_POSE PX4 tarafından okunmaz.
#      PX4 derlenmez; derleme gerekiyorsa önce: cd ~/PX4-Autopilot && make px4_sitl

SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # depo kökü (eski: ~/iha_sim_v2 sabit)
PX4_DIR=${PX4_DIR:-~/PX4-Autopilot}
# ROS_DISTRO verilmemişse kurulu olanı seç: jazzy (24.04) varsa o, yoksa humble (22.04).
if [[ -z ${ROS_DISTRO:-} ]]; then
    [[ -d /opt/ros/jazzy ]] && ROS_DISTRO=jazzy || ROS_DISTRO=humble
fi
ROS_SETUP_FILE=/opt/ros/$ROS_DISTRO/setup.bash
ROS_WS=${ROS_WS:-~/ros2_ws}
# Jazzy: /opt/ros/jazzy/setup.bash gz_tools_vendor'ın `gz`'sini ve GZ_CONFIG_PATH'i öne koyar; orada sim8.yaml
# yok → `gz sim` "komut yok" der (2026-11-20). Sistem Harmonic yapılandırmasını öne al (ROS kaynaklansa da çalışsın).
export GZ_CONFIG_PATH=/usr/share/gz${GZ_CONFIG_PATH:+:$GZ_CONFIG_PATH}
# GPU=1: WSLg Mesa varsayılanı yazılım render (llvmpipe); D3D12 sürücüsü Windows GPU'sunu kullanır (sunucu kamerası +
# pencere). GPU_ADI: MESA_D3D12_DEFAULT_ADAPTER_NAME (varsayılan NVIDIA; bu laptopta RTX 4050 / Radeon 780M). 2026-11-20.
if [[ $GPU == 1 ]]; then
    export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=${GPU_ADI:-NVIDIA}
fi
BAPP=${BAPP:-~/bapp}                                     # bapp deposu (aruco_geolocator düğümü, ARUCO=1)
PX4_BUILD=$PX4_DIR/build/px4_sitl_default
NOCAM_MODELS=$SIM_DIR/gz_models_nocam
CAM_SERVER_CONFIG=$SIM_DIR/server_nogst.config   # PX4 gz_bridge/server.config kopyası, GstCameraSystem çıkarılmış
CAM_T_MODELS=$SIM_DIR/gz_models_cam_hw/T_nadir   # RPi GS Camera + 16 mm, 1456×1088, kamera dik aşağı
CAM_T15_MODELS=$SIM_DIR/gz_models_cam_hw/T_nadir_15   # T_nadir ile aynı, update_rate 15
CAM_T10_MODELS=$SIM_DIR/gz_models_cam_hw/T_nadir_10   # T_nadir ile aynı, update_rate 10
CAM_W10_MODELS=$SIM_DIR/gz_models_cam_hw/W_nadir_10   # BAP: 6 mm, dik aşağı, 10 Hz, gürültü + distorsiyon
PY_BAPP=${PY_BAPP:-$BAPP/.venv-ros/bin/python}         # OpenCV 5 + bap_geo (ARUCO=1 düğümleri)
# Raporlanan σ95'in ortak bias tabanı (bapp GeolocatorConfig.sigma_*_bias): bapp varsayılanı GERÇEK uçak (GNSS 1,5 m,
# yaw 2° …) → simde ~5 m σ95. Sim değerleri 6 sim koşusunun PX4 − gerçek poz biasından (bapp scripts/sim_replay.py, 2026-11-20);
# --heading-beta0-deg: advanced_plane düz uçuş yan kayması (bapp scripts/heading_check.py), yaw izleyicisi yalnız tanı.
SIM_BIAS_ARGS=${SIM_BIAS_ARGS:---sigma-gnss-bias-m 0.3 --sigma-att-bias-deg 0.3 --sigma-yaw-bias-deg 0.5 --sigma-hag-bias-m 0.1 --heading-beta0-deg 2.7}
ARENA_WORLDS=$SIM_DIR/gz_worlds
VIZ_SCRIPT=$SIM_DIR/viz_trails.py
PGID_FILE=/tmp/iha_sim.pgids
LOG_MAX_BYTES=$((5 * 1024 * 1024))   # Log başına sınır; aşınca .1'e döner (tek yedek)

# UAV1_POSE: uav1'in doğma pozu (Gazebo ENU: x=Doğu, y=Kuzey, yaw 0 = Doğu'ya bakar). Yerden kalkış senaryosunda
# (2026-11-15) uav1 uzak bir üste doğar; aynı üs IHA_UAV1_SPAWN_NE="N,E" (ayrim.py) ve lora_yaklasma
# --kaynak-fark-n/-e ile de verilmelidir. Verilmezse eski konum.
UAV_POSES=("0,0,0.5,0,0,0" "${UAV1_POSE:--15,15,0.5,0,0,0}")   # uav0, uav1 (x,y,z,roll,pitch,yaw)

# stdin'i dosyaya yazar, LOG_MAX_BYTES aşılınca dosyayı .1'e taşır.
# read -n 4096: satır sonu gelmeyen akış (ör. pxh> istemi) belleği şişirmesin.
limit_log() {
    local file="$1" n=0 line
    : > "$file"
    while IFS= read -r -n 4096 line || [[ -n $line ]]; do
        printf '%s\n' "$line" >> "$file"
        if (( ++n % 100 == 0 )) && (( $(stat -c %s "$file") > LOG_MAX_BYTES )); then
            mv -f "$file" "$file.1"
        fi
    done
}
export -f limit_log
export LOG_MAX_BYTES

# Komutu yeni bir süreç grubunda başlatır; grup kimliği stop için kaydedilir.
start_bg() {
    setsid bash -c "$1" < /dev/null > /dev/null 2>&1 &
    echo $! >> "$PGID_FILE"
}

# Bu bölümdeki (GZ_PARTITION) gz sim süreçleri. Eski kalıp öldürmesi komut satırında "gz sim" geçen her süreci
# (çağıran kabuk, başka bölümdeki ayrı bir gz, ör. kalibrasyon) öldürüyordu (2026-11-20). Yalnız yürütülebilir
# dosyası ruby/gz olan ve GZ_PARTITION'ı bizimkiyle aynı süreçler seçilir.
gz_pids() {
    local p exe part
    for p in $(pgrep -f "gz sim"); do
        exe=$(readlink "/proc/$p/exe" 2>/dev/null) || continue
        [[ $exe == *ruby* || $exe == */gz || $exe == *gz-sim* ]] || continue
        part=$(tr '\0' '\n' < "/proc/$p/environ" 2>/dev/null | sed -n 's/^GZ_PARTITION=//p')
        [[ $part == "${GZ_PARTITION:-}" ]] && echo "$p"
    done
}

stop_all() {
    if [[ -f $PGID_FILE ]]; then
        while read -r pgid; do kill -- "-$pgid" 2>/dev/null; done < "$PGID_FILE"
        rm -f "$PGID_FILE"
    fi
    # Bu script dışında başlatılmış kalıntılar
    pkill -x px4 2>/dev/null
    pkill -x MicroXRCEAgent 2>/dev/null
    kill $(gz_pids) 2>/dev/null
    # Elle başlatılan yardımcı düğümler: qr_recorder.py / qr_online.py (ve qr_pass/qr_watch,
    # mavlink_takeoff, kamera köprüsü) PGID_FILE'a yazılmıyor ve px4/Agent/gz desenlerine
    # uymuyor → eski stop bunları yetim bırakıyordu (2026-10-31). Script adıyla hedefli öldür
    # (ERE; "launch_all.sh stop" komut satırına uymaz). Önce SIGTERM: kaydedici kuyruğunu
    # boşaltıp özetini, online özetini yazsın; 5 sn sonra kalanı zorla.
    local helper_pat='alan_tarama\.py|qr_recorder\.py|qr_online\.py|qr_pass\.py|qr_watch\.py|mavlink_takeoff\.py|camera_bridge\.yaml|kamikaze_bridge\.yaml|lora_relay\.py|lora_yaklasma\.py|yer_gorevi\.py|gs_kamera\.py|aruco_geolocator_node\.py|canli_kamera\.py|gz_gercek_poz\.py'
    pkill -f "$helper_pat" 2>/dev/null
    for _ in {1..5}; do pgrep -f "$helper_pat" > /dev/null || break; sleep 1; done
    pkill -9 -f "$helper_pat" 2>/dev/null
    # Bellek baskısı altındaki gz sunucusu SIGTERM'e geç yanıt verebilir; 15 sn bekle, sonra zorla.
    for _ in {1..15}; do
        pgrep -x px4 > /dev/null || [[ -n $(gz_pids) ]] || pgrep -x MicroXRCEAgent > /dev/null || return 0
        sleep 1
    done
    pkill -9 -x px4 2>/dev/null
    pkill -9 -x MicroXRCEAgent 2>/dev/null
    kill -9 $(gz_pids) 2>/dev/null
    sleep 1
}

# Log'da (veya döndürülmüş yedeğinde) desen çıkana kadar en fazla $3 sn bekler.
wait_for() {
    local file="$1" pattern="$2" timeout="$3" t=0
    until grep -qs "$pattern" "$file" "$file.1"; do
        (( t++ >= timeout )) && return 1
        sleep 1
    done
}

stop_all
if [[ $1 == stop ]]; then
    echo "Tüm süreçler durduruldu."
    exit 0
fi
rm -f /tmp/{dds,gz,gz_gui,viz_trails,px4_0,px4_1,gs_kamera,aruco_geolocator,gs_kamera_uav1,aruco_geolocator_uav1,kopru_uav1,lora_yaklasma}.log{,.1} /tmp/lora_yaklasma_sonuc.jsonl

# PX4'ün px4-rc.gzsim'deki ortamı: gz_env.sh model/world/plugin yollarını ayarlar.
source "$PX4_BUILD/rootfs/gz_env.sh"
if [[ $CAM == W10 ]]; then
    models=$CAM_W10_MODELS
    if [[ ! -f $models/advanced_plane/model.sdf ]]; then
        echo "HATA: model yok: $models/advanced_plane/model.sdf"
        exit 1
    fi
    export PX4_GZ_MODELS=$models
    WORLD=${WORLD:-iha_arena_aruco}
    echo "Model: 6 mm, aşağı bakan kamera (W_nadir_10) — $PX4_GZ_MODELS"
    export GZ_SIM_SERVER_CONFIG_PATH=$CAM_SERVER_CONFIG
    echo "Sunucu yapılandırması: $GZ_SIM_SERVER_CONFIG_PATH (GstCameraSystem yok)"
elif [[ $CAM == 1 || $CAM == T || $CAM == T15 || $CAM == T10 ]]; then
    if [[ $CAM == T || $CAM == T15 || $CAM == T10 ]]; then
        models=$CAM_T_MODELS
        [[ $CAM == T15 ]] && models=$CAM_T15_MODELS
        [[ $CAM == T10 ]] && models=$CAM_T10_MODELS
        if [[ ! -f $models/advanced_plane/model.sdf ]]; then
            echo "HATA: model yok: $models/advanced_plane/model.sdf"
            exit 1
        fi
        export PX4_GZ_MODELS=$models
        # iha_arena = özgün gri zemin (varsayılan); iha_arena_v2 = dokulu zemin (make_iha_arena.py --zemin dokulu).
        if [[ -n $WORLD && $WORLD != iha_arena && $WORLD != iha_arena_v2 && $WORLD != iha_arena_gercek ]]; then
            echo "  UYARI: CAM=$CAM yalnızca iha_arena / iha_arena_v2 / iha_arena_gercek ile çalışır (QR hedefi); WORLD=$WORLD yok sayıldı"
            WORLD=iha_arena
        fi
        WORLD=${WORLD:-iha_arena}
        echo "Model: 16 mm, aşağı bakan kamera ($(basename "$models")) — $PX4_GZ_MODELS"
    else
        echo "Model: orijinal (kameralı) — $PX4_GZ_MODELS"
    fi
    # GstCameraSystem yüklüyken sunucu kare başına ~1 RGB kare sızdırıyor (2026-10-29/30); kopyada bu satır yok.
    if [[ ! -f $CAM_SERVER_CONFIG ]]; then
        echo "HATA: sunucu yapılandırması yok: $CAM_SERVER_CONFIG"
        exit 1
    fi
    export GZ_SIM_SERVER_CONFIG_PATH=$CAM_SERVER_CONFIG
    echo "Sunucu yapılandırması: $GZ_SIM_SERVER_CONFIG_PATH (GstCameraSystem yok)"
else
    # px4-rc.gzsim çalışan bir dünya bulunca gz_env.sh'i yeniden yüklemez,
    # spawn için bu PX4_GZ_MODELS değerini kullanır.
    export PX4_GZ_MODELS=$NOCAM_MODELS
    echo "Model: kamerasız kopya — $PX4_GZ_MODELS"
fi

if [[ -n $WORLD && $WORLD != default ]]; then
    WORLD_FILE=$ARENA_WORLDS/$WORLD.sdf
    if [[ ! -f $WORLD_FILE ]]; then
        echo "HATA: dünya dosyası yok: $WORLD_FILE"
        exit 1
    fi
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$ARENA_WORLDS
    export PX4_GZ_NO_FOLLOW=1   # px4-rc.gzsim pencere kamerasını uçağa kilitlemesin
else
    WORLD_FILE=$PX4_GZ_WORLDS/default.sdf
fi
GZ_WORLD_NAME=$(sed -n 's/.*<world name="\([^"]*\)".*/\1/p' "$WORLD_FILE" | head -1)
echo "Dünya: $WORLD_FILE (ad: $GZ_WORLD_NAME)"

echo "[1/4] DDS Agent başlatılıyor..."
start_bg "MicroXRCEAgent udp4 -p 8888 2>&1 | limit_log /tmp/dds.log"

echo "[2/4] Gazebo sunucusu başlatılıyor$([[ $GUI == 1 ]] && echo " + pencere" || echo " (pencere yok)")..."
start_bg "gz sim --verbose=1 -r -s '$WORLD_FILE' 2>&1 | limit_log /tmp/gz.log"
for _ in {1..30}; do
    gz topic -l 2>/dev/null | grep -q "^/world/$GZ_WORLD_NAME/clock" && break
    sleep 1
done
if [[ $GUI == 1 ]]; then
    # Yalnızca arayüz; sunucuyla aynı ortam (gz_env.sh + PX4_GZ_MODELS), çalışan dünyaya bağlanır
    start_bg "gz sim -g 2>&1 | limit_log /tmp/gz_gui.log"
fi
if [[ $VIZ == 1 ]]; then
    if [[ $GUI == 1 ]]; then
        # İşaretçileri pencerenin MarkerManager'ı çizer; pencere hazır olana kadar istekler yanıtsız kalır
        echo "      + uçak işaretçileri (viz_trails.py)"
        start_bg "python3 -u '$VIZ_SCRIPT' --world '$GZ_WORLD_NAME' 2>&1 | limit_log /tmp/viz_trails.log"
    else
        echo "  UYARI: VIZ=1 için GUI=1 gerekli; işaretçiler başlatılmadı"
    fi
fi

# Kabuk açmadan (-d) çalışır; stdin gerekmez, pxh> istemi dönmez.
# ARUCO=1: DDS damgaları PX4 saatinde (= gz sim saati) kalsın; SYNCT=1'de ajan duvar saatine kaydırılır ve
# RTF < 1'de kare damgasıyla (sim saati) arasındaki fark büyür (CLAUDE.md "timestamp_sample 1–2 s geride").
PX4_EXTRA_ENV=""
[[ $ARUCO == 1 ]] && PX4_EXTRA_ENV="PX4_PARAM_UXRCE_DDS_SYNCT=0"
# Simülatör kaynaklı EKF hataları (2026-11-20 ölçümü, docs/aruco_sim.md; SIM_EKF_DUZELT=0 ile kapatılır):
#  - gz NavSat gecikmesiz, PX4 EKF2_GPS_DELAY 110 ms varsayıyor → konum gerçeğin ~100 ms önünde (15 m/s'de 1,3–1,5 m).
#  - Magnetometre: PX4 gz_bridge'in eksen eşlemesi gz-sim 8.15 çıktısıyla tutarsız (yöne bağlı ±10° yaw, ±4° tilt);
#    düzeltme PX4 kaynağında: px4_yamalar/gz_bridge_magnetometre.patch (uygulanmadan bu ortam eksik). Sapma
#    PX4'ün WMM tablosundan (EKF2_DECL_TYPE 3, varsayılan); gz alanı +2,68°, WMM +3,41° → ~0,7° gerçekçi kalıntı.
#    DECL_TYPE açıkça verilir: önceki bir koşunun parametre dosyasında (rootfs/<i>/parameters.bson) kalmasın.
if [[ ${SIM_EKF_DUZELT:-1} == 1 ]]; then
    PX4_EXTRA_ENV="$PX4_EXTRA_ENV PX4_PARAM_EKF2_GPS_DELAY=0 PX4_PARAM_EKF2_DECL_TYPE=3"
fi
for i in 0 1; do
    echo "[$((i + 3))/4] PX4 instance $i başlatılıyor (konum ${UAV_POSES[$i]})..."
    start_bg "cd $PX4_DIR && env $PX4_EXTRA_ENV PX4_SIM_MODEL=gz_advanced_plane PX4_SYS_AUTOSTART=4008 PX4_GZ_WORLD=$GZ_WORLD_NAME PX4_GZ_MODEL_POSE='${UAV_POSES[$i]}' $PX4_BUILD/bin/px4 -d -i $i 2>&1 | limit_log /tmp/px4_$i.log"
    wait_for /tmp/px4_$i.log "Startup script returned successfully" 60 \
        || echo "  UYARI: PX4-$i 60 sn içinde başlangıç betiğini bitirmedi"
done

echo "Kontrol ediliyor..."
# "Ready for takeoff" QGC bağlanmadan gelmez (No connection to the GCS); burada
# başlangıç betiğinin bittiği ve DDS topic'lerinin ROS 2'de göründüğü doğrulanır.
status=0
for i in 0 1; do
    if grep -qs "Startup script returned successfully" /tmp/px4_$i.log /tmp/px4_$i.log.1; then
        echo "  PX4-$i: başlangıç betiği tamamlandı"
    else
        echo "  PX4-$i: başlangıç betiği TAMAMLANMADI"
        status=1
    fi
done

source "$ROS_SETUP_FILE"
source "$ROS_WS/install/setup.bash"
# Başlangıç betiği DDS istemcisi topic'leri açmadan biter; 60 sn'ye kadar tekrar dene.
for ns in /fmu/out/ /px4_1/fmu/out/; do
    count=0
    for attempt in {1..6}; do
        count=$(ros2 topic list --no-daemon --spin-time 5 2>/dev/null | grep -c "^$ns")
        (( count > 0 )) && break
        sleep 5
    done
    if (( count > 0 )); then
        echo "  $ns: $count topic görünüyor"
    else
        echo "  $ns: topic YOK"
        status=1
    fi
done

# LoRa röle + çevrimiçi QR (opt-in, LORA=1; varsayılan kapalı → davranış değişmez). CAM=T* (aşağı bakan
# kamera) gerekir. Kamera köprüsü + qr_online (/hedef_konum) + lora_relay (/hedef_konum_role) birlikte
# başlar; üçü de start_bg ile PGID_FILE'a yazılır (stop temizler). ROS yukarıda source edildi.
# Röle parametreleri LORA_ARGS ile: LORA=1 LORA_ARGS="--gecikme-ort 60 --kayip-orani 5" ...
if [[ $LORA == 1 ]]; then
    if [[ $CAM != T && $CAM != T10 && $CAM != T15 ]]; then
        echo "  UYARI: LORA=1 için CAM=T/T10/T15 gerekir (aşağı bakan kamera); röle başlatılmadı"
    else
        echo "[LoRa] kamera köprüsü + qr_online + lora_relay başlatılıyor..."
        ROS_SETUP="source $ROS_SETUP_FILE && source $ROS_WS/install/setup.bash"
        mkdir -p /tmp/qr_online_log
        start_bg "$ROS_SETUP && ros2 run ros_gz_bridge parameter_bridge --ros-args \
            -p config_file:=$SIM_DIR/camera_bridge.yaml \
            -p qos_overrides./uav0/camera/image.publisher.reliability:=best_effort \
            -p qos_overrides./uav0/camera/image.publisher.depth:=5 \
            -p qos_overrides./uav0/camera/camera_info.publisher.reliability:=best_effort \
            -p qos_overrides./uav0/camera/camera_info.publisher.depth:=5 2>&1 | limit_log /tmp/kopru.log"
        start_bg "$ROS_SETUP && cd $SIM_DIR && python3 -u qr_online.py --log /tmp/qr_online_log 2>&1 | limit_log /tmp/qr_online.log"
        start_bg "$ROS_SETUP && cd $SIM_DIR && python3 -u lora_relay.py ${LORA_ARGS:-} 2>&1 | limit_log /tmp/lora_relay.log"
        echo "      /hedef_konum → /hedef_konum_role (varsayılan gecikme ~50 ms, kayıp %3); loglar /tmp/{kopru,qr_online,lora_relay}.log"
        # YAKLASMA=1 (LORA=1 gerektirir): uav1 yaklaşma düğümü. uav1 120 m Hold'da olunca /hedef_konum_role'ye
        # göre yönelir; düğüm uav1 hazır olana kadar bekler. Parametreler YAKLASMA_ARGS ile.
        if [[ $YAKLASMA == 1 ]]; then
            start_bg "$ROS_SETUP && cd $SIM_DIR && python3 -u lora_yaklasma.py --uav 1 --sonuc /tmp/lora_yaklasma_sonuc.jsonl ${YAKLASMA_ARGS:-} 2>&1 | limit_log /tmp/lora_yaklasma.log"
            echo "      + uav1 yaklaşma düğümü (lora_yaklasma.py); log /tmp/lora_yaklasma.log, sonuç /tmp/lora_yaklasma_sonuc.jsonl"
        fi
    fi
fi
# ArUco zinciri (ARUCO=1, CAM=W10): kamera köprüsü → gs_kamera (Bayer) → bapp aruco_geolocator → lora_relay (paket)
# [→ lora_yaklasma --girdi paket, YAKLASMA=1]. LORA=1 (QR zinciri) ile birlikte kullanılmaz.
if [[ $ARUCO == 1 ]]; then
    if [[ $CAM != W10 ]]; then
        echo "  UYARI: ARUCO=1 için CAM=W10 gerekir; ArUco zinciri başlatılmadı"
    elif [[ ! -f $BAPP/bap_ros/aruco_geolocator_node.py || ! -x $PY_BAPP ]]; then
        echo "  HATA: bapp bulunamadı ($BAPP/bap_ros/aruco_geolocator_node.py, $PY_BAPP)"; status=1
    else
        echo "[ArUco] kamera köprüsü + gs_kamera + aruco_geolocator + lora_relay (paket) başlatılıyor..."
        ROS_SETUP="source $ROS_SETUP_FILE && source $ROS_WS/install/setup.bash"
        rm -rf /tmp/aruco_log && mkdir -p /tmp/aruco_log
        start_bg "$ROS_SETUP && ros2 run ros_gz_bridge parameter_bridge --ros-args \
            -p config_file:=$SIM_DIR/camera_bridge.yaml \
            -p qos_overrides./uav0/camera/image.publisher.reliability:=best_effort \
            -p qos_overrides./uav0/camera/image.publisher.depth:=5 \
            -p qos_overrides./uav0/camera/camera_info.publisher.reliability:=best_effort \
            -p qos_overrides./uav0/camera/camera_info.publisher.depth:=5 2>&1 | limit_log /tmp/kopru.log"
        start_bg "$ROS_SETUP && cd $SIM_DIR && $PY_BAPP -u gs_kamera.py ${GS_ARGS:-} 2>&1 | limit_log /tmp/gs_kamera.log"
        start_bg "$ROS_SETUP && $PY_BAPP -u $BAPP/bap_ros/aruco_geolocator_node.py --log /tmp/aruco_log \
            --t-body-cam 0.07 0 0.1 ${ARUCO_ARGS:---corner-sensor color --camera-json $SIM_DIR/kalibrasyon/w_nadir_10_sim.json $SIM_BIAS_ARGS} 2>&1 | limit_log /tmp/aruco_geolocator.log"
        start_bg "$ROS_SETUP && cd $SIM_DIR && $PY_BAPP -u lora_relay.py --mod paket ${LORA_ARGS:-} 2>&1 | limit_log /tmp/lora_relay.log"
        echo "      loglar /tmp/{kopru,gs_kamera,aruco_geolocator,lora_relay}.log, kare günlüğü /tmp/aruco_log/"
        if [[ $YAKLASMA == 1 ]]; then
            # Kamikaze kamerası (uav1, ileri 15° aşağı) → gs_kamera → aruco_geolocator (2. örnek): dalışta marker okuma.
            # Okuma dalış hedefini DEĞİŞTİRMEZ (hedef = LoRa raporu); /bap/uav1/okuma'yı lora_yaklasma kaydeder.
            rm -rf /tmp/aruco_log_uav1 && mkdir -p /tmp/aruco_log_uav1
            start_bg "$ROS_SETUP && ros2 run ros_gz_bridge parameter_bridge --ros-args -r __node:=kamikaze_bridge \
                -p config_file:=$SIM_DIR/kamikaze_bridge.yaml \
                -p qos_overrides./uav1/camera/image.publisher.reliability:=best_effort \
                -p qos_overrides./uav1/camera/image.publisher.depth:=5 \
                -p qos_overrides./uav1/camera/camera_info.publisher.reliability:=best_effort \
                -p qos_overrides./uav1/camera/camera_info.publisher.depth:=5 2>&1 | limit_log /tmp/kopru_uav1.log"
            start_bg "$ROS_SETUP && cd $SIM_DIR && $PY_BAPP -u gs_kamera.py --giris /uav1/camera/image \
                --info /uav1/camera/camera_info --cikis /uav1/camera/image_gs --info-cikis /uav1/camera/camera_info_gs \
                --ad gs_kamera_uav1 2>&1 | limit_log /tmp/gs_kamera_uav1.log"
            start_bg "$ROS_SETUP && $PY_BAPP -u $BAPP/bap_ros/aruco_geolocator_node.py --log /tmp/aruco_log_uav1 \
                --image /uav1/camera/image_gs --camera-info /uav1/camera/camera_info_gs --fmu /px4_1/fmu \
                --mount-deg 75 0 0 --t-body-cam 0.5 0 0 --report-topic /bap/uav1/hedef_rapor --json-topic /bap/uav1/okuma \
                ${ARUCO_UAV1_ARGS:---corner-sensor color --camera-json $SIM_DIR/kalibrasyon/w_nadir_10_sim.json $SIM_BIAS_ARGS} \
                --node-name aruco_geolocator_uav1 2>&1 | limit_log /tmp/aruco_geolocator_uav1.log"
            echo "      + uav1 kamera (ileri 15° aşağı) → gs_kamera → aruco_geolocator (uav1); log /tmp/aruco_geolocator_uav1.log"
            start_bg "$ROS_SETUP && cd $SIM_DIR && BAPP=$BAPP $PY_BAPP -u lora_yaklasma.py --uav 1 --girdi paket \
                --sonuc /tmp/lora_yaklasma_sonuc.jsonl ${YAKLASMA_ARGS:-} 2>&1 | limit_log /tmp/lora_yaklasma.log"
            echo "      + uav1 yaklaşma düğümü (lora_yaklasma.py --girdi paket); log /tmp/lora_yaklasma.log"
        fi
    fi
fi
if [[ $YAKLASMA == 1 && $LORA != 1 && $ARUCO != 1 ]]; then
    echo "  UYARI: YAKLASMA=1 için LORA=1 gerekir; yaklaşma düğümü başlatılmadı"
fi

echo ""
echo "Log dosyaları: /tmp/dds.log | /tmp/gz.log$([[ $GUI == 1 ]] && echo " | /tmp/gz_gui.log")$([[ $GUI == 1 && $VIZ == 1 ]] && echo " | /tmp/viz_trails.log")$([[ $LORA == 1 ]] && echo " | /tmp/kopru.log | /tmp/qr_online.log | /tmp/lora_relay.log") | /tmp/px4_0.log | /tmp/px4_1.log  (sınır: $((LOG_MAX_BYTES / 1024 / 1024)) MB, yedek .1)"
echo "Durdurmak için: $0 stop"
echo "Sonraki adım: QGC'yi kendin aç, kalkış yap, sonra orbit node'u başlat."
exit $status
