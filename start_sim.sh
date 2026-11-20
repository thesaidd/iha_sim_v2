#!/bin/bash
# UYARI: Bu script make px4_sitl ile ORİJİNAL KAMERALI modeli açar; gz sim sunucusu
#        ~650 MB/30 sn bellek sızdırır (2026-10-29 ölçümü). Günlük kullanım için launch_all.sh.
#
# V2 Simülasyon Başlatıcı
# Terminal 1'de çalıştır, diğer terminaller için talimatları ekrana yazar.

echo "UYARI: kameralı model, bellek sızdırır — günlük kullanım için ~/iha_sim_v2/launch_all.sh"

echo "========================================="
echo " V2 Orbit Simülasyonu"
echo " UAV'lar yerde doğar: uav0 (0,0), uav1 (-15,15)"
echo "========================================="
echo ""
echo "ADIM 1 — Bu terminalde DDS Agent başlatılıyor..."
echo "Hazır olunca ADIM 2 ve 3'ü ayrı terminallerde çalıştır."
echo ""
echo "ADIM 2 (yeni terminal):"
echo "  cd ~/PX4-Autopilot && PX4_GZ_WORLD=default PX4_GZ_MODEL_POSE=\"0,0,0.5,0,0,0\" PX4_SYS_AUTOSTART=4008 make px4_sitl gz_advanced_plane"
echo ""
echo "ADIM 3 (yeni terminal, ADIM 2'den 5 sn sonra):"
echo "  cd ~/PX4-Autopilot && PX4_GZ_WORLD=default PX4_GZ_MODEL_POSE=\"-15,15,0.5,0,0,0\" PX4_SYS_AUTOSTART=4008 instance=1 ./build/px4_sitl_default/bin/px4 -i 1"
echo ""
echo "ADIM 4 (yeni terminal, her şey hazır olunca):"
echo "  source ~/ros2_ws/install/setup.bash && source /opt/ros/humble/setup.bash && python3 ~/iha_sim_v2/v2_orbit_mission.py"
echo ""
echo "-----------------------------------------"
echo "DDS Agent başlatılıyor..."
MicroXRCEAgent udp4 -p 8888
