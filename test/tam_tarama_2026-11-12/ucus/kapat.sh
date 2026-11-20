#!/bin/bash
cd ~/iha_sim_v2 && ./launch_all.sh stop
sleep 1
pgrep -af "px4|MicroXRCE|ros_gz|python3" | grep -v "pgrep\|kapat.sh" || echo "kalinti surec yok"
