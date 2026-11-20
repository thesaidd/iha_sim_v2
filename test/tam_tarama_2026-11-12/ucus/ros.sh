#!/bin/bash
# $1 = log, $2.. = python betiği + argümanlar (iha_sim_v2 içinde çalışır)
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
L="$1"; shift
cd ~/iha_sim_v2 && exec python3 -u "$@" > "$L" 2>&1
