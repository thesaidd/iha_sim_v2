#!/bin/bash
# $1 = log, $2.. = python betiği + argümanlar (iha_sim_v2 içinde)
source "$(dirname "$0")/ortam.sh"
source /opt/ros/jazzy/setup.bash; source ~/ros2_ws/install/setup.bash; source ~/venv_iha/bin/activate
L="$1"; shift
cd ~/iha_sim_v2 && exec python3 -u "$@" > "$L" 2>&1
