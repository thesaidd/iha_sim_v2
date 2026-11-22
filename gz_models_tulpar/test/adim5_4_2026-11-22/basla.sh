#!/bin/bash
source "$(dirname "$0")/ortam.sh"
source ~/venv_iha/bin/activate
cd ~/iha_sim_v2 && env -u GUI -u YAKLASMA -u VIZ UAV0=tulpar CAM=T10 LORA=1 ./launch_all.sh
