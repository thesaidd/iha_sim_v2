#!/bin/bash
# $1 = "lora" → kamera köprüsü + qr_online + röle de başlar; "gui" eklenirse pencere + işaretçiler
source "$(dirname "$0")/ortam.sh"
EK=""
[[ " $* " == *" lora "* ]] && EK="$EK LORA=1"
[[ " $* " == *" gui "* ]] && EK="$EK GUI=1 VIZ=1"
cd ~/iha_sim_v2 && env -u GUI -u YAKLASMA -u LORA -u VIZ $EK CAM=T10 WORLD=iha_arena_v2 ./launch_all.sh
