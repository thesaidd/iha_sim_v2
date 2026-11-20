#!/bin/bash
# Ayırıcı deneme: değiştirilmemiş kalkış betiğiyle uav0 (eski nokta, kontrol) + uav1 (yeni üs).
set -u
U=~/iha_sim_v2/test/yerden_kalkis_2026-11-15/ucus
D="$U/t1"
cd "$D"
bash "$U/kapat.sh" > kapatma0.log 2>&1
sleep 2
bash "$U/basla.sh" > baslatma.log 2>&1
python3 "$U/rtf.py" 6 | tee rtf0.txt
setsid bash "$U/ros.sh" "$D/iz2.log" "$U/iz2.py" "$D/iz.csv" < /dev/null > /dev/null 2>&1 &
sleep 1
setsid bash "$U/ros.sh" "$D/kalkis.log" mavlink_takeoff.py --uav 0 1 --ucus-dinamigi --watch 20 < /dev/null > /dev/null 2>&1 &
sleep 90
