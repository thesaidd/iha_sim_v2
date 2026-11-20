#!/bin/bash
# Aşama 1 (yalnız kalkış + üsse dönüş + iniş), yeni üs (200, -950), pist 90°.
set -u
U=~/iha_sim_v2/test/yerden_kalkis_2026-11-15/ucus
D="$U/a4"; cd "$D"
source "$U/ortam.sh"
bash "$U/kapat.sh" > kapatma0.log 2>&1; sleep 2
bash "$U/basla.sh" > baslatma.log 2>&1
python3 "$U/rtf.py" 6 | tee rtf0.txt
setsid bash "$U/ros.sh" "$D/iz2.log" "$U/iz2.py" "$D/iz.csv" < /dev/null > /dev/null 2>&1 &
sleep 2
setsid bash "$U/ros.sh" "$D/yak.log" lora_yaklasma.py --uav 1 --profil dalis --alcalma-siniri 4.0 \
    --baslangic yer $US_ARGS --hedef -60 -60 --uav0-yaricap 0 --guvenli-irtifa 120 \
    --sonuc "$D/sonuc.jsonl" --abort-file "$D/abort" < /dev/null > /dev/null 2>&1 &
for i in $(seq 1 260); do
    [ -s "$D/sonuc.jsonl" ] && break
    pgrep -f "lora_yaklasma[.]py --uav 1" > /dev/null || { [ $i -gt 6 ] && break; }
    sleep 5
done
sleep 3
python3 "$U/rtf.py" 5 | tee rtf1.txt
pkill -INT -f "iz2[.]py"; sleep 1
cp /tmp/px4_1.log /tmp/px4_0.log . 2>/dev/null
bash "$U/kapat.sh" > kapatma.log 2>&1
