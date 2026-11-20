#!/bin/bash
# AŞAMA 3 — tam zincir: uav0 alan taraması → QR tespiti → LoRa rölesi → uav1 YERDEN kalkış → dalış →
# tırmanış → üsse dönüş → iniş. Sabit dış kamera ile video kaydı (kayit.py).
set -u
U=~/iha_sim_v2/test/yerden_kalkis_2026-11-15/ucus
D="$U/s1"; cd "$D"
source "$U/ortam.sh"
rm -f sonuc.jsonl yak.log tarama.log kalkis0.log
bash "$U/kapat.sh" > kapatma0.log 2>&1; sleep 2
bash "$U/basla.sh" lora > baslatma.log 2>&1
python3 "$U/rtf.py" 6 | tee rtf_yerde.txt
# pencere + işaretçiler + sabit kamera (uav0 arenası ile uav1 üssü aynı kadrajda)
bash "$U/pencere.sh" >> baslatma.log 2>&1
setsid bash "$U/ros.sh" "$D/viz.log" viz_trails.py < /dev/null > /dev/null 2>&1 &
sleep 3
bash "$U/kamera_sabit.sh" -500 -1150 620 -480 60 0 | tail -1
python3 "$U/rtf.py" 6 | tee rtf_pencere.txt
# kayıt + iz
setsid python3 "$U/kayit.py" --cikti "$D" --ad zincir --sure 1500 \
    --log "$D/yak.log" "$D/tarama.log" /tmp/qr_online.log < /dev/null > kayit.log 2>&1 &
echo $! > kayit.pid
setsid bash "$U/ros.sh" "$D/iz2.log" "$U/iz2.py" "$D/iz.csv" < /dev/null > /dev/null 2>&1 &
sleep 2
# uav1 düğümü: üste, yerde bekler; ilk röle mesajı kalkış komutudur
setsid bash "$U/ros.sh" "$D/yak.log" lora_yaklasma.py --uav 1 --profil dalis --alcalma-siniri 4.0 \
    --baslangic yer $US_ARGS --guvenli-irtifa 120 --bekleme-asimi 1500 \
    --sonuc "$D/sonuc.jsonl" --abort-file "$D/abort" < /dev/null > /dev/null 2>&1 &
# uav0: kalkış (yalnız uav0) ve 100 m Hold
setsid bash "$U/ros.sh" "$D/kalkis0.log" mavlink_takeoff.py --uav 0 --watch 3000 < /dev/null > /dev/null 2>&1 &
for i in $(seq 1 60); do grep -q "Turlama başladı" kalkis0.log 2>/dev/null && break; sleep 5; done
python3 "$U/rtf.py" 6 | tee rtf_ucus.txt
# uav0 alan taraması (QR 3. hatta bulunur: --ilk-hat -40)
setsid bash "$U/ros.sh" "$D/tarama.log" alan_tarama.py --ilk-hat -40 --iz "$D/tarama_iz.csv" \
    --done-file "$D/tarama.done" --abort-file "$D/tarama.abort" --result "$D/tarama_sonuc.jsonl" < /dev/null > /dev/null 2>&1 &
for i in $(seq 1 240); do [ -s sonuc.jsonl ] && break; sleep 5; done
sleep 5
python3 "$U/rtf.py" 5 | tee rtf_son.txt
kill -INT "$(cat kayit.pid)" 2>/dev/null; sleep 4
pkill -INT -f "iz2[.]py"; sleep 1
cp /tmp/qr_online.log /tmp/lora_relay.log /tmp/px4_0.log /tmp/px4_1.log . 2>/dev/null
bash "$U/kapat.sh" > kapatma.log 2>&1
kill "$(cat /tmp/gz_gui_kayit.pid)" 2>/dev/null
