#!/bin/bash
# Tek zincir koşusu: $1 = koşu klasörü adı (ör. tekrar1). Yöntem 5.4 ile aynı; RTF her ~30 sn örneklenir.
set -u
B=~/iha_sim_v2/gz_models_tulpar/test/adim5_4_2026-11-22
D=$B/$1; mkdir -p $D; cd $D
source $B/ortam.sh
cd ~/iha_sim_v2 && ./launch_all.sh stop > $D/kapatma0.log 2>&1; sleep 3
bash $B/basla.sh > $D/baslatma.log 2>&1
if ! grep -q "PX4-0: başlangıç betiği tamamlandı" $D/baslatma.log || ! grep -q "PX4-1: başlangıç betiği tamamlandı" $D/baslatma.log; then
  echo "BASLATMA BASARISIZ" > $D/DURUM; ./launch_all.sh stop > $D/kapatma.log 2>&1; exit 1; fi
python3 $B/rtf.py 6 > $D/rtf_yerde.txt
setsid bash $B/ros.sh $D/poz.log gz_gercek_poz.py --out $D/gercek_poz.csv < /dev/null > /dev/null 2>&1 &
PYTHONPATH=~/iha_sim_v2 setsid bash $B/ros.sh $D/iz2.log $B/iz2.py $D/iz.csv < /dev/null > /dev/null 2>&1 &
(while :; do python3 $B/rtf.py 5 >> $D/rtf_surekli.txt 2>&1; sleep 25; done) < /dev/null > /dev/null 2>&1 &
RTFPID=$!
setsid bash $B/ros.sh $D/yak.log lora_yaklasma.py --uav 1 --profil dalis --alcalma-siniri 4.0 --baslangic yer $US_ARGS --guvenli-irtifa 120 --bekleme-asimi 1500 --sonuc $D/sonuc.jsonl --abort-file $D/abort < /dev/null > /dev/null 2>&1 &
sleep 8
setsid bash $B/ros.sh $D/kalkis0.log mavlink_takeoff.py --uav 0 --el-firlatma --alt 100 --watch 3000 < /dev/null > /dev/null 2>&1 &
for i in $(seq 1 40); do grep -q "Turlama başladı" $D/kalkis0.log 2>/dev/null && break; sleep 3; done
sleep 20
setsid bash $B/ros.sh $D/tarama.log alan_tarama.py --ilk-hat -40 --koridor-yok --iz $D/tarama_iz.csv --done-file $D/tarama.done --abort-file $D/tarama.abort --result $D/tarama_sonuc.jsonl < /dev/null > /dev/null 2>&1 &
for i in $(seq 1 260); do [ -s $D/sonuc.jsonl ] && break; sleep 5; done
sleep 15
cp /tmp/qr_online.log /tmp/lora_relay.log /tmp/px4_0.log /tmp/px4_1.log $D/ 2>/dev/null
kill $RTFPID 2>/dev/null
for p in $(pgrep -f "^python3 -u (gz_gercek_poz|.*iz2)"); do kill -INT $p; done
sleep 3
cd ~/iha_sim_v2 && ./launch_all.sh stop > $D/kapatma.log 2>&1
mkdir -p /tmp/zincir_ulg
R=~/PX4-Autopilot/build/px4_sitl_default/rootfs
cp "$(ls -t $R/0/log/*/*.ulg | head -1)" /tmp/zincir_ulg/$1_0.ulg
cp "$(ls -t $R/1/log/*/*.ulg | head -1)" /tmp/zincir_ulg/$1_1.ulg
[ -s $D/sonuc.jsonl ] && echo TAMAM > $D/DURUM || echo "SONUC YOK" > $D/DURUM
