#!/bin/bash
# Kullanım: gecis.sh <no> <bias>  — qr_watch (arka plan) + qr_pass; özet basar
L=~/iha_sim_v2/test/qr_kadraj_2026-10-31; N=$1; B=$2; cd ~/iha_sim_v2
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
rm -f $L/watch_$N.kod
setsid bash -c "python3 -u qr_watch.py --alt 60 --done-file $L/gecis_bitti --abort-file $L/abort --mav-log $L/mavlink.log > $L/watch_$N.log 2>&1; echo \$? > $L/watch_$N.kod" < /dev/null &
sleep 4
timeout 900 python3 -u qr_pass.py --pass-id $N --bias $B --done-file $L/gecis_bitti --abort-file $L/abort --result $L/gecis_sonuclari.jsonl > $L/pass_$N.log 2>&1
echo "qr_pass çıkış: $?"
for _ in $(seq 1 40); do [ -f $L/watch_$N.kod ] && break; sleep 0.5; done
echo "qr_watch çıkış: $(cat $L/watch_$N.kod 2>/dev/null || echo 'henüz yok')"
grep -vE '^.{10}(hat|hizalama): ' $L/pass_$N.log | grep -v RTPS | cut -c1-230
echo "--- izleyici"; grep -vE ' t= ' $L/watch_$N.log | grep -v RTPS | tail -6 | cut -c1-230
echo "--- STATUSTEXT (mavlink)"; grep 'PX4 \[sev' $L/mavlink.log | tail -3 | cut -c1-120
echo "--- kaydedici / online"; grep -E 'KAYIT' $L/kaydedici.log | tail -2; grep -c YAYIN $L/online.log
