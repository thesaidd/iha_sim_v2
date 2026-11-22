#!/bin/bash
B=~/iha_sim_v2/gz_models_tulpar/test/adim5_4_2026-11-22
for n in 1 2 3; do bash $B/calistir.sh tekrar$n; echo "tekrar$n: $(cat $B/tekrar$n/DURUM)" >> $B/tekrar_ozet.txt; done
echo BITTI >> $B/tekrar_ozet.txt
