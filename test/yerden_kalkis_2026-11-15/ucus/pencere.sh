#!/bin/bash
# Çalışan gz sunucusuna kayıt yapılandırmasıyla pencere bağlar (sunucunun ortamıyla), takibi kapatır.
set -u
SRV=$(pgrep -f "^gz sim .*-s" | head -1)
[ -z "$SRV" ] && { echo "HATA: gz sunucusu yok"; exit 1; }
set -a; eval "$(tr '\0' '\n' < /proc/$SRV/environ | grep -E '^(GZ_|LD_|PATH|HOME|DISPLAY|XDG_|WAYLAND|QT_)' | sed 's/^/export /;s/=/="/;s/$/"/')"; set +a
setsid gz sim -g --gui-config ~/iha_sim_v2/gui_kayit.config > /tmp/gz_gui_kayit.log 2>&1 &
echo $! > /tmp/gz_gui_kayit.pid
sleep 12
gz topic -t /gui/track -m gz.msgs.CameraTrack -p 'track_mode: NONE' > /dev/null 2>&1
echo "pencere acildi (pid $(cat /tmp/gz_gui_kayit.pid))"
