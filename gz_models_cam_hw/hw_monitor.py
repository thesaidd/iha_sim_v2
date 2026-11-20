#!/usr/bin/env python3
"""Gazebo sunucusu RSS/CPU/RTF izleyicisi + kamera topic hızı. Kullanım: <süre_sn> <aralık_sn> <rss_sınır_MB>"""
import os
import subprocess
import sys
import time

from gz.transport13 import Node
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.image_pb2 import Image

DURATION, INTERVAL, LIMIT_MB = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
GROW_MB, RTF_MIN = 100.0, 0.8
HZ, PAGE = os.sysconf("SC_CLK_TCK"), os.sysconf("SC_PAGE_SIZE")
pid = subprocess.run(["pgrep", "-f", "^gz sim .*-s"], capture_output=True, text=True).stdout.split()[0]


def proc():
    with open(f"/proc/{pid}/stat") as f:
        fields = f.read().rsplit(")", 1)[1].split()
    return int(fields[11]) + int(fields[12]), int(fields[21]) * PAGE / 2**20


stats = {"sim": None, "real": None}


def on_stats(msg):
    stats["sim"] = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
    stats["real"] = msg.real_time.sec + msg.real_time.nsec * 1e-9


node = Node()
node.subscribe(WorldStatistics, "/world/default/stats", on_stats)
time.sleep(1.0)
maps = open(f"/proc/{pid}/maps").read()
engines = sorted({e for e in ("libgz-rendering8-ogre.so", "libgz-rendering8-ogre2.so") if e in maps})
print(f"sunucu PID {pid}; yüklü görüntü motoru: {engines or 'yok'}", flush=True)
print(f"{'t_sn':>5} {'RSS_MB':>8} {'CPU%':>6} {'RTF':>6}", flush=True)
rows = []
t0 = time.time()
prev = (time.time(), *proc(), stats["sim"], stats["real"])
stopped = False
while True:
    now = time.time()
    ticks, rss = proc()
    cpu, rtf = float("nan"), float("nan")
    if rows:
        cpu = 100.0 * (ticks - prev[1]) / HZ / (now - prev[0])
        if stats["real"] and prev[4] and stats["real"] > prev[4]:
            rtf = (stats["sim"] - prev[3]) / (stats["real"] - prev[4])
    rows.append((now - t0, rss, cpu, rtf))
    print(f"{now - t0:5.0f} {rss:8.0f} {cpu:6.0f} {rtf:6.3f}", flush=True)
    prev = (now, ticks, rss, stats["sim"], stats["real"])
    if rss >= LIMIT_MB:
        print(f"GÜVENLİK: RSS {rss:.0f} MB ≥ {LIMIT_MB:.0f} MB — deneme durduruldu", flush=True)
        stopped = True
        break
    if rss - rows[0][1] >= GROW_MB:
        print(f"DUR: RSS ilk ölçümden {rss - rows[0][1]:.0f} MB arttı (sınır {GROW_MB:.0f}) — deneme durduruldu", flush=True)
        stopped = True
        break
    if rtf == rtf and rtf < RTF_MIN:
        print(f"DUR: RTF {rtf:.3f} < {RTF_MIN} — deneme durduruldu", flush=True)
        stopped = True
        break
    if now - t0 >= DURATION - 0.5:
        break
    time.sleep(max(0.0, t0 + len(rows) * INTERVAL - time.time()))

dur_min = (rows[-1][0] - rows[0][0]) / 60.0
rate = (rows[-1][1] - rows[0][1]) / dur_min if dur_min > 0 else float("nan")
cpus = [r[2] for r in rows[1:] if r[2] == r[2]]
rtfs = [r[3] for r in rows[1:] if r[3] == r[3]]
print(f"ÖZET başlangıç_RSS={rows[0][1]:.0f} bitiş_RSS={rows[-1][1]:.0f} artış={rate:.0f}_MB/dk "
      f"CPU={sum(cpus)/len(cpus) if cpus else float('nan'):.0f}% RTF={sum(rtfs)/len(rtfs) if rtfs else float('nan'):.3f} "
      f"süre={rows[-1][0]:.0f}_sn{' DURDURULDU' if stopped else ''}", flush=True)

# Kamera topic hızı (bellek ölçümünden SONRA; abonelik sensör davranışını değiştirebilir)
topics = [t for t in node.topic_list() if t.endswith("/sensor/camera/image")]
counts = {t: 0 for t in topics}
for t in topics:
    node.subscribe(Image, t, lambda m, t=t: counts.__setitem__(t, counts[t] + 1))
time.sleep(10.0)
for t in topics:
    print(f"KAMERA {t.split('/model/')[1].split('/')[0]}: {counts[t] / 10.0:.1f} Hz", flush=True)
if not topics:
    print("KAMERA: kamera görüntü topic'i yok", flush=True)
sys.stdout.flush()
os._exit(0)
