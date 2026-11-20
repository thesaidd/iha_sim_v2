#!/usr/bin/env python3
"""Uzun süreli gz sunucu izleyicisi: RSS, CPU, RTF (aralık ortalaması). Bitiş dosyası oluşana kadar çalışır.
Sınır aşılırsa (RSS ≥ 4096 MB ya da RTF < 0,7) <dur_dosyası>'nı yazar; tarama betiği onu görünce durur.
Kullanım: uzun_monitor.py <aralık_sn> <bitiş_dosyası> <dur_dosyası>"""
import os
import subprocess
import sys
import time

from gz.transport13 import Node
from gz.msgs10.world_stats_pb2 import WorldStatistics

INTERVAL, END_FILE, STOP_FILE = float(sys.argv[1]), sys.argv[2], sys.argv[3]
LIMIT_MB, RTF_MIN = 4096.0, 0.7
HZ, PAGE = os.sysconf("SC_CLK_TCK"), os.sysconf("SC_PAGE_SIZE")
pid = int(subprocess.run(["pgrep", "-x", "ruby"], capture_output=True, text=True).stdout.split()[0])


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
print(f"gz sunucu PID {pid}; aralık {INTERVAL:.0f} sn; sınır RSS {LIMIT_MB:.0f} MB, RTF {RTF_MIN}", flush=True)
print(f"{'saat':>8} {'t_sn':>6} {'RSS_MB':>8} {'CPU%':>6} {'RTF':>6}", flush=True)
t0 = time.time()
prev = (t0, *proc(), stats["sim"], stats["real"])
first = True
while not os.path.exists(END_FILE):
    now = time.time()
    ticks, rss = proc()
    cpu = rtf = float("nan")
    if not first:
        cpu = 100.0 * (ticks - prev[1]) / HZ / (now - prev[0])
        if stats["real"] and prev[4] and stats["real"] > prev[4]:
            rtf = (stats["sim"] - prev[3]) / (stats["real"] - prev[4])
    print(f"{time.strftime('%H:%M:%S')} {now - t0:6.0f} {rss:8.0f} {cpu:6.0f} {rtf:6.3f}", flush=True)
    prev, first = (now, ticks, rss, stats["sim"], stats["real"]), False
    reason = (f"RSS {rss:.0f} MB ≥ {LIMIT_MB:.0f}" if rss >= LIMIT_MB else
              f"RTF {rtf:.3f} < {RTF_MIN}" if rtf == rtf and rtf < RTF_MIN else None)
    if reason:
        print(f"DUR: {reason}", flush=True)
        with open(STOP_FILE, "w") as f:
            f.write(reason + "\n")
        break
    while time.time() < now + INTERVAL and not os.path.exists(END_FILE):
        time.sleep(0.5)
print("izleyici bitti", flush=True)
sys.stdout.flush()
os._exit(0)
