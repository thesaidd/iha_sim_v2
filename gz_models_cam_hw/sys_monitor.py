#!/usr/bin/env python3
"""gz sunucu + köprü + DDS Agent RSS/CPU ve RTF. Kullanım: <süre_sn> <aralık_sn>
Durur: gz RSS ≥ 4096 MB ya da RTF < 0,8 (çıkış kodu 3)."""
import os
import subprocess
import sys
import time

from gz.transport13 import Node
from gz.msgs10.world_stats_pb2 import WorldStatistics

DURATION, INTERVAL = float(sys.argv[1]), float(sys.argv[2])
LIMIT_MB, RTF_MIN = 4096.0, 0.8
HZ, PAGE = os.sysconf("SC_CLK_TCK"), os.sysconf("SC_PAGE_SIZE")


def pid_of(args):
    out = subprocess.run(["pgrep", *args], capture_output=True, text=True).stdout.split()
    return int(out[0]) if out else None


PIDS = {"gz": pid_of(["-x", "ruby"]), "kopru": pid_of(["-f", "^/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge"]),
        "dds": pid_of(["-x", "MicroXRCEAgent"])}
print("PID:", PIDS, flush=True)


def proc(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12]), int(fields[21]) * PAGE / 2**20
    except (FileNotFoundError, TypeError):
        return None, None


stats = {"sim": None, "real": None}


def on_stats(msg):
    stats["sim"] = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
    stats["real"] = msg.real_time.sec + msg.real_time.nsec * 1e-9


node = Node()
node.subscribe(WorldStatistics, "/world/default/stats", on_stats)
time.sleep(1.0)
names = list(PIDS)
print(f"{'t_sn':>5} " + " ".join(f"{n + '_MB':>9} {n + '_CPU%':>9}" for n in names) + f" {'RTF':>6}", flush=True)
t0 = time.time()
prev_t, prev = t0, {n: proc(PIDS[n]) for n in names}
prev_sim = (stats["sim"], stats["real"])
rows, code = [], 0
while True:
    now = time.time()
    cur = {n: proc(PIDS[n]) for n in names}
    cells, rtf = [], float("nan")
    for n in names:
        ticks, rss = cur[n]
        cpu = float("nan")
        if rows and ticks is not None and prev[n][0] is not None:
            cpu = 100.0 * (ticks - prev[n][0]) / HZ / (now - prev_t)
        cells.append((rss, cpu))
    if rows and stats["real"] and prev_sim[1] and stats["real"] > prev_sim[1]:
        rtf = (stats["sim"] - prev_sim[0]) / (stats["real"] - prev_sim[1])
    rows.append((now - t0, cells, rtf))
    fmt = lambda v, w: f"{v:{w}.0f}" if v is not None else f"{'YOK':>{w}}"
    print(f"{now - t0:5.0f} " + " ".join(f"{fmt(r, 9)} {fmt(c, 9)}" for r, c in cells) + f" {rtf:6.3f}", flush=True)
    prev_t, prev, prev_sim = now, cur, (stats["sim"], stats["real"])
    gz_rss = cells[0][0] or 0.0
    if gz_rss >= LIMIT_MB:
        print(f"DUR: gz RSS {gz_rss:.0f} MB ≥ {LIMIT_MB:.0f}", flush=True)
        code = 3
        break
    if rtf == rtf and rtf < RTF_MIN:
        print(f"DUR: RTF {rtf:.3f} < {RTF_MIN}", flush=True)
        code = 3
        break
    if now - t0 >= DURATION - 0.5:
        break
    time.sleep(max(0.0, t0 + len(rows) * INTERVAL - time.time()))

dur_min = (rows[-1][0] - rows[0][0]) / 60.0
for i, n in enumerate(names):
    r0, r1 = rows[0][1][i][0], rows[-1][1][i][0]
    cpus = [r[1][i][1] for r in rows[1:] if r[1][i][1] == r[1][i][1]]
    if r0 is None or r1 is None:
        print(f"ÖZET {n}: süreç yok", flush=True)
        continue
    print(f"ÖZET {n}: RSS {r0:.0f} → {r1:.0f} MB ({(r1 - r0) / dur_min if dur_min else 0:+.0f} MB/dk), "
          f"CPU ort %{sum(cpus) / len(cpus) if cpus else float('nan'):.0f}", flush=True)
rtfs = [r[2] for r in rows[1:] if r[2] == r[2]]
print(f"ÖZET RTF ort {sum(rtfs) / len(rtfs) if rtfs else float('nan'):.3f} min {min(rtfs) if rtfs else float('nan'):.3f}", flush=True)
sys.stdout.flush()
os._exit(code)
