#!/usr/bin/env python3
"""
gz_gercek_poz.py — Gazebo'daki araçların GERÇEK pozunu (sim saati damgalı) CSV'ye yazar; değerlendirme için.

Kaynak: gz-transport /world/<dünya>/dynamic_pose/info (Pose_V; model pozu, dünya ENU / gövde FLU).
Çıktı sütunları: t_sim (s), ad, x_e, y_n, z_u (ENU, m), qw, qx, qy, qz (FLU gövde → ENU). --hz ile seyreltilir.
Ayrıca /world/<dünya>/clock'tan sim ↔ duvar saati çiftleri (saat.csv) yazılır (RTF ve gecikme analizi).

Kullanım: python3 gz_gercek_poz.py --out /tmp/aruco_log/gercek_poz.csv [--hz 50] [--modeller advanced_plane_0 advanced_plane_1]
"""
import argparse
import csv
import os
import signal
import threading
import time

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from arac_modelleri import gz_ad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--dunya", default="default")
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--modeller", nargs="+", default=[gz_ad(0), gz_ad(1)])
    a = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    lock = threading.Lock()
    fh = open(a.out, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t_sim", "ad", "x_e", "y_n", "z_u", "qw", "qx", "qy", "qz"])
    fh_c = open(os.path.join(os.path.dirname(os.path.abspath(a.out)), "saat.csv"), "w", newline="")
    wc = csv.writer(fh_c)
    wc.writerow(["t_sim", "t_duvar"])
    son = {m: -1e9 for m in a.modeller}
    dt = 1.0 / a.hz
    say = {"poz": 0, "saat": 0}
    son_saat = [-1e9]

    def on_pose(m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        for p in m.pose:
            if p.name in son and t - son[p.name] >= dt - 1e-6:
                son[p.name] = t
                o, q = p.position, p.orientation
                with lock:
                    w.writerow([f"{t:.6f}", p.name, f"{o.x:.4f}", f"{o.y:.4f}", f"{o.z:.4f}",
                                f"{q.w:.7f}", f"{q.x:.7f}", f"{q.y:.7f}", f"{q.z:.7f}"])
                    say["poz"] += 1

    def on_clock(m):
        t = m.sim.sec + m.sim.nsec * 1e-9
        if t - son_saat[0] >= 0.1:
            son_saat[0] = t
            with lock:
                wc.writerow([f"{t:.6f}", f"{time.time():.6f}"])
                say["saat"] += 1

    node = GzNode()
    node.subscribe(Pose_V, f"/world/{a.dunya}/dynamic_pose/info", on_pose)
    node.subscribe(Clock, f"/world/{a.dunya}/clock", on_clock)
    dur = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: dur.set())
    signal.signal(signal.SIGINT, lambda *_: dur.set())
    print(f"gz_gercek_poz: {a.modeller} → {a.out} ({a.hz:g} Hz)", flush=True)
    while not dur.wait(5.0):
        with lock:
            fh.flush(); fh_c.flush()
        print(f"poz {say['poz']} saat {say['saat']}", flush=True)
    with lock:
        fh.close(); fh_c.close()


if __name__ == "__main__":
    main()
