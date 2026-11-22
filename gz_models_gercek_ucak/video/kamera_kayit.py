#!/usr/bin/env python3
"""gz kamera sensörlerinden (30 Hz, sim zamanı) mp4 yazar. Kullanım: kamera_kayit.py <sure_sn> <chase.mp4> <burun.mp4>"""
import sys, time, threading
import cv2, numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
dur = float(sys.argv[1]); M = "gercek_ucak_kam_0"
konf = {f"/world/default/model/{M}/link/chase_link/sensor/chase_cam/image": sys.argv[2],
        f"/world/default/model/{M}/link/burun_link/sensor/burun_cam/image": sys.argv[3]}
yaz = {}; sayac = {k: 0 for k in konf}; lock = threading.Lock(); t0 = [None]
def cb_for(topic, out):
    def cb(msg):
        w, h = msg.width, msg.height
        img = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)[:, :, ::-1].copy()
        ts = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with lock:
            if t0[0] is None: t0[0] = ts
            if topic not in yaz: yaz[topic] = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
            cv2.putText(img, f"sim t={ts - t0[0]:5.1f}s", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            yaz[topic].write(img); sayac[topic] += 1
    return cb
node = Node()
for t, o in konf.items():
    print("abone:", t, node.subscribe(Image, t, cb_for(t, o)))
time.sleep(dur)
with lock:
    for v in yaz.values(): v.release()
print("kare sayıları:", list(sayac.values()))
