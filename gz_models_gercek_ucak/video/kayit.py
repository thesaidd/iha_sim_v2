#!/usr/bin/env python3
"""GUI ekran görüntülerini ardışık kaydeder (kare hızı düşük); mp4'e çevirir. Kullanım: kayit.py <sure_sn> <cikti.mp4>"""
import glob, os, subprocess, sys, time
import cv2
dur, out = float(sys.argv[1]), sys.argv[2]
pic = os.path.expanduser("~/.gz/gui/pictures/")
seen = set(glob.glob(pic + "*.png")); frames = []; t0 = time.time()
while time.time() - t0 < dur:
    subprocess.run(["gz","service","-s","/gui/screenshot","--reqtype","gz.msgs.StringMsg","--reptype","gz.msgs.Boolean","--timeout","2000","--req",'data: ""'], capture_output=True)
    time.sleep(0.15)
    new = sorted(set(glob.glob(pic + "*.png")) - seen, key=os.path.getmtime)
    for f in new:
        seen.add(f); img = cv2.imread(f)
        if img is not None: frames.append((time.time() - t0, img))
print("kare:", len(frames), "süre:", round(time.time() - t0, 1))
if frames:
    h, w = frames[0][1].shape[:2]
    fps = max(1.0, len(frames) / (frames[-1][0] - frames[0][0] + 1e-6))
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    for ts, img in frames:
        img = cv2.resize(img, (w, h)); cv2.putText(img, f"t={ts:4.1f}s (gercek zaman)", (10, h-12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)
        vw.write(img)
    vw.release(); print("yazıldı:", out, f"fps={fps:.1f}", w, h)
