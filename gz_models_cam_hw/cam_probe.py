#!/usr/bin/env python3
"""ROS tarafı kamera ölçümü: hız, kare kaybı (stamp aralığı), uçtan uca gecikme, PNG kaydı.
Gecikme = ROS'a varış duvar saati − sim saatinin kare stamp'ına ulaştığı duvar saati
(gz /world/default/clock dinlenir). Kullanım: <süre_sn> <png_yolu>"""
import bisect
import os
import sys
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from gz.transport13 import Node as GzNode
from gz.msgs10.clock_pb2 import Clock

DURATION, PNG = float(sys.argv[1]), sys.argv[2]
lock = threading.Lock()
clock_sim, clock_wall = [], []


def on_clock(msg):
    t = msg.sim.sec + msg.sim.nsec * 1e-9
    with lock:
        clock_sim.append(t)
        clock_wall.append(time.time())


gz = GzNode()
gz.subscribe(Clock, "/world/default/clock", on_clock)

rclpy.init()
node = rclpy.create_node("cam_probe")
bridge = CvBridge()
frames, info = [], {}
saved = {"done": False}


def on_image(msg):
    wall = time.time()
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    frames.append((wall, stamp, msg.width, msg.height, msg.encoding, msg.header.frame_id))
    if not saved["done"] and len(frames) >= 20:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(PNG, img)
        saved["done"] = True


node.create_subscription(Image, "/uav0/camera/image", on_image, qos_profile_sensor_data)
node.create_subscription(CameraInfo, "/uav0/camera/camera_info",
                         lambda m: info.update(w=m.width, h=m.height, k=list(m.k), frame=m.header.frame_id),
                         qos_profile_sensor_data)
t_end = time.time() + DURATION
while time.time() < t_end:
    rclpy.spin_once(node, timeout_sec=0.1)

if len(frames) < 2:
    print("PROBE: kare yok ya da çok az:", len(frames), flush=True)
    os._exit(1)
walls = [f[0] for f in frames]
stamps = [f[1] for f in frames]
dt_stamp = [b - a for a, b in zip(stamps, stamps[1:])]
dt_stamp_sorted = sorted(dt_stamp)
median = dt_stamp_sorted[len(dt_stamp_sorted) // 2]
gaps = [d for d in dt_stamp if d > 1.5 * median]
missing = sum(round(d / median) - 1 for d in gaps)
lat = []
with lock:
    cs, cw = list(clock_sim), list(clock_wall)
for wall, stamp, *_ in frames:
    i = bisect.bisect_left(cs, stamp)
    if 0 < i < len(cs):
        lat.append(wall - cw[i])
lat.sort()
w, h, enc, frame_id = frames[0][2:]
rate = (len(frames) - 1) / (walls[-1] - walls[0])
print(f"PROBE kare: {len(frames)} / {walls[-1] - walls[0]:.1f} sn → {rate:.2f} Hz (duvar), "
      f"stamp aralığı medyan {median * 1000:.1f} ms (≈ {1 / median:.2f} Hz sim)", flush=True)
print(f"PROBE kayıp: {len(gaps)} boşluk, ~{missing} kare eksik (stamp aralığı > 1,5 × medyan); "
      f"en büyük aralık {max(dt_stamp) * 1000:.0f} ms", flush=True)
if lat:
    print(f"PROBE gecikme (uçtan uca, sim stamp → ROS varış): medyan {lat[len(lat) // 2] * 1000:.0f} ms, "
          f"p95 {lat[int(len(lat) * 0.95)] * 1000:.0f} ms, maks {lat[-1] * 1000:.0f} ms (n={len(lat)}, "
          f"saat örneği {len(cs)})", flush=True)
else:
    print("PROBE gecikme: hesaplanamadı (saat verisi yok)", flush=True)
print(f"PROBE görüntü: {w}×{h} {enc}, frame_id '{frame_id}'; camera_info: {info or 'gelmedi'}", flush=True)
print(f"PROBE PNG: {PNG if saved['done'] else 'kaydedilmedi'}", flush=True)
sys.stdout.flush()
os._exit(0)
