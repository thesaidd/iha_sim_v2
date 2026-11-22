#!/usr/bin/env python3
"""
viz_trails.py — Gazebo penceresinde uçak işaretçileri (yalnızca görsel; hiçbir araca komut göndermez).

Her uçak için 3 işaretçi (2 uçak → 6, sınır 10):
  - konum küresi, 8 m çap (uav0 kırmızı, uav1 mavi)
  - son --trail sn'lik iz (LINE_STRIP, aynı renk)
  - uçaktan zemine dikey çizgi (LINE_LIST)
Kaynak: /world/<dünya>/dynamic_pose/info (gz dünya konumu, ENU, ~50 Hz).
Hedef: arayüzün MarkerManager'ı, /marker_array (bir güncellemede tüm işaretçiler tek istekte).
Kimlikler sabit (ns "iha_viz"); her güncelleme aynı kimlikle ADD_MODIFY → sahne büyümez.
Görünürlük GUI: yalnızca pencerede çizilir, sunucu/sensörler görmez. Çarpışma ve doku yok;
Marker mesajında gölge alanı yoktur.
Yanıtsız gönderim: konum aboneliği (Python geri çağrısı, ~50 Hz) açıkken gz.transport13'ün
engelleyici request()'i yanıtı teslim alamıyor (aboneliksiz süreçte aynı istek ~1 ms'de yanıtlanıyor;
2026-10-29). İstekler arayüze ulaşıp uygulanıyor, yalnızca yanıt kayboluyor; Python bağında
yanıtsız (oneway) istek yok. Bu yüzden güncellemeler kısa --timeout (20 ms) ile gönderilir ve
yanıtsız sayılması beklenir. Kapanışta önce abonelik kaldırılır; silme isteğinin yanıtı böylece
gelir ve doğrulanır. Gerçekleşen hız 30 sn'de bir loglanır.
SIGTERM/SIGINT ile kapanırken ns'teki tüm işaretçileri siler.

Kullanım:
  python3 viz_trails.py                    # 5 Hz, 60 sn iz, uav0 + uav1
  python3 viz_trails.py --rate 2 --trail 30
Pencere (gz sim -g) kapalıyken istekler başarısız olur; betik beklemeye devam eder.
"""

import argparse
import collections
import os
import signal
import sys
import threading
import time

from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from arac_modelleri import gz_ad

NS = "iha_viz"
COLORS = {0: (0.95, 0.1, 0.1), 1: (0.1, 0.35, 1.0)}   # uav0 kırmızı, uav1 mavi
SPHERE_DIAMETER = 8.0
SPHERE_ALPHA = 0.35     # yarı saydam: takip kamerasında içindeki uçak görünsün


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


def set_color(material, rgb, alpha=1.0):
    for field in (material.ambient, material.diffuse, material.emissive):
        field.r, field.g, field.b, field.a = (*rgb, alpha)


def base_marker(marker_id: int, marker_type: int, rgb, alpha: float = 1.0) -> Marker:
    m = Marker()
    m.ns = NS
    m.id = marker_id
    m.action = Marker.ADD_MODIFY
    m.type = marker_type
    m.visibility = Marker.GUI
    set_color(m.material, rgb, alpha)
    return m


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uav", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--rate", type=float, default=5.0, help="Güncelleme hızı [Hz]")
    ap.add_argument("--trail", type=float, default=60.0, help="İz uzunluğu [sn]")
    ap.add_argument("--world", default="default")
    ap.add_argument("--timeout", type=int, default=20,
                    help="Güncelleme isteği bekleme süresi [ms]; yanıt abonelik açıkken gelmez (bkz. üst not)")
    args = ap.parse_args()
    if len(args.uav) * 3 > 10:
        ap.error("işaretçi sınırı 10 (en çok 3 uçak)")

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    node = Node()
    names = {gz_ad(u): u for u in args.uav}
    latest = {}
    lock = threading.Lock()

    def on_poses(msg: Pose_V):
        with lock:
            for p in msg.pose:
                if p.name in names:
                    latest[names[p.name]] = (p.position.x, p.position.y, p.position.z)

    topic = f"/world/{args.world}/dynamic_pose/info"
    if not node.subscribe(Pose_V, topic, on_poses):
        log(f"HATA: {topic} aboneliği kurulamadı")
        return 1
    trails = {u: collections.deque(maxlen=max(2, int(args.trail * args.rate))) for u in args.uav}
    log(f"başladı: {topic} → /marker_array, {args.rate:g} Hz, iz {args.trail:g} sn "
        f"({trails[args.uav[0]].maxlen} nokta), {len(args.uav) * 3} işaretçi (ns '{NS}')")

    period = 1.0 / args.rate
    next_t = time.monotonic()
    sent = failed = 0
    last_report, sent_at_report = time.monotonic(), 0
    while not stop.is_set():
        # Yanıt gecikirse geriye dönük telafi yok: bir sonraki güncelleme en erken şimdi + periyot
        next_t = max(next_t + period, time.monotonic())
        with lock:
            snapshot = dict(latest)
        batch = Marker_V()
        for u, (x, y, z) in snapshot.items():
            rgb = COLORS.get(u, (0.2, 0.8, 0.2))
            trails[u].append((x, y, z))

            sphere = base_marker(10 * u + 1, Marker.SPHERE, rgb, SPHERE_ALPHA)
            sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = x, y, z
            sphere.scale.x = sphere.scale.y = sphere.scale.z = SPHERE_DIAMETER

            trail = base_marker(10 * u + 2, Marker.LINE_STRIP, rgb)
            for px, py, pz in trails[u]:
                pt = trail.point.add()
                pt.x, pt.y, pt.z = px, py, pz

            drop = base_marker(10 * u + 3, Marker.LINE_LIST, rgb)
            for pz in (z, 0.05):
                pt = drop.point.add()
                pt.x, pt.y, pt.z = x, y, pz

            batch.marker.extend([sphere, trail, drop])
        if batch.marker:
            ok, _ = node.request("/marker_array", batch, Marker_V, Boolean, args.timeout)
            sent += 1
            failed += 0 if ok else 1
        now = time.monotonic()
        if now - last_report >= 30.0:
            log(f"durum: {sent} güncelleme ({(sent - sent_at_report) / (now - last_report):.1f} Hz), "
                f"{failed} yanıtsız (beklenen); uçaklar: "
                + ", ".join(f"uav{u}=({p[0]:.0f}, {p[1]:.0f}, {p[2]:.0f})" for u, p in sorted(snapshot.items())))
            last_report, sent_at_report = now, sent
        stop.wait(max(0.0, next_t - time.monotonic()))

    node.unsubscribe(topic)            # abonelik kapalıyken silme isteğinin yanıtı teslim alınır
    clear = Marker_V()
    marker = clear.marker.add()
    marker.ns = NS
    marker.action = Marker.DELETE_ALL
    # /marker (yanıt tipi Empty) yanıt döndürmüyor; /marker_array Boolean ile yanıtlıyor
    ok, reply = node.request("/marker_array", clear, Marker_V, Boolean, 3000)
    ok = ok and reply.data
    log(f"durduruldu: {sent} güncelleme, {failed} yanıtsız; işaretçiler silindi={ok}")
    return 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    # gz-transport Node'unun yıkıcısı yorumlayıcı kapanırken "terminate called" ile çöküyor;
    # işaretçiler silindikten sonra yıkıcıları atlayarak çık.
    os._exit(code)
