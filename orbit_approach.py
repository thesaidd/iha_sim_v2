#!/usr/bin/env python3
"""
orbit_approach.py — Orbit öncesi giriş: aracı DO_REPOSITION ile orbit merkezine gönderir
(PX4 Hold'a geçer ve merkez etrafında NAV_LOITER_RAD yarıçapında, saat yönünde döner),
ardından orbit düğümünün teğet devralabileceği anı bekler.

Hazır koşulu (--settle sn boyunca kesintisiz):
  merkeze uzaklık --band içinde (60–110 m), |irtifa − hedef| ≤ --alt-tol (10 m),
  |radyal hız| ≤ --max-radial-speed (5 m/s → çembere teğet uçuş),
  dönüş yönü orbit yönüyle aynı (saat yönü, θ artıyor),
  --opposite-of verilirse: bu aracın merkeze göre açısı o aracınkinden 180° ± --opposite-tol.
Merkeze giden düz uçuşta uçak banda dışarıdan girerken radyal hız yüksek olduğundan
koşul sağlanmaz; ancak Hold dönüşüne yerleşince sağlanır.

Merkez (--center-n/-e) --frame-uav aracının yerel çerçevesindedir (varsayılan uav0); her PX4
kendi doğduğu noktayı orijin aldığı için global referanslardan bu aracın çerçevesine çevrilir.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 orbit_approach.py --uav 0                    # merkez (0,0), irtifa 100 m
  python3 orbit_approach.py --uav 1 --opposite-of 0    # irtifa 120 m, uav0'ın karşısında hazır
Çıkış kodu: 0 = hazır, 1 = zaman aşımı / komut reddi.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleLocalPosition

from v2_orbit_mission import (PX4_QOS, SENSOR_QOS, get_fmu_prefix, make_vehicle_command, ne_to_latlon,
                              ref_offset_ne)


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


class Approach(Node):
    def __init__(self, uav_id: int, other_ids):
        super().__init__(f"orbit_approach_uav{uav_id}")
        self.uav_id = uav_id
        self.pos = {}      # uav_id → son VehicleLocalPosition (kendisi + frame/opposite araçları)
        self.ack = None
        self.pub_cmd = self.create_publisher(VehicleCommand, f"{get_fmu_prefix(uav_id)}/in/vehicle_command", PX4_QOS)
        for u in {uav_id, *other_ids}:
            self.create_subscription(VehicleLocalPosition, f"{get_fmu_prefix(u)}/out/vehicle_local_position_v1",
                                     lambda m, u=u: self.pos.__setitem__(u, m), SENSOR_QOS)
        self.create_subscription(VehicleCommandAck, f"{get_fmu_prefix(uav_id)}/out/vehicle_command_ack",
                                 self._on_ack, SENSOR_QOS)

    def _on_ack(self, msg: VehicleCommandAck):
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_REPOSITION:
            self.ack = msg.result

    def send_reposition(self, lat: float, lon: float, alt_amsl: float):
        self.pub_cmd.publish(make_vehicle_command(
            self.uav_id, int(self.get_clock().now().nanoseconds / 1000), VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
            -1.0,       # varsayılan hız
            1.0,        # MAV_DO_REPOSITION_FLAGS_CHANGE_MODE → Hold
            0.0,
            math.nan,   # yön serbest
            lat, lon, alt_amsl))

    def offset(self, a: int, b: int):
        """a aracının yerel orijini, b aracının çerçevesinde (N, E)."""
        pa, pb = self.pos[a], self.pos[b]
        return ref_offset_ne(pa.ref_lat, pa.ref_lon, pb.ref_lat, pb.ref_lon)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uav", type=int, default=0)
    ap.add_argument("--alt", type=float, help="Hedef irtifa [m] (varsayılan 100 + 20·uav)")
    ap.add_argument("--center-n", type=float, default=0.0)
    ap.add_argument("--center-e", type=float, default=0.0)
    ap.add_argument("--frame-uav", type=int, default=0, help="Merkezin verildiği yerel çerçevenin aracı")
    ap.add_argument("--band", type=float, nargs=2, default=[60.0, 110.0], metavar=("MIN", "MAX"))
    ap.add_argument("--alt-tol", type=float, default=10.0)
    ap.add_argument("--max-radial-speed", type=float, default=5.0)
    ap.add_argument("--opposite-of", type=int, help="Hazır için bu aracın çember üzerinde karşısında ol")
    ap.add_argument("--opposite-tol", type=float, default=30.0, help="180° etrafında tolerans [°]")
    ap.add_argument("--settle", type=float, default=5.0, help="Koşulların kesintisiz sağlanma süresi [sn]")
    ap.add_argument("--timeout", type=float, default=240.0)
    args = ap.parse_args()
    target_alt = args.alt if args.alt is not None else 100.0 + 20.0 * args.uav
    others = {args.frame_uav} | ({args.opposite_of} if args.opposite_of is not None else set())

    rclpy.init()
    node = Approach(args.uav, others)
    try:
        # Tüm araçların yerel orijini (ref_lat/lon/alt) gelene kadar bekle
        needed = {args.uav} | others
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10 and not all(
                u in node.pos and node.pos[u].xy_global and node.pos[u].z_global for u in needed):
            rclpy.spin_once(node, timeout_sec=0.1)
        missing = [u for u in needed if u not in node.pos or not node.pos[u].xy_global]
        if missing:
            log(f"[uav{args.uav}] HATA: yerel konum / global referans alınamadı: {missing}")
            return 1

        frame = node.pos[args.frame_uav]
        lat, lon = ne_to_latlon(args.center_n, args.center_e, frame.ref_lat, frame.ref_lon)
        alt_amsl = node.pos[args.uav].ref_alt + target_alt
        off_n, off_e = node.offset(args.uav, args.frame_uav)
        center_n, center_e = args.center_n - off_n, args.center_e - off_e    # bu aracın çerçevesinde
        log(f"[uav{args.uav}] DO_REPOSITION → merkez ({args.center_n:.0f}, {args.center_e:.0f}) "
            f"[uav{args.frame_uav} çerçevesi; bu araçta ({center_n:+.1f}, {center_e:+.1f})] = "
            f"{lat:.7f}, {lon:.7f}, {alt_amsl:.1f} m AMSL (yerel {target_alt:.0f} m)")

        for attempt in range(1, 6):
            node.ack = None
            node.send_reposition(lat, lon, alt_amsl)
            t = time.monotonic()
            while node.ack is None and time.monotonic() - t < 1.0:
                rclpy.spin_once(node, timeout_sec=0.05)
            if node.ack == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                log(f"[uav{args.uav}] DO_REPOSITION kabul edildi ({attempt}. deneme)")
                break
        else:
            log(f"[uav{args.uav}] HATA: DO_REPOSITION kabul edilmedi (son ACK={node.ack})")
            return 1

        start = time.monotonic()
        ok_since = None
        next_log = start
        min_sep = math.inf
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            p = node.pos[args.uav]
            dn, de = p.x - center_n, p.y - center_e
            dist = math.hypot(dn, de)
            alt = -p.z
            radial = (p.vx * dn + p.vy * de) / max(dist, 1e-3)
            clockwise = (dn * p.vy - de * p.vx) > 0.0     # dθ/dt > 0, θ = atan2(E, N)
            ready = (args.band[0] <= dist <= args.band[1] and abs(alt - target_alt) <= args.alt_tol
                     and abs(radial) <= args.max_radial_speed and clockwise)
            extra = ""
            if args.opposite_of is not None:
                q = node.pos[args.opposite_of]
                qn_off, qe_off = node.offset(args.opposite_of, args.uav)       # diğer aracın orijini, bizim çerçevede
                qn, qe = q.x + qn_off - center_n, q.y + qe_off - center_e
                sep_angle = math.degrees(math.atan2(de, dn) - math.atan2(qe, qn)) % 360.0
                sep_3d = math.sqrt((dn - qn) ** 2 + (de - qe) ** 2 + ((p.ref_alt - p.z) - (q.ref_alt - q.z)) ** 2)
                min_sep = min(min_sep, sep_3d)
                ready = ready and abs(sep_angle - 180.0) <= args.opposite_tol
                extra = f"  uav{args.opposite_of}'a açı={sep_angle:5.1f}°  3B={sep_3d:6.1f} m"
            now = time.monotonic()
            if now >= next_log:
                log(f"[uav{args.uav}] merkeze={dist:6.1f} m  irtifa={alt:6.1f} m  radyal={radial:+5.1f} m/s  "
                    f"yön={'saat' if clockwise else 'ters'}{extra}  hazır={'evet' if ready else 'hayır'}")
                next_log += 5.0
            if not ready:
                ok_since = None
            elif ok_since is None:
                ok_since = now
            elif now - ok_since >= args.settle:
                log(f"[uav{args.uav}] HAZIR: merkeze={dist:.1f} m, irtifa={alt:.1f} m, radyal={radial:+.1f} m/s"
                    f"{extra} ({now - start:.0f} sn sonra)"
                    + (f"; bekleme sırasında en küçük 3B mesafe {min_sep:.1f} m" if math.isfinite(min_sep) else ""))
                return 0
        log(f"[uav{args.uav}] HATA: {args.timeout:.0f} sn içinde hazır koşulu sağlanmadı")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
