#!/usr/bin/env python3
"""
px4_durum.py — PX4 vehicle_status_v2'den nav_state'i okur (yalnızca okuma, hiçbir şey yayınlamaz).

Neden ayrı süreç: ~/ros2_ws'teki px4_msgs VehicleStatus (v3, en büyük 87 bayt) PX4'ün gönderdiği v2 (88 bayt)
ile uyuşmuyor; Fast-DDS okuyucu geçmişi örneği reddediyor (vehicle_status_v2 ROS'a ulaşmaz). Çözüm: ham
(serileştirilmiş) abonelik + dinamik okuyucu belleği (fastdds_dinamik.xml, RMW_FASTRTPS_USE_QOS_FROM_XML=1).
Bu bayrak süreçteki TÜM ROS QoS ayarlarını devre dışı bıraktığı için yalnızca bu okuma sürecinde kullanılır;
komut/setpoint yayınlayan süreçler (orbit_abort, qr_pass) bunu alt süreç olarak çalıştırır.

CDR (XCDR1, küçük endian): 4 bayt kapsülleme başlığı + PX4 msg/versioned/VehicleStatus.msg (MESSAGE_VERSION 2)
sırası: timestamp u64 @0, armed_time u64 @8, takeoff_time u64 @16, arming_state u8 @24, ... nav_state_timestamp
u64 @32, nav_state_user_intention u8 @40, nav_state u8 @41 → baytlar [44], [45] (2026-10-31 doğrulandı: yerde
nav_state 4, arming_state 1).

Kullanım:
  python3 px4_durum.py --uav 0 --tek     # bir okuma (son 1 sn'deki en son örnek), çıkış kodu 0 = okundu, 1 = yok
  python3 px4_durum.py --uav 0 --izle    # her örnek için bir satır
Çıktı satırı: NAV <nav_state> <ad> niyet=<user_intention> arm=<arming_state> duvar=<s>
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
# PX4 v1.17.0 + eşleşen px4_msgs (msg/ ve msg/versioned/ birebir kopyalandı, 2026-11-20): VehicleStatus v1 olağan
# abonelikle okunur; ham CDR + dinamik bellek (eski çözüm) yalnız PX4_DURUM_HAM=1 ile.
HAM = os.environ.get("PX4_DURUM_HAM") == "1"
if HAM:
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = os.path.join(HERE, "fastdds_dinamik.xml")
    os.environ["RMW_FASTRTPS_USE_QOS_FROM_XML"] = "1"

import rclpy                                    # noqa: E402  (ortam değişkenlerinden sonra)
from px4_msgs.msg import VehicleStatus          # noqa: E402

from px4_durum_sabit import AUTO_LOITER, AUTO_RTL, NAV_NAMES, OFFBOARD   # noqa: E402,F401


def parse(raw: bytes):
    """(nav_state, user_intention, arming_state) ya da biçim beklenmedikse None."""
    if len(raw) < 48 or raw[:2] != b"\x00\x01":      # CDR_LE kapsülleme
        return None
    return raw[45], raw[44], raw[28]


def line(nav, intention, arming):
    return (f"NAV {nav} {NAV_NAMES.get(nav, '?')} niyet={intention} ({NAV_NAMES.get(intention, '?')}) "
            f"arm={arming} duvar={time.time():.3f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uav", type=int, default=0)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--tek", action="store_true")
    mode.add_argument("--izle", action="store_true")
    ap.add_argument("--timeout", type=float, default=4.0, help="--tek: ilk örneği bekleme süresi [sn]")
    args = ap.parse_args()
    topic = ("/fmu" if args.uav == 0 else f"/px4_{args.uav}/fmu") + "/out/" + os.environ.get(
        "PX4_STATUS_TOPIC", "vehicle_status_v2" if HAM else "vehicle_status_v1")
    rclpy.init()
    node = rclpy.create_node(f"px4_durum_uav{args.uav}")
    latest = {}

    def on_raw(raw):
        p = parse(raw) if HAM else (raw.nav_state, raw.nav_state_user_intention, raw.arming_state)
        if p is None:
            return
        latest["v"], latest["t"] = p, time.time()
        if args.izle:
            print(line(*p), flush=True)

    from rclpy.qos import qos_profile_sensor_data
    node.create_subscription(VehicleStatus, topic, on_raw, qos_profile_sensor_data, raw=HAM)
    try:
        if args.izle:
            rclpy.spin(node)
            return 0
        t0 = time.time()
        while "v" not in latest and time.time() - t0 < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "v" not in latest:
            print("NAV yok", flush=True)
            return 1
        t1 = time.time()                       # mod değişikliğinden önceki eski örneği almamak için 1 sn daha dinle
        while time.time() - t1 < 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        print(line(*latest["v"]), flush=True)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        sys.stdout.flush()
        os._exit(0 if "v" in latest else 1)


if __name__ == "__main__":
    raise SystemExit(main())
