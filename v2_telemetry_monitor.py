#!/usr/bin/env python3
"""
v2_telemetry_monitor.py
=======================
Terminal tabanlı canlı telemetri izleme aracı.

uav0 ve uav1 için VehicleLocalPosition + VehicleStatus topic'lerini izler
ve 2 Hz'de terminali temizleyip şık bir tablo yazdırır.

Kullanım:
  python3 v2_telemetry_monitor.py

Bağımlılıklar:
  - ROS 2 Humble
  - px4_msgs  (https://github.com/PX4/px4_msgs)
"""

import os
import math
import time
import logging
import pathlib
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

# ──────────────────────────────────────────────────────────────────────────────
# QoS
# ──────────────────────────────────────────────────────────────────────────────
# PX4 Publisher QoS (komutlar — TRANSIENT_LOCAL)
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Sensor / Telemetri Subscriber QoS (PX4 sensör topic'leri — VOLATILE)
# PX4 DDS Agent sensör verilerini BEST_EFFORT + VOLATILE ile yayınlar.
# TRANSIENT_LOCAL ile abone olmak mesajın reddedilmesine neden olur.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# ──────────────────────────────────────────────────────────────────────────────
# PX4 SITL DDS Topic Prefix Yardımcısı
# ──────────────────────────────────────────────────────────────────────────────

def get_fmu_prefix(uav_id: int) -> str:
    """
    PX4 SITL (MicroXRCE-DDS Agent) çoklu İHA topic namespace kuralı:
      uav_id == 0  →  /fmu
      uav_id >= 1  →  /px4_{uav_id}/fmu
    """
    if uav_id == 0:
        return "/fmu"
    return f"/px4_{uav_id}/fmu"



class C:
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    # Renkler
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    BLUE    = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN    = "\033[96m"
    WHITE   = "\033[97m"
    GRAY    = "\033[90m"
    # Arka planlar
    BG_DARK = "\033[40m"
    BG_BLUE = "\033[44m"


# ──────────────────────────────────────────────────────────────────────────────
# PX4 nav_state → İnsan okunabilir metin + renk
# ──────────────────────────────────────────────────────────────────────────────
NAV_STATE_MAP: dict[int, tuple[str, str]] = {
    0:  ("MANUAL",          C.YELLOW),
    1:  ("ALT CTL",         C.YELLOW),
    2:  ("POS CTL",         C.CYAN),
    3:  ("AUTO MISSION",    C.GREEN),
    4:  ("AUTO LOITER",     C.GREEN),
    5:  ("AUTO RTL",        C.MAGENTA),
    6:  ("ACRO",            C.YELLOW),
    8:  ("STABILIZED",      C.YELLOW),
    10: ("OFFBOARD",        C.CYAN),
    11: ("STABILIZED",      C.YELLOW),
    13: ("AUTO TAKEOFF",    C.GREEN),
    14: ("AUTO LAND",       C.MAGENTA),
    17: ("AUTO PRECLAND",   C.MAGENTA),
}


def nav_state_str(state: int) -> tuple[str, str]:
    """nav_state → (metin, renk)"""
    label, color = NAV_STATE_MAP.get(state, (f"STATE({state})", C.GRAY))
    return label, color


# ──────────────────────────────────────────────────────────────────────────────
# Tek bir İHA'nın anlık durumu
# ──────────────────────────────────────────────────────────────────────────────
class UAVSnapshot:
    def __init__(self, uav_id: int):
        self.uav_id       = uav_id
        self.connected    = False   # En az bir mesaj geldi mi?
        self.last_seen_s  = 0.0    # Son mesajın monotonic zamanı
        # VehicleStatus
        self.armed        = False
        self.nav_state    = 0
        # VehicleLocalPosition
        self.x            = 0.0
        self.y            = 0.0
        self.z            = 0.0
        self.vx           = 0.0
        self.vy           = 0.0
        self.vz           = 0.0
        self.alt_m        = 0.0    # -Z → metre
        self.speed_ms     = 0.0    # Toplam hız büyüklüğü


# ──────────────────────────────────────────────────────────────────────────────
# Ana Node
# ──────────────────────────────────────────────────────────────────────────────
class TelemetryMonitor(Node):
    """İki İHA'nın telemetrisini terminalde akan satırlar olarak gösteren izleme düğümü."""

    DISPLAY_HZ = 1  # 1 Hz — akan satır modu

    def __init__(self):
        super().__init__("v2_telemetry_monitor")
        self.start_time = time.monotonic()

        # ── Log dosyası hazırla ──────────────────────────────────────────
        log_dir = pathlib.Path("test/test-1")
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / "telemetry.log"

        tlog = logging.getLogger("telemetry")
        tlog.setLevel(logging.DEBUG)
        if not tlog.handlers:
            fh = logging.FileHandler(log_path, mode="a", encoding="utf-8")
            fh.setFormatter(logging.Formatter(
                fmt="%(asctime)s  %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            ))
            tlog.addHandler(fh)
        self._tlog = tlog
        self.get_logger().info(f"[monitor] Telemetri log dosyası: {log_path}")

        # ── İHA veri nesneleri ──────────────────────────────────────────
        self.uavs: dict[int, UAVSnapshot] = {
            0: UAVSnapshot(0),
            1: UAVSnapshot(1),
        }

        # Her İHA için abone ol
        for uid in (0, 1):
            out = f"{get_fmu_prefix(uid)}/out"

            # NOT: PX4 DDS topic'leri _v1/_v2 suffix'i ile yayınlanır.
            #      SENSOR_QOS (BEST_EFFORT + VOLATILE) zorunludur.
            self.create_subscription(
                VehicleStatus,
                f"{out}/vehicle_status_v2",          # ← _v2 suffix
                lambda msg, u=uid: self._status_cb(msg, u),
                SENSOR_QOS,                           # ← VOLATILE
            )
            self.create_subscription(
                VehicleLocalPosition,
                f"{out}/vehicle_local_position_v1",  # ← _v1 suffix
                lambda msg, u=uid: self._local_pos_cb(msg, u),
                SENSOR_QOS,                           # ← VOLATILE
            )
            self.get_logger().info(
                f"[monitor] uav{uid} → {out}/vehicle_status_v2  |  vehicle_local_position_v1"
            )

        # Ekran güncelleme timer'ı
        self.create_timer(1.0 / self.DISPLAY_HZ, self._display_cb)

    # ──────────────────────────────────────────────────────────────────────────
    # Subscriber callback'leri
    # ──────────────────────────────────────────────────────────────────────────

    def _status_cb(self, msg: VehicleStatus, uid: int):
        snap = self.uavs[uid]
        snap.connected  = True
        snap.last_seen_s = time.monotonic()
        snap.armed      = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        snap.nav_state  = msg.nav_state

    def _local_pos_cb(self, msg: VehicleLocalPosition, uid: int):
        snap = self.uavs[uid]
        snap.connected  = True
        snap.last_seen_s = time.monotonic()
        snap.x          = msg.x
        snap.y          = msg.y
        snap.z          = msg.z
        snap.vx         = msg.vx
        snap.vy         = msg.vy
        snap.vz         = msg.vz
        snap.alt_m      = -msg.z                            # NED → irtifa
        snap.speed_ms   = math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)

    # ──────────────────────────────────────────────────────────────────────────
    # Ekran render
    # ──────────────────────────────────────────────────────────────────────────

    def _display_cb(self):
        """
        1 Hz'de her iki İHA için tek satırlık özet yazar.
        os.system('clear') KULLANILMAZ — satırlar aşağı doğru akar.
        Her satır hem terminale hem test/test-1/telemetry.log dosyasına yazılır.
        """
        ts = time.strftime("%H:%M:%S")
        now = time.monotonic()

        for uid, snap in self.uavs.items():
            # Bağlantı etiketi
            stale_s = now - snap.last_seen_s
            if not snap.connected or stale_s > 3.0:
                conn = "NO_LINK"
            elif stale_s > 1.5:
                conn = f"STALE({stale_s:.1f}s)"
            else:
                conn = "OK"

            arm   = "ARMED" if snap.armed else "DISARMED"
            mode_label, _ = nav_state_str(snap.nav_state)

            line = (
                f"[{ts}] UAV{uid} "
                f"| Conn: {conn:<12}"
                f"| {arm:<8}"
                f"| Mod: {mode_label:<14}"
                f"| Z: {snap.alt_m:+7.1f}m "
                f"| X: {snap.x:+8.1f}m "
                f"| Y: {snap.y:+8.1f}m "
                f"| V: {snap.speed_ms:5.1f}m/s"
            )
            print(line)
            self._tlog.info(line)



# ──────────────────────────────────────────────────────────────────────────────
# Giriş noktası
# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TelemetryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nTelemetri izleme durduruldu.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
