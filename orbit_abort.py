#!/usr/bin/env python3
"""
orbit_abort.py — Güvenlik düğmesi: araç(lar)ı Hold (AUTO_LOITER) moduna alır.

Kullanım (ROS ortamını kendisi yükleyen sarmalayıcı ile):
  ~/iha_sim_v2/orbit_abort          # uav0 ve uav1
  ~/iha_sim_v2/orbit_abort 0        # yalnızca uav0

DO_SET_MODE(AUTO, LOITER), PX4 komutu kabul edip (VehicleCommandAck ACCEPTED) Offboard'dan
çıkana kadar (vehicle_control_mode) 0,5 sn'de bir yeniden gönderilir, en fazla 5 sn.
Doğrulama: vehicle_control_mode RTL'yi de AUTO gösterir (2026-10-30 geçiş 2'de RTL "HOLD ✔" sayıldı).
Bu yüzden ACK + AUTO'dan sonra nav_state px4_durum.py ile okunur (vehicle_status_v2, ham + ayrı süreç);
yalnızca nav_state = AUTO_LOITER (4) ise "HOLD ✔". Başka bir mod (ör. AUTO_RTL) okunursa Hold yeniden
gönderilir; süre sonunda hâlâ Hold değilse ya da nav_state okunamazsa "doğrulanamadı".
Orbit düğümü Offboard'dan çıkıldığını görünce setpoint yayınını keser ve yeniden girmez.
Çıkış kodu: 0 = hepsi Hold'da (nav_state ile doğrulandı), 1 = en az biri doğrulanamadı.
"""

import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleControlMode

from v2_orbit_mission import PX4_QOS, SENSOR_QOS, control_mode_name, get_fmu_prefix

TIMEOUT_S = 5.0
VERIFY_TIMEOUT_S = 10.0      # nav_state doğrulaması dahil toplam süre
RESEND_S = 0.5
AUTO_LOITER = 4
PX4_DURUM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "px4_durum.py")


def read_nav_state(uav_id: int):
    """(nav_state, satır) — px4_durum.py --tek; okunamazsa (None, açıklama)."""
    r = subprocess.run([sys.executable, PX4_DURUM, "--uav", str(uav_id), "--tek"], capture_output=True, text=True)
    out = [l for l in r.stdout.splitlines() if l.startswith("NAV ")]
    if r.returncode != 0 or not out or out[-1] == "NAV yok":
        return None, "nav_state okunamadı"
    return int(out[-1].split()[1]), out[-1]


class OrbitAbort(Node):
    def __init__(self, uav_ids):
        super().__init__("orbit_abort")
        self.pubs = {}
        self.mode = {}
        self.ack = {}
        for uav_id in uav_ids:
            ns = get_fmu_prefix(uav_id)
            self.pubs[uav_id] = self.create_publisher(VehicleCommand, f"{ns}/in/vehicle_command", PX4_QOS)
            self.create_subscription(
                VehicleControlMode, f"{ns}/out/vehicle_control_mode",
                lambda msg, u=uav_id: self.mode.__setitem__(u, msg), SENSOR_QOS)
            self.create_subscription(
                VehicleCommandAck, f"{ns}/out/vehicle_command_ack_v1",
                lambda msg, u=uav_id: self._on_ack(u, msg), SENSOR_QOS)

    def _on_ack(self, uav_id: int, msg: VehicleCommandAck):
        if msg.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.ack[uav_id] = msg.result

    def in_hold(self, uav_id: int) -> bool:
        mode = self.mode.get(uav_id)
        return (self.ack.get(uav_id) == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
                and mode is not None and mode.flag_control_auto_enabled
                and not mode.flag_control_offboard_enabled)

    def send_hold(self, uav_id: int):
        msg = VehicleCommand()
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        msg.command          = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1           = 1.0   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2           = 4.0   # PX4_CUSTOM_MAIN_MODE_AUTO
        msg.param3           = 3.0   # PX4_CUSTOM_SUB_MODE_AUTO_LOITER (Hold)
        msg.target_system    = uav_id + 1
        msg.target_component = 1
        msg.source_system    = 255
        msg.source_component = 1
        msg.from_external    = True
        self.pubs[uav_id].publish(msg)


def main() -> int:
    uav_ids = [int(a) for a in sys.argv[1:]] or [0, 1]
    rclpy.init()
    node = OrbitAbort(uav_ids)
    pending = set(uav_ids)
    start = time.monotonic()
    last_send = 0.0
    last_nav = {}
    while pending and time.monotonic() - start < VERIFY_TIMEOUT_S:
        rclpy.spin_once(node, timeout_sec=0.05)
        for uav_id in sorted(pending):
            if node.in_hold(uav_id):
                nav, text = read_nav_state(uav_id)      # ~1,5–2 sn; bu sırada yeniden gönderim yok
                last_nav[uav_id] = text
                if nav == AUTO_LOITER:
                    print(f"[uav{uav_id}] HOLD ✔ (DO_SET_MODE kabul, mod={control_mode_name(node.mode[uav_id])}, "
                          f"{text})", flush=True)
                    pending.discard(uav_id)
                else:
                    print(f"[uav{uav_id}] ACK + AUTO ama Hold değil: {text} — Hold yeniden gönderiliyor", flush=True)
                    node.ack.pop(uav_id, None)
                    node.send_hold(uav_id)
                    last_send = time.monotonic()
        if pending and time.monotonic() - last_send >= RESEND_S and time.monotonic() - start < TIMEOUT_S + 3.0:
            for uav_id in pending:
                node.send_hold(uav_id)
            last_send = time.monotonic()
    for uav_id in sorted(pending):
        print(f"[uav{uav_id}] HOLD ✘ — doğrulanamadı ({VERIFY_TIMEOUT_S:.0f} sn; ACK={node.ack.get(uav_id)}, "
              f"mod={control_mode_name(node.mode.get(uav_id))}, {last_nav.get(uav_id, 'nav_state okunmadı')})",
              flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if pending else 0


if __name__ == "__main__":
    raise SystemExit(main())
