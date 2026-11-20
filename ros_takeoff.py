#!/usr/bin/env python3
"""
ros_takeoff.py — PX4 aracını ROS 2/DDS VehicleCommand ile kaldırır (NAV_TAKEOFF + ARM).

mavlink_takeoff.py zaten çalışıyorken (araca GCS kalp atışı veriyor, MAVLink portunu tutuyor)
yerdeki ikinci aracı o süreci yeniden başlatmadan kaldırmak için.
Kalkış irtifası PX4'ün MIS_TAKEOFF_ALT parametresinden gelir (param7 NaN); önce ayarlayın:
  ~/PX4-Autopilot/build/px4_sitl_default/bin/px4-param --instance 1 set MIS_TAKEOFF_ALT 120

Hazır koşulu: arm, AUTO modunda (Takeoff → Hold), |irtifa − --alt| ≤ --alt-tol, --settle sn.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 ros_takeoff.py --uav 1                 # hedef irtifa 100 + 20·uav = 120 m
Çıkış kodu: 0 = hazır, 1 = komut reddi / zaman aşımı.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleControlMode, VehicleLocalPosition

from v2_orbit_mission import PX4_QOS, SENSOR_QOS, control_mode_name, get_fmu_prefix, make_vehicle_command


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


class Takeoff(Node):
    def __init__(self, uav_id: int):
        super().__init__(f"ros_takeoff_uav{uav_id}")
        ns = get_fmu_prefix(uav_id)
        self.uav_id = uav_id
        self.pos = None
        self.mode = None
        self.acks = {}
        self.pub_cmd = self.create_publisher(VehicleCommand, f"{ns}/in/vehicle_command", PX4_QOS)
        self.create_subscription(VehicleLocalPosition, f"{ns}/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "pos", m), SENSOR_QOS)
        self.create_subscription(VehicleControlMode, f"{ns}/out/vehicle_control_mode",
                                 lambda m: setattr(self, "mode", m), SENSOR_QOS)
        self.create_subscription(VehicleCommandAck, f"{ns}/out/vehicle_command_ack",
                                 lambda m: self.acks.__setitem__(m.command, m.result), SENSOR_QOS)

    def command(self, command: int, *params: float, tries: int = 5) -> bool:
        """Komutu ACCEPTED ACK gelene kadar 1 sn'de bir, en çok `tries` kez gönderir."""
        for attempt in range(1, tries + 1):
            self.acks.pop(command, None)
            self.pub_cmd.publish(make_vehicle_command(
                self.uav_id, int(self.get_clock().now().nanoseconds / 1000), command, *params))
            t = time.monotonic()
            while command not in self.acks and time.monotonic() - t < 1.0:
                rclpy.spin_once(self, timeout_sec=0.05)
            result = self.acks.get(command)
            log(f"[uav{self.uav_id}] komut {command} deneme {attempt}: ACK={result}")
            if result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return True
        return False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uav", type=int, required=True)
    ap.add_argument("--alt", type=float, help="Beklenen kalkış irtifası [m] (varsayılan 100 + 20·uav)")
    ap.add_argument("--alt-tol", type=float, default=10.0)
    ap.add_argument("--settle", type=float, default=5.0)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()
    target_alt = args.alt if args.alt is not None else 100.0 + 20.0 * args.uav

    rclpy.init()
    node = Takeoff(args.uav)
    try:
        t0 = time.monotonic()
        while (node.pos is None or node.mode is None) and time.monotonic() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pos is None or node.mode is None:
            log(f"[uav{args.uav}] HATA: konum / mod alınamadı")
            return 1
        if node.mode.flag_armed:
            log(f"[uav{args.uav}] HATA: araç zaten arm — kalkış gönderilmedi")
            return 1
        log(f"[uav{args.uav}] yerde: irtifa={-node.pos.z:.1f} m, mod={control_mode_name(node.mode)}; "
            f"hedef {target_alt:.0f} m (MIS_TAKEOFF_ALT)")

        # mavlink_takeoff.py ile aynı sıra: önce Takeoff modu (param7 NaN → MIS_TAKEOFF_ALT), sonra arm
        nan = math.nan
        if not node.command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 0, 0, 0, nan, nan, nan, nan):
            log(f"[uav{args.uav}] HATA: NAV_TAKEOFF kabul edilmedi")
            return 1
        if not node.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0):
            log(f"[uav{args.uav}] HATA: ARM kabul edilmedi")
            return 1

        start = time.monotonic()
        ok_since, next_log = None, start
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            alt = -node.pos.z
            ready = (node.mode.flag_armed and node.mode.flag_control_auto_enabled
                     and abs(alt - target_alt) <= args.alt_tol)
            now = time.monotonic()
            if now >= next_log:
                log(f"[uav{args.uav}] irtifa={alt:6.1f} m  yer_hızı={math.hypot(node.pos.vx, node.pos.vy):5.1f} m/s  "
                    f"mod={control_mode_name(node.mode)}  {'arm' if node.mode.flag_armed else 'disarm'}")
                next_log += 5.0
            if not ready:
                ok_since = None
            elif ok_since is None:
                ok_since = now
            elif now - ok_since >= args.settle:
                log(f"[uav{args.uav}] HAZIR: irtifa={alt:.1f} m ({now - start:.0f} sn sonra)")
                return 0
        log(f"[uav{args.uav}] HATA: {args.timeout:.0f} sn içinde {target_alt:.0f} ± {args.alt_tol:.0f} m'ye ulaşmadı")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
