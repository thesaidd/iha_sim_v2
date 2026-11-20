"""Pasif iz kaydı (yalnız abone): uav0 ve uav1 konumu, uav0 direk mesafesi, 2 Hz CSV."""
import math, sys, time
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
sys.path.insert(0, "/home/yusuf22_04/iha_sim_v2")
from v2_orbit_mission import SENSOR_QOS
PY = [(140, 0), (-140, 0), (0, -140), (70, 121.2)]
rclpy.init(); n = Node("iz_kayit"); pos = {}
n.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", lambda m: pos.__setitem__(0, m), SENSOR_QOS)
n.create_subscription(VehicleLocalPosition, "/px4_1/fmu/out/vehicle_local_position_v1", lambda m: pos.__setitem__(1, m), SENSOR_QOS)
f = open(sys.argv[1], "w"); f.write("saat,n0,e0,alt0,direk0,n1_uav0cerceve,e1_uav0cerceve,alt1,ayrim_yatay,ayrim_dikey\n")
nxt = time.monotonic()
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.monotonic() >= nxt and 0 in pos:
        nxt += 0.5; a = pos[0]; d = min(math.hypot(a.x - p[0], a.y - p[1]) for p in PY)
        if 1 in pos:
            b = pos[1]; n1, e1 = b.x + 15, b.y - 15
            ek = f"{n1:.1f},{e1:.1f},{-b.z:.1f},{math.hypot(a.x - n1, a.y - e1):.1f},{abs(a.z - b.z):.1f}"
        else: ek = ",,,,"
        f.write(f"{time.strftime('%H:%M:%S')},{a.x:.1f},{a.y:.1f},{-a.z:.1f},{d:.1f},{ek}\n"); f.flush()
