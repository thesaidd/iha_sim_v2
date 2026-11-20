"""İki aracın konumunu (uav0 NED) 2 Hz CSV'ye yazar; en küçük 3B ayrımı ve direk mesafesini izler. Yalnız dinler."""
import math, sys, time
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
sys.path.insert(0, "/home/yusuf22_04/iha_sim_v2")
from v2_orbit_mission import SENSOR_QOS
P = {"K": (140.0, 0.0), "G": (-140.0, 0.0), "B": (0.0, -140.0), "D": (70.0, 121.2)}
rclpy.init(); n = Node("iz2"); d = {}
n.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", lambda m: d.__setitem__(0, m), SENSOR_QOS)
n.create_subscription(VehicleLocalPosition, "/px4_1/fmu/out/vehicle_local_position_v1", lambda m: d.__setitem__(1, m), SENSOR_QOS)
f = open(sys.argv[1], "w"); f.write("saat,t,n0,e0,alt0,n1,e1,alt1,ayrim3b,direk0,direk1,vz1,v1\n"); t0 = time.time(); nxt = t0
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() >= nxt and 0 in d and 1 in d:   # 2 Hz
            nxt += 0.5
            a, b = d[0], d[1]
            n1, e1 = b.x + 200.0, b.y - 950.0   # üs (yerden kalkış senaryosu)
            s = math.sqrt((a.x - n1) ** 2 + (a.y - e1) ** 2 + (a.z - b.z) ** 2)
            p0 = min(math.hypot(a.x - p[0], a.y - p[1]) for p in P.values()); p1 = min(math.hypot(n1 - p[0], e1 - p[1]) for p in P.values())
            f.write(f"{time.strftime('%H:%M:%S')},{time.time() - t0:.1f},{a.x:.1f},{a.y:.1f},{-a.z:.1f},{n1:.1f},{e1:.1f},{-b.z:.1f},{s:.1f},{p0:.1f},{p1:.1f},{-b.vz:.2f},{math.hypot(b.vx, b.vy):.1f}\n"); f.flush()
except KeyboardInterrupt:
    pass
