"""uav0'ı düzeltilmiş qr_pass.transit ile verilen Hold noktasına taşır (Hold modu, yalnız DO_REPOSITION)."""
import math, sys, time
sys.path.insert(0, "/home/yusuf22_04/iha_sim_v2")
import rclpy
import qr_pass as q
P = (float(sys.argv[1]), float(sys.argv[2])); ALT = float(sys.argv[3])
rclpy.init(); node = q.Pass()
t0 = time.monotonic()
while (node.pos is None or not node.pos.xy_global or node.mode is None) and time.monotonic() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
p = node.pos
q.log(f"uav0 ({p.x:.0f}, {p.y:.0f}, {-p.z:.0f} m) → P=({P[0]:.0f}, {P[1]:.0f}) {ALT:.0f} m")
code, info = q.transit(node, P, ALT, 40.0)
q.log(f"transit kodu {code}: {info}")
if code: sys.exit(code)
t = time.monotonic()
while time.monotonic() - t < 400:
    rclpy.spin_once(node, timeout_sec=0.1)
    if math.hypot(node.pos.x - P[0], node.pos.y - P[1]) < 100 and abs(-node.pos.z - ALT) < 4:
        q.log(f"VARDI: P'ye {math.hypot(node.pos.x - P[0], node.pos.y - P[1]):.0f} m, irtifa {-node.pos.z:.1f} m"); sys.exit(0)
q.log("ZAMAN AŞIMI"); sys.exit(1)
