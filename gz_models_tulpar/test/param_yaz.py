import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255, source_component=190)
m.wait_heartbeat(timeout=15)
INT = {"FD_ESCS_EN","ASPD_DO_CHECKS","ASPD_SCALE_APPLY","NAV_DLL_ACT","EKF2_IMU_CTRL","FW_LAUN_DETCN_ON","RWTO_TKOFF"}
for kv in sys.argv[1:]:
    k, v = kv.split("=")
    for _ in range(6):
        m.mav.param_set_send(1, 1, k.encode(), float(v), mavutil.mavlink.MAV_PARAM_TYPE_INT32 if k in INT else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
        if r and r.param_id.rstrip("\x00") == k: print(k, "->", round(r.param_value, 4)); break
    else: print(k, "-> YAZILAMADI")
