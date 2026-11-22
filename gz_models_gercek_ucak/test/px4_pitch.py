import subprocess, threading, time, sys
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255, source_component=190)
stop=threading.Event(); st=dict(x=0,y=0,z=0,r=0)
def loop():
    while not stop.is_set():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
        m.mav.manual_control_send(1,st["x"],st["y"],st["z"],st["r"],0); time.sleep(0.05)
threading.Thread(target=loop,daemon=True).start()
m.wait_heartbeat(timeout=15)
def c(cmd,*p):
    m.mav.command_long_send(1,1,cmd,0,*(list(p)+[0]*(7-len(p)))); a=m.recv_match(type="COMMAND_ACK",blocking=True,timeout=5); return a.result if a else None
time.sleep(2); c(176,1,1); time.sleep(1); c(400,1,0); st["z"]=300; time.sleep(0.5)
B="/home/yusuf24/PX4-Autopilot/build/px4_sitl_default/bin/"
for name,x,r in (("pitch yukarı",-700,0),("yaw sağ",0,700)):
    st["x"]=x; st["r"]=r; time.sleep(0.6)
    for t in ("vehicle_torque_setpoint","actuator_servos"):
        o=subprocess.run([B+"px4-listener",t,"-n","1"],capture_output=True,text=True,timeout=8).stdout
        print(name,t,[l.strip() for l in o.splitlines() if "xyz" in l or "control" in l])
st["x"]=0;st["r"]=0
c(400,0,21196); stop.set()
