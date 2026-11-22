import subprocess, threading, time
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=255, source_component=190)
stop=threading.Event(); st=[0]
def loop():
    while not stop.is_set():
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0)
        m.mav.manual_control_send(1,0,700,st[0],0,0); time.sleep(0.05)
threading.Thread(target=loop,daemon=True).start()
m.wait_heartbeat(timeout=15)
def c(cmd,*p):
    m.mav.command_long_send(1,1,cmd,0,*(list(p)+[0]*(7-len(p)))); a=m.recv_match(type="COMMAND_ACK",blocking=True,timeout=5); return a.result if a else None
time.sleep(2); print("mode",c(176,1,1)); time.sleep(1); print("arm",c(400,1,0)); time.sleep(1)
st[0]=500; time.sleep(1.5)
hb=m.recv_match(type="HEARTBEAT",blocking=True,timeout=3); print("armed" , bool(hb.base_mode & 128), "custom", hb.custom_mode)
for t in ("/model/gercek_ucak_0/servo_0","/gercek_ucak_0/command/motor_speed"):
    print(t); print(subprocess.run(["gz","topic","-e","-t",t,"-n","1"],capture_output=True,text=True,timeout=8).stdout[:200] or "(boş)")
c(400,0,21196); stop.set()
