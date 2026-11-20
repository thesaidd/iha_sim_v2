"""uav0'ı qr_pass hizalama Hold merkezi H=(543,196)'ya 100 m'de taşır (Hold modu, DO_REPOSITION). Yol
lora_yaklasma.gecis_plani ile (tek ara nokta, HER bacak denetimli; yoksa gitmez). OFFBOARD/abort mantığı yok."""
import math, sys, time
sys.path.insert(0, "/home/yusuf22_04/iha_sim_v2")
import rclpy
from qr_pass import Pass, LOITER_R, log
from lora_yaklasma import dalis_engelleri, gecis_plani, yol_ihlalleri, _parca
ALT = 100.0
psi = math.radians(210.0); d = (math.cos(psi), math.sin(psi)); r = (-math.sin(psi), math.cos(psi))
T = (-60.0, -60.0); H = (T[0] - 650 * d[0] + LOITER_R * r[0], T[1] - 650 * d[1] + LOITER_R * r[1])
eng = dalis_engelleri(0.0, 0.0, T, uav0_yaricap=0)          # uav0 çerçevesi, yalnız arena engelleri
rclpy.init(); node = Pass()
t0 = time.monotonic()
while (node.pos is None or not node.pos.xy_global or node.mode is None) and time.monotonic() - t0 < 10:
    rclpy.spin_once(node, timeout_sec=0.1)
p = node.pos
log(f"uav0 ({p.x:.0f}, {p.y:.0f}, {-p.z:.0f} m) → H=({H[0]:.0f}, {H[1]:.0f}) {ALT:.0f} m")
if math.hypot(p.x - H[0], p.y - H[1]) < LOITER_R + 40 and abs(-p.z - 100.0) < 5:
    log("zaten H çemberinde"); sys.exit(0)
SON_ALT = ALT; ALT = float(round(-p.z))   # geçiş MEVCUT irtifada (uav1 115-120 m'de; dikey ayrım), tırmanış H'de
log(f'geçiş irtifası {ALT:.0f} m, H\'de {SON_ALT:.0f} m')
def acik(a, b): return not yol_ihlalleri(_parca(a, b, ALT, ALT, "g"), eng)[0]
t = time.monotonic(); ara = None
while time.monotonic() - t < 120:                             # çemberde dönerken açık bir plan çıkan noktayı bekle
    rclpy.spin_once(node, timeout_sec=0.1)
    ara = gecis_plani((node.pos.x, node.pos.y, ALT), H, eng)
    if ara is not None: break
if ara is None: log("GÜVENLİ YOL YOK — taşınmadı"); sys.exit(1)
for w in ara + [H]:
    son = w == H
    if not son:
        ok = node.reposition(w[0], w[1], ALT); log(f"DO_REPOSITION → ara nokta ({w[0]:.0f}, {w[1]:.0f}): {'kabul' if ok else 'RED'}")
        if not ok: sys.exit(1)
        t = time.monotonic()
        while time.monotonic() - t < 240:
            rclpy.spin_once(node, timeout_sec=0.1)
            if math.hypot(node.pos.x - w[0], node.pos.y - w[1]) < LOITER_R + 40 and acik((node.pos.x, node.pos.y), H): break
        else:
            log("ara noktadan H'ye açık bacak bulunamadı — ara noktada Hold'da"); sys.exit(1)
    here = (node.pos.x, node.pos.y)
    if son:
        assert acik(here, H), "son bacak açık değil"
        ok = node.reposition(H[0], H[1], ALT); log(f"DO_REPOSITION → H ({ALT:.0f} m): {'kabul' if ok else 'RED'} (bacak denetlendi)")
        if not ok: sys.exit(1)
t = time.monotonic(); nxt = t; tirman = False
while time.monotonic() - t < 400:
    rclpy.spin_once(node, timeout_sec=0.1)
    p = node.pos; dist = math.hypot(p.x - H[0], p.y - H[1])
    if time.monotonic() >= nxt: nxt += 15; log(f"H'ye {dist:.0f} m, irtifa {-p.z:.1f} m")
    if dist < LOITER_R + 40 and not tirman:
        tirman = True; node.reposition(H[0], H[1], SON_ALT); log(f"H çemberinde → {SON_ALT:.0f} m'ye tırmanış")
    if dist < LOITER_R + 30 and abs(-p.z - SON_ALT) < 3: log(f"VARDI: H'ye {dist:.0f} m, irtifa {-p.z:.1f} m"); sys.exit(0)
log("ZAMAN AŞIMI"); sys.exit(1)
