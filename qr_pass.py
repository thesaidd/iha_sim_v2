#!/usr/bin/env python3
"""
qr_pass.py — uav0'ı QR hedefi üzerinden düz bir hatta geçirir (bir geçiş), sonra Hold'a alır.

Geometri (uav0 yerel NED; Gazebo dünyasında x = Doğu, y = Kuzey): hedef T = (N −60, E −60), yön ψ
(varsayılan 210°: KKD → GGB). Hat T'den geçer, birim yön d = (cos ψ, sin ψ), sağ dik r = (−sin ψ, cos ψ).
s = (P − T)·d hat boyunca ilerleme, x = (P − T)·r yanal sapma (sağ +).
ψ = 210° seçimi (2026-10-30 taraması): başlangıç ve bitiş Hold çemberleri (r 80 m) ve hat arena
direklerinden ≥ 48 m uzakta, hat doğu kalkış koridorunu (N −15…+17, E > 0) kesmiyor (E = 0'da N = 44).

Akış:
  1. Uçak Hold'da (AUTO), arm, havada olmalı. Hizalama Hold merkezi H = T − pre·d + 80·r: PX4 Hold
     saat yönünde döner; rota ψ'ye döndüğü noktada uçak tam hatta olur (s ≈ −pre).
  2. (2026-11-11) Taşıma bacakları ve H Hold çemberi öteki aracın canlı konumuna göre de denetlenir
     (ayrim.py, --min-ayrim 40 m 3B): yetmiyorsa önce yerinde irtifa değişimi, olmuyorsa bekleme → rota reddi.
     Geçiş: mevcut konumdan H'ye düz yol direklere < --clear m yaklaşıyorsa ara nokta (W) seçilir.
     DO_REPOSITION (Hold modu) W → H, irtifa --alt. HER bacak komuttan önce denetlenir; güvenli rota
     yoksa (ara nokta yok / --max-hops tükendi) H'ye gidilmez: uçak son ara noktanın ya da başladığı
     Hold'da bırakılır, çıkış kodu 3 (2026-11-11; önceden 3 ara noktadan sonra denetimsiz H'ye gidiliyordu).
  3. H çevresinde bekleme; OFFBOARD setpoint akışı burada başlar (Hold'da PX4 yok sayar).
     Devralma koşulu: rota hatası ∈ [−15°, +5°] (saat yönü dönüşte rota artar; komut gecikmesini
     karşılamak için erken), |x| < 20 m, s < −(400 + 50), |irtifa − hedef| < 5 m.
  4. OFFBOARD: her döngüde setpoint = hattın uçağa en yakın noktası (+ --bias sağa kaydırma),
     hız = V·d, ivme NaN (eğrilik 0), z = −alt. PX4 sabit kanatta konum + sonlu hızı bir yol parçası
     olarak alır (NPFG, AUTO_PATH): nokta yolun üzerindeki bir nokta, hız yönü teğet. En yakın nokta
     verildiği için ileri-geri çekme ve eğrilik kaynaklı sapma yok; zamandan bağımsız (RTF/zaman
     damgası sıçramalarından etkilenmez).
  5. s ≥ --end (150 m): --done-file yazılır, Hold (DO_SET_MODE AUTO_LOITER) bu düğümden verilir; setpoint
     akışı mod değişene kadar sürer (akış kesilirse PX4 Offboard kaybı failsafe'i → RTL, 2026-10-30 geçiş 2).
     Özet --result'a (JSON satırı). Uçuşta kullanılan sürüm: yedek/qr_pass.py.ucus-2026-10-30-1925.
Abort (--abort-file ya da OFFBOARD kaybı): yayın durur, çıkış kodu 2. Ayrı izleyici: qr_watch.py.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 qr_pass.py --pass-id 1 --done-file F --abort-file G --result R.jsonl
Çıkış kodu: 0 = geçiş tamam, 1 = hazırlık hatası/zaman aşımı, 2 = abort/OFFBOARD kaybı, 3 = rota reddedildi.
"""

import argparse
import json
import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from px4_msgs.msg import (AirspeedValidated, OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleCommandAck, VehicleControlMode, VehicleLandDetected, VehicleLocalPosition)

import ayrim as ayrim_mod
from nav_izleyici import AUTO_LOITER, NavIzleyici
from v2_orbit_mission import (NAN3, PX4_QOS, SENSOR_QOS, control_mode_name, make_vehicle_command,
                              ne_to_latlon)

HERE = os.path.dirname(os.path.abspath(__file__))
PYLONS = {"K": (140.0, 0.0), "G": (-140.0, 0.0), "B": (0.0, -140.0), "D": (70.0, 121.2)}   # (N, E), 140 m boy
LOITER_R = 80.0          # NAV_LOITER_RAD (PX4 varsayılanı)
LOOP_HZ = 20


def log(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


def seg_clearance(a, b):
    """a→b doğru parçasının direklere en küçük yatay uzaklığı [m]."""
    best = math.inf
    for p in PYLONS.values():
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        best = min(best, math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1]))
    return best


def in_corridor(n, e):
    """Doğu kalkış koridoru (uçuş izleri N −15…+17) + pay: 0 < E < 300, |N| < 40. Kalkış Hold'u E ≈ 500'de
    (koridorun ucu); 300 m ötesi arena dışı, oradan geçiş serbest (2026-10-30, 1. denemede kutu sınırsızdı ve
    uçak arenanın içinden 720 m dolaştırıldı)."""
    return 0.0 < e < 300.0 and abs(n) < 40.0


def corridor_free(a, b, skip_start):
    """a→b parçası koridora girmiyor mu; a koridordaysa ilk skip_start m muaf (oradan çıkılıyor)."""
    L = math.dist(a, b)
    exempt = skip_start if in_corridor(*a) else 0.0
    for k in range(int(L // 10) + 1):
        t = min(1.0, 10.0 * k / L) if L > 0 else 0.0
        if t * L >= exempt and in_corridor(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])):
            return False
    return True


def leg_ok(a, b, clear):
    return seg_clearance(a, b) >= clear and corridor_free(a, b, 150.0)


ARRIVE_R = 120.0         # ara noktaya bu kadar yaklaşınca varılmış sayılır (Hold çemberi 80 m + pay)
EXIT_ROUTE_REJECTED = 3  # çıkış kodu: güvenli rota yok, uçak Hold'da bırakıldı


def pylon_dist(w):
    return min(math.hypot(w[0] - p[0], w[1] - p[1]) for p in PYLONS.values())


def plan_via(start, goal, clear):
    """Düz yol direk ve koridor açısından açıksa None; değilse iki bacağı da açık en kısa ara nokta.

    2026-11-11: (a) ara noktanın Hold çemberi de direklerden ≥ clear olmalı (merkez ≥ clear + LOITER_R; önce
    clear + 60 idi → çember 20 m'ye kadar yaklaşabiliyordu). Böylece ara nokta güvenli bir bekleme / geri
    çekilme noktasıdır. (b) Önce "sağlam" adaylar: w → goal bacağı clear + ARRIVE_R payıyla açık, yani
    ara noktaya varış çemberinin HER noktasından goal'e düz yol açık kalır (eski hata: 120 m kala yeniden
    planlamada düz yol kapalı çıkıp ara nokta hakkı tükeniyordu). Sağlam aday yoksa yalnız (a)'yı sağlayanlar.
    Hiç aday yoksa RuntimeError (çağıran next_leg bunu "ret"e çevirir)."""
    if leg_ok(start, goal, clear):
        return None
    best = {True: None, False: None}
    for radius in (250.0, 350.0, 450.0):
        for k in range(36):
            a = math.radians(10 * k)
            w = (radius * math.cos(a), radius * math.sin(a))
            if pylon_dist(w) < clear + LOITER_R or in_corridor(*w):
                continue
            if leg_ok(start, w, clear) and leg_ok(w, goal, clear):
                robust = seg_clearance(w, goal) >= clear + ARRIVE_R
                length = math.dist(start, w) + math.dist(w, goal)
                if best[robust] is None or length < best[robust][0]:
                    best[robust] = (length, w)
    pick = best[True] or best[False]
    if pick is None:
        raise RuntimeError("direklerden uzak ara nokta bulunamadı")
    return pick[1]


def next_leg(here, goal, clear, vias_left):
    """Bir sonraki DO_REPOSITION bacağı — DENETİMSİZ BACAK YOK (2026-11-11 düzeltmesi).
    → ("H", goal)       : here → goal düz yol açık (leg_ok)
    → ("via", w)        : düz yol kapalı, ara nokta hakkı var ve iki bacağı açık ara nokta bulundu
    → ("reject", neden) : düz yol kapalı ve (ara nokta hakkı bitti | ara nokta yok). Çağıran hiçbir şey
                          göndermez; uçak bulunduğu Hold'da kalır."""
    if leg_ok(here, goal, clear):
        return "H", goal
    why = (f"düz yol kapalı (direk {seg_clearance(here, goal):.0f} m < {clear:.0f} m"
           f"{'' if corridor_free(here, goal, 150.0) else ', koridora giriyor'})")
    if vias_left <= 0:
        return "reject", why + "; ara nokta hakkı bitti"
    try:
        return "via", plan_via(here, goal, clear)
    except RuntimeError as e:
        return "reject", why + f"; {e}"


TRACK_MARGIN = 25.0      # düz bacak denetimine eklenen iz payı [m] (2026-11-11 uçuş ölçümü, aşağıda)
COURSE_GATE = 45.0       # bacak komutu yalnız rota bacak yönüne bu kadar yakınken verilir [°]


def course_error(pos, tgt):
    """Uçağın yer rotası ile (uçak → tgt) kerterizi arasındaki fark [°]; hız < 3 m/s ise None."""
    if math.hypot(pos.vx, pos.vy) < 3.0:
        return None
    return math.degrees(math.remainder(math.atan2(pos.vy, pos.vx) - math.atan2(tgt[1] - pos.y, tgt[0] - pos.x),
                                       2 * math.pi))


def transit(node, H, alt, clear, max_hops=3, replan_wait=60.0, arrive_timeout=180.0, spin=None, clock=time.monotonic,
            margin=TRACK_MARGIN, course_gate=COURSE_GATE, ayrim=None, alt_timeout=120.0):
    """Uçağı Hold modunda (DO_REPOSITION) H'ye götürür; her bacak komuttan ÖNCE uçağın gerçek konumundan
    denetlenir. Dönüş: (kod, bilgi). kod 0 = H komutu kabul edildi, 1 = komut kabul edilmedi,
    EXIT_ROUTE_REJECTED = güvenli rota yok.

    Ret durumunda (replan_wait sn boyunca Hold çemberinde dönerken de açık bacak çıkmadıysa) güvenli geri
    çekilme: son kabul edilen ara nokta varsa oraya DO_REPOSITION yinelenir (çemberi direklerden ≥ clear —
    plan_via güvencesi); yoksa hiçbir komut gönderilmez, uçak başladığı Hold'da kalır. H'ye asla denetimsiz
    gidilmez.

    İz payı (margin) ve rota kapısı (course_gate) — 2026-11-11 uçuş bulgusu: PX4 DO_REPOSITION'da komut anındaki
    konumdan hedefe düz hattı izler, ama Hold çemberinden çıkış dönüşü yüzünden uçak hattın 27 m soluna düşüp
    sonra 15 m sağına taştı; düz hattı direğe 43 m olan bacakta gerçek iz 30 m'den geçti. Bu yüzden (1) bacaklar
    clear + margin ile planlanır/denetlenir, (2) komut, Hold çemberinde uçağın rotası bacak yönüne ±course_gate
    içindeyken verilir (çıkış dönüşü küçük kalır). Kapı replan_wait içinde tutmazsa rota reddedilir."""
    # İki araç arası ayrım (ayrim.AyrimDenetimi; 2026-11-11): her bacak ve varılacak Hold çemberi öteki aracın
    # canlı konumuna/Hold çemberine göre de denetlenir (en kötü durum 3B ≥ min_ayrim). Yetmiyorsa:
    #   (a) yalnız başlangıç irtifası yüzünden yetmiyorsa (hedef irtifada bacak uygun) önce YERİNDE irtifa
    #       değiştirilir (kendi Hold merkezine DO_REPOSITION, dikey ayrım artan yönde), sonra yeniden denetlenir;
    #   (b) değilse bacak reddedilir → çemberde beklenir → replan_wait dolunca rota reddi (güvenli geri çekilme).
    plan_clear = clear + margin
    clear_true, clear = clear, plan_clear
    spin = spin or (lambda: rclpy.spin_once(node, timeout_sec=0.1))
    info = {"bacaklar": [], "ara_nokta": 0, "ret": None}
    if pylon_dist(H) - LOITER_R < clear_true:
        info["ret"] = f"H Hold çemberi direğe {pylon_dist(H) - LOITER_R:.0f} m (< {clear_true:.0f} m)"
        log(f"ROTA REDDEDİLDİ: {info['ret']} — komut gönderilmedi, uçak mevcut Hold'da")
        return EXIT_ROUTE_REJECTED, info
    last_via = None
    while True:
        t = clock()
        waited = False
        while True:
            here = (node.pos.x, node.pos.y)
            kind, tgt = next_leg(here, H, clear, max_hops - info["ara_nokta"])
            if kind == "via" and last_via and math.dist(tgt, last_via) < 1.0:
                # Aynı ara nokta yeniden çıktı (eski hatanın tetikleyicisi): hak harcama, çemberde bekle
                kind, tgt = "reject", "ara nokta Hold'undan H'ye düz yol henüz açık değil"
            if kind != "reject" and ayrim is not None:
                alt_here = -node.pos.z
                ok_a, _, why_a = ayrim.bacak(here, tgt, alt_here, alt)
                if ok_a:
                    ok_a, _, why_a = ayrim.cember(tgt, alt)
                if not ok_a:
                    own_c = ayrim_mod.kendi_hold_merkezi(node.pos.x, node.pos.y, node.pos.vx, node.pos.vy)
                    dg = ayrim.diger()
                    uzaklasiyor = ("irtifa" in dg and abs(alt - dg["irtifa"]) > abs(alt_here - dg["irtifa"])
                                   and (alt - dg["irtifa"]) * (alt_here - dg["irtifa"]) > 0)
                    if (not info.get("yerinde_irtifa") and abs(alt - alt_here) > 5.0 and own_c is not None and uzaklasiyor
                            and ayrim.bacak(here, tgt, alt, alt)[0] and ayrim.cember(tgt, alt)[0]
                            and pylon_dist(own_c) - LOITER_R >= clear_true):
                        ok = node.reposition(own_c[0], own_c[1], alt)
                        log(f"AYRIM: {why_a}. Hedef irtifada ({alt:.0f} m) bacak uygun → önce YERİNDE irtifa değişimi "
                            f"{alt_here:.0f} → {alt:.0f} m (kendi Hold merkezi ({own_c[0]:.0f}, {own_c[1]:.0f})): "
                            f"{'kabul' if ok else 'KABUL EDİLMEDİ'}")
                        info["yerinde_irtifa"] = {"nereden_m": round(alt_here, 1), "nereye_m": alt,
                                                  "merkez": [round(own_c[0]), round(own_c[1])]}
                        if not ok:
                            return 1, info
                        ta = clock()
                        while abs(-node.pos.z - alt) > 4.0 and clock() - ta < alt_timeout:
                            spin()
                        log(f"AYRIM: yerinde irtifa değişimi {'tamam' if abs(-node.pos.z - alt) <= 4.0 else 'ZAMAN AŞIMI'}: "
                            f"irtifa {-node.pos.z:.1f} m")
                        t = clock()
                        continue
                    kind, tgt = "reject", "AYRIM — " + why_a
            # Rota kapısı yalnız gerçek bir geçiş bacağında: hedefin varış çemberi içindeysek (ör. uçak zaten H'de
            # Hold'da) merkez hep rotanın 90° sağındadır, kapı hiç tutmaz (2026-11-11 gösterim denemesinde ret).
            if kind != "reject" and course_gate is not None and math.dist(here, tgt) > ARRIVE_R:
                cerr = course_error(node.pos, tgt)
                if cerr is None or abs(cerr) > course_gate:
                    kind, tgt = "reject", (f"rota kapısı tutmadı (rota farkı "
                                           f"{'yok' if cerr is None else format(cerr, '+.0f') + '°'}, sınır ±{course_gate:.0f}°)")
            if kind != "reject" or clock() - t >= replan_wait:
                break
            if not waited:
                waited = True
                log(f"bekleme: ({here[0]:.0f}, {here[1]:.0f})'den {tgt} — Hold çemberinde uygun an "
                    f"bekleniyor (en fazla {replan_wait:.0f} sn)")
            spin()
        if kind == "reject":
            info["ret"] = tgt
            if last_via:
                ok = node.reposition(last_via[0], last_via[1], alt)
                log(f"ROTA REDDEDİLDİ: {tgt}. Güvenli geri çekilme → son ara nokta ({last_via[0]:.0f}, {last_via[1]:.0f}) "
                    f"Hold'u (çember–direk {pylon_dist(last_via) - LOITER_R:.0f} m): {'kabul' if ok else 'KABUL EDİLMEDİ'}")
                info["geri_cekilme"] = [round(last_via[0]), round(last_via[1])]
            else:
                log(f"ROTA REDDEDİLDİ: {tgt}. Komut gönderilmedi — uçak mevcut Hold'da kalıyor "
                    f"({here[0]:.0f}, {here[1]:.0f})")
                info["geri_cekilme"] = "mevcut Hold"
            return EXIT_ROUTE_REJECTED, info
        name = "H" if kind == "H" else "ara nokta"
        tn, te = tgt
        cl = seg_clearance(here, tgt)
        ok = node.reposition(tn, te, alt)
        log(f"DO_REPOSITION → {name} ({tn:.0f}, {te:.0f}), {alt:.0f} m: "
            f"{'kabul (' + str(ok) + '. deneme)' if ok else 'KABUL EDİLMEDİ'}; uçaktan yol–direk mesafesi "
            f"{cl:.0f} m (denetlendi ≥ {clear_true:.0f} + iz payı {margin:.0f}), koridor serbest; "
            f"rota farkı {'—' if course_gate is None else format(course_error(node.pos, tgt) or 0.0, '+.0f') + '°'}"
            + ("" if ayrim is None else "; ayrım: " + ayrim.bacak(here, tgt, -node.pos.z, alt)[2]))
        info["bacaklar"].append({"tur": name, "nereden": [round(here[0]), round(here[1])],
                                 "nereye": [round(tn), round(te)], "direk_m": round(cl, 1)})
        if not ok:
            return 1, info
        if kind == "H":
            return 0, info
        info["ara_nokta"] += 1
        last_via = tgt
        t = clock()
        while math.hypot(node.pos.x - tn, node.pos.y - te) > ARRIVE_R and clock() - t < arrive_timeout:
            spin()
        if math.hypot(node.pos.x - tn, node.pos.y - te) > ARRIVE_R:
            log(f"UYARI: ara noktaya {arrive_timeout:.0f} sn'de varılamadı — gerçek konumdan yeniden planlanıyor")


class Pass(Node):
    def __init__(self):
        super().__init__("qr_pass")
        self.mode = self.pos = None
        self.landed = None
        self.airspeed = math.nan
        self.ack = {}
        self.pub_ocm = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode",
                                 lambda m: setattr(self, "mode", m), SENSOR_QOS)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "pos", m), SENSOR_QOS)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected",
                                 lambda m: setattr(self, "landed", m.landed), SENSOR_QOS)
        self.create_subscription(AirspeedValidated, "/fmu/out/airspeed_validated_v1",
                                 lambda m: setattr(self, "airspeed", m.calibrated_airspeed_m_s), SENSOR_QOS)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack_v1",
                                 lambda m: self.ack.__setitem__(m.command, m.result), SENSOR_QOS)

    def ts(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def offboard(self):
        return bool(self.mode and self.mode.flag_control_offboard_enabled)

    def stream(self, position, velocity):
        ocm = OffboardControlMode()
        ocm.timestamp = self.ts()
        ocm.position = True
        self.pub_ocm.publish(ocm)
        sp = TrajectorySetpoint()
        sp.timestamp = self.ts()
        sp.position = [float(v) for v in position]
        sp.velocity = [float(v) for v in velocity]
        sp.acceleration = NAN3
        sp.jerk = NAN3
        sp.yaw = math.nan
        sp.yawspeed = math.nan
        self.pub_sp.publish(sp)

    def command(self, cmd, *params, timeout=1.0, tries=5):
        for attempt in range(1, tries + 1):
            self.ack.pop(cmd, None)
            self.pub_cmd.publish(make_vehicle_command(0, self.ts(), cmd, *params))
            t = time.monotonic()
            while cmd not in self.ack and time.monotonic() - t < timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.ack.get(cmd) == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                return attempt
        return None

    def reposition(self, n, e, alt):
        lat, lon = ne_to_latlon(n, e, self.pos.ref_lat, self.pos.ref_lon)
        return self.command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION, -1.0, 1.0, 0.0, math.nan,
                            lat, lon, self.pos.ref_alt + alt)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pass-id", type=int, required=True)
    ap.add_argument("--target-n", type=float, default=-60.0)
    ap.add_argument("--target-e", type=float, default=-60.0)
    ap.add_argument("--heading", type=float, default=210.0, help="Hat yönü ψ [°]")
    ap.add_argument("--alt", type=float, default=60.0)
    ap.add_argument("--speed", type=float, default=15.0)
    ap.add_argument("--pre", type=float, default=650.0, help="Hizalama noktası hedefin bu kadar gerisinde [m]")
    ap.add_argument("--aligned-by", type=float, default=400.0, help="Bu mesafede hatta hizalı olunmalı [m]")
    ap.add_argument("--end", type=float, default=150.0, help="Hedefi bu kadar geçince Hold [m]")
    ap.add_argument("--bias", type=float, default=0.0, help="Setpoint hattını sağa kaydır [m] (önceki geçişten düzeltme)")
    ap.add_argument("--clear", type=float, default=40.0, help="Direklere en küçük yatay uzaklık [m]")
    ap.add_argument("--align-timeout", type=float, default=300.0)
    ap.add_argument("--max-hops", type=int, default=3, help="H'ye giderken en fazla ara nokta sayısı")
    ap.add_argument("--replan-wait", type=float, default=60.0,
                    help="Açık bacak yoksa Hold çemberinde bu kadar sn beklenir, sonra rota reddedilir")
    ap.add_argument("--track-margin", type=float, default=TRACK_MARGIN,
                    help="Taşıma bacaklarında --clear'a eklenen iz payı [m]")
    ap.add_argument("--min-ayrim", type=float, default=ayrim_mod.MIN_AYRIM,
                    help="Öteki araca en küçük 3B ayrım [m] (taşıma bacakları ve Hold çemberi); 0 = denetim kapalı")
    ap.add_argument("--diger-uav", type=int, default=1, help="Ayrımı denetlenen öteki araç")
    ap.add_argument("--transit-only", action="store_true",
                    help="Yalnız H'ye taşı (Hold modu), hizalama/OFFBOARD/geçiş yapma")
    ap.add_argument("--done-file", required=True)
    ap.add_argument("--abort-file", required=True)
    ap.add_argument("--result", required=True, help="Özet JSON satırı eklenecek dosya")
    args = ap.parse_args()

    psi = math.radians(args.heading)
    d = (math.cos(psi), math.sin(psi))
    r = (-math.sin(psi), math.cos(psi))
    T = (args.target_n, args.target_e)
    H = (T[0] - args.pre * d[0] + LOITER_R * r[0], T[1] - args.pre * d[1] + LOITER_R * r[1])
    for f in (args.done_file, args.abort_file):
        if os.path.exists(f):
            os.remove(f)

    def along_cross(n, e):
        dn, de = n - T[0], e - T[1]
        return dn * d[0] + de * d[1], dn * r[0] + de * r[1]

    rclpy.init()
    node = Pass()
    nav = NavIzleyici(0)          # nav_state (vehicle_status_v2, px4_durum.py alt süreci)
    ayrim_izleyici = []
    res = {"gecis": args.pass_id, "yon_deg": args.heading, "irtifa_hedef": args.alt, "bias_m": args.bias,
           "baslangic": time.strftime("%H:%M:%S")}
    try:
        t0 = time.monotonic()
        while (node.pos is None or not node.pos.xy_global or node.mode is None) and time.monotonic() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
        p = node.pos
        if p is None or node.mode is None:
            log("HATA: konum / mod alınamadı")
            return 1
        mode = control_mode_name(node.mode)
        nav.wait_first(8.0)
        # AUTO, RTL'yi de kapsar (2026-10-30 geçiş 3 RTL tırmanışında başladı) → nav_state = AUTO_LOITER şartı
        if not node.mode.flag_armed or node.landed or -p.z < 30 or nav.nav != AUTO_LOITER:
            log(f"HATA: uçak hazır değil (arm={node.mode.flag_armed}, landed={node.landed}, irtifa={-p.z:.1f}, "
                f"mod={mode}, nav_state={nav.name()})")
            return 1
        log(f"[geçiş {args.pass_id}] hedef ({T[0]:.0f}, {T[1]:.0f}), ψ={args.heading:.0f}°, irtifa {args.alt:.0f} m, "
            f"V {args.speed:.0f} m/s, bias {args.bias:+.1f} m; hizalama Hold merkezi H=({H[0]:.0f}, {H[1]:.0f}); "
            f"uçak ({p.x:.0f}, {p.y:.0f}, {-p.z:.0f} m)")
        res["hat_direk_mesafesi_m"] = round(seg_clearance((T[0] - args.pre * d[0], T[1] - args.pre * d[1]),
                                                          (T[0] + args.end * d[0], T[1] + args.end * d[1])), 1)

        # ── 2. Geçiş: (ara nokta) → H ────────────────────────────────────────────────────────
        # Her bacak uçağın gerçek konumundan denetlenir; güvenli rota yoksa H'ye GİDİLMEZ (transit()).
        # Öteki aracın canlı konumu (doğrudan topic); Hold kestirimi için birkaç sn geçmiş toplanır.
        ayr = None
        if args.min_ayrim > 0:
            ayr = ayrim_mod.DigerAracIzleyici(node, 0, args.diger_uav, kendi_konum=lambda: node.pos,
                                              min_ayrim=args.min_ayrim, iz_payi=args.track_margin)
            ayrim_izleyici.append(ayr)
            t = time.monotonic()
            while time.monotonic() - t < (ayrim_mod.HOLD_PENCERE + 1.5 if ayr.goruldu else 3.0):
                rclpy.spin_once(node, timeout_sec=0.1)
            dg = ayr.diger()
            log("ayrım denetimi: " + (f"uav{args.diger_uav} {dg['tur']}"
                + (f", Hold merkezi ({dg['merkez'][0]:.0f}, {dg['merkez'][1]:.0f})" if dg["tur"] == "hold" else "")
                + f", irtifa {dg['irtifa']:.0f} m; eşik 3B {args.min_ayrim:.0f} m" if ayr.goruldu
                else f"uav{args.diger_uav} verisi YOK — denetim kapalı (tek araç?)"))
        code, tinfo = transit(node, H, args.alt, args.clear, max_hops=args.max_hops, replan_wait=args.replan_wait,
                              margin=args.track_margin, ayrim=ayr)
        if ayr is not None and ayr.ozet():
            log(f"ayrım: taşıma komutlarına kadar gerçekleşen en küçük 3B {ayr.ozet()}")
        res["tasima"] = tinfo
        if code:
            res.update(sonuc="rota_reddedildi" if code == EXIT_ROUTE_REJECTED else "komut_reddi")
            return code
        if args.transit_only:
            t = time.monotonic()
            next_log = t
            while time.monotonic() - t < args.align_timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                distH = math.hypot(node.pos.x - H[0], node.pos.y - H[1])
                if time.monotonic() >= next_log:
                    next_log += 10.0
                    log(f"taşıma: H'ye {distH:.0f} m, irtifa {-node.pos.z:.1f} m, direk {pylon_dist((node.pos.x, node.pos.y)):.0f} m")
                if distH <= ARRIVE_R and abs(-node.pos.z - args.alt) < 5.0:
                    log(f"TAŞIMA TAMAM (--transit-only): H'ye {distH:.0f} m, irtifa {-node.pos.z:.1f} m, Hold'da")
                    res.update(sonuc="tasima_tamam")
                    return 0
            log("HATA: taşıma zaman aşımı")
            res.update(sonuc="tasima_zaman_asimi")
            return 1

        # ── 3. Hizalama bekleme (setpoint akışı başlar) ───────────────────────────────────────
        start = time.monotonic()
        next_log = start
        engaged = None
        attempts = 0
        last_attempt = 0.0
        while engaged is None:
            rclpy.spin_once(node, timeout_sec=1.0 / LOOP_HZ)
            if os.path.exists(args.abort_file):
                log("abort-file — hazırlık durduruldu")
                return 2
            p = node.pos
            s, x = along_cross(p.x, p.y)
            node.stream((T[0] + s * d[0] + args.bias * r[0], T[1] + s * d[1] + args.bias * r[1], -args.alt),
                        (args.speed * d[0], args.speed * d[1], 0.0))
            course = math.atan2(p.vy, p.vx)
            err = math.degrees(math.remainder(course - psi, 2 * math.pi))
            alt = -p.z
            distH = math.hypot(p.x - H[0], p.y - H[1])
            ready = (-15.0 <= err <= 5.0 and abs(x) < 20.0 and s < -(args.aligned_by + 50.0)
                     and abs(alt - args.alt) < 5.0 and 50.0 <= distH <= 110.0)
            now = time.monotonic()
            if now >= next_log:
                next_log += 5.0
                log(f"hizalama: H'ye {distH:5.0f} m  s={s:+6.0f}  x={x:+6.1f} m  rota hatası={err:+6.1f}°  "
                    f"irtifa={alt:5.1f} m  yer_hızı={math.hypot(p.vx, p.vy):4.1f} m/s  mod={control_mode_name(node.mode)}")
            if node.offboard():
                engaged = now
                break
            if ready or attempts:
                if attempts >= 10:
                    log("HATA: 10 denemede OFFBOARD'a geçilemedi")
                    return 1
                if now - last_attempt >= 1.0:
                    attempts += 1
                    last_attempt = now
                    node.pub_cmd.publish(make_vehicle_command(0, node.ts(), VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0))
                    log(f"DO_SET_MODE → OFFBOARD (deneme {attempts}): s={s:+.0f} m, x={x:+.1f} m, rota hatası={err:+.1f}°")
            if now - start > args.align_timeout:
                log(f"HATA: {args.align_timeout:.0f} sn içinde hizalanamadı")
                return 1
        p = node.pos
        s, x = along_cross(p.x, p.y)
        log(f"OFFBOARD ✔ ({time.monotonic() - start:.0f} sn hizalama): s={s:+.0f} m, x={x:+.1f} m, irtifa={-p.z:.1f} m, "
            f"nav_state={nav.name()}")
        res.update(devralma_s_m=round(s, 1), devralma_x_m=round(x, 1))

        # ── 4. Hat izleme ─────────────────────────────────────────────────────────────────────
        samples = []          # (s, x, irtifa, yer_hızı, hava_hızı, monotonic)
        cross = None
        prev = None
        next_log = time.monotonic()
        while True:
            rclpy.spin_once(node, timeout_sec=1.0 / LOOP_HZ)
            if os.path.exists(args.abort_file) or not node.offboard():
                why = "abort-file" if os.path.exists(args.abort_file) else f"OFFBOARD kaybı (mod={control_mode_name(node.mode)})"
                log(f"DURDU: {why}")
                res.update(sonuc="abort", neden=why, ornek=len(samples))
                return 2
            p = node.pos
            s, x = along_cross(p.x, p.y)
            alt, gs = -p.z, math.hypot(p.vx, p.vy)
            node.stream((T[0] + s * d[0] + args.bias * r[0], T[1] + s * d[1] + args.bias * r[1], -args.alt),
                        (args.speed * d[0], args.speed * d[1], 0.0))
            samples.append((s, x, alt, gs, node.airspeed, time.monotonic()))
            if prev and prev[0] < 0.0 <= s:
                k = -prev[0] / (s - prev[0])
                cross = {"x_m": round(prev[1] + k * (x - prev[1]), 2),
                         "dikey_m": round(prev[2] + k * (alt - prev[2]) - args.alt, 2),
                         "yer_hizi": round(prev[3] + k * (gs - prev[3]), 2), "hava_hizi": round(node.airspeed, 2)}
                log(f"HEDEF ÜSTÜ: yanal {cross['x_m']:+.2f} m, dikey {cross['dikey_m']:+.2f} m, "
                    f"yer hızı {cross['yer_hizi']:.1f} m/s, hava hızı {cross['hava_hizi']:.1f} m/s")
            prev = (s, x, alt, gs)
            if time.monotonic() >= next_log:
                next_log += 1.0
                log(f"hat: s={s:+7.1f} m  x={x:+6.2f} m  irtifa={alt:6.2f} m  yer_hızı={gs:5.2f}  "
                    f"hava_hızı={node.airspeed:5.2f} m/s")
            if s >= args.end:
                break

        # ── 5. Bitiş: Hold ────────────────────────────────────────────────────────────────────
        # Hold bu düğümden verilir ve mod değişene kadar setpoint akışı sürer. 2026-10-30 geçiş 2'de Hold
        # orbit_abort alt süreciyle verilmişti; o başlarken akış kesildi, PX4 Offboard kaybı (COM_OF_LOSS_T 1 sn)
        # failsafe'i RTL'yi tetikledi ("Failsafe activated", "RTL: start return"); orbit_abort RTL'yi de AUTO
        # saydığı için "HOLD ✔" dedi.
        with open(args.done_file, "w") as f:
            f.write(f"geçiş {args.pass_id} bitti\n")
        hold_ok, last_cmd, t_hold = False, 0.0, time.monotonic()
        nav_at_end = len(nav.history)
        while time.monotonic() - t_hold < 8.0:
            rclpy.spin_once(node, timeout_sec=1.0 / LOOP_HZ)
            p = node.pos
            s, _ = along_cross(p.x, p.y)
            node.stream((T[0] + s * d[0] + args.bias * r[0], T[1] + s * d[1] + args.bias * r[1], -args.alt),
                        (args.speed * d[0], args.speed * d[1], 0.0))
            ack = node.ack.get(VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
            # Hold yalnızca nav_state = AUTO_LOITER ile kabul (AUTO bayrağı RTL'yi de kapsar)
            if (ack == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED and node.mode
                    and node.mode.flag_control_auto_enabled and not node.mode.flag_control_offboard_enabled
                    and nav.nav == AUTO_LOITER and nav.history and nav.history[-1][0] >= time.time() - (time.monotonic() - t_hold)
                    and nav.fresh(1.5)):
                hold_ok = True
                break
            if time.monotonic() - last_cmd >= 0.5:
                last_cmd = time.monotonic()
                node.ack.pop(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, None)
                node.pub_cmd.publish(make_vehicle_command(0, node.ts(), VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                                          1.0, 4.0, 3.0))   # AUTO, AUTO_LOITER (Hold)
        log(f"Hold: {'✔ (DO_SET_MODE kabul, nav_state=AUTO_LOITER; akış mod değişene kadar sürdü)' if hold_ok else '✘ 8 sn içinde doğrulanamadı'}"
            f" — nav_state değişimleri: {' → '.join(nav.name(n) for _, n in nav.history[max(0, nav_at_end - 1):])}")
        res["nav_bitis"] = nav.name()
        hold = subprocess.CompletedProcess([], 0 if hold_ok else 1)

        def window(lo, hi):
            w = [smp for smp in samples if lo <= smp[0] <= hi]
            if not w:
                return None
            xs = [smp[1] for smp in w]
            alts = [smp[2] - args.alt for smp in w]
            mean = sum(xs) / len(xs)
            return {"ornek": len(w), "x_ort": round(mean, 2), "x_std": round(math.sqrt(sum((v - mean) ** 2 for v in xs) / len(xs)), 2),
                    "x_maks_abs": round(max(map(abs, xs)), 2), "x_min_abs": round(min(map(abs, xs)), 2),
                    "dikey_ort": round(sum(alts) / len(alts), 2), "dikey_maks_abs": round(max(map(abs, alts)), 2),
                    "yer_hizi_ort": round(sum(smp[3] for smp in w) / len(w), 2),
                    "hava_hizi_ort": round(sum(smp[4] for smp in w if math.isfinite(smp[4])) / max(1, sum(1 for smp in w if math.isfinite(smp[4]))), 2)}

        res.update(sonuc="tamam", hedef_ustu=cross, hizali_bolum=window(-args.aligned_by, args.end),
                   kayit_penceresi=window(-250.0, 100.0), hold=hold.returncode == 0)
        log(f"ÖZET geçiş {args.pass_id}: hedef üstü {cross}; s∈[−{args.aligned_by:.0f}, +{args.end:.0f}] "
            f"{res['hizali_bolum']}; kayıt penceresi {res['kayit_penceresi']}")
        return 0
    finally:
        res["bitis"] = time.strftime("%H:%M:%S")
        if ayrim_izleyici and ayrim_izleyici[0].ozet():
            res["ayrim_en_kucuk"] = ayrim_izleyici[0].ozet()
            log(f"AYRIM ÖZETİ: bu koşuda uav{args.diger_uav} ile gerçekleşen en küçük 3B ayrım {res['ayrim_en_kucuk']} "
                f"(eşik {args.min_ayrim:.0f} m)")
        with open(args.result, "a") as f:
            f.write(json.dumps(res, ensure_ascii=False) + "\n")
        nav.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
