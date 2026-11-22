#!/usr/bin/env python3
"""
mavlink_takeoff.py — QGC olmadan PX4 araçlarını pymavlink ile kaldırır ve izler.

Akış (her araç için):
  1. GCS kalp atışı gönderir (1 Hz) → PX4'ün "No connection to the GCS" engeli kalkar.
  2. SIM_ONLY_PARAMS'ı yazar (yalnızca simülasyon, aşağıya bkz.).
  3. Araç zaten havadaysa kalkışı atlar; değilse MIS_TAKEOFF_ALT'ı ayarlar, Takeoff + arm.
  4. Her 5 sn'de irtifa / uçuş modu / gz sim belleğini yazar; PX4 STATUSTEXT mesajlarını
     (failsafe, landing vb.) geldiği anda tam metniyle yazar.
  5. Tüm araçlar hedefin üstünde turlamaya (LOITER) başlayınca --watch sn daha izler.
  6. Sonra kapanmaz: kalp atışını sürdürür, 30 sn'de bir durum yazar (Ctrl+C / SIGTERM ile biter).

Kullanım:
  python3 mavlink_takeoff.py                          # uav0, 100 m
  python3 mavlink_takeoff.py --uav 0 1                # iki araç, 100 / 120 m (araç başına MIS_TAKEOFF_ALT)
Çalışan bir süreç yeni komut almaz (girdi kanalı yok); havadaki uav0 için kalp atışı sürerken
yerdeki aracı kaldırmak için ros_takeoff.py (DDS) kullanılır.
  python3 mavlink_takeoff.py --uav 0 1 --takeoff 0    # ikisine kalp atışı + parametre, yalnızca uav0 kalkar
  python3 mavlink_takeoff.py --uav 1 --params-only    # yalnızca SIM_ONLY_PARAMS yaz, kalkış yok
  python3 mavlink_takeoff.py --uav 0 1 --revert-sim-params   # SIM_ONLY_PARAMS'ı geri al
  UAV0_MODEL=tulpar python3 mavlink_takeoff.py --uav 0 --el-firlatma --alt 100   # Tulpar el fırlatma (launch_all.sh UAV0=tulpar ile)

Bağlantı: PX4 instance i'nin onboard MAVLink bağlantısı (PX4 → udp 14540+i, sysid i+1).
14550'ye tüm araçlar birden yayın yaptığı için kullanılmaz.

Not: Airframe 4008'de NAV_DLL_ACT=2 (veri bağlantısı kaybında Return). Simülasyonda
SIM_ONLY_PARAMS bunu 0'a çeker; yine de kalp atışı süreci uçuş boyunca durdurulmamalı.
"""

import argparse
import math
import signal
import struct
import subprocess
import threading
import time

from pymavlink import mavutil

import arac_modelleri
import el_firlatma

# YALNIZCA SİMÜLASYON — gerçek donanıma taşınmayacak.
# isim: (simülasyon değeri, --revert-sim-params ile yazılacak PX4 varsayılanı)
# SITL parametreleri rootfs/<i>/parameters*.bson'a kaydeder; ayar yeniden başlatmada da kalır.
SIM_ONLY_PARAMS = {
    # gz AirSpeed eklentisi sıcaklık göndermiyor (0 K) → TAS %14 düşük → yanlış "sensor failure".
    # Gerçek düzeltme: PX4 src/modules/simulation/gz_plugins/airspeed + gz_bridge (açık iş).
    "ASPD_DO_CHECKS": (0, 7),
    # Aynı sıcaklık hatası yüzünden rüzgar kestiricisi ~1,185 ölçek öğreniyor; 2 (varsayılan)
    # bunu uçuşta CAS'a uyguluyor ve uçak trim hızının ~%12 altında uçuyor (2026-10-29).
    "ASPD_SCALE_APPLY": (0, 2),
    # Ölçek kapalıyken Hold'da yer hızı ~16 m/s (CAS 15,5); orbit setpoint'i 15 m/s ilerliyor.
    # Trim 14 ile yer hızını 14,5–15,5 m/s'ye çekmek için (2026-10-29).
    "FW_AIRSPD_TRIM": (14.0, 15.0),
    # Kalp atışı kısa süre kesilince (betik yeniden başlatma) RTL tetikleniyor ve bağlantı
    # geri gelse de iptal olmuyor (2026-10-29). Airframe 4008 değeri 2 = Return.
    "NAV_DLL_ACT": (0, 2),
    # Pist kalkışı (RWTO_TKOFF=1) CLIMBOUT'ta TECS'siz RWTO_MAX_THR ile MIS_TAKEOFF_ALT'a kadar çıkar:
    # 1,0 gazla 57 m/s, γ≈9° → ~560 m savrulma (2026-10-31, uav1 ulg). Mesafeyi tırmanma açısı
    # (FW_TKO_PITCH_MIN) belirler; gaz yalnızca hızı. Tahmin: ~24 m/s, γ≈19°, ~275–355 m (2026-11-01).
    # 0,6 → araçlar pistte hiç yuvarlanmadı (CLAMPED_TO_RUNWAY); 0,8 → uav0 kalktı, uav1 7,5 m/s'de takılı kaldı,
    # rotasyon yapamadı (2026-11-01). Zemin yuvarlanması CLIMBOUT ile aynı RWTO_MAX_THR'yi kullanıyor.
    # Kalkış-only seri (2026-11-01 01:15, temiz başlatma, pitch 20): 0,80 → İKİ araç da pistte (elevatör ~5 sn'de
    # doyuyor, pitch ~0, hız 13 → 7 m/s'ye düşüp kalıyor; uav1'e özgü değil); 0,825 → ikisi kalktı, rotasyon
    # 14 m/s (sınıra yakın); 0,85 → ikisi kalktı (default + iha_arena), rotasyon 15,5 m/s, elevatör doymadı,
    # 95/115 m'ye yatay 275/330 m. Eşik 0,80–0,825 → pay için 0,85.
    "RWTO_MAX_THR": (0.85, 1.0),
    "FW_TKO_PITCH_MIN": (20.0, 10.0),
    # Hold'daki uzun daire boyunca Y-ekseni accel bias sürüklenip (uav1: −0,40 @600 sn) EKF2_ABL_LIM'e dayanıyor →
    # "Preflight Fail: High Accelerometer Bias" OFFBOARD geçişini engelliyor. Sim IMU'da gerçek bias yok →
    # bit 1 (Accel Bias) kapalı: 5 = Gyro Bias + Gravity fusion (varsayılan 7).
    "EKF2_IMU_CTRL": (5, 7),
}

# UÇUŞ DİNAMİĞİ AYARI — simülatör hatası DEĞİL, gerçek donanımda da ayrıca doğrulanmalı.
# SIM_ONLY_PARAMS'tan bilerek ayrı: yukarıdakiler Gazebo/SITL kusurlarının geçici önlemleri, bunlar uçağın
# alçalma zarfını değiştirir. Yalnızca listelenen araca ve yalnızca --ucus-dinamigi verilince yazılır; bayrak
# yokken GERİ ALMA değeri yazılır (SITL parametreleri rootfs/<i>/parameters*.bson'da kalıcı → her başlatmada
# durum belli olsun). uav_id: {isim: (ayar değeri, geri alma değeri = airframe 4008, 2026-11-01 ulg)}
# Gerekçe (2026-11-10): lora_yaklasma.py dalis profili 12° alçalma ister (15 m/s'de 3,2 m/s); konum+hız Offboard'da
# TECS alçalma referansı min(FW_T_SINK_R_SP, FW_T_SINK_MAX) ile sınırlı (2,0 → en fazla ~7°).
UCUS_DINAMIGI_PARAMS = {
    1: {"FW_T_SINK_R_SP": (4.0, 2.0),
        "FW_T_SINK_MAX": (5.0, 2.7)},
}

def sim_params_for(uav_id):
    """Araç modeline göre YALNIZCA-SİMÜLASYON parametreleri (isim: (sim değeri, geri alma değeri)).
    advanced_plane (varsayılan) → SIM_ONLY_PARAMS, DEĞİŞMEDİ; tulpar → el_firlatma.TULPAR_SIM_ONLY (pist ayarları yok)."""
    if arac_modelleri.model_adi(uav_id) == "tulpar":
        return el_firlatma.TULPAR_SIM_ONLY
    return SIM_ONLY_PARAMS


def ucus_params_for(uav_id):
    """Modele özgü fırlatma/uçuş parametreleri (yalnız değer; geri alma yok). advanced_plane → boş."""
    if arac_modelleri.model_adi(uav_id) == "tulpar":
        return el_firlatma.TULPAR_UCUS_PARAMS
    return {}


def el_firlat(v: "Vehicle", alt: float, circle_alt: float, hiz: float, gecikme: float) -> bool:
    """Tulpar el fırlatma: gz yayıncıları hazırlanır → Takeoff + arm (launch-detect bekler) → `gecikme` sn sonra model
    1,5 m'ye ışınlanır ve sim süresiyle ölçülen darbe uygulanır. Araç zaten havadaysa atlanır. Başarıda True."""
    if v.armed and v.alt > 5.0:
        v.log(f"Zaten havada, el fırlatma atlandı — {v.status_line()}")
        return True
    darbe = el_firlatma.Darbe(model=arac_modelleri.gz_ad(v.uav_id), v_atis=hiz)
    darbe.hazirla()
    if not take_off(v, alt, circle_alt):
        return False
    time.sleep(gecikme)
    ok = darbe.isinla()
    sim_sure, duvar_sure = darbe.at()
    v.log(f"el fırlatma: ışınlama {'tamam' if ok else 'BAŞARISIZ'}, darbe a={hiz / darbe.t_atis:.1f} m/s² "
          f"(hedef v={hiz:.1f} m/s), sim süresi {sim_sure:.3f} s (duvar {duvar_sure:.3f} s)")
    return ok


ONBOARD_PORT_BASE = 14540   # PX4 instance i → udp 14540+i
LOITER_MODES = ("LOITER", "AUTO_LOITER", "HOLD")


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


def gz_rss_mb():
    """gz sim sunucusunun RSS belleği [MB]; bulunamazsa None."""
    pids = subprocess.run(["pgrep", "-f", "^gz sim .*-s"],
                          capture_output=True, text=True).stdout.split()
    if not pids:
        return None
    try:
        with open(f"/proc/{pids[0]}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) // 1024
    except FileNotFoundError:
        return None
    return None


class Vehicle:
    """Tek PX4 aracı: kalp atışı ve okuma iş parçacıkları, son durum alanları."""

    def __init__(self, uav_id: int):
        self.uav_id = uav_id
        self.name = f"uav{uav_id}"
        self.sysid = uav_id + 1
        self.url = f"udpin:0.0.0.0:{ONBOARD_PORT_BASE + uav_id}"
        self.m = mavutil.mavlink_connection(self.url, source_system=255, source_component=190)
        self.send_lock = threading.Lock()
        self.stop = threading.Event()

        self.alt = float("nan")       # Eve göre irtifa [m]
        self.airspeed = float("nan")  # [m/s]
        self.mode = "?"
        self.armed = False
        self.heartbeat_seen = threading.Event()
        self.texts = []               # (zaman, metin)
        self.acks = {}                # komut → sonuç
        self.params = {}              # isim → değer

        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        threading.Thread(target=self._read_loop, daemon=True).start()

    def log(self, msg: str) -> None:
        log(f"[{self.name}] {msg}")

    def _heartbeat_loop(self):
        while not self.stop.is_set():
            with self.send_lock:
                self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                          0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            time.sleep(1.0)

    def _read_loop(self):
        while not self.stop.is_set():
            msg = self.m.recv_match(blocking=True, timeout=1.0)
            if msg is None or msg.get_srcSystem() != self.sysid:
                continue
            kind = msg.get_type()
            if kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.mode = mavutil.mode_string_v10(msg)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.heartbeat_seen.set()
            elif kind == "GLOBAL_POSITION_INT":
                self.alt = msg.relative_alt / 1000.0
            elif kind == "VFR_HUD":
                self.airspeed = msg.airspeed
            elif kind == "STATUSTEXT":
                text = msg.text.rstrip("\x00")
                self.texts.append((time.time(), text))
                self.log(f"PX4 [sev {msg.severity}]: {text}")
            elif kind == "COMMAND_ACK":
                self.acks[msg.command] = msg.result
            elif kind == "PARAM_VALUE":
                value = msg.param_value
                # PX4 tamsayı parametreleri float alanında bayt bayt gönderir (2 → 2.8e-45).
                if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                    value = struct.unpack("<i", struct.pack("<f", value))[0]
                self.params[msg.param_id.rstrip("\x00")] = (value, msg.param_type)

    def get_param(self, name: str, timeout: float = 5.0):
        """(değer, tip) döndürür; zaman aşımında None."""
        self.params.pop(name, None)
        with self.send_lock:
            self.m.mav.param_request_read_send(self.sysid, 1, name.encode(), -1)
        return self._wait(lambda: self.params.get(name), timeout)

    def set_param(self, name: str, value: float, param_type: int, timeout: float = 5.0):
        """Parametreyi PX4'ün bildirdiği tiple yazar, okunan (değer, tip)'i döndürür."""
        wire = value
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            wire = struct.unpack("<f", struct.pack("<i", int(value)))[0]
        self.params.pop(name, None)
        with self.send_lock:
            self.m.mav.param_set_send(self.sysid, 1, name.encode(), wire, param_type)
        return self._wait(lambda: self.params.get(name), timeout)

    def ensure_param(self, name: str, value: float) -> bool:
        current = self.get_param(name)
        if current is None:
            self.log(f"HATA: {name} okunamadı")
            return False
        if current[0] == value:
            self.log(f"{name} = {value} (zaten ayarlı)")
            return True
        new = self.set_param(name, value, current[1])
        ok = new is not None and abs(new[0] - value) < 1e-3
        self.log(f"{name}: {current[0]} → {new[0] if new else '?'}" + ("" if ok else "  HATA: yazılamadı"))
        return ok

    def command(self, cmd: int, *params: float, timeout: float = 5.0):
        """COMMAND_LONG gönderir, COMMAND_ACK sonucunu döndürür (zaman aşımında None)."""
        p = list(params) + [0.0] * (7 - len(params))
        self.acks.pop(cmd, None)
        with self.send_lock:
            self.m.mav.command_long_send(self.sysid, 1, cmd, 0, *p)
        return self._wait(lambda: self.acks.get(cmd), timeout)

    @staticmethod
    def _wait(getter, timeout: float):
        end = time.time() + timeout
        while time.time() < end:
            value = getter()
            if value is not None:
                return value
            time.sleep(0.05)
        return None

    def status_line(self) -> str:
        rss = gz_rss_mb()
        return (f"irtifa={self.alt:7.1f} m  mod={self.mode:<12} "
                f"{'ARMED' if self.armed else 'disarmed':<8} hava_hızı={self.airspeed:5.1f} m/s  "
                f"gz_RSS={rss if rss is not None else '?'} MB")


def ack_name(result) -> str:
    if result is None:
        return "ACK yok (zaman aşımı)"
    return mavutil.mavlink.enums["MAV_RESULT"][result].name


def take_off(v: Vehicle, alt: float, circle_alt: float) -> bool:
    """Araç havadaysa atlar; değilse MIS_TAKEOFF_ALT + Takeoff + arm. Başarıda True."""
    if v.armed and v.alt > 5.0:
        v.log(f"Zaten havada, kalkış atlandı — {v.status_line()}")
        if v.alt < circle_alt:
            v.log(f"UYARI: irtifa {v.alt:.0f} m < turlama eşiği {circle_alt:.0f} m")
        return True

    if not v.ensure_param("MIS_TAKEOFF_ALT", alt):
        return False
    # PX4 v1.17: navigator parametre güncellemesini bir sonraki döngüde alıyor; hemen gelen NAV_TAKEOFF eski
    # değerle (25 m) kalkıyordu ("Using default takeoff altitude: 25.0 m", 2026-11-20). Kısa bekleme.
    time.sleep(2.0)

    # PX4 GCS sağlık kontrolü kalp atışımızı görmek için birkaç sn ister; arm reddedilirse yeniden dene.
    takeoff = arm = None
    for attempt in range(1, 6):
        # Önce Takeoff modu (param7 NaN → MIS_TAKEOFF_ALT), sonra arm.
        takeoff = v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
        arm = v.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        v.log(f"Deneme {attempt}: NAV_TAKEOFF → {ack_name(takeoff)}, ARM → {ack_name(arm)}")
        if arm == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            break
        time.sleep(2)
    if arm == mavutil.mavlink.MAV_RESULT_ACCEPTED and takeoff != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        takeoff = v.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, math.nan, math.nan, math.nan, math.nan)
        v.log(f"NAV_TAKEOFF (arm sonrası) → {ack_name(takeoff)}")
    if arm != mavutil.mavlink.MAV_RESULT_ACCEPTED or takeoff != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        v.log("HATA: arm veya kalkış kabul edilmedi")
        return False
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uav", type=int, nargs="+", default=[0], help="PX4 instance numaraları")
    ap.add_argument("--alt", type=float, nargs="+",
                    help="MIS_TAKEOFF_ALT [m]; araç başına bir değer ya da hepsi için tek değer "
                         "(varsayılan araç başına 100 + 20·uav: uav0 100, uav1 120)")
    ap.add_argument("--takeoff", type=int, nargs="+",
                    help="Kalkacak araçlar (varsayılan: --uav'daki hepsi); diğerlerine yalnızca "
                         "kalp atışı ve SIM_ONLY_PARAMS")
    ap.add_argument("--circle-alt", type=float, default=80.0, help="Turlama kabul eşiği [m]")
    ap.add_argument("--climb-timeout", type=float, default=300.0, help="Eşiğe ulaşma süresi sınırı [sn]")
    ap.add_argument("--watch", type=float, default=150.0, help="Turlama başladıktan sonra izleme [sn]")
    ap.add_argument("--ucus-dinamigi", action="store_true",
                    help="UCUS_DINAMIGI_PARAMS ayar değerlerini yaz (uav1 alçalma zarfı; dalis 12°). "
                         "Verilmezse geri alma değerleri yazılır")
    ap.add_argument("--el-firlatma", action="store_true",
                    help="Tulpar (UAV0_MODEL=tulpar): normal kalkış yerine el fırlatma (launch-detect + gz darbesi); yalnız --uav 0")
    ap.add_argument("--firlatma-hiz", type=float, default=12.0, help="el fırlatma hedef hızı [m/s]")
    ap.add_argument("--firlatma-gecikme", type=float, default=1.0, help="arm sonrası darbeye kadar bekleme [sn]")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--params-only", action="store_true", help="Yalnızca SIM_ONLY_PARAMS yaz ve çık")
    mode.add_argument("--revert-sim-params", action="store_true", help="SIM_ONLY_PARAMS'ı varsayılana döndür ve çık")
    args = ap.parse_args()

    if args.alt is None:
        alts = [100.0 + 20.0 * u for u in args.uav]
    elif len(args.alt) in (1, len(args.uav)):
        alts = args.alt * len(args.uav) if len(args.alt) == 1 else args.alt
    else:
        ap.error("--alt: tek değer ya da her araç için bir değer verin")
    takeoff_ids = args.uav if args.takeoff is None else args.takeoff
    if not set(takeoff_ids) <= set(args.uav):
        ap.error("--takeoff: yalnızca --uav'da verilen araçlar")

    if args.el_firlatma:
        if takeoff_ids != [0] or args.uav != [0]:
            ap.error("--el-firlatma yalnız --uav 0 ile (uav1'in el fırlatması yok)")
        if arac_modelleri.model_adi(0) != "tulpar":
            ap.error("--el-firlatma için UAV0_MODEL=tulpar gerekli (launch_all.sh yalnız kendi alt süreçlerine verir; bu betik ayrı kabukta çalışıyorsa: UAV0_MODEL=tulpar python3 mavlink_takeoff.py ...)")

    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))

    vehicles = []
    for uav_id in args.uav:
        v = Vehicle(uav_id)
        v.log(f"Bağlanılıyor: {v.url} (sysid {v.sysid}), GCS kalp atışı 1 Hz gönderiliyor")
        if not v.heartbeat_seen.wait(15):
            v.log("HATA: 15 sn içinde PX4 kalp atışı gelmedi")
            return 1
        v.log(f"PX4 kalp atışı alındı — {v.status_line()}")
        vehicles.append(v)

    # Simülasyona özel parametreler (tek yer: SIM_ONLY_PARAMS)
    for v in vehicles:
        for name, (sim_value, revert_value) in sim_params_for(v.uav_id).items():
            target = revert_value if args.revert_sim_params else sim_value
            if not v.ensure_param(name, target):
                return 1
        if not args.revert_sim_params:   # modele özgü fırlatma/uçuş ayarları (advanced_plane: boş → davranış aynı)
            for name, value in ucus_params_for(v.uav_id).items():
                if not v.ensure_param(name, value):
                    return 1
    # Uçuş dinamiği ayarları (SIM_ONLY_PARAMS'tan ayrı; yalnızca listelenen araç)
    for uav_id, v in zip(args.uav, vehicles):
        for name, (tuned_value, revert_value) in UCUS_DINAMIGI_PARAMS.get(uav_id, {}).items():
            target = tuned_value if args.ucus_dinamigi and not args.revert_sim_params else revert_value
            if not v.ensure_param(name, target):
                return 1
            v.log(f"uçuş dinamiği: {name} = {target} ({'AYAR' if target == tuned_value else 'geri alma / varsayılan'})")
    if args.params_only or args.revert_sim_params:
        log("Parametreler yazıldı, çıkılıyor (kalkış yok).")
        return 0

    flying = []
    for uav_id, v, alt in zip(args.uav, vehicles, alts):
        if uav_id not in takeoff_ids:
            v.log("Kalkış yok (--takeoff dışında); yalnızca kalp atışı")
            continue
        if args.el_firlatma:
            ok = el_firlat(v, alt, args.circle_alt, args.firlatma_hiz, args.firlatma_gecikme)
        else:
            ok = take_off(v, alt, args.circle_alt)
        if not ok:
            return 1
        flying.append(v)

    # Tırmanış: kalkan araçlar eşiğin üstünde ve LOITER modunda olana kadar 5 sn'de bir durum.
    start = time.time()
    while time.time() - start < args.climb_timeout:
        for v in vehicles:
            v.log(v.status_line())
        if all(v.alt >= args.circle_alt and v.mode in LOITER_MODES for v in flying):
            break
        time.sleep(5)
    else:
        log(f"HATA: {args.climb_timeout:.0f} sn içinde kalkan araçlar {args.circle_alt:.0f} m üstünde turlamaya geçmedi")
        return 1
    circling_since = time.time()
    log(f"Turlama başladı ({circling_since - start:.0f} sn sonra) — {args.watch:.0f} sn izleniyor")

    while time.time() - circling_since < args.watch:
        time.sleep(5)
        for v in vehicles:
            v.log(v.status_line())
    log("İzleme tamamlandı. Kalp atışı sürüyor (durdurmak için SIGTERM / Ctrl+C); 30 sn'de bir durum.")

    while True:
        time.sleep(30)
        for v in vehicles:
            v.log(v.status_line())


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        log("Durduruldu — kalp atışı kesildi")
