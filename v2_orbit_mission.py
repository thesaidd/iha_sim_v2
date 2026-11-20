#!/usr/bin/env python3
"""
v2_orbit_mission.py  (Faz 1 v2 — Mid-Air Takeover / Havada Devralma)
============================================================
Hedef platform: PX4 advanced_plane (SITL) — hover yapamaz, sürekli ileri hareket ister.

Durum Makinesi:
  WAITING   → Kalkış mavlink_takeoff.py / QGC ile yapılır; düğüm orbit setpoint akışı yapar
  ENGAGING  → İrtifa ≥ takeover_min_alt_m ve araç arm edilmiş: DO_SET_MODE(OFFBOARD) gönderilir,
              PX4 Offboard'a geçene kadar offboard_retry_s'de bir yeniden denenir
  ORBIT     → Offboard aktif; çember üzerinde hareketli setpoint
  EXITED    → Offboard'dan çıkıldı (orbit_abort, failsafe, operatör); yeniden girilmez,
              setpoint yayını durur, durum loglanır
  FAILED    → offboard_max_attempts denemede Offboard'a geçilemedi; yayın durur

Orbit (saat yönünde, PX4 Hold dönüşüyle aynı yön), merkez (orbit_center_n, orbit_center_e),
yarıçap R, hız V. İki setpoint modu (--mode):

  rabbit (varsayılan) — tavşan noktası, zamandan bağımsız:
    φ = uçağın merkeze göre açısı (her döngüde yerel konumdan), θ = φ + lookahead_deg
    lookahead_deg varsayılanı ≈ 4 sn'lik yol: 4·V/R rad (V=15, R=100 → 34,4°).
    Hız farkı ve zaman damgası sıçramalarından etkilenmez; iki araç arası 180° aralık
    bu modda korunmaz (ayrı aralık denetimi gerekir).
    Not: PX4 yol takibi (NPFG) verilen noktayı yola en yakın nokta sayar; lookahead > 0
    iken uçağı R/cos(lookahead) yarıçapına doğru çekebilir. lookahead_deg:=0 = en yakın nokta.

  time — eski zaman tabanlı mod:
    θ = ω·t_sim + φ_ref + uav_id·π   ← uav1 aynı çemberde 180° karşıda (çarpışma önleme)
    t_sim: PX4 zaman damgası. φ_ref (phase_ref_deg): verilmezse devralmada uçağın yönüne
    hizalanır ve loglanır. Uyarı: uçak geride kalınca dışa kayıyor (kararsız) ve ROS'a gelen
    zaman damgaları periyodik duraklamalarda ~3 sn atlıyor (2026-10-29).

Mod bilgisi: vehicle_status_v2 KULLANILMIYOR — px4_msgs'deki VehicleStatus (v3) PX4'ün
gönderdiğiyle (v2, +failure_detector_status) uyuşmuyor; mesajlar Fast-DDS'te 88/87 bayt
hatasıyla düşüyor. Arm ve Offboard durumu vehicle_control_mode'dan okunur
(flag_control_offboard_enabled ⇔ nav_state OFFBOARD). px4_msgs PX4 ile eşlenince
vehicle_status'a dönülebilir.

Setpoint: konum + çembere teğet hız + merkezcil ivme. PX4 sabit kanatta hız sonluysa
yol takibi (AUTO_PATH) yapar ve ivmeden eğrilik yarıçapını çıkarır. Kullanılmayan
alanlar NaN olmalı: ROS mesajındaki varsayılan [0,0,0] PX4'te "sıfır hızlı yol" sayılır.

Parametreler (--ros-args -p ad:=değer):
  uav_id (0)               orbit_alt_m (100 + 20·uav_id → uav0 100 m, uav1 120 m)
  orbit_center_n (0.0)     orbit_center_e (0.0)        orbit_radius_m (100.0)
  orbit_speed_ms (15.0)    takeover_min_alt_m (40.0)   feedforward (true)
  offboard_retry_s (2.0)   offboard_max_attempts (10)  log_period_s (5.0)
  lookahead_deg (NaN → 4·V/R)                          [rabbit]
  phase_ref_deg (NaN → devralmada uçağın yönüne hizala) [time]
  center_frame_uav (0)     orbit_center_n/e bu aracın yerel çerçevesinde verilir. Her PX4 kendi
                           doğduğu noktayı orijin alır (uav1, uav0'dan N+15 E−15 m); düğüm iki
                           aracın global referansından kendi çerçevesine çevirir. Çevrilene kadar
                           devralma yapılmaz.

Giriş: önce orbit_approach.py (merkezde Hold, teğet uçuş), sonra bu düğüm.

Kullanım:
  python3 v2_orbit_mission.py --ros-args -p uav_id:=0              # keşif, tavşan modu
  python3 v2_orbit_mission.py --ros-args -p uav_id:=1              # kamikaze
  python3 v2_orbit_mission.py --mode time --ros-args -p uav_id:=0  # eski zaman tabanlı mod
Acil durum: ~/iha_sim_v2/orbit_abort [uav_id ...]  → Hold (AUTO_LOITER)

Koordinat sistemi: PX4 NED
  X → Kuzey | Y → Doğu | Z → Aşağı (negatif = yukarı)

Bağımlılıklar:
  - ROS 2 Humble
  - px4_msgs  (https://github.com/PX4/px4_msgs)
"""

import argparse
import math
import sys
import time
import logging
import pathlib
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from px4_msgs.msg import (
    AirspeedValidated,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
)

# ──────────────────────────────────────────────────────────────────────────────
# Log altyapısı
# ──────────────────────────────────────────────────────────────────────────────
LOG_DIR = pathlib.Path("test/test-1")


def setup_file_logger(uav_id: int) -> logging.Logger:
    """
    Her İHA için ayrı bir dosya loggerı oluşturur.
    Çıkış: test/test-1/uav{uav_id}_mission.log
    """
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / f"uav{uav_id}_mission.log"

    logger = logging.getLogger(f"uav{uav_id}_mission")
    logger.setLevel(logging.DEBUG)

    if not logger.handlers:   # çoklu init'te handler çiftlenmesini önle
        fh = logging.FileHandler(log_path, mode="a", encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(logging.Formatter(
            fmt="%(asctime)s  %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        ))
        logger.addHandler(fh)

    return logger

# ──────────────────────────────────────────────────────────────────────────────
# Sabitler
# ──────────────────────────────────────────────────────────────────────────────
LOOP_HZ              = 20      # Ana döngü frekansı [Hz]
NAN3                 = [math.nan] * 3

# PX4 Publisher QoS (Offboard komutlar — TRANSIENT_LOCAL)
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Sensor / Telemetri Subscriber QoS (PX4 sensör topic'leri — VOLATILE)
# PX4 DDS Agent sensör verilerini BEST_EFFORT + VOLATILE ile yayınlar.
# TRANSIENT_LOCAL ile abone olmak mesajın reddedilmesine neden olur.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

def control_mode_name(msg) -> str:
    """vehicle_control_mode bayraklarından kaba mod adı (AUTO = Hold/RTL/Mission/Takeoff/Land)."""
    if msg is None:
        return "?"
    if msg.flag_control_offboard_enabled:
        return "OFFBOARD"
    if msg.flag_control_auto_enabled:
        return "AUTO"
    if msg.flag_control_position_enabled:
        return "POSCTL"
    if msg.flag_control_altitude_enabled:
        return "ALTCTL"
    if msg.flag_control_manual_enabled:
        return "MANUAL"
    return "DİĞER"


# ──────────────────────────────────────────────────────────────────────────────
# Durum etiketleri
# ──────────────────────────────────────────────────────────────────────────────
class State:
    WAITING  = "WAITING"    # İrtifa + arm bekleniyor; setpoint akışı sürüyor
    ENGAGING = "ENGAGING"   # DO_SET_MODE(OFFBOARD) gönderiliyor, PX4 onayı bekleniyor
    ORBIT    = "ORBIT"      # Offboard modu aktif; hareketli dairesel setpoint
    EXITED   = "EXITED"     # Offboard'dan çıkıldı; yeniden girilmez
    FAILED   = "FAILED"     # Offboard'a geçilemedi


# ──────────────────────────────────────────────────────────────────────────────
# PX4 SITL DDS Topic Prefix Yardımcısı
# ──────────────────────────────────────────────────────────────────────────────

def get_fmu_prefix(uav_id: int) -> str:
    """
    PX4 SITL (MicroXRCE-DDS Agent) çoklu İHA topic namespace kuralı:
      uav_id == 0  →  /fmu            (varsayılan instance)
      uav_id >= 1  →  /px4_{uav_id}/fmu

    Örnekler:
      get_fmu_prefix(0)  →  "/fmu"
      get_fmu_prefix(1)  →  "/px4_1/fmu"
    """
    if uav_id == 0:
        return "/fmu"
    return f"/px4_{uav_id}/fmu"


EARTH_RADIUS_M = 6371000.0


def ref_offset_ne(lat: float, lon: float, ref_lat: float, ref_lon: float):
    """(lat, lon) noktasının (ref_lat, ref_lon) orijinli yerel çerçevedeki (N, E) konumu [m]."""
    north = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    east = math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    return north, east


def ne_to_latlon(north: float, east: float, ref_lat: float, ref_lon: float):
    """ref_offset_ne'nin tersi: yerel (N, E) → (lat, lon) [derece]."""
    lat = ref_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat))))
    return lat, lon


def make_vehicle_command(uav_id: int, timestamp_us: int, command: int, *params: float) -> VehicleCommand:
    """
    SysID alanları dolu VehicleCommand; params → param1..param7 (verilmeyenler 0).

    SysID Kuralları (PX4 çoklu İHA):
      target_system   = uav_id + 1   → uav0→1, uav1→2
                        PX4 SITL instance ID'leri 1'den başlar.
                        Yanlış ID'li komutlar sessizce görmezden gelinir!
      target_component = 1           → Otopilot bileşeni
      source_system   = 255          → GCS / Dış Bilgisayar kimliği
      source_component = 1
      from_external   = True         → Komutun dışarıdan geldiğini bildirir
      timestamp       = mikrosaniye (PX4 beklentisi)
    """
    msg = VehicleCommand()
    msg.timestamp        = timestamp_us
    msg.command          = command
    for i, value in enumerate(params, start=1):
        setattr(msg, f"param{i}", float(value))
    msg.target_system    = uav_id + 1
    msg.target_component = 1
    msg.source_system    = 255
    msg.source_component = 1
    msg.from_external    = True
    return msg


def circle_setpoint(theta: float, center_n: float, center_e: float, radius: float,
                    speed: float, altitude: float, feedforward: bool):
    """
    Çemberin θ açısındaki setpoint'i: (konum, hız, ivme), NED.
    Konum (N, E) = merkez + R·(cos θ, sin θ); hareket θ artan yönde (saat yönü).
    """
    position = [center_n + radius * math.cos(theta),
                center_e + radius * math.sin(theta),
                -altitude]
    if not feedforward:
        return position, NAN3, NAN3
    velocity = [-speed * math.sin(theta), speed * math.cos(theta), 0.0]
    accel = speed * speed / radius
    acceleration = [-accel * math.cos(theta), -accel * math.sin(theta), 0.0]
    return position, velocity, acceleration


def orbit_setpoint(t_sim: float, center_n: float, center_e: float, radius: float,
                   speed: float, altitude: float, phase: float, feedforward: bool):
    """Zaman tabanlı mod: t_sim anındaki (θ, konum, hız, ivme); θ, ω = V/R ile artar."""
    theta = math.fmod(speed / radius * t_sim, 2.0 * math.pi) + phase
    return (theta,) + circle_setpoint(theta, center_n, center_e, radius, speed, altitude, feedforward)


def rabbit_setpoint(north: float, east: float, center_n: float, center_e: float, radius: float,
                    speed: float, altitude: float, lookahead: float, feedforward: bool):
    """Tavşan modu: uçağın açısı φ + lookahead'deki (θ, konum, hız, ivme)."""
    phi = math.atan2(east - center_e, north - center_n)
    if not math.isfinite(phi):
        phi = 0.0   # konum henüz yok; Offboard'dan önce yalnızca akış için
    theta = phi + lookahead
    return (theta,) + circle_setpoint(theta, center_n, center_e, radius, speed, altitude, feedforward)


class FixedWingOrbitMission(Node):
    """
    Sabit kanatlı İHA için Faz-1 görev düğümü.

    uav_id = 0  →  /fmu/...          faz ofseti = 0
    uav_id = 1  →  /px4_1/fmu/...    faz ofseti = π  (180° geride, tam karşı tarafta)
    """

    def __init__(self, mode: str = "rabbit"):
        super().__init__("fixed_wing_orbit_mission")
        self.mode = mode

        # ── Parametreler ─────────────────────────────────────────────────────
        # dynamic_typing: "-p orbit_alt_m:=120" gibi tamsayı değerler de kabul edilsin
        def param(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(dynamic_typing=True)).value

        self.uav_id: int = int(param("uav_id", 0))
        ns = get_fmu_prefix(self.uav_id)

        self.orbit_alt      = float(param("orbit_alt_m", 100.0 + 20.0 * self.uav_id))
        self.center_param   = (float(param("orbit_center_n", 0.0)), float(param("orbit_center_e", 0.0)))
        self.center_frame   = int(param("center_frame_uav", 0))
        # Kendi çerçevemizdeki merkez; center_frame başka araçsa referanslar gelince düzeltilir
        self.center_n, self.center_e = self.center_param
        self.center_ready   = self.center_frame == self.uav_id
        self.own_ref        = None     # (ref_lat, ref_lon)
        self.frame_ref      = None
        self.radius         = float(param("orbit_radius_m", 100.0))
        self.speed          = float(param("orbit_speed_ms", 15.0))
        self.takeover_alt   = float(param("takeover_min_alt_m", 40.0))
        self.feedforward    = bool(param("feedforward", True))
        self.retry_s        = float(param("offboard_retry_s", 2.0))
        self.max_attempts   = int(param("offboard_max_attempts", 10))
        self.log_period     = float(param("log_period_s", 5.0))
        self.phase_ref      = math.radians(float(param("phase_ref_deg", math.nan)))  # NaN → hizala [time]
        lookahead_deg       = float(param("lookahead_deg", math.nan))                 # NaN → 4 sn'lik yol [rabbit]
        if math.isnan(lookahead_deg):
            lookahead_deg = math.degrees(4.0 * self.speed / self.radius)
        self.lookahead      = math.radians(lookahead_deg)

        self.phase_offset: float = math.pi * self.uav_id  # 0 veya π

        # ── Dosya loggerı başlat ──────────────────────────────────────────
        self._flog = setup_file_logger(self.uav_id)

        if self.mode == "rabbit":
            mode_info = (f"mod=rabbit lookahead={math.degrees(self.lookahead):.1f}° "
                         f"(≈{self.lookahead * self.radius:.0f} m yay, "
                         f"{self.lookahead * self.radius / self.speed:.1f} sn)")
        else:
            phase_ref = ("devralmada hizalanacak" if math.isnan(self.phase_ref)
                         else f"{math.degrees(self.phase_ref):.1f}°")
            mode_info = f"mod=time faz={math.degrees(self.phase_offset):.0f}° φ_ref={phase_ref}"
        self._log(
            f"[uav{self.uav_id}] Başlatıldı — "
            f"merkez=({self.center_param[0]:.0f}, {self.center_param[1]:.0f}) [uav{self.center_frame} çerçevesi] "
            f"R={self.radius:.0f} m "
            f"V={self.speed:.1f} m/s irtifa={self.orbit_alt:.0f} m | {mode_info} | "
            f"devralma: irtifa ≥ {self.takeover_alt:.0f} m + arm | "
            f"feedforward={'açık' if self.feedforward else 'kapalı'} | "
            f"FMU prefix: {ns} | Log: {LOG_DIR}/uav{self.uav_id}_mission.log"
        )

        # ── İç durum ─────────────────────────────────────────────────────────
        self.state          = State.WAITING
        self.control_mode   = None     # Son VehicleControlMode (henüz gelmediyse None)
        self.armed          = False
        self.offboard       = False    # PX4 Offboard'da mı
        self.local_x        = math.nan  # VehicleLocalPosition [m NED]
        self.local_y        = math.nan
        self.local_z        = math.nan
        self.t_sim          = None     # Son konum mesajının PX4 zamanı [s]
        self.airspeed       = math.nan  # Kalibre hava hızı [m/s]
        self.attempts       = 0        # Offboard deneme sayısı
        self.last_attempt_t = None     # Son deneme [monotonic s]
        self.last_log_t     = 0.0      # Son periyodik log [monotonic s]

        # ── Publisher'lar ────────────────────────────────────────────────────
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            f"{ns}/in/offboard_control_mode",
            PX4_QOS,
        )
        self.pub_trajectory = self.create_publisher(
            TrajectorySetpoint,
            f"{ns}/in/trajectory_setpoint",
            PX4_QOS,
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand,
            f"{ns}/in/vehicle_command",
            PX4_QOS,
        )

        # ── Subscriber'lar ───────────────────────────────────────────────────
        # NOT: PX4 DDS Agent sensör topic'lerini _v1/_v2 suffix ile yayınlar.
        #      SENSOR_QOS (BEST_EFFORT + VOLATILE) zorunludur; aksi halde
        #      ROS 2 QoS uyuşmazlığı nedeniyle mesajlar reddedilir.
        self.sub_control_mode = self.create_subscription(
            VehicleControlMode,
            f"{ns}/out/vehicle_control_mode",       # ← sürüm 0, suffix yok
            self._control_mode_cb,
            SENSOR_QOS,                              # ← VOLATILE
        )
        self.sub_local_pos = self.create_subscription(
            VehicleLocalPosition,
            f"{ns}/out/vehicle_local_position_v1",  # ← _v1 suffix
            self._local_position_cb,
            SENSOR_QOS,                              # ← VOLATILE
        )
        self.sub_airspeed = self.create_subscription(
            AirspeedValidated,
            f"{ns}/out/airspeed_validated_v1",      # ← _v1 suffix
            self._airspeed_cb,
            SENSOR_QOS,
        )
        if not self.center_ready:
            self.sub_frame_pos = self.create_subscription(
                VehicleLocalPosition,
                f"{get_fmu_prefix(self.center_frame)}/out/vehicle_local_position_v1",
                self._frame_position_cb,
                SENSOR_QOS,
            )

        # ── Ana döngü ────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / LOOP_HZ, self._timer_cb)

    # ──────────────────────────────────────────────────────────────────────────
    # Log yardımcısı
    # ──────────────────────────────────────────────────────────────────────────

    def _log(self, msg: str, warn: bool = False) -> None:
        """
        Mesajı hem ROS 2 logger'a hem de dosyaya (uav{id}_mission.log) yazar.
        Kod içindeki tüm get_logger() çağrıları bu metotla yapılır.
        """
        if warn:
            self.get_logger().warning(msg)
            self._flog.warning(msg)
        else:
            self.get_logger().info(msg)
            self._flog.info(msg)

    def _control_mode_cb(self, msg: VehicleControlMode):
        self.control_mode = msg
        self.armed = msg.flag_armed
        self.offboard = msg.flag_control_offboard_enabled

    def _local_position_cb(self, msg: VehicleLocalPosition):
        """Anlık NED konumunu ve PX4 zamanını günceller."""
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z  # Negatif = yukarıda
        self.t_sim = msg.timestamp * 1e-6
        if msg.xy_global and self.own_ref is None:
            self.own_ref = (msg.ref_lat, msg.ref_lon)
            self._resolve_center()

    def _frame_position_cb(self, msg: VehicleLocalPosition):
        if msg.xy_global and self.frame_ref is None:
            self.frame_ref = (msg.ref_lat, msg.ref_lon)
            self._resolve_center()

    def _resolve_center(self):
        """Merkezi center_frame aracının çerçevesinden kendi yerel çerçevemize çevirir (bir kez)."""
        if self.center_ready or self.own_ref is None or self.frame_ref is None:
            return
        off_n, off_e = ref_offset_ne(*self.own_ref, *self.frame_ref)   # kendi orijinimiz, onun çerçevesinde
        self.center_n = self.center_param[0] - off_n
        self.center_e = self.center_param[1] - off_e
        self.center_ready = True
        self._log(
            f"[uav{self.uav_id}] Merkez çerçevesi: orijinimiz uav{self.center_frame} çerçevesinde "
            f"N={off_n:+.1f} E={off_e:+.1f} m → kendi çerçevemizde merkez=({self.center_n:+.1f}, {self.center_e:+.1f})"
        )

    def _airspeed_cb(self, msg: AirspeedValidated):
        self.airspeed = msg.calibrated_airspeed_m_s

    # ──────────────────────────────────────────────────────────────────────────
    # Ana döngü
    # ──────────────────────────────────────────────────────────────────────────

    def _timer_cb(self):
        """
        Mid-Air Takeover döngüsü (20 Hz):
          1. WAITING/ENGAGING/ORBIT'te OffboardControlMode (kalp atışı) ve orbit
             setpoint'i her döngüde yayınlanır; PX4 Offboard'a geçmeden önce
             setpoint akışı görmek ister.
          2. WAITING: irtifa ≥ takeover_min_alt_m ve arm → ENGAGING.
          3. ENGAGING: PX4 Offboard'a geçene kadar komut yeniden gönderilir.
          4. ORBIT: Offboard'dan çıkılırsa EXITED; yeniden girilmez.
        Mod/arm bilgisi vehicle_control_mode'dan gelir (bkz. dosya başı).
        """
        if self.state in (State.EXITED, State.FAILED):
            return

        now = time.monotonic()
        periodic = now - self.last_log_t >= self.log_period
        if periodic:
            self.last_log_t = now

        # ─ Daima: Offboard kalp atışı + orbit setpoint ────────────────────
        self._publish_offboard_control_mode()
        theta = self._publish_orbit_setpoint()

        altitude = -self.local_z
        mode = control_mode_name(self.control_mode)

        # ─ WAITING: irtifa + arm bekle ────────────────────────────────────
        if self.state == State.WAITING:
            if altitude >= self.takeover_alt and self.armed and self.center_ready:
                self._log(
                    f"[uav{self.uav_id}] ✅ Devralma koşulları sağlandı "
                    f"(irtifa={altitude:.1f} m, arm, mod={mode}). "
                    f"Görev Bilgisayarı kontrolü (OFFBOARD) devralıyor!"
                )
                if self.mode == "time" and math.isnan(self.phase_ref):
                    self._align_phase()
                    theta = self._publish_orbit_setpoint()   # Offboard komutundan önce hizalı setpoint
                self.state = State.ENGAGING
                self.attempts = 0
                self.last_attempt_t = None
            else:
                if periodic:
                    self._log(
                        f"[uav{self.uav_id}] Bekleniyor — irtifa={altitude:.1f} m "
                        f"(hedef ≥ {self.takeover_alt:.0f} m), "
                        f"{'arm' if self.armed else 'arm DEĞİL'}, mod={mode}"
                        + ("" if self.center_ready else f", merkez çerçevesi (uav{self.center_frame}) bekleniyor")
                    )
                return

        # ─ ENGAGING: PX4 Offboard'a geçene kadar yeniden dene ────────────
        if self.state == State.ENGAGING:
            if self.offboard:
                self._log(f"[uav{self.uav_id}] ✈ OFFBOARD aktif ({self.attempts}. denemede) — orbit başladı")
                self.state = State.ORBIT
            elif not self.armed or altitude < self.takeover_alt:
                self._log(
                    f"[uav{self.uav_id}] Devralma koşulu kayboldu (irtifa={altitude:.1f} m, "
                    f"{'arm' if self.armed else 'arm DEĞİL'}) — beklemeye dönülüyor", warn=True)
                self.state = State.WAITING
            elif self.last_attempt_t is None or now - self.last_attempt_t >= self.retry_s:
                if self.attempts >= self.max_attempts:
                    self._log(
                        f"[uav{self.uav_id}] ❌ {self.max_attempts} denemede OFFBOARD'a geçilemedi "
                        f"(mod={mode}) — setpoint yayını durduruldu", warn=True)
                    self.state = State.FAILED
                    return
                self.attempts += 1
                self.last_attempt_t = now
                self._send_offboard_mode_cmd()
            return

        # ─ ORBIT: Offboard'dan çıkılırsa logla, yeniden girme ─────────────
        if not self.offboard:
            self._log(
                f"[uav{self.uav_id}] ⚠ OFFBOARD'dan çıkıldı: mod={mode}, irtifa={altitude:.1f} m. "
                f"Yeniden girilmeyecek; setpoint yayını durduruldu.", warn=True)
            self.state = State.EXITED
            return

        if periodic:
            dist = math.hypot(self.local_x - self.center_n, self.local_y - self.center_e)
            phi = math.atan2(self.local_y - self.center_e, self.local_x - self.center_n)
            lead = math.degrees(math.remainder(theta - phi, 2.0 * math.pi))
            self._log(
                f"[uav{self.uav_id}] ORBIT θ={math.degrees(theta) % 360:6.1f}°  setpoint−uçak={lead:+6.1f}°  "
                f"irtifa={altitude:6.1f} m  merkeze={dist:6.1f} m  "
                f"yarıçap_hatası={dist - self.radius:+6.1f} m  hava_hızı={self.airspeed:5.1f} m/s"
            )


    def _align_phase(self):
        """φ_ref'i, şu anki θ setpoint'i uçağın merkeze göre yönüne denk gelecek şekilde seçer."""
        bearing = math.atan2(self.local_y - self.center_e, self.local_x - self.center_n)
        omega_t = math.fmod(self.speed / self.radius * (self.t_sim or 0.0), 2.0 * math.pi)
        self.phase_ref = math.remainder(bearing - omega_t - self.phase_offset, 2.0 * math.pi)
        self._log(
            f"[uav{self.uav_id}] Faz uçağa hizalandı: yön={math.degrees(bearing) % 360:.1f}°, "
            f"φ_ref={math.degrees(self.phase_ref):.1f}° "
            f"(diğer araç için: -p phase_ref_deg:={math.degrees(self.phase_ref):.1f})"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Yardımcı yayın fonksiyonları
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_offboard_control_mode(self):
        """
        Offboard modunu canlı tutmak için zorunlu heartbeat mesajı.
        Kesilirse PX4 ~0.5 s içinde Offboard'dan çıkar.
        """
        msg = OffboardControlMode()
        msg.timestamp    = self._ts()
        msg.position     = True   # Pozisyon kontrolü
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self.pub_offboard_mode.publish(msg)

    def _publish_orbit_setpoint(self) -> float:
        """Şu anki orbit setpoint'ini yayınlar, θ'yı döndürür (time: φ_ref henüz yoksa 0 alınır)."""
        if self.mode == "rabbit":
            theta, position, velocity, acceleration = rabbit_setpoint(
                self.local_x, self.local_y, self.center_n, self.center_e, self.radius,
                self.speed, self.orbit_alt, self.lookahead, self.feedforward)
        else:
            phase = (0.0 if math.isnan(self.phase_ref) else self.phase_ref) + self.phase_offset
            theta, position, velocity, acceleration = orbit_setpoint(
                self.t_sim or 0.0, self.center_n, self.center_e, self.radius,
                self.speed, self.orbit_alt, phase, self.feedforward)
        self._publish_setpoint(position, velocity, acceleration)
        return theta

    def _publish_setpoint(self, position, velocity, acceleration):
        """NED konum + (isteğe bağlı) hız/ivme setpoint'i; kullanılmayan alanlar NaN."""
        msg = TrajectorySetpoint()
        msg.timestamp    = self._ts()
        msg.position     = [float(v) for v in position]
        msg.velocity     = [float(v) for v in velocity]
        msg.acceleration = [float(v) for v in acceleration]
        msg.jerk         = NAN3
        msg.yaw          = math.nan
        msg.yawspeed     = math.nan
        self.pub_trajectory.publish(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # VehicleCommand yardımcısı — SysID kuralları
    # ──────────────────────────────────────────────────────────────────────────

    def _ts(self) -> int:
        """
        PX4'ün beklediği mikrosaniye cinsinden zaman damgası.
        ROS saatini kullanır — _publish_* ve _send_offboard_mode_cmd() için ortak kaynak.
        """
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_offboard_mode_cmd(self):
        """Offboard mod komutu — SysID doğru; ENGAGING'de retry_s'de bir çağrılır."""
        msg = make_vehicle_command(
            self.uav_id, self._ts(), VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            6.0)    # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        self.pub_vehicle_cmd.publish(msg)
        self._log(
            f"[uav{self.uav_id}] ✈ DO_SET_MODE → OFFBOARD | deneme {self.attempts}/{self.max_attempts} | "
            f"mod={control_mode_name(self.control_mode)} | target_system={msg.target_system}"
        )


# ──────────────────────────────────────────────────────────────────────────────
# Giriş noktası
# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    parser = argparse.ArgumentParser(description="Sabit kanat orbit görev düğümü (bkz. dosya başı)")
    parser.add_argument("--mode", choices=("rabbit", "time"), default="rabbit",
                        help="Setpoint modu: rabbit = tavşan noktası (varsayılan), time = eski zaman tabanlı")
    cli = parser.parse_args(remove_ros_args(sys.argv if args is None else args)[1:])

    rclpy.init(args=args)
    node = FixedWingOrbitMission(cli.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # rclpy.spin() zaten Ctrl+C'yi yakalar; burada sadece kullanıcıya bilgi.
        node.get_logger().info(f"[uav{node.uav_id}] Kullanıcı tarafından durduruldu.")
    finally:
        # destroy_node güvenlidir; shutdown yalnızca rclpy hâlâ aktifse çağrılır.
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
