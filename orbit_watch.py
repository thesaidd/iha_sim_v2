#!/usr/bin/env python3
"""
orbit_watch.py — Orbit testini PX4 topic'lerinden bağımsız izler, gerekirse orbit_abort çağırır.

Bir ya da birden çok aracı izler (--uav 0 1). Konumlar --frame-uav aracının yerel çerçevesine
çevrilir (her PX4 kendi doğduğu noktayı orijin alır; uav1, uav0'dan N+15 E−15 m).
İzlenen tüm araçlar OFFBOARD'a geçince süre başlar; her --period sn'de araç başına mod, irtifa,
merkeze uzaklık, yarıçap hatası, hava hızı, yer hızı, tavşan−uçak açı farkı ve gz sim belleği,
iki araçta ayrıca çember açıları farkı (ikinci − birinci, 0–360°) ve 3B mesafe yazılır.

Tavşan−uçak açı farkı: aracın yayınlanan trajectory_setpoint'inin merkeze göre açısı − aracın
açısı (−180…180). Tavşan modunda ≈ lookahead_deg'de sabit kalmalı.

Kurallar (0,5 sn'de bir denetlenir):
  Araç başına, her zaman: |irtifa − hedef| > --alt-tol (20 m), Offboard'dan beklenmeyen çıkış
               (failsafe ya da dış komut), iniş algılanması
  Yakalama:    |yarıçap hatası| < --capture-tol (30 m), --capture-hold (10 sn) kesintisiz
  Yakalamadan önce: --capture-timeout (90 sn) içinde yakalanmazsa abort
  Yakalamadan sonra: |yarıçap hatası| > --track-tol (40 m) ise abort
  Ortak:       gz RSS > başlangıç + --mem-growth-mb; iki araç arası 3B mesafe < --min-separation
Herhangi bir abort → --abort-uavs'taki tüm araçlar Hold (varsayılan: izlenen araçlar).
Duraklama: izleyicinin kendi döngüsünde > --stall-gap sn boşluk; özet, her duraklamanın
öncesindeki ve sonraki 5 sn içindeki yarıçap hatasını karşılaştırır.
SIGTERM: izleme durur, özet basılır, Hold verilmez.

Mod bilgisi vehicle_control_mode'dan okunur; vehicle_status_v2 px4_msgs uyumsuzluğu
nedeniyle ROS'a ulaşmıyor (bkz. v2_orbit_mission.py). Failsafe ayrıntısı (RTL/Land)
mavlink_takeoff.py logundaki PX4 STATUSTEXT satırlarında görülür.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 orbit_watch.py --uav 0 --duration 240
  python3 orbit_watch.py --uav 0 1 --duration 240
Çıkış kodu: 0 = başarılı, 2 = abort, 3 = Offboard'a geçilmedi, 5 = SIGTERM ile durduruldu.
"""

import argparse
import math
import os
import signal
import subprocess
import time
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from px4_msgs.msg import (AirspeedValidated, TrajectorySetpoint, VehicleControlMode, VehicleLandDetected,
                          VehicleLocalPosition)

from mavlink_takeoff import gz_rss_mb
from v2_orbit_mission import SENSOR_QOS, control_mode_name, get_fmu_prefix, ref_offset_ne

HERE = os.path.dirname(os.path.abspath(__file__))


class Watch(Node):
    def __init__(self, uav_ids, frame_uav: int):
        super().__init__("orbit_watch_" + "_".join(f"uav{u}" for u in uav_ids))
        self.v = {}
        for u in sorted({*uav_ids, frame_uav}):
            st = SimpleNamespace(mode=None, pos=None, airspeed=math.nan, landed=None, setpoint=None)
            self.v[u] = st
            ns = get_fmu_prefix(u)
            self.create_subscription(TrajectorySetpoint, f"{ns}/in/trajectory_setpoint",
                                     lambda m, st=st: setattr(st, "setpoint", m), SENSOR_QOS)
            self.create_subscription(VehicleControlMode, f"{ns}/out/vehicle_control_mode",
                                     lambda m, st=st: setattr(st, "mode", m), SENSOR_QOS)
            self.create_subscription(VehicleLocalPosition, f"{ns}/out/vehicle_local_position_v1",
                                     lambda m, st=st: setattr(st, "pos", m), SENSOR_QOS)
            self.create_subscription(AirspeedValidated, f"{ns}/out/airspeed_validated_v1",
                                     lambda m, st=st: setattr(st, "airspeed", m.calibrated_airspeed_m_s), SENSOR_QOS)
            self.create_subscription(VehicleLandDetected, f"{ns}/out/vehicle_land_detected",
                                     lambda m, st=st: setattr(st, "landed", m.landed), SENSOR_QOS)


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


def stats(values):
    mean = sum(values) / len(values)
    return mean, math.sqrt(sum((x - mean) ** 2 for x in values) / len(values))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uav", type=int, nargs="+", default=[0])
    ap.add_argument("--alt", type=float, nargs="+", help="Hedef irtifa(lar) [m] (varsayılan 100 + 20·uav)")
    ap.add_argument("--frame-uav", type=int, default=0, help="Ortak çerçeve ve merkezin aracı")
    ap.add_argument("--center-n", type=float, default=0.0)
    ap.add_argument("--center-e", type=float, default=0.0)
    ap.add_argument("--radius", type=float, default=100.0)
    ap.add_argument("--duration", type=float, default=240.0, help="OFFBOARD sonrası izleme [sn]")
    ap.add_argument("--period", type=float, default=5.0)
    ap.add_argument("--engage-timeout", type=float, default=60.0, help="OFFBOARD bekleme sınırı [sn]")
    ap.add_argument("--alt-tol", type=float, default=20.0)
    ap.add_argument("--capture-tol", type=float, default=30.0)
    ap.add_argument("--capture-hold", type=float, default=10.0)
    ap.add_argument("--capture-timeout", type=float, default=90.0)
    ap.add_argument("--track-tol", type=float, default=40.0)
    ap.add_argument("--min-separation", type=float, default=40.0, help="İki araç arası en küçük 3B mesafe [m]")
    ap.add_argument("--mem-growth-mb", type=float, default=100.0)
    ap.add_argument("--stall-gap", type=float, default=1.5, help="Duraklama sayılan döngü boşluğu [sn]")
    ap.add_argument("--abort-uavs", type=int, nargs="+", help="Abort'ta Hold'a alınacaklar (varsayılan: --uav)")
    ap.add_argument("--no-hold-at-end", action="store_true", help="Başarıda araçları Hold'a alma")
    args = ap.parse_args()
    uavs = args.uav
    if args.alt is not None and len(args.alt) not in (1, len(uavs)):
        ap.error("--alt: tek değer ya da her araç için bir değer verin")
    targets = {u: (args.alt[i if len(args.alt) > 1 else 0] if args.alt else 100.0 + 20.0 * u)
               for i, u in enumerate(uavs)}
    abort_uavs = args.abort_uavs or uavs
    pair = uavs[:2] if len(uavs) >= 2 else None

    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))

    # rclpy'nin kendi SIGINT/SIGTERM işleyicisi bağlamı kapatıp spin_once'tan istisna fırlatır;
    # kapatılmazsa yukarıdaki işleyicinin yerine geçer ve özet basılmaz.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    w = Watch(uavs, args.frame_uav)
    rss0 = gz_rss_mb()
    log(f"izleme: {', '.join(f'uav{u} {targets[u]:.0f} m' for u in uavs)}; merkez ({args.center_n:.0f}, "
        f"{args.center_e:.0f}) [uav{args.frame_uav} çerçevesi], R {args.radius:.0f} m, gz RSS başlangıç {rss0} MB; "
        f"abort → {abort_uavs}")

    offsets = {}     # aracın yerel orijini, ortak çerçevede (N, E)

    def offboard(u) -> bool:
        st = w.v[u]
        return bool(st.mode and st.mode.flag_control_offboard_enabled)

    def snapshot(u):
        st, (on, oe) = w.v[u], offsets[u]
        p, sp = st.pos, st.setpoint
        dn, de = p.x + on - args.center_n, p.y + oe - args.center_e
        dist = math.hypot(dn, de)
        angle = math.atan2(de, dn)
        lead = math.nan
        if sp is not None:
            sp_angle = math.atan2(sp.position[1] + oe - args.center_e, sp.position[0] + on - args.center_n)
            lead = math.degrees(math.remainder(sp_angle - angle, 2.0 * math.pi))
        return {"mod": control_mode_name(st.mode), "irtifa": -p.z, "merkeze": dist,
                "yarıçap_hatası": dist - args.radius, "hava_hızı": st.airspeed, "yer_hızı": math.hypot(p.vx, p.vy),
                "açı_farkı": lead, "açı": math.degrees(angle), "n": dn, "e": de, "amsl": p.ref_alt - p.z}

    def joint(snaps):
        a, b = snaps[pair[0]], snaps[pair[1]]
        sep_angle = (b["açı"] - a["açı"]) % 360.0
        sep_3d = math.sqrt((a["n"] - b["n"]) ** 2 + (a["e"] - b["e"]) ** 2 + (a["amsl"] - b["amsl"]) ** 2)
        return sep_angle, sep_3d

    def abort(reason: str, u, snaps) -> int:
        who = f"uav{u}" if u is not None else "ortak"
        values = "; ".join(
            f"uav{x}: mod={s['mod']} irtifa={s['irtifa']:.1f} m merkeze={s['merkeze']:.1f} m "
            f"yarıçap_hatası={s['yarıçap_hatası']:+.1f} m hava_hızı={s['hava_hızı']:.1f} yer_hızı={s['yer_hızı']:.1f} "
            f"tavşan−uçak={s['açı_farkı']:+.1f}°" for x, s in snaps.items())
        if pair:
            sep_angle, sep_3d = joint(snaps)
            values += f"; açı farkı={sep_angle:.1f}° 3B={sep_3d:.1f} m"
        log(f"ABORT ({who}): {reason} | o an: {values}")
        subprocess.run([os.path.join(HERE, "orbit_abort"), *map(str, abort_uavs)])
        return 2

    # Global referanslar ve OFFBOARD bekle
    start = time.monotonic()
    while time.monotonic() - start < args.engage_timeout and not stop["flag"]:
        rclpy.spin_once(w, timeout_sec=0.1)
        if all(w.v[u].pos is not None and w.v[u].pos.xy_global for u in w.v) and all(offboard(u) for u in uavs):
            break
    else:
        missing = [u for u in uavs if not offboard(u)]
        log(f"{args.engage_timeout:.0f} sn içinde OFFBOARD'a geçmeyen: {missing} "
            f"(mod: {', '.join(f'uav{u}={control_mode_name(w.v[u].mode)}' for u in uavs)})")
        return 5 if stop["flag"] else 3
    frame = w.v[args.frame_uav].pos
    for u in uavs:
        p = w.v[u].pos
        offsets[u] = ref_offset_ne(p.ref_lat, p.ref_lon, frame.ref_lat, frame.ref_lon)
        log(f"uav{u} orijini uav{args.frame_uav} çerçevesinde: N={offsets[u][0]:+.1f} E={offsets[u][1]:+.1f} m")

    engaged = time.monotonic()
    log(f"Tüm araçlar OFFBOARD — {args.duration:.0f} sn izleniyor")
    state = {u: SimpleNamespace(rows=[], checks=[], captured=None, within_since=None) for u in uavs}
    joint_rows, joint_checks, stalls = [], [], []
    next_row, next_check, last_loop = engaged, engaged, engaged
    code = 0
    while True:
        rclpy.spin_once(w, timeout_sec=0.05)
        now = time.monotonic()
        elapsed = now - engaged
        if now - last_loop > args.stall_gap:
            stalls.append((last_loop - engaged, now - last_loop))
            log(f"DURAKLAMA: t={last_loop - engaged:.1f} sn, {now - last_loop:.1f} sn")
        last_loop = now
        if stop["flag"]:
            log("SIGTERM — izleme durduruldu (Hold verilmedi)")
            code = 5
            break
        snaps = None
        if now >= next_row:
            snaps = {u: snapshot(u) for u in uavs}
            gz_mb = gz_rss_mb()
            for u, s in snaps.items():
                rows = state[u].rows
                s["t"], s["gz_MB"] = elapsed, gz_mb
                s["açı_değişimi"] = math.nan
                if rows:
                    step = math.remainder(math.radians(s["açı_farkı"] - rows[-1]["açı_farkı"]), 2.0 * math.pi)
                    s["açı_değişimi"] = math.degrees(step) / (s["t"] - rows[-1]["t"])
                rows.append(s)
                log(f"[uav{u}] t={elapsed:5.0f} sn  mod={s['mod']:<8} irtifa={s['irtifa']:6.1f} m  "
                    f"merkeze={s['merkeze']:6.1f} m  yarıçap_hatası={s['yarıçap_hatası']:+6.1f} m  "
                    f"hava_hızı={s['hava_hızı']:5.1f}  yer_hızı={s['yer_hızı']:5.1f} m/s  "
                    f"tavşan−uçak={s['açı_farkı']:+6.1f}°  gz={gz_mb} MB")
            if pair:
                sep_angle, sep_3d = joint(snaps)
                joint_rows.append({"t": elapsed, "açı": sep_angle, "3B": sep_3d})
                log(f"[uav{pair[0]}–uav{pair[1]}] t={elapsed:5.0f} sn  açı farkı={sep_angle:6.1f}°  3B mesafe={sep_3d:6.1f} m")
            next_row += args.period
        if elapsed >= args.duration:
            break
        if now < next_check:
            continue
        next_check = now + 0.5

        snaps = snaps or {u: snapshot(u) for u in uavs}
        reason = None
        for u, s in snaps.items():
            st = state[u]
            err = s["yarıçap_hatası"]
            st.checks.append((elapsed, err))
            if not offboard(u):
                reason = (u, "Offboard'dan beklenmeyen çıkış (failsafe ya da dış komut)")
            elif w.v[u].landed:
                reason = (u, "iniş algılandı")
            elif abs(s["irtifa"] - targets[u]) > args.alt_tol:
                reason = (u, f"irtifa hedeften {s['irtifa'] - targets[u]:+.1f} m saptı (sınır ±{args.alt_tol:.0f} m)")
            elif st.captured is None:
                if abs(err) < args.capture_tol:
                    st.within_since = elapsed if st.within_since is None else st.within_since
                    if elapsed - st.within_since >= args.capture_hold:
                        st.captured = elapsed
                        log(f"[uav{u}] YAKALANDI: t={elapsed:.0f} sn (hata {args.capture_hold:.0f} sn boyunca "
                            f"< {args.capture_tol:.0f} m)")
                else:
                    st.within_since = None
                if st.captured is None and elapsed > args.capture_timeout:
                    reason = (u, f"{args.capture_timeout:.0f} sn içinde yakalanmadı")
            elif abs(err) > args.track_tol:
                reason = (u, f"yakalama sonrası yarıçap hatası {err:+.1f} m > {args.track_tol:.0f} m")
            if reason:
                break
        if reason is None and pair:
            sep_angle, sep_3d = joint(snaps)
            joint_checks.append((elapsed, sep_angle, sep_3d))
            if sep_3d < args.min_separation:
                reason = (None, f"iki araç arası 3B mesafe {sep_3d:.1f} m < {args.min_separation:.0f} m")
        if reason is None and rss0 is not None:
            gz_mb = gz_rss_mb()
            if gz_mb is not None and gz_mb > rss0 + args.mem_growth_mb:
                reason = (None, f"gz RSS {gz_mb} MB > {rss0} + {args.mem_growth_mb:.0f} MB")
        if reason:
            code = abort(reason[1], reason[0], snaps)
            break

    # ── Özet ────────────────────────────────────────────────────────────────
    for u in uavs:
        st = state[u]
        print(f"\n### uav{u} (hedef {targets[u]:.0f} m)")
        print("| t (sn) | mod | irtifa (m) | merkeze (m) | yarıçap hatası (m) | hava hızı (m/s) | "
              "yer hızı (m/s) | tavşan−uçak (°) | değişim (°/sn) | gz (MB) |")
        print("|---|---|---|---|---|---|---|---|---|---|")
        for r in st.rows:
            print(f"| {r['t']:.0f} | {r['mod']} | {r['irtifa']:.1f} | {r['merkeze']:.1f} | {r['yarıçap_hatası']:+.1f} | "
                  f"{r['hava_hızı']:.1f} | {r['yer_hızı']:.1f} | {r['açı_farkı']:+.1f} | {r['açı_değişimi']:+.2f} | "
                  f"{r['gz_MB']} |")
        after = [r for r in st.rows if st.captured is not None and r["t"] >= st.captured]
        if after:
            signed = [r["yarıçap_hatası"] for r in after]
            mean_e, std_e = stats(signed)
            leads = [r["açı_farkı"] for r in after]
            alts = [r["irtifa"] for r in after]
            gs = [r["yer_hızı"] for r in after]
            print(f"\nuav{u} yakalama: {st.captured:.0f}. sn. Sonrası ({len(after)} örnek): yarıçap hatası ort |e| "
                  f"{sum(map(abs, signed))/len(signed):.1f} m, maks |e| {max(map(abs, signed)):.1f} m, "
                  f"ort e {mean_e:+.1f} m, std {std_e:.1f} m; tavşan−uçak {min(leads):+.1f}…{max(leads):+.1f}°; "
                  f"irtifa {min(alts):.1f}–{max(alts):.1f} m; ort. yer hızı {sum(gs)/len(gs):.1f} m/s")
        else:
            print(f"\nuav{u}: yakalama olmadı.")

    if pair and joint_rows:
        print(f"\n### uav{pair[0]}–uav{pair[1]}")
        print("| t (sn) | açı farkı (°) | 3B mesafe (m) |")
        print("|---|---|---|")
        for r in joint_rows:
            print(f"| {r['t']:.0f} | {r['açı']:.1f} | {r['3B']:.1f} |")
        angles = [r["açı"] for r in joint_rows]
        unwrapped = [angles[0]]        # 0/360 geçişlerini aç; ortalama değişim için
        for a in angles[1:]:
            unwrapped.append(unwrapped[-1] + math.degrees(math.remainder(math.radians(a - unwrapped[-1]), 2 * math.pi)))
        span = joint_rows[-1]["t"] - joint_rows[0]["t"]
        rate = (unwrapped[-1] - unwrapped[0]) / span if span > 0 else math.nan
        min_3d = min(c[2] for c in joint_checks) if joint_checks else min(r["3B"] for r in joint_rows)
        print(f"\naçı farkı: başlangıç {angles[0]:.1f}°, bitiş {angles[-1]:.1f}°, en küçük {min(angles):.1f}°, "
              f"en büyük {max(angles):.1f}°; ortalama değişim {rate:+.3f} °/sn; en küçük 3B mesafe {min_3d:.1f} m")

    print(f"\nDuraklamalar ({len(stalls)}):")
    for t0, dur in stalls:
        parts = []
        for u in uavs:
            before = [e for t, e in state[u].checks if t <= t0]
            after_win = [e for t, e in state[u].checks if t0 + dur < t <= t0 + dur + 5.0]
            if before and after_win:
                jump = max(after_win, key=lambda e: abs(e - before[-1])) - before[-1]
                parts.append(f"uav{u} öncesi {before[-1]:+.1f} m, sonraki 5 sn en büyük değişim {jump:+.1f} m")
            else:
                parts.append(f"uav{u} yetersiz veri")
        print(f"  t={t0:6.1f} sn, {dur:.1f} sn: " + "; ".join(parts))

    if code == 0:
        log(f"Test başarılı ({args.duration:.0f} sn).")
        if not args.no_hold_at_end:
            subprocess.run([os.path.join(HERE, "orbit_abort"), *map(str, abort_uavs)])
    w.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
