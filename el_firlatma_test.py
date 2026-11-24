#!/usr/bin/env python3
"""el_firlatma_test.py — Adım 4 kuru testi (simülasyon/gz/PX4 gerekmez, dosya yazmaz).

1) Varsayılan davranış DEĞİŞMEDİ: mavlink_takeoff.py'nin advanced_plane parametre tabloları, Adım 3'ün son
   commit'indeki (TABAN) dosyayla birebir aynı (git show ile okunur).
2) Tulpar tabloları: pist ayarları yok, geri alma değerleri PX4 varsayılanı, UCUS değerleri 4030_gz_tulpar airframe'iyle aynı.
3) Darbe fiziği: hız = F·t/m, CG tork telafisi sıfırlıyor, sabitler uret_model.py çıktısıyla (kütle, CG z) uyumlu.
4) --el-firlatma doğrulaması (yanlış kullanım reddi).
Çıkış kodu 0 = hepsi geçti.
"""
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.pop("UAV0_MODEL", None)
os.environ.pop("UAV1_MODEL", None)
import arac_modelleri  # noqa: E402
import el_firlatma  # noqa: E402
import mavlink_takeoff as mt  # noqa: E402

TABAN = "ee97a24"   # Adım 3 commit'i (mavlink_takeoff.py'ye dokunulmamış son hal)
n = 0


def ok(cond, msg):
    global n
    if not cond:
        print(f"✘ {msg}")
        sys.exit(1)
    n += 1
    print(f"✔ {msg}")


# --- 1) varsayılan = taban
r = subprocess.run(["git", "-C", HERE, "show", f"{TABAN}:mavlink_takeoff.py"], capture_output=True, text=True)
if r.returncode == 0:
    ns = {}
    src = r.stdout.split("\ndef log(")[0]            # yalnız modül başı (importlar + tablolar)
    exec(compile(src, "taban_mavlink_takeoff", "exec"), ns)
    ok(mt.SIM_ONLY_PARAMS == ns["SIM_ONLY_PARAMS"], f"SIM_ONLY_PARAMS tabanla ({TABAN}) birebir aynı ({len(mt.SIM_ONLY_PARAMS)} parametre)")
    ok(mt.UCUS_DINAMIGI_PARAMS == ns["UCUS_DINAMIGI_PARAMS"], "UCUS_DINAMIGI_PARAMS tabanla birebir aynı")
else:
    print(f"! taban commit {TABAN} okunamadı — taban karşılaştırması ATLANDI: {r.stderr.strip()[:80]}")
for u in (0, 1):
    ok(mt.sim_params_for(u) == mt.SIM_ONLY_PARAMS, f"UAV{u}_MODEL yok: sim_params_for({u}) = SIM_ONLY_PARAMS (advanced_plane)")
    ok(mt.ucus_params_for(u) == {}, f"UAV{u}_MODEL yok: modele özgü ek parametre yok")
ok(arac_modelleri.gz_ad(0) == "advanced_plane_0", "varsayılan gz adı advanced_plane_0")

# --- 2) Tulpar tabloları
os.environ["UAV0_MODEL"] = "tulpar"
sim0, sim1 = mt.sim_params_for(0), mt.sim_params_for(1)
ok(set(sim0) == {"FD_ESCS_EN", "ASPD_DO_CHECKS", "ASPD_SCALE_APPLY", "NAV_DLL_ACT", "EKF2_IMU_CTRL"}, "tulpar SIM_ONLY: beş parametre")
ok("RWTO_MAX_THR" not in sim0 and "FW_TKO_PITCH_MIN" not in sim0 and "FW_AIRSPD_TRIM" not in sim0,
   "tulpar SIM_ONLY'de pist/advanced_plane ayarları (RWTO_MAX_THR, FW_TKO_PITCH_MIN=20, FW_AIRSPD_TRIM=14) yok")
ok(sim1 == mt.SIM_ONLY_PARAMS, "UAV0_MODEL=tulpar iken uav1 hâlâ advanced_plane tablosunu alır")
PX4_VARSAYILAN = {"FD_ESCS_EN": 1, "ASPD_DO_CHECKS": 7, "ASPD_SCALE_APPLY": 2, "NAV_DLL_ACT": 2, "EKF2_IMU_CTRL": 7}
ok(all(sim0[k][1] == v for k, v in PX4_VARSAYILAN.items()), "tulpar geri alma değerleri PX4 varsayılanı (NAV_DLL_ACT: airframe 2)")
ok(mt.ucus_params_for(1) == {} and len(mt.ucus_params_for(0)) == 20, "tulpar uçuş parametreleri 20 adet, uav1'e uygulanmaz")
airframe = open(os.path.join(HERE, "gz_models_tulpar", "4030_gz_tulpar"), encoding="utf-8").read()
farkli = []
for k, v in el_firlatma.TULPAR_UCUS_PARAMS.items():
    m = re.findall(rf"^param set-default {k} (\S+)", airframe, re.M)
    if m and abs(float(m[-1]) - v) > 1e-9:
        farkli.append((k, m[-1], v))
ok(not farkli, f"UCUS değerleri 4030_gz_tulpar ile aynı (farklı: {farkli})")
ok("FD_ESCS_EN 0" in airframe, "airframe'de FD_ESCS_EN 0 (sim-only) var")

# --- 3) darbe fiziği
for v, p in ((12.0, 10.0), (10.0, 5.0), (15.0, 15.0)):
    fx, fz, ty = el_firlatma.hesapla(v, 0.30, math.radians(p))
    hiz = math.hypot(fx, fz - el_firlatma.KUTLE * el_firlatma.G) * 0.30 / el_firlatma.KUTLE
    ok(abs(hiz - v) < 1e-9, f"darbe hızı = F·t/m = {hiz:.3f} m/s (hedef {v}, pitch {p}°)")
    th = math.radians(p)
    rx, rz = -el_firlatma.CG_Z * math.sin(th), el_firlatma.CG_Z * math.cos(th)
    tork_cg = -(rz * fx - rx * fz) + ty        # orijindeki kuvvetin CG'ye göre torku + telafi
    ok(abs(tork_cg) < 1e-9, f"CG tork telafisi sıfırlıyor (kalan {tork_cg:.1e} N·m)")
fx, fz, ty = el_firlatma.hesapla(12.0, 0.30, math.radians(10.0))
ok(abs(fx - 90.6) < 0.1 and abs(fz - 38.5) < 0.1 and abs(ty - 2.54) < 0.01, "A5'te uçan değerler (Fx 90,6 / Fz 38,5 / Ty 2,54)")
d = el_firlatma.Darbe(v_atis=12.0)
ok((d.fx, d.fz, d.ty) == (fx, fz, ty) and d.model == "tulpar_0", "Darbe sınıfı hesapla ile aynı kuvvetleri üretir, model tulpar_0")
for varyant in ("nocam", "T10", "W10"):
    sdf = open(os.path.join(HERE, "gz_models_tulpar", varyant, "tulpar", "model.sdf"), encoding="utf-8").read()
    kutle = float(re.search(r"<inertial>\s*<pose>[^<]*</pose>\s*<mass>([0-9.e+-]+)</mass>", sdf).group(1))
    cgz = float(re.search(r"<inertial>\s*<pose>\S+ \S+ (\S+) [^<]*</pose>\s*<mass>", sdf).group(1))
    ok(abs(kutle - el_firlatma.KUTLE) < 1e-6 and abs(cgz - el_firlatma.CG_Z_VARYANT[varyant]) < 0.0005,
       f"{varyant}: model kütlesi {kutle} kg, CG z {cgz:.4f} m ↔ el_firlatma sabitleri ({el_firlatma.KUTLE}, {el_firlatma.CG_Z_VARYANT[varyant]})")
ok(el_firlatma.Darbe(cg_z=0.0244).ty != el_firlatma.Darbe(cg_z=0.0265).ty and el_firlatma.Darbe().cg_z == el_firlatma.CG_Z,
   "Darbe: cg_z verilirse tork buna göre hesaplanır, verilmezse nocam değeri (hazirla'da varyant algılanır)")

# --- 4) CLI doğrulaması
env = dict(os.environ)
env.pop("UAV0_MODEL", None)
for args, env_i, parca in ((["--uav", "0", "1", "--el-firlatma"], env, "yalnız --uav 0"),
                           (["--uav", "0", "--el-firlatma"], env, "UAV0_MODEL=tulpar gerekli"),
                           (["--uav", "0", "1", "--el-firlatma"], dict(env, UAV0_MODEL="tulpar"), "yalnız --uav 0")):
    r = subprocess.run([sys.executable, os.path.join(HERE, "mavlink_takeoff.py")] + args, capture_output=True, text=True, env=env_i)
    ok(r.returncode == 2 and parca in r.stderr, f"mavlink_takeoff {' '.join(args)} (UAV0_MODEL={env_i.get('UAV0_MODEL', '-')}) reddedilir: '{parca}'")

print(f"\nTAMAM: {n} doğrulama geçti")
