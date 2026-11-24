#!/usr/bin/env python3
"""el_firlatma.py — Tulpar'ı Gazebo'da el fırlatmayla havalandırmanın gz tarafı + Tulpar parametre tabloları.

Gerçek uçak el fırlatmayla kalkar (iniş takımı yok). Simülasyonda: PX4 Takeoff modunda arm edilir (launch-detect motoru
fırlatma algılanınca açar) → model yerden 1,5 m'ye ışınlanır, burun yukarı → kısa bir kuvvet darbesi (ApplyLinkWrench,
gz-sim-apply-link-wrench-system server.config'te yüklü) ≈ 12 m/s verir. Darbe süresi SİM saatiyle (`/world/<w>/clock`)
ölçülür: RTF < 1'de duvar saatiyle ölçmek darbeyi uzatırdı. MAVLink tarafı (parametre, Takeoff+arm) mavlink_takeoff.py'de.

Kuvvet link orijininden uygulanır, CG ise z'de ofsetli (Tulpar: +0,0265 m) → telafi torku eklenir (ölçüm: telafisiz darbe
~30° burun-yukarı savurdu, 2026-11-21). Tüm sabitler `gz_models_tulpar/uret_model.py` çıktısıyla uyumlu olmalı
(el_firlatma_test.py kontrol eder).

Kullanım (komut): python3 el_firlatma.py hesap [v_atis] [pitch_derece]    → darbe kuvvet/torkunu yazdırır (gz gerekmez)
"""
import math
import subprocess
import sys
import time

# --- Tulpar fiziksel sabitleri (uret_model.py: KUTLE, CG z) ---
KUTLE = 2.3           # kg
CG_Z = 0.0265         # m, CG'nin base_link orijininden yukarı ofseti (nocam, kam)
# Varyanta göre CG z: T10'da kamera kütlesi (0,12 kg, z=0) CG'yi 2 mm aşağı çeker (uret_model.py çıktısı)
# W10 (BAP 6 mm) kamerası T10 ile aynı kütle/konumda → aynı CG; varyant_algila() kameralı modeli "T10" sayar (aynı değer).
CG_Z_VARYANT = {"nocam": 0.0265, "T10": 0.0244, "W10": 0.0244}
G = 9.81

# --- parametre tabloları: isim: (fırlatma/sim değeri, --revert-sim-params ile yazılacak PX4 varsayılanı) ---
# YALNIZCA SİMÜLASYON — gerçek uçağa taşınmaz:
TULPAR_SIM_ONLY = {
    "FD_ESCS_EN": (0, 1),         # gz_bridge ESC "armed" bayrağını yalnız motor hızı > 0 iken kurar; el fırlatmada motor
                                  # kalkışa kadar kapalı → "ESC failure" failsafe'i arm'ı düşürür (2026-11-21, A4)
    "ASPD_DO_CHECKS": (0, 7),     # gz AirSpeed sıcaklık göndermiyor (0 K) → TAS yanlış → "sensor failure"
    "ASPD_SCALE_APPLY": (0, 2),   # aynı hatadan öğrenilen ölçek uygulanmasın
    "NAV_DLL_ACT": (0, 2),        # veri bağlantısı kaybı eylemi kapalı (kalp atışı kesilirse RTL tetiklenmesin)
    "EKF2_IMU_CTRL": (5, 7),      # sim IMU'da gerçek ivmeölçer bias'ı yok; bias kestirimi kapalı
}
# Fırlatma/uçuş ayarları (gerçek uçakta da ayrıca doğrulanmalı). Değerler 4030_gz_tulpar airframe'iyle aynı olmalı
# (el_firlatma_test.py karşılaştırır); kayıtlı parametreler `param set-default`'ı ezdiği için her fırlatmada yazılır.
# Geri alma değeri YOK: model değişince launch_all.sh uav0 parametrelerini yedekleyip sıfırlar.
TULPAR_UCUS_PARAMS = {
    "COM_DISARM_LAND": 10.0,   # Takeoff modunda fırlatma beklerken yerde "indi" sanıp disarm etmesin (airframe'de yok)
    "FW_LAUN_DETCN_ON": 1,
    "FW_LAUN_AC_THLD": 20.0,
    "FW_LAUN_AC_T": 0.05,
    "FW_LAUN_MOT_DEL": 0.3,
    "RWTO_TKOFF": 0,
    "FW_AIRSPD_TRIM": 15.0,
    "FW_AIRSPD_MIN": 11.0,
    "FW_AIRSPD_MAX": 24.0,
    "FW_THR_TRIM": 0.36,
    "FW_THR_MAX": 0.9,
    "FW_THR_MIN": 0.05,
    "FW_PSP_OFF": -2.0,
    "FW_TKO_PITCH_MIN": 10.0,
    "FW_P_LIM_MAX": 22.0,
    "FW_PR_I": 0.1,
    "FW_RR_I": 0.05,
    "FW_RR_P": 0.08,
    "FW_RR_FF": 0.12,
    "FW_R_LIM": 35.0,
}


def hesapla(v_atis=12.0, t_atis=0.30, pitch_rad=math.radians(10.0), kutle=KUTLE, cg_z=CG_Z):
    """Darbe kuvvet/torku (dünya çerçevesi): a = v/t; ağırlığı da taşır (uçak darbe boyunca düşmesin).
    Dönüş: (Fx, Fz, Ty). Ty: kuvvet link orijininde, CG cg_z yukarıda; burun yukarı pitch_rad ile dönmüş gövde z ekseni."""
    a = v_atis / t_atis
    fx = kutle * a * math.cos(pitch_rad)
    fz = kutle * G + kutle * a * math.sin(pitch_rad)
    rx, rz = -cg_z * math.sin(pitch_rad), cg_z * math.cos(pitch_rad)
    ty = rz * fx - rx * fz
    return fx, fz, ty


class Darbe:
    """gz.transport13 ile persistent wrench + clear. Yayıncılar ÖNCEDEN kurulur (hazirla), darbe anında gecikme olmasın."""

    def __init__(self, model="tulpar_0", dunya="default", v_atis=12.0, t_atis=0.30, pitch_deg=10.0,
                 yukseklik=1.5, x=0.0, y=0.0, cg_z=None):
        self.model, self.dunya = model, dunya
        self.v_atis, self.t_atis = v_atis, t_atis
        self.pitch = math.radians(pitch_deg)
        self.yukseklik, self.x, self.y = yukseklik, x, y
        self.sim_zamani = None   # son /clock değeri [s]
        self.cg_z_verilen = cg_z
        self.varyant = None
        self._hesapla(CG_Z if cg_z is None else cg_z)

    def _hesapla(self, cg_z):
        self.cg_z = cg_z
        self.fx, self.fz, self.ty = hesapla(self.v_atis, self.t_atis, self.pitch, cg_z=cg_z)

    def varyant_algila(self):
        """Çalışan modelin varyantı: aşağı bakan kamera sensörü (`.../base_link/sensor/camera/image`) varsa T10, yoksa nocam."""
        r = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True)
        kamera = f"/world/{self.dunya}/model/{self.model}/link/base_link/sensor/camera/image"
        self.varyant = "T10" if kamera in r.stdout.split() else "nocam"
        return self.varyant

    def hazirla(self):
        from gz.transport13 import Node
        from gz.msgs10.clock_pb2 import Clock
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        self._node = Node()
        self._pw = self._node.advertise(f"/world/{self.dunya}/wrench/persistent", EntityWrench)
        self._pc = self._node.advertise(f"/world/{self.dunya}/wrench/clear", Entity)

        def on_clock(m):
            self.sim_zamani = m.sim.sec + m.sim.nsec * 1e-9
        self._node.subscribe(Clock, f"/world/{self.dunya}/clock", on_clock)
        w = EntityWrench()
        w.entity.name = f"{self.model}::base_link"
        w.entity.type = Entity.LINK
        w.wrench.force.x, w.wrench.force.z, w.wrench.torque.y = self.fx, self.fz, self.ty
        c = Entity()
        c.name = f"{self.model}::base_link"
        c.type = Entity.LINK
        self._w, self._c = w, c
        if self.cg_z_verilen is None:
            self._hesapla(CG_Z_VARYANT[self.varyant_algila()])
            self._w.wrench.force.x, self._w.wrench.force.z, self._w.wrench.torque.y = self.fx, self.fz, self.ty
        time.sleep(0.6)   # yayıncı/abone eşleşmesi
        if self.sim_zamani is None:
            raise RuntimeError(f"/world/{self.dunya}/clock gelmiyor (gz sunucusu çalışıyor mu?)")

    def isinla(self):
        """Modeli (x, y, yukseklik)'e, burun yukarı pitch ile taşır (yaw 0 → doğuya bakar; yaw-0 spawn kısıtı)."""
        s, c = math.sin(-self.pitch / 2), math.cos(self.pitch / 2)   # gz: burun yukarı = negatif pitch
        r = subprocess.run(
            ["gz", "service", "-s", f"/world/{self.dunya}/set_pose", "--reqtype", "gz.msgs.Pose",
             "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req",
             f'name: "{self.model}" position {{x: {self.x} y: {self.y} z: {self.yukseklik}}} '
             f'orientation {{x: 0 y: {s} z: 0 w: {c}}}'], capture_output=True, text=True)
        return "data: true" in r.stdout

    def at(self, zaman_asimi=10.0):
        """Darbeyi uygular, t_atis kadar SİM süresi bekler, kaldırır. Dönüş: (sim_sure, duvar_sure)."""
        w0 = time.time()
        s0 = self.sim_zamani
        self._pw.publish(self._w)
        while self.sim_zamani - s0 < self.t_atis and time.time() - w0 < zaman_asimi:
            time.sleep(0.002)
        self._pc.publish(self._c)
        return self.sim_zamani - s0, time.time() - w0


def _main(argv):
    if len(argv) >= 2 and argv[1] == "hesap":
        v = float(argv[2]) if len(argv) > 2 else 12.0
        p = float(argv[3]) if len(argv) > 3 else 10.0
        fx, fz, ty = hesapla(v, 0.30, math.radians(p))
        print(f"v={v} m/s, pitch={p}°, t=0,30 s → Fx={fx:.1f} N  Fz={fz:.1f} N  Ty={ty:.2f} N·m  (ivme {v/0.30:.1f} m/s²)")
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(_main(sys.argv))
