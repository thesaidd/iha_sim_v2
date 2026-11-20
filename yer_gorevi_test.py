#!/usr/bin/env python3
"""
yer_gorevi_test.py — uav1 "yerden kalkış" senaryosunun KURU testi (uçuş / simülasyon / ROS / PX4 YOK).

  [1] Birim: üs pozu, iniş deseni ve görev öğeleri, katılma öngörüsü, kalkış öncesi plan (direk payı, uav0 Hold'una
      3B ayrım, katılma mesafesi, dönüş rotası; NO-GO durumları), görev yükleme el sıkışması.
  [2] Üs için erişilebilirlik: kutu içindeki hedef × tarama hattı × uav0 çıkış Hold'u durumlarının kaçında plan var.
  [3] Sahte kinematik araçla akış: lora_yaklasma._dalis_akisi'nın KENDİSİ (--baslangic yer) uçtan uca koşturulur:
      yerde bekleme → plan → görev yükleme → arm + kalkış → OFFBOARD, hatta katılma → hat / durma / tırmanış (mevcut
      kod) → dönüş rotası → görev devri → iniş → disarm. Ayrıca: NO-GO'da hiç arm edilmediği, kalkış başarısızlığında
      disarm, görev moduna geçilemezse Hold, hedefsiz (Aşama 1) akış. Sayılar uçuş öngörüsü DEĞİLDİR.

Kullanım: python3 yer_gorevi_test.py [--cikti KLASOR]      Çıkış kodu 0 = tüm doğrulamalar geçti.
"""
import argparse
import datetime
import json
import math
import os
import types

import ayrim as ayrim_mod
import lora_yaklasma as ly
import yer_gorevi as yg

US = (200.0, -950.0)            # üs, uav0 NED (tasarım: test/yerden_kalkis_2026-11-15/tasarim.md)
PIST = 90.0                     # gz yaw 0 — aero kısıtı (2026-11-16 kök nedeni); başka yön denenmemeli
QR0 = (-60.0, -60.0)            # gerçek QR, uav0 NED
HATLAR = (-100, -85, -70, -55, -40, 40, 54, 67, 81)
GEO = ly.dalis_geometrisi(alcalma_siniri=4.0, suzulme_siniri=2.4)
N_OK = 0


def tamam(kosul, ad):
    global N_OK
    assert kosul, ad
    N_OK += 1


def u1(p):
    return (p[0] - US[0], p[1] - US[1])


def engel(T, uav0=None, direk_pay=45.0):
    if uav0 is None:
        return ly.dalis_engelleri(US[0], US[1], T, uav0_yaricap=0.0, direk_pay=direk_pay)
    return ly.dalis_engelleri(US[0], US[1], T, uav0_irtifa=uav0[2], direk_pay=direk_pay, uav0_merkez=u1(uav0))


def plan_icin(T0, uav0=None, **kw):
    T = u1(T0)
    return yg.yer_plani(T, GEO, engel(T, uav0), PIST, merkez=u1((0.0, 0.0)),
                        uav0=None if uav0 is None else {"merkez": u1(uav0), "irtifa": uav0[2]}, **kw), T


def direk_mesafesi(n, e):
    """uav1 NED noktasının en yakın direk eksenine yatay uzaklığı."""
    return min(math.hypot(n - (dn - US[0]), e - (de - US[1])) for ad, dn, de, _y, _r in ly.ARENA_ENGELLER_UAV0
               if ad.startswith("direk"))


# ---- [1] birim ----------------------------------------------------------------------------------------

def birim():
    tamam(yg.gz_poz(200.0, -775.0, 105.0) == "-775,200,0.5,0,0,-0.2618", "gz pozu (ENU, yaw = 90° − kerteriz)")
    tamam(yg.gz_poz(15.0, -15.0, 90.0) == "-15,15,0.5,0,0,0.0000", "eski uav1 pozu ile uyum (yaw 0 = Doğu)")
    # iniş deseni
    d = yg.inis_deseni(PIST)
    dl, rl = yg._birim(PIST)
    tamam(abs(math.dist(d["Lc"], d["G"]) - 80.0) < 1e-6 and abs(math.dist(d["G"], d["Td"]) - 350.0) < 1e-6, "orbit teğet noktası = yaklaşma girişi")
    tamam(d["Td"][0] * dl[0] + d["Td"][1] * dl[1] < 0 and abs(d["Td"][0] * rl[0] + d["Td"][1] * rl[1]) < 1e-6, "Td üssün gerisinde, pist ekseninde")
    tamam((d["Lc"][0] - d["G"][0]) * rl[0] + (d["Lc"][1] - d["G"][1]) * rl[1] > 79.9, "orbit merkezi yaklaşma hattının sağında (saat yönü)")
    tamam(d["suzulme_deg"] < 8.0, "süzülme açısı FW_LND_ANG (8°) altında")
    tamam(math.dist(d["Lc"], u1((0.0, 0.0))) > 900.0, "iniş orbiti arenadan uzak tarafta")
    og = yg.inis_gorevi(d, lambda n, e, a, b: (n, e), 0.0, 0.0)
    tamam([o["cmd"] for o in og] == [31, 21] and og[0]["p1"] == 1.0 and og[0]["p4"] == 1.0 and og[0]["p2"] == 80.0, "görev: LOITER_TO_ALT (yön şartı, teğet çıkış) → LAND")
    tamam(og[0]["alt"] == 25.0 and og[1]["alt"] == 0.0 and (og[1]["lat"], og[1]["lon"]) == d["Td"], "görev irtifaları ve iniş noktası")
    # katılma öngörüsü
    T = u1(QR0)
    k = yg.katil_benzetimi((0.0, 0.0), 90.0, 22.0, (0.0, 800.0), 90.0, 250.0)
    tamam(k["vardi"] and k["s_otur"] is not None and k["s_otur"] < -700.0 and abs(k["x_A"]) < 1.0, "hizalı kalkış: hemen oturur")
    k = yg.katil_benzetimi((0.0, 0.0), 90.0, 22.0, (150.0, 800.0), 90.0, 250.0)
    tamam(k["vardi"] and k["s_otur"] is not None and k["s_otur"] < -250.0 and abs(abs(k["x0"]) - 150.0) < 1e-6, "150 m yanal kaçıklık: A'dan önce oturur")
    k = yg.katil_benzetimi((0.0, 0.0), 90.0, 22.0, (150.0, 330.0), 90.0, 250.0)
    tamam(k["s_otur"] is None or k["s_otur"] > -250.0, "150 m yanal + 80 m mesafe: oturamaz")
    # plan: gerçek hedef, dört çıkış Hold'u
    for u0 in ((310.0, 25.0, 60.0), (310.0, 10.0, 60.0), (-310.0, -135.0, 60.0), (-310.0, -150.0, 60.0)):
        p, T = plan_icin(QR0, u0)
        tamam(p["uygun"] and abs(p["yon_sapma_deg"]) <= 20.0 and p["giris"] == "yer", f"gerçek hedef, uav0 Hold {u0[:2]}: plan var")
        pts, hn = ly.sabit_yol(T, p["yon_deg"], GEO, bas_irtifa=120.0, guvenli_irtifa=120.0, tirmanma_hizi=3.0, tirmanma_tol=5.0)
        hat = [q for q in pts if q[3] in ("hat", "tırmanış")]
        tamam(min(direk_mesafesi(q[0], q[1]) for q in hat) >= 45.0 + 1.5 - 1e-6, "hat + tırmanış direklere ≥ 45 m (bağımsız ölçüm)")
        tamam(hn["cikis_m"] >= 320.0, "denetlenen tırmanış uzantısı ≥ 320 m")
        cm = u1(u0)
        en = min(math.hypot(max(0.0, math.hypot(q[0] - cm[0], q[1] - cm[1]) - 80.0), max(0.0, 50.0 - q[2], q[2] - 70.0)) for q in hat)
        tamam(en >= 110.0 - 1e-6 and p["uav0_en_kucuk_3b_m"] >= 110.0, f"uav0 Hold çemberine 3B ≥ 110 m ({en:.0f})")
        rota = p["donus"]["rota"]
        ek = min(direk_mesafesi(a[0] + (b[0] - a[0]) * i / 200, a[1] + (b[1] - a[1]) * i / 200)
                 for a, b in zip(rota, rota[1:]) for i in range(201))
        tamam(ek >= 45.0 + 25.0 + 1.5 - 1e-6 and rota[-1] == tuple(round(v, 1) for v in yg.inis_deseni(PIST)["Lc"]), f"dönüş bacakları direklere ≥ 70 m ({ek:.0f}), iniş orbitinde biter")
        tamam(p["giris_detay"]["s_otur"] <= -250.0 + 37.5, "katılma: alçalma başından ≥ yarım düz-giriş önce oturur")
    p0, _ = plan_icin(QR0, None, sapma_max=0.0)
    tamam(not p0["uygun"] and "direk_B" in p0["neden"], "sapmasız doğrudan kerteriz B direğine takılır → NO-GO")
    # NO-GO durumları
    p, _ = plan_icin((150.0, -620.0))
    tamam(not p["uygun"] and "katılma" in p["neden"], "üsse çok yakın hedef: katılma mesafesi → NO-GO")
    p, _ = plan_icin((0.0, -120.0))
    tamam(not p["uygun"], "direğin dibindeki hedef → NO-GO")
    pk, Tq = plan_icin(QR0)
    s_orta = -100.0
    dd, _r = ly.yon_birim(pk["yon_deg"])
    u0_hat = (Tq[0] + s_orta * dd[0] + US[0], Tq[1] + s_orta * dd[1] + US[1], 40.0)
    p, _ = plan_icin(QR0, u0_hat)
    tamam((not p["uygun"]) or abs(p["yon_deg"] - pk["yon_deg"]) > 1.0, "uav0 Hold'u hattın üstünde: o yön reddedilir")
    p, _ = plan_icin(QR0, (QR0[0], QR0[1], 30.0))
    tamam(not p["uygun"] and "uav0" in p["neden"], "uav0 hedefin üstünde Hold'da → NO-GO (hiçbir yön açık değil)")
    # dönüş rotası yoksa NO-GO (denetimsiz bacak yok): iniş orbitini kapatan sahte engel
    T = u1(QR0)
    Lc = yg.inis_deseni(PIST)["Lc"]
    kapali = engel(T) + [{"ad": "sahte", "n": Lc[0] + 100.0, "e": Lc[1], "yuk": 300.0, "r": 5.0, "pay_y": 45.0, "pay_d": 15.0}]
    p = yg.yer_plani(T, GEO, kapali, PIST, merkez=u1((0.0, 0.0)))
    tamam(not p["uygun"], "dönüş rotası yoksa NO-GO (kalkıştan önce; ilk ret başka yönün nedeni olabilir)")
    tamam(yg.donus_rotasi((0.0, 900.0), 90.0, Lc, kapali, 120.0, u1((0.0, 0.0))) is None, "donus_rotasi: kapalı orbit → None")
    yakin = engel(T) + [{"ad": "sahte", "n": 20.0, "e": 60.0, "yuk": 20.0, "r": 5.0, "pay_y": 45.0, "pay_d": 15.0}]
    p = yg.yer_plani(T, GEO, yakin, PIST, merkez=u1((0.0, 0.0)))
    tamam(not p["uygun"] and "üs uygun değil" in p["neden"], "pist üzerinde engel → üs uygun değil")
    # hedefsiz plan (Aşama 1)
    d1 = yg.duz_plan(engel((0.0, 0.0)), PIST, 120.0, 25.0, 60.0, merkez=u1((0.0, 0.0)))
    tamam(d1["uygun"] and len(d1["donus"]["rota"]) >= 2 and math.hypot(*d1["C"]) < 450.0, "hedefsiz plan: 60 m, üs çevresinde")
    # görev yükleme el sıkışması (sahte MAVLink)
    class _Mav:
        def __init__(self, dis):
            self.dis, self.gonderilen = dis, []

        def mission_count_send(self, sysid, comp, n, tip):
            self.n = n
            self.dis._istek(0)

        def mission_item_int_send(self, sysid, comp, seq, frame, cmd, cur, auto, p1, p2, p3, p4, lat, lon, alt, tip):
            self.gonderilen.append((seq, frame, cmd, lat, lon, alt))
            if seq + 1 < self.n:
                self.dis._istek(seq + 1)
            else:
                self.dis._ack(self.dis.ack_tip)
        def mission_set_current_send(self, sysid, comp, seq):
            import time
            self.dis.set_current.append(seq)
            s = 0 if (seq == 0 and len(self.dis.set_current) >= self.dis.gereken) else self.dis.kalan_seq
            self.dis.messages["MISSION_CURRENT"] = types.SimpleNamespace(seq=s, _timestamp=time.time() + 1.0)

    class _Baglanti:
        def __init__(self, ack_tip=0, kalan_seq=0, gereken=1):
            self.messages, self.ack_tip = {}, ack_tip
            self.set_current, self.kalan_seq, self.gereken = [], kalan_seq, gereken
            self.mav = _Mav(self)

        def _istek(self, seq):
            import time
            self.messages["MISSION_REQUEST_INT"] = types.SimpleNamespace(seq=seq, _timestamp=time.time() + 1e-3 * (seq + 1))

        def _ack(self, tip):
            import time
            self.messages["MISSION_ACK"] = types.SimpleNamespace(type=tip, _timestamp=time.time() + 1.0)

    import threading
    for ack, bekle in ((0, True), (5, False)):
        arac = types.SimpleNamespace(m=_Baglanti(ack), send_lock=threading.Lock(), sysid=2)
        ok, _a = yg.gorev_yukle(arac, og, asim=3.0)
        tamam(ok == bekle and [g[0] for g in arac.m.mav.gonderilen] == [0, 1] and arac.m.mav.gonderilen[0][1] == 6, f"görev yükleme (ACK {ack})")
        tamam(arac.m.set_current == ([0] if bekle else []), "yükleme sonrası MISSION_SET_CURRENT(0) yalnız ACK ✔ iken")
    # a3 arızası: PX4 görev dizini (dataman'da kalıcı) 1. öğede takılı → set_current yinelenir, doğrulanmazsa HATA
    arac = types.SimpleNamespace(m=_Baglanti(0, kalan_seq=1, gereken=3), send_lock=threading.Lock(), sysid=2)
    ok, acik = yg.gorev_yukle(arac, og, asim=3.0)
    tamam(ok and len(arac.m.set_current) >= 2 and "0. öğede" in acik, f"takılı görev dizini: set_current yinelendi ({len(arac.m.set_current)}×)")
    arac = types.SimpleNamespace(m=_Baglanti(0, kalan_seq=1, gereken=999), send_lock=threading.Lock(), sysid=2)
    ok, acik = yg.gorev_yukle(arac, og, asim=1.0)
    tamam((not ok) and "0. öğeye alınamadı" in acik, "görev dizini 0'a alınamazsa yükleme BAŞARISIZ (kalkış yok)")
    tamam(set(yg.INIS_PARAMS) == {"FW_LND_USETER", "MIS_TKO_LAND_REQ"}, "INIS_PARAMS listesi")
    # kalkış komutu: MIS_TAKEOFF_ALT önce yazılır, GERİ OKUNARAK doğrulanır, sonra take_off (a1 yarışı)
    import mavlink_takeoff as mt

    class _Par:
        def __init__(self, geri):
            self.yazilan, self.geri, self.cagri = [], geri, []

        def ensure_param(self, ad, deger):
            self.yazilan.append((ad, deger)); return True

        def get_param(self, ad, timeout=5.0):
            return None if self.geri is None else (self.geri, 9)
    eski_to = mt.take_off
    try:
        for geri, bekle in ((25.0, True), (120.0, False), (None, False)):
            v = _Par(geri)
            mt.take_off = lambda a_, alt, c_, v=v: (v.cagri.append(alt), True)[1]
            ok = yg.kalkis_komutu(v, 25.0, param_bekleme=0.0, log=lambda m: None)
            tamam(ok == bekle and v.yazilan == [("MIS_TAKEOFF_ALT", 25.0)] and v.cagri == ([25.0] if bekle else []),
                  f"kalkış komutu: MIS_TAKEOFF_ALT geri okuma {geri} → {'take_off' if bekle else 'iptal'}")
    finally:
        mt.take_off = eski_to


# ---- [2] erişilebilirlik ---------------------------------------------------------------------------------

def erisilebilirlik():
    say = {"toplam": 0, "uygun": 0, "sapma0": 0, "red": {}}
    for c in HATLAR:
        for n in (-140, -100, -60, -20, 20, 60, 100, 140):
            for cikis in (1, -1):
                p, _ = plan_icin((float(n), float(c)), (cikis * 310.0, c + cikis * 80.0, 60.0))
                say["toplam"] += 1
                if p["uygun"]:
                    say["uygun"] += 1
                    say["sapma0"] += p["yon_sapma_deg"] == 0
                else:
                    k = (p.get("ilk_red") or p["neden"]).split("(")[0].strip()
                    say["red"][k] = say["red"].get(k, 0) + 1
    tamam(say["toplam"] == 144 and say["uygun"] >= 60, f"üs erişilebilirliği ≥ 60/144 ({say['uygun']})")
    return say


# ---- [3] sahte kinematik araçla akış -------------------------------------------------------------------

class _Zaman:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def time(self):
        return self.t

    def strftime(self, f):
        return f"T+{self.t - 1000.0:6.1f}"


class _VC:
    VEHICLE_CMD_DO_SET_MODE, VEHICLE_CMD_DO_REPOSITION = 176, 192


class _VCA:
    VEHICLE_CMD_RESULT_ACCEPTED = 0


class _YerArac:
    """Kaba araç modeli (düğüm arayüzü lora_yaklasma.Yaklasma ile aynı adlar). Modlar: yer → kalkis (pist yönünde
    hızlanma, 40 m koşu, tırmanış) → hold (kalkış irtifasında çember) → offboard (hat setpoint izleme; dikey TECS
    referans modeli) → gorev (iniş orbiti → teğet çıkış → süzülme → yer → 2 sn sonra disarm)."""

    def __init__(self, zaman, pist_deg, kalkis_irtifa, desen, kalkar=True, gorev_kabul=True):
        # kalkar: True = normal; False = hiç yuvarlanmaz (arm sonrası COM_DISARM_PRFLT);
        # "yuvarlanma" = a1 arızası (yerde 8 m/s yuvarlanır, hiç havalanmaz, PX4 normal disarm'ı reddeder)
        self.z, self.kalkar, self.gorev_kabul, self.desen = zaman, kalkar, gorev_kabul, desen
        self.n = self.e = self.h = 0.0
        self.rota = math.radians(pist_deg)
        self.V, self.vzv, self.ref = 0.0, 0.0, 0.0
        self.mod, self.armed, self.landed = "yer", False, True
        self.pist_izi = 0.0
        self.kalkis_irtifa = kalkis_irtifa
        self.sp, self.ack, self.komutlar, self.mav_komut = None, {}, [], []
        self.hold_c, self.hold_alt = (0.0, 0.0), 0.0
        self.hedef, self.pos0, self.iz = None, None, []
        self.t_yer = None
        self.pub_cmd = types.SimpleNamespace(publish=self._komut)

    @property
    def pos(self):
        return types.SimpleNamespace(x=self.n, y=self.e, z=-self.h, vx=self.V * math.cos(self.rota), vy=self.V * math.sin(self.rota),
                                     vz=-self.vzv, ref_lat=0.0, ref_lon=0.0, ref_alt=0.0, heading=self.rota, xy_valid=True)

    @property
    def mode(self):
        o = self.mod == "offboard"
        return types.SimpleNamespace(flag_control_offboard_enabled=o, flag_control_auto_enabled=not o, flag_armed=self.armed)

    def offboard(self):
        return self.mod == "offboard"

    def ts(self):
        return int(self.z.t * 1e6)

    def stream(self, position, velocity):
        self.sp = (position, velocity)

    def nav(self):
        return {"yer": 4, "kalkis": 17, "hold": 4, "offboard": 14, "gorev": 3}[self.mod]

    def kalkis_komutu(self, irtifa):
        self.mav_komut.append(("takeoff+arm", irtifa))
        self.armed = True
        if self.kalkar is True:
            self.mod = "kalkis"
        elif self.kalkar == "yuvarlanma":
            self.mod = "yuvarlanma"
        return True

    def command(self, cmd, *prm):
        self.mav_komut.append((cmd, prm))
        if cmd == yg.MAV_CMD_COMPONENT_ARM_DISARM and prm and prm[0] == 0:
            zorla = len(prm) > 1 and prm[1] == yg.ZORLA_DISARM
            if not self.landed and not zorla:
                return 4                                # PX4: "Disarming denied: not landed" (a1'de görüldü)
            self.armed = False
            if self.mod == "yuvarlanma":
                self.mod = "fren"                       # motor kesildi: yerde yavaşlar
            return 0
        return 0

    def _komut(self, msg):
        cmd, prm = msg
        self.komutlar.append((round(self.z.t - 1000.0, 1), cmd, prm))
        if cmd == _VC.VEHICLE_CMD_DO_SET_MODE and prm[1] == 6.0 and self.mod in ("hold", "kalkis", "offboard"):
            assert self.sp is not None, "setpoint akışı olmadan OFFBOARD istendi"
            assert not self.landed, "yerdeyken OFFBOARD istendi"
            self.mod = "offboard"; self.ref = self.h
        elif cmd == _VC.VEHICLE_CMD_DO_SET_MODE and prm[1] == 4.0 and prm[2] == 4.0:
            if self.gorev_kabul and not self.landed:
                self.mod = "gorev"; self.ref = self.h; self.faz = "orbit"
        elif cmd == _VC.VEHICLE_CMD_DO_SET_MODE and prm[1] == 4.0 and not self.landed:
            self.mod = "hold"
            self.hold_c, self.hold_alt = (self.n - 80.0 * math.sin(self.rota), self.e + 80.0 * math.cos(self.rota)), self.h
        self.ack[cmd] = _VCA.VEHICLE_CMD_RESULT_ACCEPTED

    def _don(self, ist, dt, w=0.35):
        fark = (ist - self.rota + math.pi) % (2 * math.pi) - math.pi
        self.rota += max(-w * dt, min(w * dt, fark))

    def _dikey(self, alt_kom, dt, sink=2.4):
        ad = max(-4.0 * dt, min(5.0 * dt, alt_kom - self.ref))
        self.ref += ad
        vk = max(-sink, min(6.0, (self.ref - self.h) / 5.0 + 0.3 * ad / dt))
        self.vzv += (vk - self.vzv) * dt
        self.h += self.vzv * dt

    def _cember(self, c, R):
        dn, de = c[0] - self.n, c[1] - self.e
        uz = math.hypot(dn, de)
        return math.atan2(de, dn) - (math.pi / 2 - max(-0.6, min(math.pi / 2, math.atan2(uz - R, 40.0) * 2)))

    def adim(self, dt):
        if self.mod in ("yuvarlanma", "fren"):
            # a1 arızası: hızlanır, 13 m/s'de rotasyon denemesiyle 8 m/s'ye düşer, hiç havalanmaz; land_detected
            # "yerde değil" der (yer hızı > LNDFW_VEL_XY_MAX) → normal disarm reddedilir.
            hedef_v = 0.0 if self.mod == "fren" else (8.0 if self.pist_izi > 20.0 else 14.0)
            self.V += (hedef_v - self.V) * dt / 1.5
            self.landed = self.V < 5.0
            self.h = 0.0
            self.vzv = 0.0
            self.n += self.V * math.cos(self.rota) * dt
            self.e += self.V * math.sin(self.rota) * dt
            self.pist_izi += self.V * dt
            self.iz.append((self.n, self.e, self.h, self.mod))
            return
        if self.mod == "yer":
            if self.t_yer is not None and self.armed and self.z.t - self.t_yer > 2.0:
                self.armed = False                      # COM_DISARM_LAND
            elif self.t_yer is None and self.armed and not self.kalkar:
                self.t_arm = getattr(self, "t_arm", self.z.t)
                if self.z.t - self.t_arm > 10.0:
                    self.armed = False                  # COM_DISARM_PRFLT
            return
        if self.mod == "kalkis":
            self.V = min(22.0, self.V + 4.0 * dt)
            if math.hypot(self.n, self.e) > 40.0:
                self.vzv = 6.0; self.h += self.vzv * dt
                self.landed = self.h < 0.3
            if self.h >= self.kalkis_irtifa:
                self.mod = "hold"; self.ref = self.h
                self.hold_c, self.hold_alt = (self.n - 80.0 * math.sin(self.rota), self.e + 80.0 * math.cos(self.rota)), self.h
        elif self.mod == "hold":
            self.V += (15.0 - self.V) * dt / 6.0
            self._don(self._cember(self.hold_c, 80.0), dt)
            self._dikey(self.hold_alt, dt)
        elif self.mod == "offboard":
            self.V += (15.0 - self.V) * dt / 6.0
            (pn, pe, pz), (vn, ve, _vz) = self.sp
            psi = math.atan2(ve, vn)
            x = -(self.n - pn) * math.sin(psi) + (self.e - pe) * math.cos(psi)
            self._don(psi - math.atan2(x, 60.0), dt, 3.5 / max(self.V, 1.0))
            self._dikey(-pz, dt)
        elif self.mod == "gorev":
            d = self.desen
            dl, _rl = yg._birim(d["inis_yonu_deg"])
            if self.faz == "orbit":
                self.V += (15.0 - self.V) * dt / 6.0
                self._don(self._cember(d["Lc"], 80.0), dt)
                yakin = math.hypot(self.n - d["Lc"][0], self.e - d["Lc"][1]) < 110.0
                self._dikey(d["giris_irtifa_m"] if yakin else self.h, dt, 2.0)
                rh = (math.degrees(self.rota) - d["inis_yonu_deg"] + 180.0) % 360.0 - 180.0
                if self.h <= d["giris_irtifa_m"] + 2.0 and abs(rh) < 10.0 and math.dist((self.n, self.e), d["G"]) < 40.0:
                    self.faz = "suzulme"
            else:
                self.V += (11.0 - self.V) * dt / 4.0
                x = -(self.n - d["Td"][0]) * dl[1] + (self.e - d["Td"][1]) * dl[0]
                self._don(math.radians(d["inis_yonu_deg"]) - math.atan2(x, 40.0), dt)
                kalan = -((self.n - d["Td"][0]) * dl[0] + (self.e - d["Td"][1]) * dl[1])
                self.h = max(0.0, min(self.h, d["giris_irtifa_m"] * kalan / d["yaklasma_m"]))
                self.vzv = -1.0
                if self.h <= 0.0:
                    self.landed = True; self.mod = "yer"; self.V = 0.0; self.vzv = 0.0; self.t_yer = self.z.t
        self.n += self.V * math.cos(self.rota) * dt
        self.e += self.V * math.sin(self.rota) * dt
        self.iz.append((self.n, self.e, self.h, self.mod))


def sahte_akis(ek_arg=(), hedef0=QR0, kalkar=True, gorev_kabul=True, gorev_yukleme=True):  # noqa: C901
    """_dalis_akisi'yı (--baslangic yer) sahte araçla koştur. Dönüş: (kod, res, araç, log satırları)."""
    args = ly.build_parser().parse_args(["--profil", "dalis", "--alcalma-siniri", "4.0", "--baslangic", "yer", "--uav0-yaricap", "0",
                                         "--kaynak-fark-n", str(US[0]), "--kaynak-fark-e", str(US[1]), "--pist-yonu", str(PIST), *ek_arg])
    zaman = _Zaman()
    desen = yg.inis_deseni(args.pist_yonu, args.td_uzaklik, args.inis_yaklasma, args.inis_giris_irtifa, args.loiter_yaricap)
    arac = _YerArac(zaman, PIST, args.kalkis_irtifa, desen, kalkar, gorev_kabul)
    arac.hedef = u1(hedef0)

    def spin_once(node, timeout_sec=0.05):
        dt = max(0.02, min(timeout_sec, 0.1))
        zaman.t += dt
        node.adim(dt)
    adlar = {3: "AUTO_MISSION", 4: "AUTO_LOITER", 14: "OFFBOARD", 17: "AUTO_TAKEOFF"}
    nav = type("Nav", (), {"nav": property(lambda self: arac.nav()), "fresh": lambda self, a=2.0: True,
                           "name": lambda self: adlar[arac.nav()]})()
    res = {}
    c = types.SimpleNamespace(
        node=arac, nav=nav, res=res, rclpy=types.SimpleNamespace(spin_once=spin_once), LOOP_HZ=20, tespit_t=zaman.t,
        rtf_ort=lambda: 0.95, abort_nedeni=lambda: None, abort_nedeni_kritik=lambda: None, AUTO_LOITER=4,
        VehicleCommand=_VC, VehicleCommandAck=_VCA,
        make_vehicle_command=lambda uav, ts, cmd, *prm: (cmd, tuple(float(v) for v in prm)),
        ne_to_latlon=lambda n, e, rl, ro: (n, e),
        control_mode_name=lambda m: "OFFBOARD" if m.flag_control_offboard_enabled else "AUTO", arac=arac)
    satirlar = []
    eski = (ly.time, ly._log, yg.time, yg.kalkis_komutu, yg.gorev_yukle)
    ly.time = yg.time = zaman
    ly._log = lambda m: satirlar.append(f"{zaman.strftime('')}  {m}")
    yg.kalkis_komutu = lambda a, irtifa, param_bekleme=0.0, log=None: a.kalkis_komutu(irtifa)
    yg.gorev_yukle = lambda a, og, asim=10.0: (gorev_yukleme, "sahte ACK")
    try:
        kod = ly._dalis_akisi(args, c)
    finally:
        ly.time, ly._log, yg.time, yg.kalkis_komutu, yg.gorev_yukle = eski
    return kod, res, arac, satirlar


def akislar(cikti):
    ozet = []
    # A) tam görev
    kod, res, a, log = sahte_akis()
    open(os.path.join(cikti, "akis_tam.log"), "w").write("\n".join(log) + "\n")
    T = u1(QR0)
    tamam(kod == 0 and res["sonuc"] == "indi", f"tam görev: indi ve disarm oldu (kod {kod}, {res.get('sonuc')}, {res.get('neden')})")
    tamam(res["plan"]["giris"] == "yer" and "ara_noktalar" in res["plan"] and not any(k[1] == 192 for k in a.komutlar), "hizalanma çemberi yok (DO_REPOSITION hiç gönderilmedi)")
    tamam(a.mav_komut[0][0] == "takeoff+arm" and res["kalkis"]["yer_kosusu_m"] > 30.0, "arm kalkış komutuyla birlikte; teker kesme ölçüldü")
    tamam(abs(res["hat_girisi"]["yanal_m"]) < 10.0 and abs(res["hat_girisi"]["irtifa_m"] - GEO["giris_irtifa_m"]) < 6.0, f"hat girişi: yanal {res['hat_girisi']['yanal_m']} m, irtifa {res['hat_girisi']['irtifa_m']} m")
    tamam(res["durma_nedeni"] == "durma-mesafesi (3b)" and res["durma"]["3b_m"] <= 20.0, f"3B durma ({res['durma']['3b_m']} m)")
    tamam(res["dip_irtifa_m"] >= GEO["min_irtifa_m"], "dip irtifa tabanın üstünde")
    ucus = [q for q in a.iz if q[2] > 1.0]
    en_direk = min(direk_mesafesi(q[0], q[1]) for q in ucus if q[2] < 155.0)
    tamam(en_direk >= 45.0, f"tüm uçuş izi direklere ≥ 45 m ({en_direk:.0f} m)")
    tamam(abs(res["inis"]["yanal_m"]) < 5.0 and abs(res["inis"]["boyuna_m"]) < 30.0 and not a.armed and a.landed, f"iniş pist ekseninde (yanal {res['inis']['yanal_m']}, boyuna {res['inis']['boyuna_m']})")
    sira = [s for s in log if any(k in s for k in ("KALKIŞ KOMUTU", "TEKER KESME", "OFFBOARD ✔", "HAT GİRİŞİ", "DURDU", "DÖNÜŞ bacağı 1", "AUTO_MISSION ✔", "YER ALGILANDI", "DISARM ✔"))]
    tamam(len(sira) == 9, f"aşama sırası tam ({len(sira)}/9)")
    ozet.append(("tam görev", kod, res))
    # B) NO-GO: hiç arm edilmez
    kod, res, a, log = sahte_akis(hedef0=(0.0, -120.0))
    tamam(kod == 3 and res["sonuc"] == "no-go" and not a.mav_komut and not a.komutlar and not a.armed and a.landed, "NO-GO: uav1 hiç arm edilmedi, komut yok")
    tamam(any("KALKMADI" in s for s in log), "NO-GO nedeni loglandı")
    ozet.append(("NO-GO (direğe yakın hedef)", kod, res))
    kod, res, a, log = sahte_akis(gorev_yukleme=False)
    tamam(kod == 3 and not a.mav_komut and not a.armed, "iniş görevi yüklenemezse kalkış yok")
    # C) kalkış başarısız (hiç yuvarlanmadı) → normal disarm yeter
    kod, res, a, log = sahte_akis(kalkar=False, ek_arg=("--kalkis-asimi", "8", "--kalkis-irtifa-asimi", "6"))
    tamam(kod == 4 and res["sonuc"] == "kalkis_basarisiz" and not a.armed and res["kalkis"]["guvenli_mod"] == "disarm", "kalkış başarısız (yerde duruyor): normal disarm yetti")
    tamam(not any(k[1] == 176 and k[2][1] == 6.0 for k in a.komutlar), "kalkış başarısızken OFFBOARD istenmedi")
    tamam(not any(isinstance(k[1], tuple) and len(k[1]) > 1 and k[1][1] == yg.ZORLA_DISARM for k in a.mav_komut), "gereksiz zorla disarm yok")
    ozet.append(("kalkış başarısız (yerde)", kod, res))
    # C2) a1 arızası: yerde yuvarlanır, havalanmaz → irtifa kapısı → normal disarm reddedilir → ZORLA disarm
    kod, res, a, log = sahte_akis(kalkar="yuvarlanma", ek_arg=("--kalkis-irtifa-asimi", "15"))
    open(os.path.join(cikti, "akis_yuvarlanma.log"), "w").write("\n".join(log) + "\n")
    k = res["kalkis"]
    tamam(kod == 4 and res["sonuc"] == "kalkis_basarisiz" and "irtifa kazanılmadı" in k["hata"], f"yuvarlanma: irtifa kapısı tetikledi ({k.get('hata')})")
    tamam(k["disarm_ack"] == 4 and k.get("zorla_disarm_ack") == 0 and k["guvenli_mod"] == "zorla_disarm" and not a.armed, "normal disarm reddedildi → zorla disarm (21196) ile motor kesildi")
    tamam(any(isinstance(x[1], tuple) and len(x[1]) > 1 and x[1][1] == yg.ZORLA_DISARM for x in a.mav_komut) and k["durum"]["yerde"], "zorla disarm yalnız 'yerde' doğrulandıktan sonra")
    v0 = a.V
    for _ in range(300):                                 # motor kesik: 15 sn daha modeli ilerlet
        a.adim(0.05)
    tamam(a.mod == "fren" and a.V < 0.5 * v0 and a.V < 2.0 and max(q[2] for q in a.iz) < 0.5, f"motor kesildikten sonra araç yerde yavaşladı ({v0:.1f} → {a.V:.1f} m/s)")
    tamam(not any(x[1] == 176 and x[2][1] == 6.0 for x in a.komutlar), "yuvarlanmada OFFBOARD hiç istenmedi")
    ozet.append(("a1 arızası (yuvarlanma)", kod, res))
    # C3) pist sonu kapısı: irtifa kapısından önce mesafe aşılırsa
    kod, res, a, log = sahte_akis(kalkar="yuvarlanma", ek_arg=("--kalkis-irtifa-asimi", "600", "--pist-boyu", "60"))
    tamam(kod == 4 and "pist sonu" in res["kalkis"]["hata"] and not a.armed, f"pist sonu kapısı ({res['kalkis'].get('hata')})")
    tamam(res["kalkis"]["durum"]["yerde"] and max(q[2] for q in a.iz) < 0.5, "pist sonu: araç yerde durduruldu")
    ozet.append(("pist sonu kapısı", kod, res))
    # C4) havada başarısızlık: zorla disarm ASLA
    class _Hep:
        def __getattr__(self, k):
            raise AssertionError("havadayken komut gönderildi")
    kod2, res2, a2, _l = sahte_akis(ek_arg=("--kalkis-irtifa", "4000", "--kalkis-asimi", "25", "--kalkis-irtifa-asimi", "600"))
    k2 = res2["kalkis"]
    tamam(kod2 == 4 and not k2["durum"]["yerde"] and k2["guvenli_mod"] == "hold" and a2.armed, "havada kalkış başarısızlığı: Hold, disarm YOK")
    tamam(not any(x[0] == yg.MAV_CMD_COMPONENT_ARM_DISARM for x in a2.mav_komut if x[0] != "takeoff+arm"), "havadayken hiçbir disarm komutu gönderilmedi")
    ozet.append(("havada kalkış başarısızlığı", kod2, res2))
    # D) görev moduna geçilemedi → Hold (güvenli mod)
    kod, res, a, log = sahte_akis(gorev_kabul=False)
    tamam(kod == 4 and res["sonuc"] == "donus_basarisiz" and a.mod == "hold" and res["donus"]["guvenli_mod_hold"], "görev modu reddi: Hold'a alındı")
    ozet.append(("görev modu reddi", kod, res))
    # E) hedefsiz (Aşama 1)
    kod, res, a, log = sahte_akis(ek_arg=("--yalniz-kalkis-inis", "--guvenli-irtifa", "60"))
    open(os.path.join(cikti, "akis_hedefsiz.log"), "w").write("\n".join(log) + "\n")
    tamam(kod == 0 and res["sonuc"] == "indi" and "durma" not in res and max(q[2] for q in a.iz) < 75.0, f"hedefsiz akış: kalkış → dönüş → iniş (kod {kod})")
    tamam(abs(res["inis"]["yanal_m"]) < 5.0, "hedefsiz akış: iniş pist ekseninde")
    ozet.append(("hedefsiz (Aşama 1)", kod, res))
    return ozet


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cikti", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "test",
                                                    f"yer_gorevi_{datetime.date.today().isoformat()}"))
    args = ap.parse_args()
    os.makedirs(args.cikti, exist_ok=True)
    birim()
    print(f"[1] birim: {N_OK} doğrulama ✔")
    say = erisilebilirlik()
    print(f"[2] üs ({US[0]:.0f}, {US[1]:.0f}), pist {PIST:.0f}°: {say['uygun']}/{say['toplam']} durumda plan var "
          f"(sapmasız {say['sapma0']}); retler: {dict(sorted(say['red'].items(), key=lambda kv: -kv[1])[:4])}")
    n1 = N_OK
    ozet = akislar(args.cikti)
    print(f"[3] sahte akış: {N_OK - n1} doğrulama ✔")
    for ad, kod, res in ozet:
        print(f"    {ad}: kod {kod}, sonuç {res.get('sonuc')}"
              + (f", kalkış {res['kalkis'].get('kalkis_mesafe_m')} m, hat girişi yanal {res['hat_girisi']['yanal_m']} m, "
                 f"3B {res['durma']['3b_m']} m, iniş yanal {res['inis']['yanal_m']} m" if "hat_girisi" in res and "inis" in res and "yanal_m" in res["inis"] else ""))
    with open(os.path.join(args.cikti, "kuru_sonuc.json"), "w") as f:
        json.dump({"erisilebilirlik": say, "akislar": [{"ad": a, "kod": k, "res": r} for a, k, r in ozet]}, f, ensure_ascii=False, indent=1, default=str)
    print(f"\n{N_OK} doğrulama geçti; çıktı {args.cikti}/")


if __name__ == "__main__":
    main()
