#!/usr/bin/env python3
"""
ayrim.py — iki araç arası ayrım denetimi için ortak yardımcı (qr_pass.py taşıma mantığı kullanır;
lora_yaklasma.py planlamasında da kullanılabilecek şekilde yazıldı).

Neden: 2026-11-11 gösteriminde uav0 H'ye taşınırken uav1'in Hold çemberinin yanından geçti; iki araç
yatay 54 m / dikey 21 m yaklaştı. Taşıma mantığı yalnız sabit engelleri (direkler) denetliyordu.

İki katman:
  1. SAF FONKSİYONLAR (ROS yok, kuru testte kullanılır): öteki aracın yakın geçmişinden "Hold'da mı, çemberi
     nerede" kestirimi (hold_tahmini) ve bir bacağın / bir Hold çemberinin öteki araca göre en kötü durum 3B
     ayrımı (bacak_ayrimi, cember_ayrimi).
  2. DigerAracIzleyici: öteki aracın vehicle_local_position'ını doğrudan topic'ten dinler, kendi aracın
     yerel NED çerçevesine çevirir, geçmişi tutar, gerçekleşen en küçük 3B ayrımı kaydeder.

Model (muhafazakâr):
  - Öteki araç Hold'daysa (saat yönünde, yarıçap HOLD_R) işgal ettiği bölge = çember merkezi çevresinde
    HOLD_R + HOLD_TOL yarıçaplı disk, irtifa bandı = son irtifa ± IRTIFA_BANDI. Merkez hız vektöründen
    kestirilir (merkez = konum + HOLD_R · sağ-dik(hız)); son HOLD_PENCERE sn'deki kestirimler birbirini
    tutuyorsa Hold kabul edilir.
  - Öteki araç duruyorsa (yerde): bölge = nokta.
  - Öteki araç hareketli (Hold'da değil) ya da verisi bayatsa: nerede olacağı bilinmiyor → yalnız DİKEY ayrım
    tek başına yetiyorsa bacak kabul edilir, yoksa reddedilir (çağıran bekler).
  - Kendi bacağımız: a → b düz hattı ± iz_payi (Hold'dan çıkış dönüşü + NPFG taşması, qr_pass TRACK_MARGIN),
    irtifa aralığı [min(alt_a, alt_b), max(...)] (PX4 irtifayı yol boyunca değiştirir).
  - En kötü durum 3B ayrımı = hypot(max(0, yatay boşluk), dikey boşluk) ≥ min_ayrim olmalı.

Çerçeve: her PX4 örneğinin yerel NED orijini kendi doğma noktasıdır. SPAWN_NE = doğma noktaları, uav0
çerçevesinde (launch_all.sh UAV_POSES: uav1 ENU (−15, 15) → NED (N 15, E −15)).
"""
import collections
import math
import os
import time

SPAWN_NE = {0: (0.0, 0.0), 1: (15.0, -15.0)}
# uav1 başka bir üsten doğuyorsa (yerden kalkış senaryosu, 2026-11-15; launch_all.sh UAV1_POSE ile birlikte):
# IHA_UAV1_SPAWN_NE="N,E" (uav0 çerçevesi, m). Verilmezse eski (15, −15).
if os.environ.get("IHA_UAV1_SPAWN_NE"):
    SPAWN_NE[1] = tuple(float(v) for v in os.environ["IHA_UAV1_SPAWN_NE"].split(","))
TOPIC_ONEK = {0: "/fmu", 1: "/px4_1/fmu"}
HOLD_R = 80.0            # NAV_LOITER_RAD
HOLD_TOL = 20.0          # çember izleme toleransı [m]
IRTIFA_BANDI = 10.0      # Hold'da irtifa salınımı payı [m] (ölçülen ±6 m)
HOLD_PENCERE = 5.0       # Hold kestirimi için gereken geçmiş [sn]
HOLD_TUTARLILIK = 25.0   # merkez kestirimlerinin ortalamadan en büyük sapması [m]
BAYAT = 2.0              # bu kadar sn'den eski veri bayat
MIN_AYRIM = 40.0         # projedeki diğer ayrım eşikleriyle aynı (lora_yaklasma --ayrim-min, direk payı)


def cerceve_cevir(n, e, kaynak, hedef):
    """kaynak aracın yerel NED'indeki (n, e) noktasını hedef aracın yerel NED'ine çevirir."""
    return (n + SPAWN_NE[kaynak][0] - SPAWN_NE[hedef][0], e + SPAWN_NE[kaynak][1] - SPAWN_NE[hedef][1])


def hold_tahmini(gecmis, simdi, hold_r=HOLD_R):
    """gecmis: [(t, n, e, irtifa, vn, ve), …] (eski → yeni). Öteki aracın durumu:
    {"tur": "yok" | "bayat" | "duruyor" | "hold" | "hareketli", "konum", "irtifa", "merkez", "yaricap"}."""
    if not gecmis:
        return {"tur": "yok"}
    t, n, e, alt, vn, ve = gecmis[-1]
    d = {"konum": (n, e), "irtifa": alt, "yas": simdi - t}
    if simdi - t > BAYAT:
        return dict(d, tur="bayat")
    if math.hypot(vn, ve) < 3.0:
        return dict(d, tur="duruyor", merkez=(n, e), yaricap=0.0)
    pencere = [g for g in gecmis if simdi - g[0] <= HOLD_PENCERE + 1.0]
    if pencere[-1][0] - pencere[0][0] < HOLD_PENCERE * 0.8 or len(pencere) < 5:
        return dict(d, tur="hareketli", neden="geçmiş kısa")
    merkezler = []
    for _, pn, pe, _, pvn, pve in pencere:
        v = math.hypot(pvn, pve)
        if v < 3.0:
            return dict(d, tur="hareketli", neden="hız düşük")
        merkezler.append((pn - hold_r * pve / v, pe + hold_r * pvn / v))      # sağ-dik = (−ve, vn)
    mn = sum(m[0] for m in merkezler) / len(merkezler)
    me = sum(m[1] for m in merkezler) / len(merkezler)
    sapma = max(math.hypot(m[0] - mn, m[1] - me) for m in merkezler)
    if sapma > HOLD_TUTARLILIK:
        return dict(d, tur="hareketli", neden=f"merkez sapması {sapma:.0f} m")
    alts = [g[3] for g in pencere]
    return dict(d, tur="hold", merkez=(mn, me), yaricap=hold_r + HOLD_TOL, irtifa_min=min(alts), irtifa_max=max(alts))


def _nokta_parca(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dy - p[1])


def _dikey_bosluk(alt_lo, alt_hi, diger):
    lo = diger.get("irtifa_min", diger["irtifa"]) - IRTIFA_BANDI
    hi = diger.get("irtifa_max", diger["irtifa"]) + IRTIFA_BANDI
    return max(0.0, lo - alt_hi, alt_lo - hi)


def _karar(yatay, dikey, min_ayrim, diger, ne):
    en_kotu = math.hypot(max(0.0, yatay), dikey) if yatay is not None else dikey
    ok = en_kotu >= min_ayrim
    y = "bilinmiyor" if yatay is None else f"{yatay:.0f} m"
    return ok, en_kotu, (f"{ne}: öteki araç {diger['tur']}, en kötü durum yatay boşluk {y}, dikey boşluk {dikey:.0f} m → "
                         f"3B {en_kotu:.0f} m {'≥' if ok else '<'} {min_ayrim:.0f} m")


def bacak_ayrimi(a, b, alt_a, alt_b, diger, min_ayrim=MIN_AYRIM, iz_payi=25.0):
    """a → b bacağının (irtifa alt_a → alt_b) öteki araca en kötü durum 3B ayrımı.
    Dönüş: (uygun, en_kotu_m, açıklama). diger = hold_tahmini() çıktısı."""
    if diger["tur"] == "yok":
        return True, math.inf, "öteki araç verisi yok — ayrım denetimi yapılamadı"
    dikey = _dikey_bosluk(min(alt_a, alt_b), max(alt_a, alt_b), diger)
    if diger["tur"] in ("bayat", "hareketli"):
        return _karar(None, dikey, min_ayrim, diger, "bacak")
    yatay = _nokta_parca(diger["merkez"], a, b) - diger["yaricap"] - iz_payi
    return _karar(yatay, dikey, min_ayrim, diger, "bacak")


def cember_ayrimi(merkez, alt, diger, min_ayrim=MIN_AYRIM, hold_r=HOLD_R):
    """merkez çevresinde alt irtifasında tutulacak Hold çemberinin öteki araca en kötü durum 3B ayrımı."""
    if diger["tur"] == "yok":
        return True, math.inf, "öteki araç verisi yok — ayrım denetimi yapılamadı"
    dikey = _dikey_bosluk(alt, alt, diger)
    if diger["tur"] in ("bayat", "hareketli"):
        return _karar(None, dikey, min_ayrim, diger, "Hold çemberi")
    yatay = math.dist(merkez, diger["merkez"]) - diger["yaricap"] - (hold_r + HOLD_TOL)
    return _karar(yatay, dikey, min_ayrim, diger, "Hold çemberi")


def kendi_hold_merkezi(n, e, vn, ve, hold_r=HOLD_R):
    """Saat yönünde Hold'da dönen aracın çember merkezi kestirimi (hız < 3 m/s ise None)."""
    v = math.hypot(vn, ve)
    return None if v < 3.0 else (n - hold_r * ve / v, e + hold_r * vn / v)


class AyrimDenetimi:
    """Öteki aracın geçmişini tutar; bacak / çember denetimini ve gerçekleşen en küçük ayrımı verir.
    ROS'tan bağımsız: guncelle() ile beslenir (kuru testte doğrudan, uçuşta DigerAracIzleyici üzerinden)."""

    def __init__(self, min_ayrim=MIN_AYRIM, iz_payi=25.0, saat=time.monotonic):
        self.min_ayrim, self.iz_payi, self.saat = min_ayrim, iz_payi, saat
        self.gecmis = collections.deque(maxlen=400)
        self.en_kucuk = None          # (3B, yatay, dikey, duvar saati)
        self.goruldu = False

    def guncelle(self, n, e, alt, vn, ve, kendi=None):
        """Öteki aracın (kendi çerçevemizdeki) yeni konumu; kendi = (n, e, alt) verilirse en küçük ayrım izlenir."""
        self.goruldu = True
        self.gecmis.append((self.saat(), n, e, alt, vn, ve))
        if kendi is not None:
            y, d = math.hypot(kendi[0] - n, kendi[1] - e), abs(kendi[2] - alt)
            u = math.hypot(y, d)
            if self.en_kucuk is None or u < self.en_kucuk[0]:
                self.en_kucuk = (u, y, d, time.strftime("%H:%M:%S"))

    def diger(self):
        return hold_tahmini(list(self.gecmis), self.saat())

    def bacak(self, a, b, alt_a, alt_b):
        if not self.goruldu:
            return True, math.inf, "öteki araç hiç görülmedi — ayrım denetimi kapalı"
        return bacak_ayrimi(a, b, alt_a, alt_b, self.diger(), self.min_ayrim, self.iz_payi)

    def cember(self, merkez, alt):
        if not self.goruldu:
            return True, math.inf, "öteki araç hiç görülmedi — ayrım denetimi kapalı"
        return cember_ayrimi(merkez, alt, self.diger(), self.min_ayrim)

    def ozet(self):
        if self.en_kucuk is None:
            return None
        u, y, d, s = self.en_kucuk
        return {"3b_m": round(u, 1), "yatay_m": round(y, 1), "dikey_m": round(d, 1), "saat": s}


class DigerAracIzleyici(AyrimDenetimi):
    """Öteki aracın konumunu doğrudan PX4 topic'inden dinler (verilen rclpy düğümüne abonelik ekler).
    kendi_konum: çağrıldığında kendi aracın VehicleLocalPosition mesajını (ya da None) döndüren fonksiyon."""

    def __init__(self, node, kendi_uav, diger_uav, kendi_konum=None, min_ayrim=MIN_AYRIM, iz_payi=25.0):
        super().__init__(min_ayrim, iz_payi)
        from px4_msgs.msg import VehicleLocalPosition
        from v2_orbit_mission import SENSOR_QOS
        self.kendi_uav, self.diger_uav, self.kendi_konum = kendi_uav, diger_uav, kendi_konum
        node.create_subscription(VehicleLocalPosition, f"{TOPIC_ONEK[diger_uav]}/out/vehicle_local_position_v1",
                                 self._mesaj, SENSOR_QOS)

    def _mesaj(self, m):
        n, e = cerceve_cevir(m.x, m.y, self.diger_uav, self.kendi_uav)
        k = self.kendi_konum() if self.kendi_konum else None
        self.guncelle(n, e, -m.z, m.vx, m.vy, None if k is None else (k.x, k.y, -k.z))
