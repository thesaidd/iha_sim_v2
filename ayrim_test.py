#!/usr/bin/env python3
"""
ayrim_test.py — ayrim.py kuru testi (ROS yok). 2026-11-11 gösterimindeki yakınlaşmayı (uav0 H'ye giderken
uav1'e yatay 54 m / dikey 21 m) KAYITLI İZDEN yeniden oynatır ve yeni denetimin bacağı reddettiğini doğrular.

Kullanım: python3 ayrim_test.py
"""
import csv
import math
import os
import sys

import ayrim as A

IZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test/dalis_gosterim_2026-11-11/iz.csv")
H = (543.0, 196.0)
n_ok = 0


def dogrula(k, ad):
    global n_ok
    if not k:
        print("✘", ad)
        sys.exit(1)
    n_ok += 1
    print("✔", ad)


def sn(s):
    h, m, x = s.split(":")
    return int(h) * 3600 + int(m) * 60 + int(x)


# ── 1. Çerçeve çevirimi ───────────────────────────────────────────────────────────────────────────
dogrula(A.cerceve_cevir(0, 0, 1, 0) == (15.0, -15.0) and A.cerceve_cevir(15, -15, 0, 1) == (0.0, 0.0),
        "çerçeve: uav1 orijini uav0 çerçevesinde (15, −15), tersi tutarlı")

# ── 2. Sentetik Hold: merkez ve tür kestirimi ─────────────────────────────────────────────────────
def cember_gecmisi(c, alt, t0, sure, hz=2.0, r=80.0, v=15.0, faz=0.0):
    g = []
    for i in range(int(sure * hz) + 1):
        t = t0 + i / hz
        a = faz + v / r * (t - t0)                    # saat yönü (NED'de açı artar)
        g.append((t, c[0] + r * math.cos(a), c[1] + r * math.sin(a), alt, -v * math.sin(a), v * math.cos(a)))
    return g

g = cember_gecmisi((45.7, 330.3), 119.0, 0.0, 8.0)
d = A.hold_tahmini(g, 8.0)
dogrula(d["tur"] == "hold" and math.dist(d["merkez"], (45.7, 330.3)) < 1.0 and d["yaricap"] == 100.0,
        f"sentetik Hold: tür hold, merkez ({d['merkez'][0]:.1f}, {d['merkez'][1]:.1f}), bölge yarıçapı {d['yaricap']:.0f} m")
dogrula(A.hold_tahmini(g, 11.0)["tur"] == "bayat", "veri 3 sn eski → bayat")
duz = [(t / 2, 10.0 * t, 0.0, 100.0, 15.0, 0.0) for t in range(17)]
dogrula(A.hold_tahmini(duz, 8.0)["tur"] == "hareketli", "düz uçan araç → hareketli (Hold değil)")
dogrula(A.hold_tahmini([(8.0, 15.0, -15.0, 0.2, 0.0, 0.0)], 8.0)["tur"] == "duruyor", "yerde duran araç → duruyor (nokta bölge)")
dogrula(A.hold_tahmini([], 0.0)["tur"] == "yok", "geçmiş yok → yok")

# ── 3. Kayıtlı iz: 2026-11-11 06:49 olayı ─────────────────────────────────────────────────────────
rows = [r for r in csv.DictReader(open(IZ)) if r["n1_uav0cerceve"]]
gercek = min(rows[:200], key=lambda r: math.hypot(float(r["ayrim_yatay"]), float(r["ayrim_dikey"])))
dogrula(abs(float(gercek["ayrim_yatay"]) - 54) < 2 and abs(float(gercek["ayrim_dikey"]) - 21) < 1.5,
        f"kayıtlı olay: {gercek['saat']} yatay {gercek['ayrim_yatay']} m / dikey {gercek['ayrim_dikey']} m "
        f"(3B {math.hypot(float(gercek['ayrim_yatay']), float(gercek['ayrim_dikey'])):.0f} m)")
# Komut anı 06:49:14 (qr_pass logu). O ana kadarki uav1 izini (2 Hz; hız = fark) denetime besle.
saat = [0.0]
den = A.AyrimDenetimi(saat=lambda: saat[0])
once, t_komut, kendi = None, sn("06:49:14"), None
# Kayıt 2 Hz (satır başına 0,5 sn; saat sütunu saniye çözünürlüklü). İlk satırlar bayat/tekrarlı → konum
# gerçekten ilerleyen satırlar kullanılır. Kayıt komuttan ~4 sn önce başlıyor, Hold kestirimi ≥ 5 sn ister;
# uav1'in izi uav0'dan bağımsız olduğu için 3 sn daha oynatılır (qr_pass de karar öncesi 6,5 sn dinler).
for i, r in enumerate(rows):
    if sn(r["saat"]) > t_komut + 3:
        break
    tt = 0.5 * i
    p = (float(r["n1_uav0cerceve"]), float(r["e1_uav0cerceve"]), float(r["alt1"]))
    if sn(r["saat"]) <= t_komut:
        kendi = (float(r["n0"]), float(r["e0"]), float(r["alt0"]))
    if once and 3.0 < math.dist(p[:2], once[1][:2]) / (tt - once[0]) < 30.0:
        dt = tt - once[0]
        saat[0] = tt
        den.guncelle(p[0], p[1], p[2], (p[0] - once[1][0]) / dt, (p[1] - once[1][1]) / dt, kendi)
    once = (tt, p)
dg = den.diger()
dogrula(dg["tur"] == "hold" and math.dist(dg["merkez"], (45.7, 330.3)) < 25.0,
        f"kayıtlı izden uav1: tür {dg['tur']}, merkez ({dg['merkez'][0]:.0f}, {dg['merkez'][1]:.0f}) "
        f"(en küçük kareler uyumu (46, 330)), irtifa {dg['irtifa']:.0f} m")
ok, en_kotu, neden = den.bacak(kendi[:2], H, kendi[2], 100.0)
dogrula(not ok and en_kotu < 40.0, f"olaydaki bacak ({kendi[0]:.0f}, {kendi[1]:.0f}, {kendi[2]:.0f} m) → H @100 m REDDEDİLİR: {neden}")
ok60, en60, neden60 = den.bacak(kendi[:2], H, 60.0, 60.0)
dogrula(ok60 and en60 >= 40.0, f"aynı bacak 60 m'de (dikey ayrım yeterli) KABUL: {neden60}")
ok_k, _, neden_k = den.bacak(kendi[:2], H, kendi[2], 60.0)
dogrula(not ok_k, f"99 → 60 m alçalarak giden bacak (başlangıçta dikey yetersiz) RET: {neden_k}")
dogrula(den.cember(H, 100.0)[0], f"H Hold çemberi @100 m uav1 çemberinden uzak → uygun: {den.cember(H, 100.0)[2]}")
dogrula(not den.cember((150.0, 330.0), 110.0)[0], "uav1 çemberine komşu Hold çemberi @110 m → RET")
dogrula(den.ozet() is not None and den.ozet()["3b_m"] > 0, f"gerçekleşen en küçük ayrım izleniyor: {den.ozet()}")

# ── 4. Öteki araç hareketli / bayat: yalnız dikey ayrım yetiyorsa kabul ───────────────────────────
saat[0] = 8.0
hd = A.AyrimDenetimi(saat=lambda: saat[0])
for g_ in duz:
    hd.gecmis.append(g_)
hd.goruldu = True
dogrula(not hd.bacak((0, 0), (500, 0), 100, 100)[0] and hd.bacak((0, 0), (500, 0), 40, 40)[0],
        "öteki araç hareketli: aynı irtifada RET, 60 m altında KABUL")
saat[0] = 20.0
dogrula(not hd.bacak((0, 0), (500, 0), 100, 100)[0], "öteki araç verisi bayat: aynı irtifada RET")
dogrula(A.AyrimDenetimi().bacak((0, 0), (1, 1), 0, 0)[0], "öteki araç hiç görülmedi (tek araç oturumu): denetim kapalı, engellemez")

# ── 5. Kendi Hold merkezi ─────────────────────────────────────────────────────────────────────────
t_, n_, e_, _, vn_, ve_ = g[5]
dogrula(math.dist(A.kendi_hold_merkezi(n_, e_, vn_, ve_), (45.7, 330.3)) < 1e-6, "kendi Hold merkezi kestirimi doğru")

print(f"\nTAMAM: {n_ok} doğrulama geçti")
