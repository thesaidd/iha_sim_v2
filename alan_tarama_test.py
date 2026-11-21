#!/usr/bin/env python3
"""
alan_tarama_test.py — alan taraması planı ve düğüm akışının kuru testi (uçuş yok, ROS düğümü yok).

Sahte uçak: kinematik model (15 m/s, dönüş hızı sınırı = V / 45 m, yol-teğet güdümü; Hold'da saat yönü 80 m
çember). PX4'ün gerçek izleme hatasını TEMSİL ETMEZ — amaç mantık: ilkel geçişleri, sıra, denetimler, tespit →
çıkış Hold'u, ayrım / koridor / abort dalları. Gerçek izleme hatası uçuşta ölçülecek.

Kullanım:  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && python3 alan_tarama_test.py
"""
import math
import random
import sys
import types

import alan_tarama as at
import ayrim as ay
import qr_pass as q
import tarama_plan as tp

q.log = lambda m: None
DT = 0.05
n_assert = 0
EXIT = at.EXIT_ROUTE_REJECTED


def dogrula(kosul, ad):
    global n_assert
    n_assert += 1
    print(("✔ " if kosul else "✘ ") + ad)
    if not kosul:
        sys.exit(1)


class SahteUcak:
    class P:
        pass

    def __init__(self, n, e, alt=100.0, hedef_qr=None, W=16.0, offboard_kayip_t=None, offboard_kabul=True):
        self.pos = self.P()
        self.pos.x, self.pos.y, self.pos.z, self.pos.vx, self.pos.vy = n, e, -alt, 15.0, 0.0
        self.psi, self.t = 0.0, 0.0
        self.mod = "hold"
        self.hold_merkez, self.hold_alt = (n, e + 80.0), alt
        self.sp = None
        self.tespitler, self.iz, self.komutlar = [], [], []
        self.hedef_qr, self.W = hedef_qr, W
        self.offboard_kayip_t, self.offboard_kabul = offboard_kayip_t, offboard_kabul
        self.akis_sayisi, self.son_akis_t = 0, -1.0
        self.her_adim = None
        self.maks_donus_hizi = 0.0

    # ── düğüm arayüzü ──
    def offboard(self):
        return self.mod == "offboard"

    def mod_adi(self):
        return self.mod

    def stream_yol(self, konum, hiz, ivme):
        self.sp = (konum, hiz, ivme)
        self.akis_sayisi += 1
        self.son_akis_t = self.t

    def offboard_iste(self):
        if self.offboard_kabul and self.sp is not None:
            self.mod = "offboard"

    def hold_iste(self):
        self.mod, self.hold_merkez, self.hold_alt = "hold", (self.pos.x, self.pos.y), -self.pos.z
        self.hold_t = self.t

    def hold_dogrulandi(self, t0):
        return self.mod == "hold" and getattr(self, "hold_t", -1) >= t0

    def reposition(self, n, e, alt):
        self.komutlar.append(((self.pos.x, self.pos.y), (n, e), alt))
        self.mod, self.hold_merkez, self.hold_alt = "hold", (n, e), alt
        return 1

    def clock(self):
        return self.t

    # ── model ──
    def spin(self):
        self.t += DT
        V = 15.0
        if self.offboard_kayip_t is not None and self.t >= self.offboard_kayip_t and self.mod == "offboard":
            self.mod, self.hold_merkez = "hold", (self.pos.x, self.pos.y)       # failsafe benzeri
        if self.mod == "offboard":
            (pn, pe, pd), (vn, ve, _), _ = self.sp
            yol = math.atan2(ve, vn)
            x = -(self.pos.x - pn) * math.sin(yol) + (self.pos.y - pe) * math.cos(yol)
            ist, alt_h = yol - math.atan(x / 40.0), -pd
        else:
            c = self.hold_merkez
            dn, de = self.pos.x - c[0], self.pos.y - c[1]
            dist = math.hypot(dn, de)
            ist = math.atan2(de, dn) + math.pi / 2 + math.atan(1.5 * (dist - 80.0) / 80.0)
            alt_h = self.hold_alt
        w = max(-V / 45.0, min(V / 45.0, 2.0 * math.remainder(ist - self.psi, 2 * math.pi)))
        self.maks_donus_hizi = max(self.maks_donus_hizi, abs(w))
        self.psi += w * DT
        self.pos.vx, self.pos.vy = V * math.cos(self.psi), V * math.sin(self.psi)
        self.pos.x += self.pos.vx * DT
        self.pos.y += self.pos.vy * DT
        self.pos.z -= max(-2.0, min(2.0, (alt_h + self.pos.z))) * DT
        self.iz.append((self.t, self.pos.x, self.pos.y, -self.pos.z, self.mod))
        if self.hedef_qr and self.mod == "offboard" and abs(w) < 0.02 and int(self.t / DT) % 2 == 0:
            # kamera modeli: kanatlar düzken, hedef gövde çerçevesinde |yanal| ≤ W/2 ve |ileri| ≤ 6 m ise 10 Hz tespit
            dn, de = self.hedef_qr[0] - self.pos.x, self.hedef_qr[1] - self.pos.y
            ileri = dn * math.cos(self.psi) + de * math.sin(self.psi)
            yanal = -dn * math.sin(self.psi) + de * math.cos(self.psi)
            if abs(yanal) <= self.W / 2 and abs(ileri) <= 6.0:
                self.tespitler.append({"x_kuzey": self.hedef_qr[0] + 1.0, "y_dogu": self.hedef_qr[1] - 0.8,
                                       "icerik": "HEDEF-01", "damga_sim": self.t, "kare_no": len(self.tespitler)})
        if self.her_adim:
            self.her_adim(self)


def arg(**kw):
    a = at.arguman().parse_args([])
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def kos(plan, ucak, ayr=None, **kw):
    res, kayit = {}, []
    kod = at.ucus(ucak, plan, arg(**kw), ayr, spin=ucak.spin, saat=ucak.clock, res=res, kayit=kayit.append)
    return kod, res, kayit


# ══ 1. Geometri ilkelleri ═════════════════════════════════════════════════════════════════════════
rng = random.Random(7)
ok = True
for _ in range(400):
    S, psi = (rng.uniform(-300, 300), rng.uniform(-300, 300)), rng.uniform(-math.pi, math.pi)
    y, ek, R = rng.choice([-1, 1]) * rng.uniform(5, 320), rng.choice([0.0, 30.0]), 85.0
    il, tur = tp.u_donus(S, psi, y, R, ek)
    n, e, h = tp.ilkel_nokta(il[-1], il[-1]["uzunluk"])
    rr = (-math.sin(psi), math.cos(psi))
    hedef = (S[0] + y * rr[0], S[1] + y * rr[1])
    ok &= math.hypot(n - hedef[0], e - hedef[1]) < 1e-6 and abs(math.remainder(h - psi - math.pi, 2 * math.pi)) < 1e-9
    ok &= (tur == "stadyum") == (abs(y) >= 2 * R - 1e-9) and all(p["R"] >= R - 1e-9 for p in il if p["tur"] == "yay")
    for a, b in zip(il, il[1:]):
        n1, e1, h1 = tp.ilkel_nokta(a, a["uzunluk"])
        n2, e2, h2 = tp.ilkel_nokta(b, 0.0)
        ok &= math.hypot(n1 - n2, e1 - e2) < 1e-6 and abs(math.remainder(h1 - h2, 2 * math.pi)) < 1e-9
dogrula(ok, "u_donus: 400 rastgele dönüş — bitiş hedef hattın başında, ters yönde, sürekli; D ≥ 2R ⇔ stadyum; yay R ≥ 85")
ok = True
for _ in range(300):
    pr, _, _ = tp.yay((rng.uniform(-100, 100), rng.uniform(-100, 100)), rng.uniform(-3, 3), rng.choice([-1, 1]),
                      rng.uniform(0.3, 4.5), 85.0)
    il = rng.uniform(0, pr["uzunluk"])
    n, e, h = tp.ilkel_nokta(pr, il)
    il2, x = tp.ilkel_ilerleme(pr, n + 3 * (-math.sin(h)), e + 3 * math.cos(h), il)      # 3 m sağda
    ok &= abs(il2 - il) < 3.0 and abs(x - 3.0) < 1e-6
    (pn, pe, _), (vn, ve, _), (an, ae, _) = tp.ilkel_setpoint(pr, il, 15.0, 60.0)
    ok &= abs(math.hypot(an, ae) - 225 / 85) < 1e-9 and abs(vn * an + ve * ae) < 1e-9
    ok &= ((pr["merkez"][0] - pn) * an + (pr["merkez"][1] - pe) * ae) > 0                    # ivme merkeze bakar
dogrula(ok, "yay ilkeli: ilerleme/yanal sapma tutarlı; setpoint ivmesi merkezcil (V²/R), hıza dik")

# ══ 2. Plan (varsayılan: direk payı 40 m) ve seçenekler ═══════════════════════════════════════════
BAS = (-30.0, 290.0)
planlar = {hp: tp.planla(tp.Param(hat_payi=hp), BAS) for hp in (40.0, 20.0)}
for hp, pl in planlar.items():
    p, o = pl.p, pl.ozet()
    dogrula(pl.dogrula() == [], f"[pay {hp:.0f}] plan doğrulaması boş (süreklilik, direk, çıkış Hold'ları, yarıçap)")
    dogrula(sorted(pl.sira) == list(range(len(pl.hatlar))) and all(pl.yon(k) == -pl.yon(k + 1) for k in range(len(pl.sira) - 1)),
            f"[pay {hp:.0f}] {o['hat_sayisi']} hattın her biri bir kez, yön her hatta değişiyor")
    dogrula(all(min(q.pylon_dist(x) for x in tp.ornekle([pl.hat_ilkeli(k)])) >= hp - 1e-6 for k in range(len(pl.sira))),
            f"[pay {hp:.0f}] her hat (uzatmalar dahil) direklerden ≥ {hp:.0f} m (en küçük {o['hat_direk_min_m']})")
    dogrula(all(tp.yol_uygun(p, b["ilkeller"], p.donus_payi)[0] for b in pl.baglar) and o["donus_direk_min_m"] >= 50,
            f"[pay {hp:.0f}] her dönüş direklerden ≥ {p.donus_payi:.0f} m (uçlar hariç; en küçük {o['donus_direk_min_m']}), koridor dışı")
    dogrula(all((b["tur"] == "stadyum") == (b["D"] >= 2 * p.r_donus - 1e-6) for b in pl.baglar)
            and all(pr["R"] >= p.r_min for b in pl.baglar for pr in b["ilkeller"] if pr["tur"] == "yay"),
            f"[pay {hp:.0f}] geçişler: {o['stadyum']} tanesi D ≥ {2 * p.r_donus:.0f} m (stadyum), {o['ampul']} tanesi dış bağlantı (ampul); hiçbir yay < R_min")
    aralik = [b - a for a, b in zip(pl.hatlar, pl.hatlar[1:])]
    dogrula(all(d <= p.hat_araligi + 1e-6 or any(a < c < b for lo, hi, _ in tp.bloklu_bantlar(p) for c in [(lo + hi) / 2])
                for d, a, b in zip(aralik, pl.hatlar, pl.hatlar[1:])),
            f"[pay {hp:.0f}] komşu hat aralığı ≤ W; yalnız direk bantları boş")
    for k in range(len(pl.sira)):
        c = pl.cikis(k)
        ok = c and q.pylon_dist(c["M"]) - 80 >= 65 and q.pylon_dist(c["P"]) - 80 >= 65
    dogrula(all(pl.cikis(k) and min(q.pylon_dist(pl.cikis(k)[m]) for m in "PM") - 80 >= 65 for k in range(len(pl.sira))),
            f"[pay {hp:.0f}] her hat sonu için çıkış Hold çemberleri direklerden ≥ 65 m")
plan = planlar[40.0]
sey = tp.planla_seyrek(tp.Param(), BAS)
dogrula(sey is not None and sey.dogrula() == [] and sey.ozet()["ampul"] == 0 and sey.ozet()["geometrik_kapsama"] < 0.15,
        f"atlamasız seyrek karşılaştırma planı: {sey.ozet()['hat_sayisi']} hat, kapsama %{100 * sey.ozet()['geometrik_kapsama']:.0f}")
dogrula(tp.kitap_sirasi(8, 4) == [0, 4, 1, 5, 2, 6, 3, 7], "kitap_sirasi: 0, k, 1, k+1, …")

# ══ 3. Uçtan uca: tespit yok → tüm hatlar, çıkış Hold'u ═══════════════════════════════════════════
u = SahteUcak(-30.0, 210.0)
kod, res, kayit = kos(plan, u)
ob = [r for r in u.iz if r[4] == "offboard"]
dogrula(kod == 0 and res["sonuc"] == "bulunamadi" and len(res["ucuslar"]) == len(plan.sira)
        and [x["hat"] for x in res["ucuslar"]] == plan.sira, f"tespitsiz koşu: {len(plan.sira)} hat plan sırasıyla uçuldu, sonuç 'bulunamadi'")
dogrula(max(x["x_maks_m"] for x in res["ucuslar"]) < 3.0,
        f"kutu içinde hatta yanal sapma < 3 m (model; en büyük {max(x['x_maks_m'] for x in res['ucuslar'])} m)")
dmin_hat = min(x["direk_min_m"] for x in res["ucuslar"])
dmin_tum = min(q.pylon_dist((r[1], r[2])) for r in u.iz)
dogrula(dmin_hat >= 40 - 3 and dmin_tum >= 40 - 3, f"gerçekleşen iz direklerden ≥ 37 m (hatlarda {dmin_hat}, tüm uçuşta {dmin_tum:.1f})")
dogrula(not any(q.in_corridor(r[1], r[2]) for r in ob if abs(r[1]) > 150), "dönüşler kalkış koridoruna girmedi")
dogrula(u.maks_donus_hizi <= 15 / 45 + 1e-9 and all(abs(r[3] - 60) < 3 for r in ob[200:]), "irtifa 60 ± 3 m; dönüş hızı model sınırında")
dogrula(u.mod == "hold" and res["hold"] and res["cikis_hold"]["kabul"]
        and q.pylon_dist(u.hold_merkez) - 80 >= 65, f"bitiş: Hold ✔, çıkış merkezi {res['cikis_hold']['merkez']} direklerden uzak")
sure_model, sure_plan = res["tarama_sn"], plan.ozet()["sure_sn"]
dogrula(abs(sure_model - sure_plan - (plan.cikis(len(plan.sira) - 1)["L"] + 150) / 15) < 0.08 * sure_plan,
        f"tarama süresi plan ile uyumlu (model {sure_model:.0f} sn, plan {sure_plan} sn + giriş/çıkış)")
# kapsama: uçulan izden — kutu içi 5 m ızgara noktalarından, kanatlar düzken W/2 yanal içinde geçilenlerin oranı
kap = set()
for _, n, e, _, m in ob[::4]:
    if abs(n) <= 150:
        for k in range(int(math.floor((e - 8) / 5)) + 1, int(math.floor((e + 8) / 5)) + 1):
            if abs(k * 5) <= 150:
                kap.add((round(n / 5), k))
oran = len(kap) / (61 * 61)
dogrula(abs(oran - plan.ozet()["geometrik_kapsama"]) < 0.04, f"izden ölçülen kapsama %{100 * oran:.1f} ≈ plan %{100 * plan.ozet()['geometrik_kapsama']:.1f}")

# ══ 4. Tespit → tarama durur → çıkış Hold'u (zincire geçiş) ═══════════════════════════════════════
u = SahteUcak(-30.0, 210.0, hedef_qr=(-60.0, -60.0))
kod, res, kayit = kos(plan, u)
kaçıncı = [plan.hatlar[i] for i in plan.sira].index(min(plan.hatlar, key=lambda c: abs(c + 60))) + 1
dogrula(kod == 0 and res["sonuc"] == "tespit" and res["ucular_hat"] == kaçıncı,
        f"gerçek QR (−60, −60): {res['ucular_hat']}/{len(plan.sira)}. hatta tespit, tarama başından {res['tespit']['tarama_sn']} sn sonra")
u = SahteUcak(-30.0, 210.0, hedef_qr=(-60.0, -68.0))
kod, res, kayit = kos(plan, u)
kaçıncı = [plan.hatlar[i] for i in plan.sira].index(min(plan.hatlar, key=lambda c: abs(c + 68))) + 1
dogrula(kod == 0 and res["sonuc"] == "tespit" and res["ucular_hat"] == kaçıncı < len(plan.sira) and res["ucmayan_hat"],
        f"hedef (−60, −68): {res['ucular_hat']}. hatta tespit (tarama {res['tespit']['tarama_sn']} sn), sonraki hat AÇILMADI "
        f"(uçulmayan {len(res['ucmayan_hat'])} hat)")
dogrula(abs(res["tespit"]["x_kuzey"] + 59) < 1e-6 and abs(res["tespit"]["y_dogu"] + 68.8) < 1e-6 and res["tespit"]["icerik"] == "HEDEF-01" and res["dur_nedeni"] == "tespit",
        "tespit kaydı sonuçta (konum, içerik, hat, süre)")
son_ob = [r for r in u.iz if r[4] == "offboard"][-1]
c_hat = res["ucuslar"][-1]["c_m"]
dogrula(abs(son_ob[2] - c_hat) < 3 and abs(son_ob[1]) >= plan.p.T + 55 and u.mod == "hold" and res["cikis_hold"]["kabul"],
        f"tespitten sonra hat bitirildi, düz çıkış, Hold'a geçiş N={son_ob[1]:.0f} (kutu dışı), merkez {res['cikis_hold']['merkez']}")
t_tespit = u.tespitler[0]["damga_sim"]
dogrula(all(q.pylon_dist((r[1], r[2])) >= 37 for r in u.iz if r[0] >= t_tespit), "tespit sonrası iz direklerden ≥ 37 m")
dogrula(not at.tespit_gecerli({"x_kuzey": 0, "y_dogu": 0, "icerik": "BASKA"}, plan.p, "HEDEF-01")
        and not at.tespit_gecerli({"x_kuzey": 400, "y_dogu": 0, "icerik": "HEDEF-01"}, plan.p, "HEDEF-01")
        and at.tespit_gecerli({"x_kuzey": 0, "y_dogu": 0, "icerik": "X"}, plan.p, ""),
        "tespit süzgeci: yanlış içerik / kutu dışı konum sayılmaz")
u = SahteUcak(-30.0, 210.0, hedef_qr=(0.0, 0.0))          # K–G direk bandı: kapsama dışı
kod, res, _ = kos(plan, u)
dogrula(kod == 0 and res["sonuc"] == "bulunamadi" and not u.tespitler, "kapsama dışı hedef (0, 0; direk bandı): 'bulunamadi' — dürüst sonuç")
u = SahteUcak(-30.0, 210.0, hedef_qr=(100.0, plan.hatlar[plan.sira[-1]] + 5.0))
kod, res, _ = kos(plan, u)
dogrula(kod == 0 and res["sonuc"] == "tespit" and res["ucular_hat"] == len(plan.sira), "son hattaki hedef: son hatta tespit")

# ══ 5. Ayrım (ayrim.py) ve koridor ═══════════════════════════════════════════════════════════════
def uav1(ayr, u, merkez, alt, hareket="hold"):
    def f(_):
        if int(u.t / DT) % 4:
            return
        if hareket == "hold":
            a = u.t * 15 / 80
            ayr.guncelle(merkez[0] + 80 * math.cos(a), merkez[1] + 80 * math.sin(a), alt, -15 * math.sin(a), 15 * math.cos(a),
                         (u.pos.x, u.pos.y, -u.pos.z))
        elif hareket == "yerde":
            ayr.guncelle(15.0, -15.0, 0.0, 0.0, 0.0, (u.pos.x, u.pos.y, -u.pos.z))
        else:
            ayr.guncelle(merkez[0] + 10 * u.t, merkez[1], alt, 10.0, 0.0, (u.pos.x, u.pos.y, -u.pos.z))
    return f


def ayrimli(merkez, alt, hareket="hold", bas=(-30.0, 210.0), **kw):
    u = SahteUcak(bas[0], bas[1], **kw)
    ayr = ay.AyrimDenetimi(saat=u.clock)
    u.her_adim = uav1(ayr, u, merkez, alt, hareket)
    for _ in range(int(8 / DT)):
        u.spin()
    return u, ayr


u, ayr = ayrimli((45.0, 334.0), 119.0)
kod, res, _ = kos(plan, u, ayr)
dogrula(kod == 0 and len(res["ucuslar"]) == len(plan.sira) and not res["atlanan"],
        "uav1 kalkış Hold'unda (45, 334) @119 m: dikey ayrım yetiyor, hiçbir hat ertelenmedi")
u, ayr = ayrimli((0.0, 160.0), 65.0, bas=(-350.0, -150.0))
kod, res, _ = kos(plan, u, ayr)
bati = [c for c in plan.hatlar if c < 0]
dogrula(kod == 0 and res["sonuc"] == "kesildi" and sorted(x["c_m"] for x in res["ucuslar"]) == [round(c, 1) for c in bati]
        and len(res["ucmayan_hat"]) == len(plan.hatlar) - len(bati) and all("AYRIM" in v for v in res["atlanan"].values()),
        f"uav1 doğu hatlarının üstünde Hold'da (0, 160) @65 m: doğu hatları ertelendi, batı hatları ({len(bati)}) öne alınıp uçuldu, "
        f"sonra 'kesildi' (uçulmayan {res['ucmayan_hat']}), çıkış Hold'u")
u, ayr = ayrimli((0.0, 0.0), 62.0, "hareketli")
kod, res, _ = kos(plan, u, ayr)
dogrula(kod == EXIT and not u.komutlar and u.akis_sayisi == 0 and res["sonuc"] == "rota_reddedildi",
        "uav1 aynı irtifada hareketli (yeri belirsiz): tarama BAŞLAMADI, komut yok, kod 3")
u, ayr = ayrimli((0.0, 0.0), 0.0, "yerde")
kod, res, _ = kos(plan, u, ayr)
dogrula(all(x["c_m"] < 0 for x in res.get("ucuslar", [])) and res["sonuc"] in ("kesildi", "rota_reddedildi"),
        f"uav1 yerde (kalkmamış): koridoru kesen doğu hatları uçulmadı (sonuç '{res['sonuc']}')")

# ══ 6. Hata dalları ══════════════════════════════════════════════════════════════════════════════
u = SahteUcak(-30.0, 210.0, offboard_kayip_t=200.0)
kod, res, _ = kos(plan, u)
n_akis = u.akis_sayisi
for _ in range(100):
    u.spin()
dogrula(kod == 2 and res["sonuc"] == "abort" and u.akis_sayisi == n_akis, "OFFBOARD kaybı: kod 2, setpoint akışı kesildi")
u = SahteUcak(-30.0, 210.0, offboard_kabul=False)
kod, res, _ = kos(plan, u, align_timeout=400.0)
dogrula(kod == 1 and res["sonuc"] == "offboard_yok", "OFFBOARD reddi: 10 denemeden sonra kod 1, tarama yok")
u = SahteUcak(140.0, 100.0)                      # K ile D direkleri arasında, direğe yakın Hold: taşıma reddi beklenir
kod, res, _ = kos(plan, u, replan_wait=5.0)
dogrula(kod in (0, 3) and (kod == 0 or (res["sonuc"] == "rota_reddedildi" and u.akis_sayisi == 0)),
        f"zor başlangıç (140, 100): taşıma {'kabul' if kod == 0 else 'REDDEDİLDİ → kod 3, OFFBOARD yok'}; denetimsiz bacak yok")
dogrula(all(q.seg_clearance(a, b) >= 65 - 1e-6 for a, b, _ in u.komutlar if math.dist(a, b) > q.ARRIVE_R) or kod == 3,
        "taşıma bacakları direk + iz payı denetimli")

# ══ 7. Geniş kapsamlı seçenek (direk payı 20 m) uçtan uca ═════════════════════════════════════════
u = SahteUcak(-30.0, 210.0)
kod, res, _ = kos(planlar[20.0], u)
dogrula(kod == 0 and len(res["ucuslar"]) == len(planlar[20.0].sira)
        and min(q.pylon_dist((r[1], r[2])) for r in u.iz) >= 17.0,
        f"[pay 20] {len(res['ucuslar'])} hat uçuldu, iz direklerden ≥ 17 m ({min(q.pylon_dist((r[1], r[2])) for r in u.iz):.1f}), "
        f"{res['tarama_sn']:.0f} sn")

# ══ 8. Kameradan şerit, en kısa sıra, tam tarama, daire girişi (2026-11-20) ══════════════════════
KJ = "kalibrasyon/w_nadir_10_sim.json"
W5, d5 = at.serit_kameradan(arg(serit_kamera=KJ, alt=100.0, kenar_px=5.0), kayit=lambda m: None)
W15, _ = at.serit_kameradan(arg(serit_kamera=KJ, alt=100.0, kenar_px=15.0), kayit=lambda m: None)
_, d60 = at.serit_kameradan(arg(serit_kamera=KJ, alt=100.0, kenar_px=15.0, yan_ortusme=0.6), kayit=lambda m: None)
dogrula(abs(W5 - 72.0) < 0.5 and d5 == W5 and W15 < W5 and abs(d60 - 30.3) < 0.5,
        f"--serit-kamera: W {W5:.1f} m (kenar 5 px), {W15:.1f} m (15 px); %60 yan örtüşme → aralık {d60:.1f} m")
a_d = dict(serit_kamera=KJ, alt=100.0, kenar_px=15.0, daire=100.0, koridor_yok=True)
p_d = at.param_olustur(arg(uzatma=60.0, **a_d), kayit=lambda m: None)
pl_kisa = at.plan_kur(p_d, arg(uzatma=60.0, **a_d), None, kayit=lambda m: None)
pl_cim = at.plan_kur(p_d, arg(uzatma=60.0, sira="cim", **a_d), None, kayit=lambda m: None)
p_eski = at.param_olustur(arg(serit=70.0, hat_araligi=70.0, alt=100.0, daire=100.0, uzatma=100.0), kayit=lambda m: None)
pl_eski = at.plan_kur(p_eski, arg(serit=70.0, hat_araligi=70.0, alt=100.0, daire=100.0, sira="cim"), None, kayit=lambda m: None)
o_k, o_c, o_e = pl_kisa.ozet(), pl_cim.ozet(), pl_eski.ozet()
dogrula(not pl_kisa.dogrula() and o_k["toplam_m"] <= o_c["toplam_m"] < o_e["toplam_m"]
        and o_k["geometrik_kapsama"] >= o_e["geometrik_kapsama"],
        f"daire R 100: en kısa sıra {o_k['toplam_m']} m ≤ çim {o_c['toplam_m']} m < eski (W 70, uzatma 100) {o_e['toplam_m']} m; "
        f"kapsama %{100 * o_k['geometrik_kapsama']:.1f} ≥ %{100 * o_e['geometrik_kapsama']:.1f}")
yur = at.Yurutucu(pl_kisa, kayit=lambda m: None)
yur.baslat()
c0, g0 = pl_kisa.hatlar[pl_kisa.sira[0]], pl_kisa.g0
dis = p_d.nokta(-g0 * (tp.daire_kiris(p_d, c0) / 2 + 5.0), c0)
ic = p_d.nokta(-g0 * (tp.daire_kiris(p_d, c0) / 2 - 5.0), c0)
yur.adim(*dis)
once = yur.ucuslar[-1]["x_giris_m"]
yur.adim(*ic)
dogrula(once is None and yur.ucuslar[-1]["x_giris_m"] is not None,
        "daire modu: hat girişi (x_giris) daireye girişte ölçülür, kiriş dışında değil")
u = SahteUcak(-30.0, 210.0, hedef_qr=(-60.0, -68.0))          # bölüm 4: bu hedef taramayı son hattan önce keser
kod, res, kayit = kos(plan, u, tam_tarama=True)
dogrula(kod == 0 and res["tespit"] is not None and res["tespit"]["ucus_sirasi"] < len(plan.sira)
        and res["ucular_hat"] == len(plan.sira) and res["dur_nedeni"] == "tüm hatlar uçuldu",
        f"--tam-tarama: tespit {res['tespit']['ucus_sirasi']}. hatta kaydedildi, tarama {res['ucular_hat']}/{len(plan.sira)} hat sürdü")

print(f"\n{n_assert} doğrulama geçti")
