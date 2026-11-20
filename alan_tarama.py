#!/usr/bin/env python3
"""
alan_tarama.py — uav0 ile alan taraması: hedef konumu BİLİNMEDEN 300 × 300 m kutuyu (arena merkezli) atlamalı
sıralı tek yönlü boustrophedon ile tarar; tarama boyunca qr_online.py çalışır; ilk tespitte tarama durur
(uçulan hat bitirilir, yeni hat açılmaz), uav0 güvenli çıkış Hold'una alınır. Röle → uav1 zinciri
(qr_online /hedef_konum → lora_relay /hedef_konum_role → lora_yaklasma) aynı mesajla kendiliğinden tetiklenir;
bu düğüm zincirin hazır olduğunu başta denetler ve tespiti sonuç dosyasına yazar.

Plan: tarama_plan.py (hat yerleşimi, atlamalı sıra, dönüş bağlantıları, direk/koridor denetimi).
Tasarım ve hesaplar: test/tarama_tasarim_2026-11-11/tasarim.md.

Uçuş yöntemi (yalnız standart PX4 Offboard + TrajectorySetpoint; özel güdüm yok):
  1. Uçak Hold'da olmalı. İlk hattın hizalama Hold'una qr_pass.transit() ile taşınır (direk + iz payı +
     koridor + ayrim.py denetimli; güvenli rota yoksa çıkış kodu 3).
  2. Hizalama: qr_pass deseni — saat yönü Hold hattın 80 m sağında; rota hat yönüne döndüğünde OFFBOARD.
  3. OFFBOARD: her döngüde setpoint = yol ilkelinin uçağa en yakın noktası, hız = V·teğet; yaylarda ivme =
     merkezcil (V²/R) → PX4 bunu yol eğriliği olarak alır (FixedWingModeManager: trajectory_setpoint
     konum + hız + ivme → navigatePathTangent). Düz hatta ivme NaN (qr_pass ile aynı, uçuşta kanıtlı).
     Zamandan bağımsız (RTF / zaman damgası sıçramalarından etkilenmez).
  4. Her hat bitiminde sıradaki hat seçilir: direk/koridor (plan) + öteki araca ayrım (ayrim.py, canlı).
     Uygun olmayan hat ertelenir; uygun hat kalmazsa tarama kesilir.
  5. Tespit ya da tarama bitişi: hat doğrultusunda çıkış noktasına düz uçulur, Hold (DO_SET_MODE AUTO_LOITER,
     akış mod değişene kadar sürer — qr_pass'te kanıtlı), sonra açık Hold merkezine DO_REPOSITION.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 alan_tarama.py --yalniz-plan                       # planı yazdır, uçuş yok
  python3 alan_tarama.py --done-file F --abort-file G --result R.jsonl
Çıkış kodu: 0 = tarama bitti (tespit ya da tüm hatlar), 1 = hazırlık hatası, 2 = abort / OFFBOARD kaybı,
            3 = güvenli rota yok / ayrım nedeniyle başlanamadı.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import ayrim as ayrim_mod
import qr_pass as q
import tarama_plan as tp

LOOP_HZ = 20
EXIT_ROUTE_REJECTED = q.EXIT_ROUTE_REJECTED


def log(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


class Yurutucu:
    """Planı ilkel ilkel yürütür (ROS'suz; kuru testte sahte uçakla, uçuşta düğümle aynı sınıf).

    adim(n, e) → (konum, hız, ivme) setpoint'i. faz: "hat" | "donus" | "cikis" | "hold" (hold = setpoint'i
    çıkış hattı boyunca sürdür, çağıran Hold komutunu verir)."""

    def __init__(self, plan, ayrim=None, koridor_serbest=None, kayit=log):
        self.plan, self.p = plan, plan.p
        self.ayrim, self.kayit = ayrim, kayit
        self.koridor_serbest = koridor_serbest or (lambda: True)
        self.kalan = list(plan.sira)            # henüz uçulmamış hatlar (plan sırasıyla)
        self.ucuslar = []                       # [{hat, c, yon, x_giris, x_maks, ...}]
        self.tespit = None
        self.dur_nedeni = None                  # tarama neden bitti
        self.faz = None
        self.ilkeller, self.i, self.ilerleme = [], 0, 0.0
        self.hat_no = self.g = None
        self.cikis = None
        self.atlanan = {}                       # hat → son ret nedeni
        self.donusler = []                      # [{tur, D_m, x_maks_m (|yanal|), x_son_m, direk_min_m}]

    # ── durum geçişleri ──
    def baslat(self):
        self.hat_no, self.g = self.kalan.pop(0), self.plan.g0
        self._hat_ac()

    def _hat_ac(self):
        c = self.plan.hatlar[self.hat_no]
        self.faz, self.ilkeller, self.i, self.ilerleme = "hat", [tp.hat_ilkeli(self.p, c, self.g)], 0, 0.0
        self.ucuslar.append({"hat": self.hat_no, "c_m": round(c, 1), "yon": self.g, "x_giris_m": None, "x_maks_m": 0.0,
                             "direk_min_m": math.inf})
        self.kayit(f"HAT {len(self.ucuslar)}/{len(self.plan.sira)}: c={c:+.1f} m, yön {'ileri' if self.g > 0 else 'geri'}"
                   f" (kalan {len(self.kalan)})")

    def hat_denetimi(self, j, g_yeni, bag):
        """Sıradaki hat adayı için canlı denetim → (uygun, neden)."""
        p, c = self.p, self.plan.hatlar[j]
        if bag["ilkeller"] is None:
            return False, "dönüş bağlantısı yok: " + bag.get("neden", "")
        if tp.cikis(p, c, g_yeni) is None:
            return False, "güvenli çıkış Hold'u yok"
        pr = tp.hat_ilkeli(p, c, g_yeni)
        if tp.hat_uygun(p, c)[2] and not self.koridor_serbest():
            return False, "hat kalkış koridorunu kesiyor ve koridor serbest değil"
        if self.ayrim is not None:
            ok, _, neden = self.ayrim.bacak(pr["a"], pr["b"], p.irtifa, p.irtifa)
            if not ok:
                return False, "AYRIM — " + neden
            for b in bag["ilkeller"]:               # dönüş: ilkellerin kirişleri (iz payı ayrim içinde)
                a0, a1 = tp.ilkel_nokta(b, 0.0)[:2], tp.ilkel_nokta(b, b["uzunluk"])[:2]
                am = tp.ilkel_nokta(b, b["uzunluk"] / 2)[:2]
                for u, v in ((a0, am), (am, a1)):
                    ok, _, neden = self.ayrim.bacak(u, v, p.irtifa, p.irtifa)
                    if not ok:
                        return False, "AYRIM (dönüş) — " + neden
        return True, ""

    def _hat_bitti(self):
        if self.tespit is not None:
            return self._cikisa_gec("tespit")
        if self.ayrim is not None and self.ayrim.en_kucuk is not None and self.ayrim.en_kucuk[0] < self.ayrim.min_ayrim:
            return self._cikisa_gec(f"canlı ayrım ihlali (3B {self.ayrim.en_kucuk[0]:.0f} m < {self.ayrim.min_ayrim:.0f} m)")
        if not self.kalan:
            return self._cikisa_gec("tüm hatlar uçuldu")
        c = self.plan.hatlar[self.hat_no]
        for j in list(self.kalan):
            bag = tp.baglanti(self.p, c, self.plan.hatlar[j], self.g)
            ok, neden = self.hat_denetimi(j, -self.g, bag)
            if ok:
                if j != self.kalan[0]:
                    self.kayit(f"SIRA DEĞİŞTİ: {[self.atlanan.get(k) for k in self.kalan[:self.kalan.index(j)]]} → "
                               f"hat c={self.plan.hatlar[j]:+.1f} öne alındı")
                self.kalan.remove(j)
                self.faz, self.ilkeller, self.i, self.ilerleme = "donus", bag["ilkeller"], 0, 0.0
                self._sonraki = j
                self.donusler.append({"tur": bag["tur"], "D_m": round(bag["D"], 1), "x_maks_m": 0.0, "x_son_m": None,
                                      "direk_min_m": math.inf})
                self.kayit(f"DÖNÜŞ → c={self.plan.hatlar[j]:+.1f} m: {bag['tur']}, D {bag['D']:.0f} m, "
                           f"uzunluk {bag['uzunluk']:.0f} m, direk ≥ {bag['direk_m']:.0f} m")
                return
            self.atlanan[j] = neden
            self.kayit(f"hat c={self.plan.hatlar[j]:+.1f} şimdi uçulamıyor: {neden}")
        self._cikisa_gec("kalan hatlar uçulamıyor: " + "; ".join(
            f"c={self.plan.hatlar[j]:+.1f}: {self.atlanan[j]}" for j in self.kalan))

    def _cikisa_gec(self, neden):
        self.dur_nedeni = neden
        self.cikis = tp.cikis(self.p, self.plan.hatlar[self.hat_no], self.g)
        E = tp.ilkel_nokta(self.ilkeller[-1], self.ilkeller[-1]["uzunluk"])[:2]
        self.faz, self.ilkeller, self.i, self.ilerleme = "cikis", [tp.hat(E, self.cikis["P"])], 0, 0.0
        self.kayit(f"TARAMA DURDU ({neden}) → çıkış: {self.cikis['L']:.0f} m düz, Hold'a geçiş "
                   f"({self.cikis['P'][0]:.0f}, {self.cikis['P'][1]:.0f}), Hold merkezi "
                   f"({self.cikis['M'][0]:.0f}, {self.cikis['M'][1]:.0f}), çember–direk ≥ {self.cikis['direk_m']:.0f} m")

    def tespit_bildir(self, d):
        """İlk geçerli tespit: yeni hat açılmaz. True = bu çağrı ilk tespitti."""
        if self.tespit is not None:
            return False
        self.tespit = dict(d, hat=self.hat_no, faz=self.faz, ucus_sirasi=len(self.ucuslar))
        self.kayit(f"TESPİT: {d} — faz {self.faz}; uçulan hat bitirilip çıkış Hold'una gidilecek")
        return True

    # ── her döngü ──
    def adim(self, n, e):
        pr = self.ilkeller[self.i]
        il, x = tp.ilkel_ilerleme(pr, n, e, self.ilerleme)
        self.ilerleme = max(self.ilerleme, min(il, pr["uzunluk"] + 200.0))
        if self.faz == "hat":
            u = self.ucuslar[-1]
            t, _ = self.p.tc(n, e)
            if abs(t) <= self.p.yari:
                if u["x_giris_m"] is None:
                    u["x_giris_m"] = round(x, 2)
                u["x_maks_m"] = round(max(u["x_maks_m"], abs(x)), 2)
            u["direk_min_m"] = round(min(u["direk_min_m"], q.pylon_dist((n, e))), 1)
            if abs(t) > self.p.yari and u["x_giris_m"] is None:
                u["x_uzatma_maks_m"] = round(max(u.get("x_uzatma_maks_m", 0.0), abs(x)), 2)   # kutuya girmeden önceki 40 m
        elif self.faz == "donus":
            dn_ = self.donusler[-1]
            dn_["x_maks_m"] = round(max(dn_["x_maks_m"], abs(x)), 2)
            dn_["x_son_m"] = round(x, 2)
            dn_["direk_min_m"] = round(min(dn_["direk_min_m"], q.pylon_dist((n, e))), 1)
        self.son_x = x
        while self.faz != "hold" and self.ilerleme >= self.ilkeller[self.i]["uzunluk"] - 0.5:
            tasma = self.ilerleme - self.ilkeller[self.i]["uzunluk"]
            if self.i + 1 < len(self.ilkeller):
                self.i, self.ilerleme = self.i + 1, max(0.0, tasma)
            elif self.faz == "hat":
                self._hat_bitti()
            elif self.faz == "donus":
                self.hat_no, self.g = self._sonraki, -self.g
                self._hat_ac()
            elif self.faz == "cikis":
                self.faz = "hold"
                self.ilerleme = self.ilkeller[self.i]["uzunluk"]
                break
            il, _ = tp.ilkel_ilerleme(self.ilkeller[self.i], n, e, self.ilerleme)
            self.ilerleme = max(self.ilerleme, il)
        pr = self.ilkeller[self.i]
        il = self.ilerleme
        if self.faz == "hold":                     # çıkış hattının uzantısında en yakın nokta (akış sürsün)
            il = max(il, tp.ilkel_ilerleme(pr, n, e)[0])
        return tp.ilkel_setpoint(pr, il, self.p.hiz, self.p.irtifa)


def tespit_gecerli(d, p, icerik):
    """/hedef_konum JSON'u tarama tespiti sayılır mı: içerik eşleşmeli, konum kutunun içinde (± 20 m) olmalı."""
    if icerik and d.get("icerik") != icerik:
        return False
    try:
        t, c = p.tc(float(d["x_kuzey"]), float(d["y_dogu"]))
    except (KeyError, TypeError, ValueError):
        return False
    return abs(t) <= p.yari + 20 and abs(c) <= p.yari + 20


def ucus(node, plan, args, ayr=None, spin=None, saat=time.monotonic, res=None, kayit=log):
    """Taşıma → hizalama → OFFBOARD tarama → Hold. node arayüzü: pos, offboard(), stream_yol(), reposition(),
    offboard_iste(), hold_iste(), hold_dogrulandi(t0), mod_adi(), tespitler (liste; düğüm doldurur)."""
    p = plan.p
    res = {} if res is None else res
    gi = plan.giris()
    d, r = p.d(plan.g0), p.r(plan.g0)
    S = gi["bas"]

    def koridor_serbest():
        if ayr is None or not ayr.goruldu:
            return True
        dg = ayr.diger()
        return dg["tur"] in ("hold", "hareketli") and dg.get("irtifa", 0.0) > 30.0

    yur = Yurutucu(plan, ayr, koridor_serbest, kayit)
    res["plan"] = plan.ozet()
    # ── 0. İlk hat canlı denetimi (ayrım / koridor) ──
    c0 = plan.hatlar[plan.sira[0]]
    pr0 = tp.hat_ilkeli(p, c0, plan.g0)
    if tp.hat_uygun(p, c0)[2] and not koridor_serbest():
        kayit("ROTA REDDEDİLDİ: ilk hat kalkış koridorunu kesiyor ve öteki araç havada değil — komut gönderilmedi")
        res.update(sonuc="rota_reddedildi", neden="koridor")
        return EXIT_ROUTE_REJECTED
    if ayr is not None:
        ok, _, neden = ayr.bacak(gi["teget"], pr0["b"], p.irtifa, p.irtifa)
        if not ok:
            kayit(f"ROTA REDDEDİLDİ: ilk hat — {neden} — komut gönderilmedi, uçak mevcut Hold'da")
            res.update(sonuc="rota_reddedildi", neden="AYRIM — " + neden)
            return EXIT_ROUTE_REJECTED
    # ── 1. Taşıma ──
    kod, tinfo = q.transit(node, gi["H"], p.irtifa, args.clear, max_hops=args.max_hops, replan_wait=args.replan_wait,
                           spin=spin, clock=saat, margin=args.track_margin, ayrim=ayr)
    res["tasima"] = tinfo
    if kod:
        res.update(sonuc="rota_reddedildi" if kod == EXIT_ROUTE_REJECTED else "komut_reddi")
        return kod
    # ── 2. Hizalama ──
    yur.baslat()
    t0 = saat()
    sonraki_log, deneme, son_deneme = t0, 0, -1e9
    while True:
        spin()
        if args.abort_file and os.path.exists(args.abort_file):
            kayit("abort-file — hazırlık durduruldu")
            res.update(sonuc="abort", neden="abort-file (hizalama)")
            return 2
        ps = node.pos
        dn, de = ps.x - S[0], ps.y - S[1]
        s, x = dn * d[0] + de * d[1], dn * r[0] + de * r[1]
        node.stream_yol((S[0] + s * d[0], S[1] + s * d[1], -p.irtifa), (p.hiz * d[0], p.hiz * d[1], 0.0), None)
        err = math.degrees(math.remainder(math.atan2(ps.vy, ps.vx) - math.atan2(d[1], d[0]), 2 * math.pi))
        distH = math.hypot(ps.x - gi["H"][0], ps.y - gi["H"][1])
        hazir = (-15.0 <= err <= 5.0 and abs(x) < 20.0 and s < -args.hizali_mesafe and abs(-ps.z - p.irtifa) < 5.0
                 and 50.0 <= distH <= 110.0)
        simdi = saat()
        if simdi >= sonraki_log:
            sonraki_log += 5.0
            kayit(f"hizalama: H'ye {distH:5.0f} m  s={s:+6.0f}  x={x:+6.1f} m  rota hatası={err:+6.1f}°  "
                  f"irtifa={-ps.z:5.1f} m  mod={node.mod_adi()}")
        if node.offboard():
            break
        if hazir or deneme:
            if deneme >= 10:
                kayit("HATA: 10 denemede OFFBOARD'a geçilemedi")
                res.update(sonuc="offboard_yok")
                return 1
            if simdi - son_deneme >= 1.0:
                deneme, son_deneme = deneme + 1, simdi
                node.offboard_iste()
                kayit(f"DO_SET_MODE → OFFBOARD (deneme {deneme}): s={s:+.0f} m, x={x:+.1f} m, rota hatası={err:+.1f}°")
        if simdi - t0 > args.align_timeout:
            kayit(f"HATA: {args.align_timeout:.0f} sn içinde hizalanamadı")
            res.update(sonuc="hizalama_zaman_asimi")
            return 1
    kayit(f"OFFBOARD ✔ ({saat() - t0:.0f} sn hizalama): s={s:+.0f} m, x={x:+.1f} m, irtifa={-node.pos.z:.1f} m")
    res.update(devralma_s_m=round(s, 1), devralma_x_m=round(x, 1))
    # Canlı ayrım ihlali kuralı yalnız TARAMA sırasındaki ayrıma bakar: taşıma dönemindeki en küçük değer ayrı saklanır
    onceki_ayrim = None
    if ayr is not None:
        onceki_ayrim, ayr.en_kucuk = ayr.en_kucuk, None
        res["ayrim_tasima"] = None if onceki_ayrim is None else round(onceki_ayrim[0], 1)
    # ── 3. Tarama ──
    t_tarama = saat()
    iz, iz_t = None, t_tarama
    if getattr(args, "iz", None):
        iz = open(args.iz, "w")
        iz.write("t,n,e,irtifa,vn,ve,faz,ilkel,hat_sirasi,yanal\n")
    sonraki_log = t_tarama
    islenen = 0
    while yur.faz != "hold":
        spin()
        if (args.abort_file and os.path.exists(args.abort_file)) or not node.offboard():
            neden = "abort-file" if args.abort_file and os.path.exists(args.abort_file) else f"OFFBOARD kaybı (mod={node.mod_adi()})"
            kayit(f"DURDU: {neden} — setpoint akışı kesildi")
            res.update(sonuc="abort", neden=neden, ucuslar=yur.ucuslar, tespit=yur.tespit)
            return 2
        while islenen < len(node.tespitler):
            dd = node.tespitler[islenen]
            islenen += 1
            if tespit_gecerli(dd, p, args.icerik) and yur.tespit_bildir(
                    {k: dd.get(k) for k in ("x_kuzey", "y_dogu", "icerik", "damga_sim", "kare_no")}):
                yur.tespit["tarama_sn"] = round(saat() - t_tarama, 1)
                yur.tespit["ucak"] = [round(node.pos.x, 1), round(node.pos.y, 1), round(-node.pos.z, 1)]
        ps = node.pos
        konum, hiz, ivme = yur.adim(ps.x, ps.y)
        node.stream_yol(konum, hiz, ivme)
        if iz is not None and saat() >= iz_t:
            iz_t += 0.2
            iz.write(f"{saat() - t_tarama:.2f},{ps.x:.2f},{ps.y:.2f},{-ps.z:.2f},{ps.vx:.2f},{ps.vy:.2f},{yur.faz},"
                     f"{yur.ilkeller[yur.i]['tur']},{len(yur.ucuslar)},{getattr(yur, 'son_x', 0.0):.2f}\n")
        if saat() >= sonraki_log:
            sonraki_log += 2.0
            pr = yur.ilkeller[yur.i]
            _, x = tp.ilkel_ilerleme(pr, ps.x, ps.y, yur.ilerleme)
            kayit(f"{yur.faz:5s} {pr['tur']} {yur.ilerleme:6.0f}/{pr['uzunluk']:.0f} m  x={x:+6.2f} m  irtifa={-ps.z:6.2f} m  "
                  f"yer_hızı={math.hypot(ps.vx, ps.vy):5.2f}  direk={q.pylon_dist((ps.x, ps.y)):.0f} m"
                  + ("" if ayr is None or ayr.en_kucuk is None else f"  ayrım_min={ayr.en_kucuk[0]:.0f} m"))
    if ayr is not None:
        res["ayrim_tarama"] = None if ayr.en_kucuk is None else round(ayr.en_kucuk[0], 1)
        if onceki_ayrim is not None and (ayr.en_kucuk is None or onceki_ayrim[0] < ayr.en_kucuk[0]):
            ayr.en_kucuk = onceki_ayrim              # genel özet (main) tüm koşunun en küçüğünü yazar
    res.update(tarama_sn=round(saat() - t_tarama, 1), donusler=yur.donusler, ucuslar=yur.ucuslar, ucular_hat=len(yur.ucuslar),
               tespit=yur.tespit, dur_nedeni=yur.dur_nedeni, atlanan={str(k): v for k, v in yur.atlanan.items()},
               ucmayan_hat=[round(plan.hatlar[j], 1) for j in yur.kalan])
    if iz is not None:
        iz.close()
    # ── 4. Hold (akış mod değişene kadar sürer) ──
    if args.done_file:
        with open(args.done_file, "w") as f:
            f.write(f"tarama bitti: {yur.dur_nedeni}\n")
    t_hold, son_komut, hold_ok = saat(), -1e9, False
    while saat() - t_hold < 8.0:
        spin()
        ps = node.pos
        node.stream_yol(*yur.adim(ps.x, ps.y))
        if node.hold_dogrulandi(t_hold):
            hold_ok = True
            break
        if saat() - son_komut >= 0.5:
            son_komut = saat()
            node.hold_iste()
    kayit(f"Hold: {'✔ (nav_state AUTO_LOITER; akış mod değişene kadar sürdü)' if hold_ok else '✘ 8 sn içinde doğrulanamadı'}")
    res["hold"] = hold_ok
    if hold_ok:
        M = yur.cikis["M"]
        ok_a, neden_a = True, ""
        if ayr is not None:
            ok_a, _, neden_a = ayr.cember(M, p.irtifa)
        if ok_a:
            ok = node.reposition(M[0], M[1], p.irtifa)
            kayit(f"DO_REPOSITION → çıkış Hold merkezi ({M[0]:.0f}, {M[1]:.0f}), {p.irtifa:.0f} m: "
                  f"{'kabul' if ok else 'KABUL EDİLMEDİ'}")
            res["cikis_hold"] = {"merkez": [round(M[0]), round(M[1])], "kabul": bool(ok)}
        else:
            kayit(f"çıkış Hold merkezi komutu verilmedi ({neden_a}) — uçak geçiş noktasındaki Hold'da")
            res["cikis_hold"] = {"merkez": None, "neden": neden_a}
    res["sonuc"] = "tespit" if yur.tespit else ("bulunamadi" if not yur.kalan else "kesildi")
    kayit(f"ÖZET: {res['sonuc']} — {len(yur.ucuslar)}/{len(plan.sira)} hat, tarama {res['tarama_sn']:.0f} sn; "
          f"neden: {yur.dur_nedeni}" + (f"; tespit {yur.tespit}" if yur.tespit else ""))
    return 0 if hold_ok else 1


def param_olustur(args):
    return tp.Param(psi0=args.yon, yari=args.kutu / 2, merkez=(args.merkez_n, args.merkez_e), W=args.serit,
                    hat_araligi=args.hat_araligi if args.hat_araligi else args.serit, r_min=args.r_min,
                    r_donus=args.r_donus, uzatma=args.uzatma, hat_payi=args.hat_payi, donus_payi=args.donus_payi,
                    hiz=args.speed, irtifa=args.alt, on_mesafe=args.on_mesafe)


def plan_yazdir(plan, kayit=print):
    p, o = plan.p, plan.ozet()
    kayit(f"PLAN: {o['hat_sayisi']} hat (aralık ≤ {p.hat_araligi:.1f} m, W {p.W:.1f} m), yön {p.psi0:.0f}°, "
          f"toplam {o['toplam_m']} m ≈ {o['sure_sn']} sn; geometrik kapsama %{100 * o['geometrik_kapsama']:.1f}, "
          f"C {o['C']:.2f}; dönüşler: {o['stadyum']} stadyum + {o['ampul']} ampul, R {p.r_donus:.0f} m; "
          f"direk: hat ≥ {o['hat_direk_min_m']} m, dönüş ≥ {o['donus_direk_min_m']} m")
    kayit("  sıra (yanal c, m): " + " → ".join(f"{plan.hatlar[i]:+.0f}{'↑' if plan.yon(k) > 0 else '↓'}"
                                               for k, i in enumerate(plan.sira)))
    gi = plan.giris()
    kayit(f"  giriş: hizalama Hold merkezi ({gi['H'][0]:.0f}, {gi['H'][1]:.0f}), hat başı ({gi['bas'][0]:.0f}, {gi['bas'][1]:.0f})")
    hata = plan.dogrula()
    kayit("  doğrulama: " + ("uygun" if not hata else "HATA — " + "; ".join(hata)))
    return not hata


def arguman():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kutu", type=float, default=300.0, help="Kare kutu kenarı [m]")
    ap.add_argument("--merkez-n", type=float, default=0.0)
    ap.add_argument("--merkez-e", type=float, default=0.0)
    ap.add_argument("--yon", type=float, default=0.0, choices=(0.0, 90.0), help="Hat yönü [°] (0: K–G hatlar)")
    ap.add_argument("--serit", type=float, default=16.0, help="Güvenilir tespit şeridi W [m] (60 m irtifa, Adım 1)")
    ap.add_argument("--hat-araligi", type=float, default=0.0, help="En büyük hat aralığı [m]; 0 = W")
    ap.add_argument("--alt", type=float, default=60.0)
    ap.add_argument("--speed", type=float, default=15.0)
    ap.add_argument("--r-min", type=float, default=q.LOITER_R, help="En küçük dönüş yarıçapı (NAV_LOITER_RAD) [m]")
    ap.add_argument("--r-donus", type=float, default=85.0, help="Dönüş yayı yarıçapı [m] (atlama eşiği 2×)")
    ap.add_argument("--uzatma", type=float, default=100.0,
                    help="Hat uçlarında kutu dışı giriş/taşma payı [m] (uçuşta ölçüldü: 40 m yetmiyor, 100 m'de kutu içi hata < 1 m)")
    ap.add_argument("--hat-payi", type=float, default=40.0, help="Düz hat – direk en küçük yatay mesafe [m]")
    ap.add_argument("--donus-payi", type=float, default=55.0, help="Dönüş – direk en küçük yatay mesafe [m]")
    ap.add_argument("--on-mesafe", type=float, default=150.0, help="Hizalama teğet noktası ilk hat başının gerisinde [m]")
    ap.add_argument("--hizali-mesafe", type=float, default=30.0, help="Devralma en geç hat başından bu kadar önce [m]")
    ap.add_argument("--baslangic", type=float, nargs=2, default=None, metavar=("N", "E"),
                    help="Sıra seçiminde uçağın başlangıç konumu (verilmezse canlı konum)")
    ap.add_argument("--icerik", default="HEDEF-01", help="Beklenen QR içeriği ('' = çözülmüş her içerik)")
    ap.add_argument("--clear", type=float, default=40.0, help="Taşıma bacakları – direk [m] (qr_pass)")
    ap.add_argument("--track-margin", type=float, default=q.TRACK_MARGIN)
    ap.add_argument("--max-hops", type=int, default=3)
    ap.add_argument("--replan-wait", type=float, default=60.0)
    ap.add_argument("--align-timeout", type=float, default=300.0)
    ap.add_argument("--min-ayrim", type=float, default=ayrim_mod.MIN_AYRIM, help="Öteki araca en küçük 3B ayrım [m]; 0 = kapalı")
    ap.add_argument("--diger-uav", type=int, default=1)
    ap.add_argument("--qr-baslat", action="store_true", help="/hedef_konum yayıncısı yoksa qr_online.py'yi bu düğüm başlatsın")
    ap.add_argument("--qr-log", default="/tmp/qr_online_log")
    ap.add_argument("--tespitsiz", action="store_true", help="qr_online olmadan da başla (yalnız yol doğrulama uçuşu)")
    ap.add_argument("--hat-sec", type=float, nargs="+", default=None, metavar="C",
                    help="Dilim uçuşu: yalnız bu yanal konumlara (m) en yakın hatlar uçulur")
    ap.add_argument("--ilk-hat", type=float, default=None, help="İlk uçulacak hat (yanal konum, m); verilmezse en kısa sıra")
    ap.add_argument("--iz", help="Tarama izini (5 Hz, PX4 kestirimi) bu CSV'ye yaz")
    ap.add_argument("--yalniz-plan", action="store_true", help="Planı yazdır ve çık (ROS düğümü açılmaz)")
    ap.add_argument("--done-file")
    ap.add_argument("--abort-file")
    ap.add_argument("--result")
    return ap


def main() -> int:
    args = arguman().parse_args()
    p = param_olustur(args)
    if args.yalniz_plan:
        plan = tp.planla(p, tuple(args.baslangic) if args.baslangic else None, sec=args.hat_sec, ilk=args.ilk_hat)
        if plan is None:
            print("PLAN YOK: bu parametrelerle uçulabilir hat/sıra bulunamadı")
            return 1
        return 0 if plan_yazdir(plan) else 1
    for f in (args.done_file, args.abort_file):
        if not f:
            print("--done-file ve --abort-file gerekli (ya da --yalniz-plan)")
            return 1
        if os.path.exists(f):
            os.remove(f)

    import rclpy
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck
    from std_msgs.msg import String
    from nav_izleyici import AUTO_LOITER, NavIzleyici
    from v2_orbit_mission import NAN3, control_mode_name, make_vehicle_command

    class Tarama(q.Pass):
        def __init__(self):
            super().__init__()
            self.tespitler = []
            self.nav = None
            self.create_subscription(String, "/hedef_konum", self._hedef, 10)

        def _hedef(self, m):
            try:
                self.tespitler.append(json.loads(m.data))
            except ValueError:
                pass

        def stream_yol(self, konum, hiz, ivme):
            ocm = OffboardControlMode()
            ocm.timestamp = self.ts()
            ocm.position = True
            self.pub_ocm.publish(ocm)
            sp = TrajectorySetpoint()
            sp.timestamp = self.ts()
            sp.position = [float(v) for v in konum]
            sp.velocity = [float(v) for v in hiz]
            sp.acceleration = NAN3 if ivme is None or not all(math.isfinite(v) for v in ivme) else [float(v) for v in ivme]
            sp.jerk = NAN3
            sp.yaw = math.nan
            sp.yawspeed = math.nan
            self.pub_sp.publish(sp)

        def mod_adi(self):
            return control_mode_name(self.mode)

        def offboard_iste(self):
            self.pub_cmd.publish(make_vehicle_command(0, self.ts(), VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0))

        def hold_iste(self):
            self.ack.pop(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, None)
            self.pub_cmd.publish(make_vehicle_command(0, self.ts(), VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0))

        def hold_dogrulandi(self, t0):
            nav = self.nav
            return bool(self.ack.get(VehicleCommand.VEHICLE_CMD_DO_SET_MODE) == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
                        and self.mode and self.mode.flag_control_auto_enabled and not self.mode.flag_control_offboard_enabled
                        and nav.nav == AUTO_LOITER and nav.fresh(1.5)
                        and nav.history and nav.history[-1][0] >= time.time() - (time.monotonic() - t0))

    rclpy.init()
    node = Tarama()
    nav = node.nav = NavIzleyici(0)
    res = {"baslangic": time.strftime("%H:%M:%S"), "arg": {k: v for k, v in vars(args).items() if not k.endswith("_file")}}
    qr_proc, ayr = None, None
    try:
        t0 = time.monotonic()
        while (node.pos is None or not node.pos.xy_global or node.mode is None) and time.monotonic() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.1)
        ps = node.pos
        if ps is None or node.mode is None:
            log("HATA: konum / mod alınamadı")
            return 1
        nav.wait_first(8.0)
        if not node.mode.flag_armed or node.landed or -ps.z < 30 or nav.nav != AUTO_LOITER:
            log(f"HATA: uçak hazır değil (arm={node.mode.flag_armed}, landed={node.landed}, irtifa={-ps.z:.1f}, "
                f"nav_state={nav.name()})")
            return 1
        # ── tespit zinciri hazır mı ──
        yay_n = node.count_publishers("/hedef_konum")
        if yay_n == 0 and args.qr_baslat:
            os.makedirs(args.qr_log, exist_ok=True)
            qr_proc = subprocess.Popen([sys.executable, "-u", os.path.join(q.HERE, "qr_online.py"), "--log", args.qr_log],
                                       stdout=open(os.path.join(args.qr_log, "qr_online_tarama.log"), "w"),
                                       stderr=subprocess.STDOUT)
            t = time.monotonic()
            while node.count_publishers("/hedef_konum") == 0 and time.monotonic() - t < 20:
                rclpy.spin_once(node, timeout_sec=0.2)
            yay_n = node.count_publishers("/hedef_konum")
            log(f"qr_online.py bu düğümce başlatıldı (pid {qr_proc.pid}); /hedef_konum yayıncısı: {yay_n}")
        zincir = {"qr_online": yay_n, "kamera": node.count_publishers("/uav0/camera/image"),
                  "role_dinleyen": node.count_subscribers("/hedef_konum") - 1,
                  "yaklasma_dinleyen": node.count_subscribers("/hedef_konum_role")}
        res["zincir"] = zincir
        log(f"zincir: qr_online yayıncı {zincir['qr_online']}, kamera köprüsü {zincir['kamera']}, lora_relay dinleyici "
            f"{zincir['role_dinleyen']}, /hedef_konum_role dinleyici (uav1) {zincir['yaklasma_dinleyen']}")
        if (zincir["qr_online"] == 0 or zincir["kamera"] == 0) and not args.tespitsiz:
            log("HATA: tarama boyunca tespit çalışmalı — qr_online / kamera köprüsü yok (LORA=1 ile başlat, "
                "--qr-baslat ya da yalnız yol doğrulaması için --tespitsiz)")
            return 1
        if zincir["role_dinleyen"] <= 0 or zincir["yaklasma_dinleyen"] <= 0:
            log("UYARI: röle / uav1 yaklaşma düğümü dinlemiyor — tespit olsa da zincir devam ETMEYECEK")
        # ── ayrım ──
        if args.min_ayrim > 0:
            ayr = ayrim_mod.DigerAracIzleyici(node, 0, args.diger_uav, kendi_konum=lambda: node.pos,
                                              min_ayrim=args.min_ayrim, iz_payi=args.track_margin)
            t = time.monotonic()
            while time.monotonic() - t < (ayrim_mod.HOLD_PENCERE + 1.5 if ayr.goruldu else 3.0):
                rclpy.spin_once(node, timeout_sec=0.1)
            dg = ayr.diger()
            log("ayrım denetimi: " + (f"uav{args.diger_uav} {dg['tur']}, irtifa {dg.get('irtifa', float('nan')):.0f} m; "
                                      f"eşik 3B {args.min_ayrim:.0f} m" if ayr.goruldu
                                      else f"uav{args.diger_uav} verisi YOK — denetim kapalı (tek araç?)"))
        plan = tp.planla(p, tuple(args.baslangic) if args.baslangic else (node.pos.x, node.pos.y), sec=args.hat_sec, ilk=args.ilk_hat)
        if plan is None or not plan_yazdir(plan, log):
            log("HATA: uygun plan yok")
            res.update(sonuc="plan_yok")
            return 1
        node.tespitler.clear()
        return ucus(node, plan, args, ayr, spin=lambda: rclpy.spin_once(node, timeout_sec=1.0 / LOOP_HZ), res=res)
    finally:
        res["bitis"] = time.strftime("%H:%M:%S")
        if ayr is not None and ayr.ozet():
            res["ayrim_en_kucuk"] = ayr.ozet()
            log(f"AYRIM ÖZETİ: uav{args.diger_uav} ile gerçekleşen en küçük 3B ayrım {res['ayrim_en_kucuk']}")
        if args.result:
            with open(args.result, "a") as f:
                f.write(json.dumps(res, ensure_ascii=False, default=str) + "\n")
        if qr_proc is not None:
            log("not: bu düğümün başlattığı qr_online.py çalışır bırakıldı (zincir sürsün); launch_all.sh stop kapatır")
        nav.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
