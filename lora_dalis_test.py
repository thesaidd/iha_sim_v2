#!/usr/bin/env python3
"""
lora_dalis_test.py — lora_yaklasma.py "dalis" profilinin KURU testi (uçuş / simülasyon / ROS YOK).

Üç bölüm:
  1. Birim testleri: çerçeve, durma noktası, profil irtifası, açı kırpma, durma/devralma/uav0 kararları,
     engel denetimi, geçiş planı (assert; biri tutmazsa çıkış kodu 1).
  2. Senaryo tablosu: farklı başlangıç konumları × iki PX4 alçalma sınırı (bugünkü 2,0 m/s ve önerilen sim
     parametresi 4,0 m/s) → seçilen yön, giriş türü, açı, süre, açıklık, uçulabilirlik. Şekil değişmezliği
     (A'dan sonrası her başlangıçta aynı) assert edilir.
  4. Sahte düğümle akış duman testi: _dalis_akisi'nın kendisi kaba bir araç modeline karşı uçtan uca koşturulur
     (komut sırası, mod geçişleri, kapılar, NO-GO yolları). Zaman sahte; ROS/PX4 yok.
  3. Kinematik benzetim: sabit yer hızı + TECS irtifa referans MODELİ (referans alçalma sınırı, TC, FF; dikey hız
     birinci derece gecikme). Amaç uçuşu öngörmek DEĞİL; komut_irtifa öne-almasının ve durma_karari'nin 3B'de
     (ıskalamadan) tetiklendiğini, dip irtifasının tabanın üstünde kaldığını görmek. Gerçek değerler uçuşta ölçülmeli.

Çıktı: test/lora_dalis_<tarih>/ (tablo.md, kuru_sonuc.json, benzetim.csv).
Kullanım: python3 lora_dalis_test.py [--cikti KLASOR]
"""
import argparse
import csv
import datetime
import json
import math
import os

from lora_yaklasma import (ARENA_ENGELLER_UAV0, _parca, aci_farki, dalis_engelleri, dalis_geometrisi, dalis_plani,
                           devralma_kosulu, dogrudan_kosulu, durma_karari, gecis_plani, giris_karari, hat_irtifa, hat_koordinat,
                           hat_noktalari, komut_irtifa, sabit_yol, uav0_kapisi, uav1_hedef, yol_ihlalleri, yon_birim)

DN, DE = 15.0, -15.0                       # uav1 kaynağı − uav0 kaynağı
T = uav1_hedef(-60.0, -60.0, DN, DE)       # QR hedefi, uav1 NED = (−75, −45)
UCULAMAZ_ACI = 25.0                        # önceki turların "uçulamaz" sınırı (> 25–30° dalış)
SINIRLAR = (("bugünkü PX4 (2,0 m/s)", 2.0, 2.7), ("önerilen sim param (4,0 m/s)", 4.0, 5.0))


def yaklasik(a, b, tol=1e-6):
    return abs(a - b) <= tol


def birim_testleri():
    n = 0
    # çerçeve
    assert T == (-75.0, -45.0); n += 1
    d, r = yon_birim(90.0)
    assert yaklasik(d[0], 0, 1e-9) and yaklasik(d[1], 1) and yaklasik(r[0], -1) and yaklasik(r[1], 0, 1e-9); n += 1
    s, x = hat_koordinat(T[0] - 100.0, T[1] + 7.0, T, 0.0)          # yön kuzey: 100 m geride, 7 m sağda
    assert yaklasik(s, -100.0) and yaklasik(x, 7.0); n += 1
    assert yaklasik(aci_farki(350.0, 10.0), -20.0) and yaklasik(aci_farki(10.0, 350.0), 20.0); n += 1
    # durma noktası: 3B tam durma-mesafesi
    g = dalis_geometrisi()
    assert yaklasik(g["x_durma_m"], math.sqrt(20 ** 2 - 15 ** 2)); n += 1
    assert yaklasik(math.hypot(g["x_durma_m"], hat_irtifa(-g["x_durma_m"], g)), 20.0, 1e-9); n += 1
    # açı kırpma: bugünkü sınırla 12° istenir, atan(0,9·2/15) = 6,84° verilir
    assert g["aci_kirpildi"] and yaklasik(g["aci_etkin_deg"], 6.84, 0.01) and yaklasik(g["alcalma_m_s"], 1.8, 0.01); n += 1
    g4 = dalis_geometrisi(alcalma_siniri=4.0)
    assert not g4["aci_kirpildi"] and g4["aci_etkin_deg"] == 12.0; n += 1
    assert dalis_geometrisi(dalis_acisi=40.0, alcalma_siniri=20.0)["aci_etkin_deg"] == 20.0; n += 1   # azami 20°
    # profil: sürekli, düz girişte sabit, alçalmada eğim, taban
    for gg in (g, g4):
        L, duz = gg["hat_uzunlugu_m"], gg["duz_giris_m"]
        assert hat_irtifa(-L - 50, gg) == gg["giris_irtifa_m"] == hat_irtifa(-L + duz, gg); n += 1
        assert yaklasik(hat_irtifa(-L + duz + 1e-6, gg), gg["giris_irtifa_m"], 1e-3); n += 1          # süreklilik
        e = (hat_irtifa(-150.0, gg) - hat_irtifa(-100.0, gg)) / 50.0
        assert yaklasik(e, math.tan(math.radians(gg["aci_etkin_deg"])), 2e-4); n += 1
        assert hat_irtifa(200.0, gg) == gg["min_irtifa_m"] and komut_irtifa(0.0, gg) >= gg["min_irtifa_m"]; n += 1
        assert komut_irtifa(-150.0, gg) < hat_irtifa(-150.0, gg); n += 1                              # öne alma
    # 'dogrudan' ön-alçalma: A'da giriş irtifası, geriye doğru başlangıç irtifasına kadar yükselir
    assert yaklasik(hat_irtifa(-250.0, g, 120.0), g["giris_irtifa_m"]) and hat_irtifa(-5000.0, g, 120.0) == 120.0; n += 1
    # tutarsız girdiler
    assert not dalis_geometrisi(durma_irtifa=25.0)["uygun"] and not dalis_geometrisi(hat_uzunlugu=60.0)["uygun"]; n += 1
    assert not dalis_geometrisi(durma_irtifa=9.0, durma_mesafesi=20.0, alcalma_siniri=4.0)["uygun"]; n += 1   # dip < taban
    # durma kararı
    assert durma_karari(-13.0, 0.0, 15.0, g) == "durma-mesafesi (3b)" and durma_karari(-60.0, 0.0, 22.0, g) is None; n += 1
    assert durma_karari(-60.0, 0.0, 7.5, g).startswith("irtifa tabanı"); n += 1
    assert durma_karari(0.5, 0.0, 30.0, g).startswith("hedef geçildi"); n += 1
    assert durma_karari(-120.0, 45.0, 30.0, g).startswith("hat dışı"); n += 1
    assert durma_karari(-900.0, 120.0, 120.0, g) is None; n += 1                 # ön-alçalmada hat dışı kuralı yok
    # doğrudan giriş kapısı: ters rota / geniş yanal / yetersiz mesafe reddedilir
    assert dogrudan_kosulu(-1300.0, 20.0, 10.0, 120.0, g) == (True, ""); n += 1
    assert dogrudan_kosulu(-1300.0, 20.0, 170.0, 120.0, g)[1] == "rota"; n += 1
    assert dogrudan_kosulu(-1300.0, 200.0, 0.0, 120.0, g)[1] == "yanal"; n += 1
    assert dogrudan_kosulu(-700.0, 0.0, 0.0, 120.0, g)[1] == "mesafe"; n += 1
    # devralma kapısı
    L = g["hat_uzunlugu_m"]
    assert devralma_kosulu(-L, 2.0, -5.0, g["giris_irtifa_m"], g) == (True, ""); n += 1
    assert devralma_kosulu(-L, 2.0, 20.0, g["giris_irtifa_m"], g)[1] == "rota"; n += 1
    assert devralma_kosulu(-L, 25.0, 0.0, g["giris_irtifa_m"], g)[1] == "yanal"; n += 1
    assert devralma_kosulu(-L, 2.0, 0.0, g["giris_irtifa_m"] + 30, g)[1] == "irtifa"; n += 1
    assert devralma_kosulu(-L + 60.0, 2.0, 0.0, g["giris_irtifa_m"], g)[1] == "konum"; n += 1
    # engeller: uav0 NED → uav1 NED, uav0 Hold çemberi
    eng = dalis_engelleri(DN, DE, T)
    k = next(e_ for e_ in eng if e_["ad"] == "direk_K")
    assert (k["n"], k["e"]) == (125.0, 15.0) and len(eng) == len(ARENA_ENGELLER_UAV0) + 1; n += 1
    u0 = next(e_ for e_ in eng if e_["ad"] == "uav0_bekleme")
    assert yaklasik(math.hypot(u0["n"] - T[0], u0["e"] - T[1]), 150.0, 1e-6) and u0["r"] == 100.0; n += 1
    assert len(dalis_engelleri(DN, DE, T, uav0_yaricap=0)) == len(ARENA_ENGELLER_UAV0); n += 1
    # ihlal: direğin dibinden 120 m'de geçen yol ihlal; 160 m'de (tepe 140 + 15 pay üstü) değil
    assert yol_ihlalleri([(125.0, 20.0, 120.0, "t")], eng)[0] and not yol_ihlalleri([(125.0, 20.0, 160.0, "t")], eng)[0]; n += 1
    # uav0 çemberi: 60 m'de içinden geçmek ihlal, 120 m'de (60 + 40 ayrım üstü) değil
    assert yol_ihlalleri([(u0["n"], u0["e"], 60.0, "t")], eng)[0] and not yol_ihlalleri([(u0["n"], u0["e"], 120.0, "t")], eng)[0]; n += 1
    # geçiş planı: dönen her bacak gerçekten açık; düz yol direkten geçiyorsa ara nokta ya da None (denetimsiz yol yok)
    bas, Hh = (125.0, -200.0, 120.0), (125.0, 250.0)                # düz yol direk_K'nın üstünden geçer
    ara = gecis_plani(bas, Hh, eng, merkez=(-DN, -DE))
    assert ara is not None and len(ara) == 1; n += 1
    dur = [(bas[0], bas[1])] + ara + [Hh]
    for i in range(len(dur) - 1):
        assert not yol_ihlalleri(_parca(dur[i], dur[i + 1], 120.0, 120.0, "g"), eng)[0]; n += 1
    assert gecis_plani((0.0, 300.0, 120.0), (0.0, 420.0), eng) == []; n += 1
    # NO-GO: uav0 payı çok büyükse hiçbir yön açık değildir → plan uygun değil, neden yazılı
    kapali = dalis_plani((0.0, 300.0, 120.0), T, g, dalis_engelleri(DN, DE, T, uav0_pay=400.0), merkez=(-DN, -DE))
    assert not kapali["uygun"] and "açık değil" in kapali["neden"]; n += 1
    # uav0 kapısı
    yol = [q for q in sabit_yol(T, 280.0, g)[0] if q[3] in ("hat", "tırmanış")]
    c0 = (u0["n"], u0["e"])
    assert not uav0_kapisi(None, yol, c0, 100.0)[0]; n += 1                                  # veri yok → geçme
    assert uav0_kapisi((c0[0] + 50.0, c0[1]), yol, c0, 100.0)[0]; n += 1                     # öngörülen çemberde
    assert not uav0_kapisi((T[0], T[1]), yol, c0, 100.0)[0]; n += 1                          # hedefin üstünde
    assert uav0_kapisi((T[0] + 2000.0, T[1]), yol, c0, 100.0)[0]; n += 1                     # çok uzakta
    # giriş kararı: hattın uzantısında yeterince geride → dogrudan; yakında → hizalan
    dd, _ = yon_birim(280.0)
    assert giris_karari((T[0] - 1300 * dd[0], T[1] - 1300 * dd[1], 120.0), T, 280.0, g)[0] == "dogrudan"; n += 1
    assert giris_karari((T[0] - 400 * dd[0], T[1] - 400 * dd[1], 120.0), T, 280.0, g)[0] == "hizalan"; n += 1
    assert giris_karari((T[0] + 50.0, T[1], 120.0), T, 280.0, g)[0] == "hizalan"; n += 1
    # --- 2026-11-12 teşhisi: uçulabilir açı (süzülme sınırı) + devralmada dikey durum kapısı ---
    gs = dalis_geometrisi(alcalma_siniri=4.0, suzulme_siniri=2.4)
    assert gs["aci_kirpildi"] and yaklasik(gs["aci_etkin_deg"], 8.19, 0.01) and gs["aci_istenen_deg"] == 12.0; n += 1
    assert gs["uygun"] and gs["pullup_dip_tahmin_m"] >= gs["min_irtifa_m"]; n += 1
    assert yaklasik(hat_irtifa(-gs["x_durma_m"], gs), 15.0); n += 1                          # S değişmedi (3B 20 m @ 15 m)
    assert dalis_geometrisi(alcalma_siniri=4.0, suzulme_siniri=9.0)["aci_etkin_deg"] == 12.0; n += 1     # büyük → etkisiz
    assert dalis_geometrisi(suzulme_siniri=2.4)["aci_etkin_deg"] == g["aci_etkin_deg"]; n += 1          # PX4 sınırı küçükse o geçerli
    assert g4["aci_etkin_deg"] == 12.0; n += 1                                               # verilmezse eski davranış
    # ölçülen alçalma yeteneği (ort 2,1–2,35 m/s) ile: eski 12° hat ıskalar / kıl payı; yeni hat hep 3B'de durur
    eski = benzetim(g4, 4.0, 2.1, x0=8.0, x_uz=1e6)
    assert eski["durma_nedeni"].startswith("hedef geçildi") and eski["durma_3b_m"] > 20.5; n += 1
    for sink in (2.1, 2.35, 3.0):
        for x0 in (0.0, 8.0):
            r = benzetim(gs, 4.0, sink, x0=x0, x_uz=1e6)
            assert r["durma_nedeni"] == "durma-mesafesi (3b)" and r["durma_3b_m"] <= 20.0, (sink, x0, r)
            assert r["durma_irtifa_m"] >= gs["min_irtifa_m"] + 4.0, (sink, x0, r)
    n += 2
    # dikey kapı: uçuştaki beş devralma (irtifa − giriş, dikey hız): iyi üçü geçer, kötü ikisi bekler
    Ls, gi = gs["hat_uzunlugu_m"], gs["giris_irtifa_m"]
    for dh, vz in ((-0.7, -0.56), (0.4, -0.34), (4.9, -0.64)):                               # D1–D3 (2026-11-10/11)
        assert devralma_kosulu(-Ls, 2.0, -5.0, gi + dh, gs, dikey_hiz=vz) == (True, ""), (dh, vz)
    for dh, vz in ((1.9, 0.87), (2.2, 0.17)):                                                # D4–D5 (2026-11-12)
        assert devralma_kosulu(-Ls, 2.0, -5.0, gi + dh, gs, dikey_hiz=vz) == (False, "dikey"), (dh, vz)
    n += 2
    assert devralma_kosulu(-Ls, 2.0, -5.0, gi + 1.9, gs) == (True, ""); n += 1               # dikey_hiz verilmezse eski davranış
    assert devralma_kosulu(-Ls, 2.0, 20.0, gi, gs, dikey_hiz=2.0)[1] == "rota"; n += 1       # öncelik sırası korunur
    return n


def benzetim(geo, sink_ref, sink_max, tc=5.0, ff=0.3, tau=1.0, x0=0.0, x_uz=60.0, guvenli=120.0, tol=5.0,
             clmb_ref=5.0, clmb_max=6.0, dt=0.02, kayit=None):
    """Hat üzerinde sabit yer hızıyla ilerleyen nokta kütle + TECS irtifa referans modeli (PX4 TECS.cpp'den):
    referans hedefe sink_ref / clmb_ref hız sınırıyla gider; dikey hız komutu = (ref − h)/tc + ff·ref_hızı,
    [−sink_max, clmb_max]; gerçek dikey hız komutu tau zaman sabitiyle izler. Yanal sapma x0'dan x_uz uzunluk
    sabitiyle söner (NPFG yakınsaması için kaba model)."""
    V = geo["hiz"]
    s = -geo["hat_uzunlugu_m"] - 10.0
    h = ref = geo["giris_irtifa_m"]
    vz = 0.0
    s_bas = s
    t = 0.0
    dal = None
    neden = None
    while t < 120.0:
        x = x0 * math.exp(-(s - s_bas) / x_uz)
        neden = durma_karari(s, x, h, geo)
        if neden:
            break
        if dal is None and s >= -geo["hat_uzunlugu_m"] + geo["duz_giris_m"]:
            dal = (s, h, t)
        kom = komut_irtifa(s, geo)
        adim = max(-sink_ref * dt, min(clmb_ref * dt, kom - ref))
        ref += adim
        vz_kom = max(-sink_max, min(clmb_max, (ref - h) / tc + ff * adim / dt))
        vz += (vz_kom - vz) * dt / tau
        h += vz * dt
        s += V * dt
        t += dt
        if kayit is not None and int(t / dt) % 10 == 0:
            kayit.append((round(t, 2), round(s, 1), round(x, 2), round(h, 2), round(hat_irtifa(s, geo), 2), round(kom, 2), round(vz, 2), "hat"))
    dur = {"neden": neden, "s": s, "x": x, "irtifa": h, "3b": math.sqrt(s * s + x * x + h * h), "vz": vz, "t": t}
    aci = math.degrees(math.atan2(dal[1] - h, s - dal[0])) if dal and s > dal[0] else float("nan")
    # tırmanış
    dip = h
    ref = h                                     # mod/komut değişiminde referans mevcut irtifadan başlar
    t_pu = 0.0
    while h < guvenli - tol and t_pu < 90.0:
        adim = max(-sink_ref * dt, min(clmb_ref * dt, guvenli - ref))
        ref += adim
        vz_kom = max(-sink_max, min(clmb_max, (ref - h) / tc + ff * adim / dt))
        vz += (vz_kom - vz) * dt / tau
        h += vz * dt
        s += V * dt
        t += dt
        t_pu += dt
        dip = min(dip, h)
        if kayit is not None and int(t / dt) % 10 == 0:
            kayit.append((round(t, 2), round(s, 1), 0.0, round(h, 2), "", guvenli, round(vz, 2), "tırmanış"))
    return {"durma_nedeni": dur["neden"], "durma_3b_m": round(dur["3b"], 2), "durma_irtifa_m": round(dur["irtifa"], 2),
            "durma_s_m": round(dur["s"], 1), "durma_vz_m_s": round(dur["vz"], 2), "olculen_aci_deg": round(aci, 2),
            "dip_m": round(dip, 2), "tirmanma_sn": round(t_pu, 1), "cikis_m": round(s - dur["s"], 0)}


def senaryolar():
    d280, _ = yon_birim(280.0)
    return [
        ("S1 çok yakın (50 m K)", (T[0] + 50.0, T[1], 120.0)),
        ("S2 belgeleme koşusu (82 m)", (-19.5, 14.0, 114.3)),
        ("S3 gösterim koşusu (110 m)", (-173.7, -81.6, 114.9)),
        ("S4 orta (250 m GD)", (T[0] - 177.0, T[1] + 177.0, 120.0)),
        ("S5 deneme 2 (410 m D)", (33.5, 349.3, 121.7)),
        ("S6 uzak (600 m B)", (T[0], T[1] - 600.0, 120.0)),
        ("S7 uzak (600 m KD)", (T[0] + 424.0, T[1] + 424.0, 120.0)),
        ("S8 çok uzak, hat üstünde (1300 m)", (T[0] - 1300.0 * d280[0], T[1] - 1300.0 * d280[1], 120.0)),
    ]


# ---- [4] Sahte düğümle akış duman testi (ROS/PX4 YOK) ---------------------------------------------
# _dalis_akisi'nın KENDİSİ (komut sırası, mod geçişleri, kapılar, zaman aşımları) kaba bir araç modeline karşı
# koşturulur. Model: Hold'da saat yönünde çember (vektör alanı), OFFBOARD'da verilen hat setpoint'ini izleme
# (yanal: rota = hat yönü − atan(x/30); dikey: benzetim() ile aynı TECS referans modeli), dönüş hızı sınırlı.
# Zaman sahte (lora_yaklasma.time yamalanır) → saniyeler içinde biter. Sayılar uçuş öngörüsü DEĞİLDİR.

class _SahteZaman:
    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def strftime(self, f):
        return f"T+{self.t - 1000.0:6.1f}"


class _VC:
    VEHICLE_CMD_DO_SET_MODE, VEHICLE_CMD_DO_REPOSITION = 176, 192


class _VCA:
    VEHICLE_CMD_RESULT_ACCEPTED = 0


class _Arac:
    def __init__(self, zaman, bas, rota_deg, sink_ref, sink_max, uav0, V=15.0, R=80.0):
        import types
        self.z, self.V, self.R, self.sink_ref, self.sink_max = zaman, V, R, sink_ref, sink_max
        self.n, self.e, self.h = bas
        self.rota = math.radians(rota_deg)
        self.vzv = 0.0
        self.ref = self.h
        self.hold_c, self.hold_alt = (self.n + R * -math.sin(self.rota), self.e + R * math.cos(self.rota)), self.h
        self.offb = False
        self.sp = None
        self.ack = {}
        self.komutlar = []
        self.hedef = None
        self.pub_cmd = types.SimpleNamespace(publish=self._komut)
        self.pos0 = None if uav0 is None else types.SimpleNamespace(x=uav0[0], y=uav0[1], z=-uav0[2])
        self.min_h = self.h
        self.iz = []

    # --- düğüm arayüzü (lora_yaklasma.Yaklasma ile aynı adlar) ---
    @property
    def pos(self):
        import types
        return types.SimpleNamespace(x=self.n, y=self.e, z=-self.h, vx=self.V * math.cos(self.rota),
                                     vy=self.V * math.sin(self.rota), vz=-self.vzv, ref_lat=0.0, ref_lon=0.0, ref_alt=0.0)

    @property
    def mode(self):
        import types
        return types.SimpleNamespace(flag_control_offboard_enabled=self.offb, flag_control_auto_enabled=not self.offb,
                                     flag_armed=True)

    def offboard(self):
        return self.offb

    def ts(self):
        return int(self.z.t * 1e6)

    def stream(self, position, velocity):
        self.sp = (position, velocity)

    def _komut(self, msg):
        cmd, prm = msg
        self.komutlar.append((round(self.z.t - 1000.0, 1), cmd, prm))
        if cmd == _VC.VEHICLE_CMD_DO_REPOSITION:
            self.hold_c, self.hold_alt = (prm[4], prm[5]), prm[6]
        elif cmd == _VC.VEHICLE_CMD_DO_SET_MODE and prm[1] == 6.0:
            assert self.sp is not None, "setpoint akışı olmadan OFFBOARD istendi"
            self.offb = True
            self.ref = self.h
        elif cmd == _VC.VEHICLE_CMD_DO_SET_MODE and prm[1] == 4.0:
            self.offb = False
            self.hold_c, self.hold_alt = (self.n, self.e), self.h
        self.ack[cmd] = _VCA.VEHICLE_CMD_RESULT_ACCEPTED

    def adim(self, dt):
        if self.offb:
            (pn, pe, pz), (vn, ve, _vz) = self.sp
            psi = math.atan2(ve, vn)
            x = -(self.n - pn) * math.sin(psi) + (self.e - pe) * math.cos(psi)
            ist = psi - math.atan2(x, 30.0)
            alt_kom = -pz
        else:
            dn, de = self.hold_c[0] - self.n, self.hold_c[1] - self.e
            uz = math.hypot(dn, de)
            ist = math.atan2(de, dn) - (math.pi / 2 - max(-0.6, min(math.pi / 2, math.atan2(uz - self.R, 40.0) * 2)))
            alt_kom = self.hold_alt
        fark = (ist - self.rota + math.pi) % (2 * math.pi) - math.pi
        self.rota += max(-0.35 * dt, min(0.35 * dt, fark))
        ad = max(-self.sink_ref * dt, min(5.0 * dt, alt_kom - self.ref))
        self.ref += ad
        vk = max(-self.sink_max, min(6.0, (self.ref - self.h) / 5.0 + 0.3 * ad / dt))
        self.vzv += (vk - self.vzv) * dt / 1.0
        self.h += self.vzv * dt
        self.n += self.V * math.cos(self.rota) * dt
        self.e += self.V * math.sin(self.rota) * dt
        self.min_h = min(self.min_h, self.h)
        self.iz.append((self.n, self.e, self.h))


def sahte_kosu(bas, rota_deg, sinir, sink_max, uav0="cember", ek_arg=()):
    """_dalis_akisi'yı sahte araçla uçtan uca koştur. Dönüş: (çıkış kodu, res, araç)."""
    import types
    import lora_yaklasma as ly
    # Bu duman testleri uav0'ı SABİT ön-hesapla (qr_pass 210° hattı) sınar; canlı kaynak: lora_tarama_zincir_test.py
    args = ly.build_parser().parse_args(["--profil", "dalis", "--alcalma-siniri", str(sinir), "--uav0-kaynak", "sabit", "--baslangic", "hava", *ek_arg])
    zaman = _SahteZaman()
    eng = dalis_engelleri(DN, DE, T)
    c0 = next((g["n"], g["e"]) for g in eng if g["ad"] == "uav0_bekleme")
    u0 = {"cember": (c0[0] + 60.0 + DN, c0[1] + DE, 60.0), "hedefte": (T[0] + DN, T[1] + DE, 60.0), "yok": None}[uav0]
    arac = _Arac(zaman, bas, rota_deg, sinir, sink_max, u0)      # pos0 uav0 NED'inde yayınlanır
    arac.hedef = T

    def spin_once(node, timeout_sec=0.05):
        dt = max(0.02, min(timeout_sec, 0.1))
        zaman.t += dt
        node.adim(dt)
    nav_sinif = type("Nav", (), {"nav": property(lambda self: "OFFBOARD" if arac.offb else "LOITER"),
                                 "fresh": lambda self, a=2.0: True,
                                 "name": lambda self: "OFFBOARD" if arac.offb else "AUTO_LOITER"})
    res = {}
    c = types.SimpleNamespace(
        node=arac, nav=nav_sinif(), res=res, rclpy=types.SimpleNamespace(spin_once=spin_once), LOOP_HZ=20,
        tespit_t=zaman.t, rtf_ort=lambda: 0.95, abort_nedeni=lambda: None, abort_nedeni_kritik=lambda: None,
        AUTO_LOITER="LOITER", VehicleCommand=_VC, VehicleCommandAck=_VCA,
        make_vehicle_command=lambda uav, ts, cmd, *prm: (cmd, tuple(float(v) for v in prm)),
        ne_to_latlon=lambda n, e, rl, ro: (n, e), control_mode_name=lambda m: "OFFBOARD" if m.flag_control_offboard_enabled else "AUTO")
    eski_t, eski_log = ly.time, ly._log
    satirlar = []
    ly.time = zaman
    ly._log = lambda m: satirlar.append(f"{zaman.strftime('')}  {m}")
    try:
        kod = ly._dalis_akisi(args, c)
    finally:
        ly.time, ly._log = eski_t, eski_log
    return kod, res, arac, satirlar, eng


def duman_testleri(cikti, md):
    d280, _ = yon_birim(280.0)
    durumlar = [
        ("D1 çok yakın (S1), bugünkü PX4", (T[0] + 50.0, T[1], 120.0), 0.0, 2.0, 2.7, "cember", (), 0),
        ("D2 gösterim (S3, ara noktalı), bugünkü PX4", (-173.7, -81.6, 114.9), 200.0, 2.0, 2.7, "cember", (), 0),
        ("D3 deneme 2 (S5), önerilen sim param", (33.5, 349.3, 121.7), 90.0, 4.0, 5.0, "cember", (), 0),
        ("D4 çok uzak hat üstü (S8, dogrudan), önerilen", (T[0] - 1300 * d280[0], T[1] - 1300 * d280[1], 120.0), 100.0, 4.0, 5.0, "cember", (), 0),
        ("D5 uav0 hedefin üstünde kaldı → kapı geçilmez, NO-GO", (T[0] + 50.0, T[1], 120.0), 0.0, 2.0, 2.7, "hedefte", ("--hizalama-asimi", "150"), 3),
        ("D6 uav0 konumu gelmiyor → NO-GO", (T[0] + 50.0, T[1], 120.0), 0.0, 2.0, 2.7, "yok", ("--hizalama-asimi", "150"), 3),
        ("D7 açık yön yok (uav0 payı 400) → plan NO-GO", (T[0] + 50.0, T[1], 120.0), 0.0, 2.0, 2.7, "cember", ("--uav0-pay", "400"), 3),
    ]
    md.append("\n## Sahte düğümle akış duman testi — MODEL, uçuş değil\n\n| durum | kod | sonuç | hizalama sn | devralma s / x / irtifa | "
              "durma nedeni | 3B / irtifa | ölçülen açı° | dip | en küçük irtifa | engel ihlali | toplam sn |\n" + "|---|" * 12)
    for ad, bas, rota, sinir, smax, uav0, ek, beklenen in durumlar:
        kod, res, arac, satirlar, eng = sahte_kosu(bas, rota, sinir, smax, uav0, ek)
        assert kod == beklenen, (ad, kod, satirlar[-3:])
        ihl, _ = yol_ihlalleri([(n, e, h, "iz") for n, e, h in arac.iz[::5]], [g for g in eng])
        if kod == 0:
            assert res["sonuc"] == "tamam" and res["durma_nedeni"] == "durma-mesafesi (3b)", (ad, res)
            assert res["durma"]["3b_m"] <= 20.0 and res["dip_irtifa_m"] >= 8.0, (ad, res)
            assert not ihl, (ad, ihl[:3])
            assert not arac.offb                                     # Hold'da bitti
            dv, du = res["devralma"], res["durma"]
            sat = (f"| {ad} | {kod} | {res['sonuc']} | {res['hizalama_sn']:.0f} | {dv['s_m']:+.0f} / {dv['x_m']:+.1f} / {dv['irtifa_m']:.1f} | "
                   f"{res['durma_nedeni']} | {du['3b_m']:.1f} / {du['irtifa_m']:.1f} | {res['dalis']['olculen_aci_deg']} | "
                   f"{res['dip_irtifa_m']:.1f} | {arac.min_h:.1f} | yok | {res['toplam_manevra_sn']:.0f} |")
        else:
            assert res["sonuc"] == "no-go" and not arac.offb, (ad, res)
            assert all(k[1] != _VC.VEHICLE_CMD_DO_SET_MODE for k in arac.komutlar), ad   # OFFBOARD hiç istenmedi
            sat = f"| {ad} | {kod} | NO-GO: {res['neden'][:70]} | — | — | — | — | — | — | {arac.min_h:.1f} | {'yok' if not ihl else ihl[0]} | — |"
        md.append(sat)
        print("    " + sat)
        open(os.path.join(cikti, f"duman_{ad.split()[0]}.log"), "w").write("\n".join(satirlar) + "\n")
    return len(durumlar)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cikti", default=None)
    args = ap.parse_args()
    cikti = args.cikti or os.path.join(os.path.dirname(os.path.abspath(__file__)), "test",
                                       f"lora_dalis_{datetime.date.today().isoformat()}")
    os.makedirs(cikti, exist_ok=True)
    md = []

    n = birim_testleri()
    print(f"[1] birim testleri: {n} assert ✔")
    md.append(f"# dalis profili kuru test — {datetime.date.today().isoformat()}\n\nUçuş YOK. Hedef uav1 NED {T}. "
              f"Birim testleri: {n} assert ✔\n")

    sonuc = {"hedef_uav1_NED": T, "birim_assert": n, "sinirlar": {}}
    for ad, sinir, sink_max in SINIRLAR:
        geo = dalis_geometrisi(alcalma_siniri=sinir)
        eng = dalis_engelleri(DN, DE, T)
        acik_yonler = [y for y in range(360) if not yol_ihlalleri(sabit_yol(T, y, geo)[0], eng)[0]]
        print(f"\n[2] {ad}: açı {geo['aci_istenen_deg']}° → {geo['aci_etkin_deg']}° (alçalma {geo['alcalma_m_s']} m/s), "
              f"giriş irtifası {geo['giris_irtifa_m']:.1f} m, dalış {geo['dalis_yatay_m']:.1f} m / −{geo['dalis_irtifa_kaybi_m']} m / "
              f"{geo['dalis_sn']} sn, durma S: hedefin {geo['x_durma_m']:.1f} m gerisi @ {geo['durma_irtifa_m']} m, "
              f"dip tahmini {geo['pullup_dip_tahmin_m']} m")
        print(f"    sabit yol açısından açık yönler (1° tarama): {len(acik_yonler)}° → {acik_yonler}")
        bas = (f"| {'başlangıç':<34} | yatay | geliş° | yön° | sapma° | giriş | ara nokta | açı° | giriş irt. | "
               f"dalış kaybı | 3B dur / irt. | geçiş+çember+hat+tırm. = toplam sn | en küçük açıklık (pay üstü) | uçulabilir |")
        md.append(f"\n## {ad}\n\nProfil (başlangıçtan bağımsız): istenen {geo['aci_istenen_deg']}° → **etkin {geo['aci_etkin_deg']}°** "
                  f"(alçalma {geo['alcalma_m_s']} m/s), hat {geo['hat_uzunlugu_m']:.0f} m = düz giriş {geo['duz_giris_m']:.0f} + dalış "
                  f"{geo['dalis_yatay_m']:.1f} m (−{geo['dalis_irtifa_kaybi_m']} m, {geo['dalis_sn']} sn) + durma payı {geo['x_durma_m']:.1f} m; "
                  f"giriş irtifası {geo['giris_irtifa_m']:.1f} m; durma 3B {geo['durma_mesafesi_m']:.0f} m @ {geo['durma_irtifa_m']:.0f} m; "
                  f"pull-up dip tahmini {geo['pullup_dip_tahmin_m']} m (taban {geo['min_irtifa_m']:.0f} m). "
                  f"Açık yönler: {len(acik_yonler)}° ({acik_yonler[0] if acik_yonler else '-'}–{acik_yonler[-1] if acik_yonler else '-'}°).\n\n"
                  + bas + "\n|" + "---|" * 14)
        satirlar = {}
        sekil = None
        for sad, b in senaryolar():
            p = dalis_plani(b, T, geo, eng, merkez=(-DN, -DE))
            assert p["uygun"], (sad, p.get("neden"))
            assert geo["aci_etkin_deg"] <= 20.0 < UCULAMAZ_ACI
            # şekil değişmezliği: A → S profili hat koordinatında her başlangıçta aynı
            pts, hn = sabit_yol(T, p["yon_deg"], geo, bas_irtifa=b[2])
            imza = [(round(hat_koordinat(q[0], q[1], T, p["yon_deg"])[0], 3), round(q[2], 3)) for q in pts if q[3] == "hat"]
            sekil = sekil or imza
            assert imza == sekil, sad
            # seçilen planın tüm parçaları gerçekten açık (bağımsız yeniden denetim)
            assert not yol_ihlalleri(pts if p["giris"] == "hizalan" else [q for q in pts if q[3] != "hizalanma çemberi"], eng)[0]
            dur = [(b[0], b[1])] + [tuple(w) for w in p["ara_noktalar"]] + ([hn["H"]] if p["giris"] == "hizalan" else [])
            for i in range(len(dur) - 1):
                assert not yol_ihlalleri(_parca(dur[i], dur[i + 1], b[2], b[2], "g"), eng)[0], sad
            su = p["sure_sn"]
            ek = ", ".join(f"{k} {v:+.1f}" for k, v in list(p["en_kucuk_aciklik_m"].items())[:2])
            ara = "; ".join(f"({w[0]:.0f}, {w[1]:.0f})" for w in p["ara_noktalar"]) or "—"
            sat = (f"| {sad:<34} | {p['baslangic_yatay_m']:.0f} | {p['gelis_yonu_deg']:.0f} | {p['yon_deg']:.0f} | "
                   f"{p['yon_sapma_deg']:+.0f} | {p['giris']} | {ara} | {geo['aci_etkin_deg']} | {geo['giris_irtifa_m']:.1f} | "
                   f"{geo['dalis_irtifa_kaybi_m']} | {geo['durma_mesafesi_m']:.0f} / {geo['durma_irtifa_m']:.0f} | "
                   f"{su['gecis']:.0f}+{su['cember']:.0f}+{su['hat']:.0f}+{su['tirmanma']:.0f} = {su['toplam']:.0f} | {ek} | evet |")
            md.append(sat)
            print("    " + sat)
            satirlar[sad] = p
        sonuc["sinirlar"][ad] = {"geo": geo, "acik_yonler": acik_yonler, "senaryolar": satirlar}

        # [3] kinematik benzetim (model)
        print(f"\n[3] kinematik benzetim ({ad}; MODEL — uçuş değil):")
        md.append(f"\n### Kinematik benzetim ({ad}) — MODEL, uçuş değil\n\n| durum | durma nedeni | 3B dur | irtifa | s | "
                  f"ölçülen açı° | dip | tırmanış sn | çıkış m |\n|---|---|---|---|---|---|---|---|---|")
        ben = {}
        durumlar = [("nominal (öne alma 3,5 sn, τ 1,0)", {}, geo),
                    ("öne alma YOK", {}, dalis_geometrisi(alcalma_siniri=sinir, irtifa_onalma=0.0)),
                    ("yavaş tepki τ 2,0", {"tau": 2.0}, geo), ("hızlı tepki τ 0,5", {"tau": 0.5}, geo),
                    ("devralma yanal 10 m", {"x0": 10.0}, geo), ("devralma yanal 19 m", {"x0": 19.0}, geo)]
        for dad, kw, gg in durumlar:
            kayit = [] if dad.startswith("nominal") else None
            r = benzetim(gg, sinir, sink_max, kayit=kayit, **kw)
            ben[dad] = r
            sat = (f"| {dad} | {r['durma_nedeni']} | {r['durma_3b_m']} | {r['durma_irtifa_m']} | {r['durma_s_m']} | "
                   f"{r['olculen_aci_deg']} | {r['dip_m']} | {r['tirmanma_sn']} | {r['cikis_m']:.0f} |")
            md.append(sat)
            print("    " + sat)
            if kayit:
                with open(os.path.join(cikti, f"benzetim_{sinir:.0f}ms.csv"), "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["t_s", "s_m", "x_m", "irtifa_m", "profil_m", "komut_m", "vz_m_s", "asama"])
                    w.writerows(kayit)
        nom = ben["nominal (öne alma 3,5 sn, τ 1,0)"]
        assert nom["durma_nedeni"] == "durma-mesafesi (3b)", nom
        assert nom["dip_m"] >= geo["min_irtifa_m"], nom
        # planın çıkış yolu varsayımı benzetimden kısa kalmamalı (denetlenen yol, uçulandan uzun olmalı)
        hn = hat_noktalari(T, 280.0, geo)
        assert hn["cikis_m"] >= nom["cikis_m"] - 15.0, (hn["cikis_m"], nom["cikis_m"])
        sonuc["sinirlar"][ad]["benzetim"] = ben

    print("\n[4] sahte düğümle akış duman testi (MODEL — uçuş değil):")
    sonuc["duman_durum_sayisi"] = duman_testleri(cikti, md)

    json.dump(sonuc, open(os.path.join(cikti, "kuru_sonuc.json"), "w"), indent=1, ensure_ascii=False, default=str)
    open(os.path.join(cikti, "tablo.md"), "w").write("\n".join(md) + "\n")
    print(f"\nTÜM KURU TESTLER ✔  çıktı: {cikti}/ (tablo.md, kuru_sonuc.json, benzetim_*.csv, duman_*.log)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
