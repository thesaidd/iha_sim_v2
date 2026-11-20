#!/usr/bin/env python3
"""
yer_gorevi.py — uav1 "YERDEN KALKIŞ" senaryosu (SİMÜLASYON; 2026-11-15). lora_yaklasma.py --baslangic yer kullanır.

Akış: uav1 üste, pistte, yerde ve DISARMED bekler → ilk /hedef_konum_role mesajı komuttur → (uav0'ın çıkış
Hold'u ölçülür) → PLAN (kalkıştan ÖNCE; uygun yön yoksa NO-GO: uav1 hiç kalkmaz) → iniş görevi PX4'e yüklenir →
arm + pist kalkışı (mavlink_takeoff.take_off — yeniden yazılmadı) → kalkış irtifası geçilince OFFBOARD, hizalanma
çemberi YOK: doğrudan yaklaşma hattına katılma → lora_yaklasma'nın MEVCUT hat / durma / tırmanış mantığı →
denetlenmiş dönüş rotası (OFFBOARD, güvenli irtifada) → PX4 görev inişi (LOITER_TO_ALT + LAND) → disarm.

PX4 v1.17 kaynak bulguları (src/modules; ayrıntı test/yerden_kalkis_2026-11-15/tasarim.md):
  - Kalkış (FixedWingModeManager::control_auto_takeoff, navigator/takeoff.cpp): pist yönü = arm anındaki yaw
    (_launch_current_yaw); açıklık irtifası = MIS_TAKEOFF_ALT; irtifa geçilince navigator o noktada Hold'a döner
    (GO_TO_LOITER). OFFBOARD devralması bu geçişten sonra yapılır.
  - İniş: pist yönüne hizalı DÜZ iniş yalnız görevde ve önceki yol noktası geçerliyken mümkün
    (FW_POSCTRL_MODE_AUTO_LANDING_STRAIGHT); Land modu / RTL dairesel iner (hizasız). Desen: LOITER_TO_ALT
    (teğet çıkış, loiter_exit_xtrack) → LAND; FeasibilityChecker süzülme açısını FW_LND_ANG ile sınar.
  - Yer algılanınca COM_DISARM_LAND (2 sn) sonra otomatik disarm; arm edip kalkmayan araç COM_DISARM_PRFLT
    (10 sn) sonra disarm olur → arm, kalkış komutuyla BİRLİKTE verilir.

Güvenlik: denetimsiz bacak yok. Plan (katılma + hat + tırmanış + dönüş + iniş deseni) kalkıştan önce direk /
bina / uav0 Hold çemberi açısından denetlenir; dönüş rotası bulunamazsa da NO-GO. Uçuşta bir aşama başarısız olursa
araç Hold'a alınır (kalkış yerde başarısızsa disarm) ve neden loglanır.
"""
import math
import time

import ayrim as ayrim_mod

NAV_AUTO_MISSION, NAV_AUTO_LOITER, NAV_OFFBOARD, NAV_AUTO_TAKEOFF, NAV_AUTO_LAND = 3, 4, 14, 17, 18
MAV_CMD_NAV_LAND, MAV_CMD_NAV_LOITER_TO_ALT, MAV_CMD_COMPONENT_ARM_DISARM = 21, 31, 400
ZORLA_DISARM = 21196.0        # MAV_CMD_COMPONENT_ARM_DISARM param2 (PX4: ön denetimleri atlar)
YERDE_IRTIFA, YERDE_VZ = 1.0, 0.5    # 'araç yerde' ölçütü (zorla disarm yalnız bu durumda)

# İNİŞ / YERDEN KALKIŞ parametreleri — YALNIZCA SİMÜLASYON senaryosu için; gerçek donanıma körlemesine TAŞINMAZ.
# isim: (senaryo değeri, geri alma değeri = airframe 4008 / PX4 varsayılanı). Yalnız uav1'e yazılır.
INIS_PARAMS = {
    # Düz inişte flare irtifası arazi kestirimiyle (mesafe sensörü) belirlenir; sensör geçerli ölçüm vermezse
    # FW_LND_ABORT bit 0/1 inişi iptal eder (getLandingTerrainAltitudeEstimate). Simülasyon zemini düz ve iniş
    # noktası irtifası kesin (ev irtifası) → arazi kestirimi kapalı, flare iniş noktası irtifasına göre.
    "FW_LND_USETER": (0, 1),
    # Görevde kalkış / iniş öğesi zorunluluğu olmasın (varsayılan zaten 0; açıkça sabitlenir): görev yalnız
    # iniş desenidir, kalkış Takeoff moduyla yapılır.
    "MIS_TKO_LAND_REQ": (0, 0),
}


def _ly():
    import lora_yaklasma as ly           # geç içe aktarma (lora_yaklasma bu modülü fonksiyon içinde yükler)
    return ly


def _birim(yon_deg):
    a = math.radians(yon_deg)
    return (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))


def _aci(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


SIM_PIST_YONU = 90.0          # gz yaw 0; aşağıdaki kısıt yüzünden tek güvenli pist yönü
SIM_YAW_TOL = 1.0


def sim_yaw_uygun(pist_yonu_deg, tol=SIM_YAW_TOL):
    """SİMÜLATÖR KISITI (2026-11-16, uçuşla saptandı): `advanced_plane` modelinin AdvancedLiftDrag aerodinamiği
    yalnız gz yaw 0 (NED 90°, doğu) spawn'ında doğru çalışıyor. yaw −15° (pist 105°) ile uçak gövdesi 105°'yi
    gösterirken TAM DOĞUYA kayıyor (gerçek yanal kayma −15°), gerçek pitch 0,000°'de kalıyor, elevatör doyuyor ve
    uçak hiç kalkamıyor — aynı koşuda yaw 0 ile doğan uav0 normal kalktı (ulg kanıtı: ucus/t1). Gerçek donanımda
    böyle bir kısıt YOKTUR; modeldeki/eklentideki yön kısıtıdır."""
    return abs((pist_yonu_deg - SIM_PIST_YONU + 180.0) % 360.0 - 180.0) <= tol


def gz_poz(us_n, us_e, pist_yonu_deg, z=0.5):
    """Üs (uav0 NED) + pist yönü → PX4_GZ_MODEL_POSE metni (Gazebo ENU: x = Doğu, y = Kuzey, yaw 0 = Doğu)."""
    return f"{us_e:g},{us_n:g},{z:g},0,0,{math.radians(90.0 - pist_yonu_deg):.4f}"


# ---- Saf geometri / plan (ROS yok) -----------------------------------------------------------------

def katil_benzetimi(P, rota_deg, V0, T, yon_deg, L, V=15.0, a_yanal=3.5, L1=60.0, hiz_tau=6.0, dt=0.1, asim=150.0,
                    otur_x=5.0, otur_rota=5.0):
    """Kalkış sonu noktasından (P, rota, hız V0) yaklaşma hattına katılmanın kaba kinematik öngörüsü: uçak hattın
    en yakın noktasını izler (rota isteği = hat yönü − atan(x / L1)), dönüş hızı yanal ivmeyle sınırlı, hız V0'dan
    V'ye hiz_tau ile iner. Amaç uçuşu öngörmek DEĞİL; "hatta oturmaya mesafe yetiyor mu" denetimi ve katılma izinin
    engel denetimi için örnek noktalar. Dönüş: {"iz": [(n, e)], "s_otur": oturduğu s (ya da None),
    "x_A", "rota_A": A'daki yanal / rota hatası, "s0", "x0"}."""
    ly = _ly()
    n, e = P
    rota = math.radians(rota_deg)
    v = V0
    s0, x0 = ly.hat_koordinat(n, e, T, yon_deg)
    iz, s_otur, t = [(n, e)], None, 0.0
    s, x = s0, x0
    while s < -L and t < asim:
        ist = math.radians(yon_deg) - math.atan2(x, L1)
        fark = (ist - rota + math.pi) % (2 * math.pi) - math.pi
        w = a_yanal / max(v, 1.0)
        rota += max(-w * dt, min(w * dt, fark))
        v += (V - v) * dt / hiz_tau
        n += v * math.cos(rota) * dt
        e += v * math.sin(rota) * dt
        t += dt
        s, x = ly.hat_koordinat(n, e, T, yon_deg)
        rh = _aci(math.degrees(rota), yon_deg)
        if abs(x) <= otur_x and abs(rh) <= otur_rota:
            if s_otur is None:
                s_otur = s
        else:
            s_otur = None
        if len(iz) == 0 or math.hypot(n - iz[-1][0], e - iz[-1][1]) >= 5.0:
            iz.append((n, e))
    return {"iz": iz, "s_otur": s_otur, "x_A": x, "rota_A": _aci(math.degrees(rota), yon_deg), "s0": s0, "x0": x0,
            "vardi": s >= -L}


def inis_deseni(pist_yonu_deg, td_uzaklik=50.0, yaklasma_m=350.0, giris_irtifa=25.0, yaricap=80.0):
    """İniş deseni, uav1 NED (üs = orijin, pist ekseni üsten pist_yonu doğrultusunda). İniş yönü = KALKIŞ YÖNÜ
    (pist arena tarafına bakar): yaklaşma hattı ve orbit üssün ARENADAN UZAK tarafında kalır — arena tarafına
    konunca orbit + görev devri bölgesi direklere / uav0 Hold'una yaklaşıyordu (üs seçimi hesabı, 2026-11-15).
    Teker koyma noktası Td üssün td_uzaklik m gerisinde (rulo üsse doğru, pist ekseninde). Yaklaşma girişi G = Td'nin
    yaklasma_m gerisi, giris_irtifa'da; orbit merkezi Lc = G'nin yaricap m SAĞI (saat yönü orbit → teğet çıkış G'de,
    rota = iniş yönü)."""
    inis_yonu = pist_yonu_deg % 360.0
    dl, rl = _birim(inis_yonu)
    Td = (-td_uzaklik * dl[0], -td_uzaklik * dl[1])
    G = (Td[0] - yaklasma_m * dl[0], Td[1] - yaklasma_m * dl[1])
    Lc = (G[0] + yaricap * rl[0], G[1] + yaricap * rl[1])
    return {"inis_yonu_deg": inis_yonu, "Td": Td, "G": G, "Lc": Lc, "yaricap": yaricap, "giris_irtifa_m": giris_irtifa,
            "yaklasma_m": yaklasma_m, "suzulme_deg": math.degrees(math.atan2(giris_irtifa, yaklasma_m))}


def inis_gorevi(desen, ne_to_latlon, ref_lat, ref_lon):
    """PX4 görev öğeleri (MISSION_ITEM_INT alanları; irtifa eve göre): LOITER_TO_ALT (yön şartı + teğet çıkış) → LAND."""
    la, lo = ne_to_latlon(desen["Lc"][0], desen["Lc"][1], ref_lat, ref_lon)
    ta, to = ne_to_latlon(desen["Td"][0], desen["Td"][1], ref_lat, ref_lon)
    return [
        {"cmd": MAV_CMD_NAV_LOITER_TO_ALT, "p1": 1.0, "p2": desen["yaricap"], "p3": 0.0, "p4": 1.0,
         "lat": la, "lon": lo, "alt": desen["giris_irtifa_m"]},
        {"cmd": MAV_CMD_NAV_LAND, "p1": 0.0, "p2": 0.0, "p3": 0.0, "p4": math.nan, "lat": ta, "lon": to, "alt": 0.0},
    ]


def _paylı(engeller, ek):
    return [dict(g, pay_y=g["pay_y"] + ek) for g in engeller]


def _disk_acik(merkez, yaricap, alt, engeller):
    """merkez çevresinde yaricap m'lik bölge (dönüş taşması / orbit) alt irtifasında engellerden uzak mı."""
    en = math.inf
    for g in engeller:
        if alt >= g["yuk"] + g["pay_d"]:
            continue
        en = min(en, math.hypot(merkez[0] - g["n"], merkez[1] - g["e"]) - g["r"] - g["pay_y"] - yaricap)
    return en >= 0.0, en


def donus_rotasi(C, yon_deg, Lc, engeller, irtifa, merkez=(0.0, 0.0), iz_payi=25.0, donus_disk=150.0,
                 kucuk_disk=80.0, lc_disk=250.0, lc_alt=25.0, son_bacak_alt=None):
    """Tırmanış sonu C'den (rota = hat yönü) iniş orbit merkezi Lc'ye, güvenli irtifada, OFFBOARD düz bacaklarla
    dönüş rotası. Her bacak direk payı + iz_payi ile, her köşe (dönüş taşması) bir disk olarak denetlenir; 90°'den
    büyük dönüşte disk = donus_disk. Lc çevresi (görev devri + orbit, lc_alt … irtifa) ayrıca denetlenir.
    Dönüş: {"rota": [C, ara…, Lc], "uzunluk_m", "en_kucuk_m"} ya da None — denetimsiz bacak YOK."""
    ly = _ly()
    eng_bacak = _paylı(engeller, iz_payi)

    def bacak_acik(a, b, alt=None):
        alt = irtifa if alt is None else alt
        ihl, acik = ly.yol_ihlalleri(ly._parca(a, b, alt, alt, "dönüş"), eng_bacak)
        return (not ihl), (min(acik.values()) if acik else math.inf)

    ok_lc1, a1 = _disk_acik(Lc, lc_disk, irtifa, engeller)
    ok_lc2, a2 = _disk_acik(Lc, 100.0, lc_alt, engeller)
    if not (ok_lc1 and ok_lc2):
        return None

    def dene(noktalar):
        en, onceki = math.inf, yon_deg
        for i in range(len(noktalar) - 1):
            a, b = noktalar[i], noktalar[i + 1]
            if math.dist(a, b) < 1.0:
                return None
            yb = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            ok, d1 = _disk_acik(a, donus_disk if abs(_aci(yb, onceki)) > 90.0 else kucuk_disk, irtifa, engeller)
            if not ok:
                return None
            # son bacakta iniş deseni irtifasına alçalınıyorsa o bacak DAHA DÜŞÜK irtifada denetlenir
            ok, d2 = bacak_acik(a, b, son_bacak_alt if (son_bacak_alt is not None and i == len(noktalar) - 2) else None)
            if not ok:
                return None
            en, onceki = min(en, d1, d2), yb
        return en
    adaylar = [[C, Lc]]
    halka = [(merkez[0] + R * math.cos(math.radians(k)), merkez[1] + R * math.sin(math.radians(k)))
             for R in (300.0, 400.0, 500.0, 600.0) for k in range(0, 360, 15)]
    adaylar += [[C, w, Lc] for w in halka]
    en_iyi = None
    for nk in adaylar:
        en = dene(nk)
        if en is None:
            continue
        uz = sum(math.dist(nk[i], nk[i + 1]) for i in range(len(nk) - 1))
        if en_iyi is None or uz < en_iyi[0]:
            en_iyi = (uz, nk, en)
    if en_iyi is None:                                   # iki ara nokta (kaba halka)
        kaba = [(merkez[0] + R * math.cos(math.radians(k)), merkez[1] + R * math.sin(math.radians(k)))
                for R in (350.0, 500.0) for k in range(0, 360, 30)]
        for w1 in kaba:
            for w2 in kaba:
                if w1 == w2:
                    continue
                uz = math.dist(C, w1) + math.dist(w1, w2) + math.dist(w2, Lc)
                if en_iyi is not None and uz >= en_iyi[0]:
                    continue
                en = dene([C, w1, w2, Lc])
                if en is not None:
                    en_iyi = (uz, [C, w1, w2, Lc], en)
    if en_iyi is None:
        return None
    return {"rota": [(round(p[0], 1), round(p[1], 1)) for p in en_iyi[1]], "uzunluk_m": round(en_iyi[0], 1),
            "en_kucuk_m": round(min(en_iyi[2], a1, a2), 1)}


def yer_plani(T, geo, engeller, pist_yonu_deg, kalkis_mesafe=120.0, kalkis_irtifa=25.0, kalkis_hizi=22.0, uav0=None,
              yaricap=80.0, guvenli_irtifa=120.0, tirmanma_hizi=3.0, tirmanma_tol=5.0, sapma_max=20.0, yon_adim=5.0,
              uav0_min=110.0, merkez=(0.0, 0.0), desen=None, iz_payi=25.0, katil_kw=None, desen_irtifa=60.0):
    """Yerden kalkış planı, uav1 NED (üs = orijin). KALKIŞTAN ÖNCE hesaplanır.
    Kerteriz = kalkış sonu noktasından (üs + kalkis_mesafe · pist yönü) hedefe doğrudan yön; uygun değilse ±yon_adim
    adımlarla ±sapma_max'a kadar kaydırılır. Her aday için: (1) katılma: kinematik öngörüyle uçak alçalma başından
    en az yarım düz-giriş önce hatta oturmalı; (2) katılma izi (+iz_payi), hat, tırmanış (S → C) ve C'deki Hold
    çemberi engellere (direk payı engeller listesinde) açık; (3) uav0'ın çıkış Hold çemberine en kötü durum 3B
    ≥ uav0_min (ayrim.bacak_ayrimi; çember yarıçapı 80 m, irtifa bandı ±10 m); (4) dönüş rotası + iniş deseni var.
    uav0 = {"merkez": (n, e), "irtifa": m} (uav1 NED) ya da None. Dönüş: dalis_plani ile aynı anahtarlar + "yer"."""
    ly = _ly()
    dp, _ = _birim(pist_yonu_deg)
    Pk = (kalkis_mesafe * dp[0], kalkis_mesafe * dp[1])
    yatay = math.hypot(T[0] - Pk[0], T[1] - Pk[1])
    kerteriz = math.degrees(math.atan2(T[1] - Pk[1], T[0] - Pk[0])) % 360.0
    plan = {"hedef": [round(T[0], 1), round(T[1], 1)], "baslangic_yatay_m": round(math.hypot(T[0], T[1]), 1),
            "gelis_yonu_deg": round(kerteriz, 1), "tercih_yon_deg": round(kerteriz, 1), "geo": geo, "uygun": False,
            "giris": "yer", "ara_noktalar": [],
            "yer": {"pist_yonu_deg": pist_yonu_deg, "kalkis_mesafe_m": kalkis_mesafe, "kalkis_irtifa_m": kalkis_irtifa,
                    "kalkis_sonu": [round(Pk[0], 1), round(Pk[1], 1)], "kalkis_sonu_hedef_m": round(yatay, 1)}}
    if not geo["uygun"]:
        plan["neden"] = "; ".join(geo["nedenler"])
        return plan
    desen = desen or inis_deseni(pist_yonu_deg)
    plan["yer"]["inis"] = {k: ([round(x, 1) for x in v] if isinstance(v, tuple) else round(v, 2) if isinstance(v, float) else v)
                           for k, v in desen.items()}
    # (0) pist + kalkış uzantısı ve iniş hattı: her irtifada engellerden uzak (yerde tüm engeller sayılır)
    yer_hatti = ly._parca((0.0, 0.0), (1.5 * kalkis_mesafe * dp[0], 1.5 * kalkis_mesafe * dp[1]), 0.0, 0.0, "pist/kalkış")
    yer_hatti += ly._parca(desen["G"], (0.0, 0.0), 0.0, 0.0, "iniş hattı")
    ihl, _a = ly.yol_ihlalleri(yer_hatti, engeller)
    if ihl:
        plan["neden"] = f"üs uygun değil: {ihl[0][0]} ↔ {ihl[0][1]} ({ihl[0][2]} m)"
        return plan
    L, duz = geo["hat_uzunlugu_m"], geo["duz_giris_m"]
    katil_irtifa = max(kalkis_irtifa, geo["giris_irtifa_m"])
    diger = None
    if uav0 is not None:
        diger = {"tur": "hold", "merkez": tuple(uav0["merkez"]), "yaricap": ayrim_mod.HOLD_R, "irtifa": uav0["irtifa"],
                 "konum": tuple(uav0["merkez"])}
    eng_katil = _paylı(engeller, iz_payi)
    adaylar = [0.0]
    for k in range(1, int(sapma_max / yon_adim) + 1):
        adaylar += [k * yon_adim, -k * yon_adim]
    ilk_red = None

    def red(neden):
        nonlocal ilk_red
        if ilk_red is None:
            ilk_red = neden
    kw = dict(yaricap=yaricap, guvenli_irtifa=guvenli_irtifa, tirmanma_hizi=tirmanma_hizi, tirmanma_tol=tirmanma_tol)
    for sapma in adaylar:
        yon = (kerteriz + sapma) % 360.0
        kat = katil_benzetimi(Pk, pist_yonu_deg, kalkis_hizi, T, yon, L, V=geo["hiz"], **(katil_kw or {}))
        if not kat["vardi"] or kat["s_otur"] is None or kat["s_otur"] > -L + 0.5 * duz:
            red(f"katılma mesafesi yetmiyor (yön {yon:.0f}°: kalkış sonu s={kat['s0']:.0f} m, x={kat['x0']:+.0f} m; "
                f"oturma s={'yok' if kat['s_otur'] is None else round(kat['s_otur'])})")
            continue
        pts, hn = ly.sabit_yol(T, yon, geo, bas_irtifa=guvenli_irtifa, **kw)
        pts = [q for q in pts if q[3] != "hizalanma çemberi"]
        ihl, acik = ly.yol_ihlalleri(pts, engeller)
        kpts = [(n, e, katil_irtifa, "katılma") for n, e in kat["iz"]]
        ihl_k, acik_k = ly.yol_ihlalleri(kpts, eng_katil)
        for k_, v_ in acik_k.items():
            acik[k_] = min(acik.get(k_, math.inf), v_)
        if ihl or ihl_k:
            b = (ihl or ihl_k)[0]
            red(f"{b[0]} ↔ {b[1]} ({b[2]} m)")
            continue
        uav0_en = math.inf
        if diger is not None:
            kotu = None
            dizi = kpts + [q for q in pts if q[3] in ("hat", "tırmanış")]
            for i in range(len(dizi) - 1):
                a, b = dizi[i], dizi[i + 1]
                if a[3] != b[3] and not (a[3] == "katılma" and b[3] == "hat"):
                    continue
                ok, en, _m = ayrim_mod.bacak_ayrimi((a[0], a[1]), (b[0], b[1]), a[2], b[2], diger, uav0_min,
                                                    iz_payi if a[3] == "katılma" else 0.0)
                uav0_en = min(uav0_en, en)
                if not ok and kotu is None:
                    kotu = (a[3], en)
            okc, enc, _m = ayrim_mod.cember_ayrimi(hn["C"], guvenli_irtifa, diger, ayrim_mod.MIN_AYRIM)
            if kotu is not None or not okc:
                red(f"uav0 Hold çemberine ayrım yetmiyor ({kotu[0] if kotu else 'son Hold çemberi'}: "
                    f"{(kotu[1] if kotu else enc):.0f} m < {uav0_min if kotu else ayrim_mod.MIN_AYRIM:.0f})")
                continue
        don = donus_rotasi(hn["C"], yon, desen["Lc"], engeller, guvenli_irtifa, merkez, iz_payi,
                           lc_alt=desen["giris_irtifa_m"], son_bacak_alt=min(desen_irtifa, guvenli_irtifa))
        if don is None:
            red("dönüş rotası yok (C → iniş orbiti)")
            continue
        hat_sn = (L - geo["x_durma_m"]) / geo["hiz"]
        katil_m = sum(math.dist(kat["iz"][i], kat["iz"][i + 1]) for i in range(len(kat["iz"]) - 1))
        plan.update({
            "uygun": True, "yon_deg": round(yon, 1), "yon_sapma_deg": sapma, "ilk_red": ilk_red,
            "giris_detay": {"s": round(kat["s0"], 1), "x": round(kat["x0"], 1), "s_otur": round(kat["s_otur"], 1),
                            "rota_farki_deg": round(_aci(yon, pist_yonu_deg), 1)},
            "A": [round(v, 1) for v in hn["A"]], "H": [round(v, 1) for v in hn["H"]],
            "S": [round(v, 1) for v in hn["S"]], "C": [round(v, 1) for v in hn["C"]],
            "cikis_m": round(hn["cikis_m"], 1), "gecis_m": round(katil_m, 1),
            "en_kucuk_aciklik_m": {k_: round(v, 1) for k_, v in sorted(acik.items(), key=lambda kv: kv[1])[:3]},
            "uav0_en_kucuk_3b_m": None if uav0_en == math.inf else round(uav0_en, 1),
            "donus": don,
            "sure_sn": {"katilma": round(katil_m / geo["hiz"], 1), "hat": round(hat_sn, 1), "dalis": geo["dalis_sn"],
                        "tirmanma": round(hn["tirmanma_sn"], 1), "donus": round(don["uzunluk_m"] / geo["hiz"], 1)}})
        return plan
    plan["ilk_red"] = ilk_red
    plan["neden"] = f"±{sapma_max:.0f}° içinde uygun yön yok (ilk ret: {ilk_red})"
    return plan


# ---- MAVLink yardımcıları (mavlink_takeoff.Vehicle üzerinden) ------------------------------------------

def parametreleri_yaz(arac, uav_id=1, log=print):
    """uav1'e SIM_ONLY_PARAMS + UCUS_DINAMIGI_PARAMS (ayar) + INIS_PARAMS yazar. Dönüş: başarı."""
    import mavlink_takeoff as mt
    tum = {k: v[0] for k, v in mt.SIM_ONLY_PARAMS.items()}
    tum.update({k: v[0] for k, v in mt.UCUS_DINAMIGI_PARAMS.get(uav_id, {}).items()})
    tum.update({k: v[0] for k, v in INIS_PARAMS.items()})
    for ad, deger in tum.items():
        if not arac.ensure_param(ad, deger):
            log(f"HATA: parametre yazılamadı: {ad}")
            return False
    return True


def kalkis_komutu(arac, irtifa, param_bekleme=1.5, log=print):
    """Kanıtlanmış pist kalkış mantığı: mavlink_takeoff.take_off (MIS_TAKEOFF_ALT + NAV_TAKEOFF + arm, yeniden
    denemeli) — YENİDEN YAZILMADI, olduğu gibi çağrılır.
    MIS_TAKEOFF_ALT yarışı (2026-11-15 a1): parametre yazıldıktan 0,2 sn sonra NAV_TAKEOFF verilince navigator hâlâ
    ESKİ değeri okuyor ("Using default takeoff altitude: 120.0 m"; PX4 parametre bildirimi toplu/gecikmeli). Bu yüzden
    irtifa burada önceden yazılır, geri okunarak doğrulanır ve param_bekleme sn beklenir; take_off içindeki
    ensure_param artık 'zaten ayarlı' bulur (kanıtlanmış sıra bozulmaz)."""
    import mavlink_takeoff as mt
    if not arac.ensure_param("MIS_TAKEOFF_ALT", irtifa):
        return False
    time.sleep(param_bekleme)
    okunan = arac.get_param("MIS_TAKEOFF_ALT")
    if okunan is None or abs(okunan[0] - irtifa) > 1e-3:
        log(f"HATA: MIS_TAKEOFF_ALT doğrulanamadı (okunan {okunan})")
        return False
    log(f"MIS_TAKEOFF_ALT = {irtifa:.0f} m doğrulandı ({param_bekleme:.1f} sn bekleme sonrası)")
    return mt.take_off(arac, irtifa, irtifa)


def gorev_yukle(arac, ogeler, asim=10.0):
    """Görev öğelerini MAVLink görev protokolüyle (MISSION_COUNT → MISSION_REQUEST(_INT) → MISSION_ITEM_INT →
    MISSION_ACK) yükler. arac = mavlink_takeoff.Vehicle (okuma iş parçacığı mesajları tüketir; son mesajlar
    arac.m.messages sözlüğünden izlenir). Dönüş: (başarı, açıklama)."""
    m = arac.m
    t0 = time.time()
    with arac.send_lock:
        m.mav.mission_count_send(arac.sysid, 1, len(ogeler), 0)
    son = None
    while time.time() - t0 < asim:
        for tip in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            r = m.messages.get(tip)
            if r is None or getattr(r, "_timestamp", 0.0) < t0 or (tip, r.seq, r._timestamp) == son:
                continue
            son = (tip, r.seq, r._timestamp)
            if not 0 <= r.seq < len(ogeler):
                return False, f"geçersiz öğe isteği {r.seq}"
            o = ogeler[r.seq]
            with arac.send_lock:
                m.mav.mission_item_int_send(arac.sysid, 1, r.seq, 6, o["cmd"], 0, 1, o["p1"], o["p2"], o["p3"], o["p4"],
                                            int(round(o["lat"] * 1e7)), int(round(o["lon"] * 1e7)), float(o["alt"]), 0)
        a = m.messages.get("MISSION_ACK")
        if a is not None and getattr(a, "_timestamp", 0.0) >= t0:
            if a.type != 0:
                return False, f"MISSION_ACK tip {a.type}"
            return gorev_basa_al(arac, len(ogeler))
        time.sleep(0.02)
    return False, "zaman aşımı (MISSION_ACK yok)"


def gorev_basa_al(arac, n_oge, asim=5.0):
    """Görev sırasını 0. öğeye al ve DOĞRULA. Neden (2026-11-16 a3 arızası): PX4 görev dizinini (seq_current)
    `dataman`'da KALICI tutuyor; önceki uçuşun görevi 1. öğede (LAND) bitmişse yeni görev yüklendiğinde dizin
    sıfırlanmıyor ve AUTO_MISSION doğrudan LAND'den başlıyor → yaklaşma yol noktası olmadığı için PX4 DAİRESEL
    iniş yapıyor (iz, iniş noktası çevresinde R 80 m çember; teker koyma pist ekseninin 78 m kuzeyinde, ulg
    kanıtı). MISSION_SET_CURRENT(0) + MISSION_CURRENT doğrulaması bunu kapatıyor."""
    m = arac.m
    bitis = time.time() + asim
    t0 = time.time()
    with arac.send_lock:
        m.mav.mission_set_current_send(arac.sysid, 1, 0)
    while time.time() < bitis:
        cur = m.messages.get("MISSION_CURRENT")
        if cur is not None and getattr(cur, "_timestamp", 0.0) >= t0:
            if cur.seq == 0:
                return True, f"MISSION_ACK ✔, sıra 0. öğede (toplam {n_oge})"
            t0 = time.time()
            with arac.send_lock:
                m.mav.mission_set_current_send(arac.sysid, 1, 0)
        time.sleep(0.05)
    son = getattr(m.messages.get("MISSION_CURRENT"), "seq", "?")
    return False, f"görev sırası 0. öğeye alınamadı (MISSION_CURRENT seq={son})"


# ---- Uçuş aşamaları (lora_yaklasma._dalis_akisi çağırır; c = uçuş bağlamı) --------------------------------

def yerde_bekle(args, c, kosul, asim, ad, log):
    """uav1 yerde ve disarmed beklerken kosul() sağlanana kadar bekle. Dönüş: None ya da neden."""
    import os
    node, rclpy = c.node, c.rclpy
    t0 = time.monotonic(); nxt = t0
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
        if os.path.exists(args.abort_file):
            return "abort: abort-file"
        if node.mode is not None and node.mode.flag_armed:
            return "abort: araç beklenmedik biçimde arm oldu"
        if kosul():
            return None
        if time.monotonic() - t0 > asim:
            return f"zaman aşımı ({ad}, {asim:.0f} sn)"
        if time.monotonic() >= nxt:
            nxt += 10.0
            log(f"{ad} (uav1 yerde, disarmed)")


def kalkis(args, c, arac, log):
    """Arm + pist kalkışı; teker kesme ve kalkış irtifasının geçilmesini bekler. Dönüş: (başarı, ölçümler).
    Başarısızlıkta güvenli durum: yerdeyse disarm; havadaysa PX4 Takeoff modu zaten Hold'a geçer (ayrıca Hold komutu)."""
    node, rclpy, res = c.node, c.rclpy, c.res
    p0 = (node.pos.x, node.pos.y)
    t_komut = time.monotonic()
    olc = {"komut_saat": time.strftime("%H:%M:%S")}
    res["kalkis"] = olc

    def guvenli_dur(neden):
        """Kalkış başarısız: motoru kes ve aracı YERDE durdur. PX4 'Takeoff detected'ten sonra normal DISARM'ı
        'not landed' diye reddediyor (2026-11-15 a1: araç 40 sn pistte yuvarlandı). Bu yüzden normal disarm
        reddedilirse ZORLA disarm (param2 21196) gönderilir — ama YALNIZCA araç gerçekten yerdeyken
        (irtifa < 1 m ve |dikey hız| < 0,5 m/s, ölçümle doğrulanır). Havadaki araca asla."""
        pp = node.pos
        alt_s, vz_s = -pp.z, abs(pp.vz)
        yerde = alt_s < YERDE_IRTIFA and vz_s < YERDE_VZ
        log(f"KALKIŞ BAŞARISIZ: {neden} — irtifa {alt_s:.1f} m, |vz| {vz_s:.1f} m/s, yer_hızı "
            f"{math.hypot(pp.vx, pp.vy):.1f} m/s → {'DISARM' if yerde else 'HAVADA: Hold'}")
        olc["hata"] = neden
        olc["durum"] = {"irtifa_m": round(alt_s, 2), "vz_m_s": round(-pp.vz, 2),
                        "yer_hizi_m_s": round(math.hypot(pp.vx, pp.vy), 1), "yerde": yerde}
        if not yerde:                                   # havada: zorla disarm YOK, güvenli mod Hold
            node.pub_cmd.publish(c.make_vehicle_command(args.uav, node.ts(), c.VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                                        1.0, 4.0, 3.0))
            olc["guvenli_mod"] = "hold"
            return False, olc
        olc["disarm_ack"] = arac.command(MAV_CMD_COMPONENT_ARM_DISARM, 0)
        t_d = time.monotonic()
        while time.monotonic() - t_d < 3.0:             # disarm oldu mu
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.mode is not None and not node.mode.flag_armed:
                olc["guvenli_mod"] = "disarm"
                log("DISARM ✔ — araç yerde, motor kesildi")
                return False, olc
        pp = node.pos
        if -pp.z < YERDE_IRTIFA and abs(pp.vz) < YERDE_VZ:
            log(f"DISARM reddedildi (ack {olc['disarm_ack']}) ve araç hâlâ yerde → ZORLA DISARM (21196)")
            olc["zorla_disarm_ack"] = arac.command(MAV_CMD_COMPONENT_ARM_DISARM, 0, ZORLA_DISARM)
            t_d = time.monotonic()
            while time.monotonic() - t_d < 3.0:
                rclpy.spin_once(node, timeout_sec=0.05)
                if node.mode is not None and not node.mode.flag_armed:
                    olc["guvenli_mod"] = "zorla_disarm"
                    log("ZORLA DISARM ✔ — motor kesildi, araç yerde duracak")
                    return False, olc
            olc["guvenli_mod"] = "disarm_basarisiz"
            log("UYARI: zorla disarm da sonuç vermedi — araç hâlâ ARMED (elle müdahale gerekir)")
        else:
            olc["guvenli_mod"] = "havalandi_disarm_iptal"
            log("UYARI: disarm beklenirken araç yerden ayrıldı — zorla disarm GÖNDERİLMEDİ")
        return False, olc
    if not kalkis_komutu(arac, args.kalkis_irtifa, args.param_bekleme, log):
        return guvenli_dur("arm / kalkış komutu kabul edilmedi")
    olc["arm_sn"] = round(time.monotonic() - t_komut, 2)
    log(f"KALKIŞ komutu kabul edildi (arm + Takeoff, MIS_TAKEOFF_ALT {args.kalkis_irtifa:.0f} m; {olc['arm_sn']:.1f} sn)")
    t_teker = None; nxt = time.monotonic(); v_maks = 0.0
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        pp = node.pos
        alt, yer_hizi = -pp.z, math.hypot(pp.vx, pp.vy)
        v_maks = max(v_maks, yer_hizi)
        gecen = time.monotonic() - t_komut
        havada = (node.landed is False) and alt > 1.0
        if t_teker is None and havada and -pp.vz > 0.3:
            t_teker = time.monotonic()
            olc["teker_kesme_sn"] = round(t_teker - t_komut, 2)
            olc["yer_kosusu_m"] = round(math.hypot(pp.x - p0[0], pp.y - p0[1]), 1)
            olc["teker_kesme_hizi_m_s"] = round(yer_hizi, 1)
            log(f"TEKER KESME: komuttan {olc['teker_kesme_sn']:.1f} sn sonra, yer koşusu {olc['yer_kosusu_m']:.0f} m, "
                f"yer hızı {yer_hizi:.1f} m/s")
        # Havalanma YALNIZCA irtifa + dikey hızla doğrulanır: PX4 "Takeoff detected" yanlış bildirebiliyor
        # (a1: yerde yuvarlanırken bildirdi) → land_detected / STATUSTEXT'e güvenilmez.
        if t_teker is None:
            if node.mode is not None and not node.mode.flag_armed and gecen > 3.0:
                return guvenli_dur("araç kalkmadan disarm oldu (COM_DISARM_PRFLT?)")
            if gecen > args.kalkis_irtifa_asimi and alt < args.kalkis_irtifa_esik:
                return guvenli_dur(f"{args.kalkis_irtifa_asimi:.0f} sn'de irtifa kazanılmadı "
                                   f"(irtifa {alt:.1f} m < {args.kalkis_irtifa_esik:.0f} m)")
            if math.hypot(pp.x - p0[0], pp.y - p0[1]) > args.pist_boyu:
                return guvenli_dur(f"pist sonu aşıldı (üsten {math.hypot(pp.x - p0[0], pp.y - p0[1]):.0f} m "
                                   f"> {args.pist_boyu:.0f} m), araç hâlâ yerde")
            if gecen > args.kalkis_asimi:
                return guvenli_dur(f"{args.kalkis_asimi:.0f} sn içinde teker kesilmedi")
        else:
            if alt >= args.kalkis_irtifa - 1.0:
                olc["kalkis_mesafe_m"] = round(math.hypot(pp.x - p0[0], pp.y - p0[1]), 1)
                olc["kalkis_irtifa_sn"] = round(gecen, 2)
                olc["kalkis_sonu_hizi_m_s"] = round(yer_hizi, 1)
                olc["kalkis_sonu"] = [round(pp.x, 1), round(pp.y, 1), round(alt, 1)]
                olc["kalkis_sonu_rota_deg"] = round(math.degrees(math.atan2(pp.vy, pp.vx)) % 360.0, 1)
                olc["en_yuksek_yer_hizi_m_s"] = round(v_maks, 1)
                log(f"KALKIŞ İRTİFASI geçildi: {alt:.1f} m, üsten yatay {olc['kalkis_mesafe_m']:.0f} m, "
                    f"komuttan {gecen:.1f} sn, yer hızı {yer_hizi:.1f} m/s, rota {olc['kalkis_sonu_rota_deg']:.0f}°")
                return True, olc
            if time.monotonic() - t_teker > args.kalkis_asimi:
                return guvenli_dur(f"teker kesildi ama {args.kalkis_asimi:.0f} sn'de {args.kalkis_irtifa:.0f} m'ye "
                                   f"çıkılamadı (irtifa {alt:.1f} m)")
        if time.monotonic() >= nxt:
            nxt += 2.0
            log(f"kalkış: irtifa {alt:5.1f} m  yer_hızı {yer_hizi:4.1f} m/s  üsten {math.hypot(pp.x - p0[0], pp.y - p0[1]):5.0f} m  "
                f"arm={node.mode.flag_armed if node.mode else '?'}  yerde={node.landed}")


def _hold_ver(args, c, log, neden):
    """Güvenli mod: Hold (AUTO_LOITER). OFFBOARD'dayken akış mod değişene kadar sürer."""
    node, nav = c.node, c.nav
    log(f"GÜVENLİ MOD: {neden} → Hold")
    t0 = time.monotonic(); son = 0.0
    while time.monotonic() - t0 < 10.0:
        c.rclpy.spin_once(node, timeout_sec=0.05)
        pp = node.pos
        v = math.hypot(pp.vx, pp.vy) or 1.0
        node.stream((pp.x + 200.0 * pp.vx / v, pp.y + 200.0 * pp.vy / v, -args.guvenli_irtifa),
                    (args.hiz * pp.vx / v, args.hiz * pp.vy / v, 0.0))
        if nav.nav == NAV_AUTO_LOITER and nav.fresh(1.5) and not node.offboard():
            return True
        if time.monotonic() - son >= 0.5:
            son = time.monotonic()
            node.pub_cmd.publish(c.make_vehicle_command(args.uav, node.ts(), c.VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                                        1.0, 4.0, 3.0))
    return False


def donus_inis(args, c, arac, plan, T, yon_deg, log):
    """Tırmanıştan sonra: hat üzerinde C'ye → denetlenmiş dönüş rotası (OFFBOARD, güvenli irtifa) → Lc yakınında
    PX4 görevine (iniş deseni) devir → teker koyma → disarm. Dönüş: çıkış kodu (0 = indi ve disarm oldu)."""
    node, nav, res, rclpy = c.node, c.nav, c.res, c.rclpy
    ly = _ly()
    VC = c.VehicleCommand
    don, desen = plan["donus"], plan["yer"]["inis"]
    rota = [tuple(p) for p in don["rota"]]
    alt_sp = args.guvenli_irtifa
    # İNİŞ DESENİ İRTİFASI (2026-11-16 a3 bulgusu): uav1 iniş orbitine 120 m'de gelince PX4 LOITER_TO_ALT
    # öğesini atlayıp DAİRESEL inişe geçti (teker koyma pist ekseninin 68 m kuzeyinde; iz, iniş noktası
    # çevresinde 80 m yarıçaplı çember). 60 m'de gelen a2 uçuşunda DÜZ iniş çalıştı (yanal 0,0 m). Bu yüzden
    # son bacakta desen irtifasına inilir (plan bu bacağı o irtifada denetledi).
    desen_alt = min(args.inis_desen_irtifa, args.guvenli_irtifa)
    olc = {"rota": don["rota"]}
    res["donus"] = olc
    t_bas = time.monotonic()
    Rt = 60.0

    def guvenli(neden, kod=4):
        ok = _hold_ver(args, c, log, neden)
        olc["hata"] = neden; olc["guvenli_mod_hold"] = ok
        res["sonuc"] = "donus_basarisiz"; res["neden"] = neden
        return kod
    # 0) hat üzerinde C'ye kadar (tırmanış bacağı — planda denetlendi)
    d, _r = _birim(yon_deg)
    sC, _x = ly.hat_koordinat(rota[0][0], rota[0][1], T, yon_deg)
    s, _x = ly.hat_koordinat(node.pos.x, node.pos.y, T, yon_deg)
    if s > sC + 100.0:
        return guvenli(f"tırmanış planlanan C'nin {s - sC:.0f} m ötesinde bitti (denetlenmemiş bölge)")
    y1 = math.degrees(math.atan2(rota[1][1] - rota[0][1], rota[1][0] - rota[0][0]))
    on = min(120.0, Rt * math.tan(math.radians(min(abs(_aci(y1, yon_deg)), 170.0)) / 2.0))
    while True:
        rclpy.spin_once(node, timeout_sec=1.0 / c.LOOP_HZ)
        if not node.offboard():
            log(f"ABORT: OFFBOARD kaybı (dönüş, mod={c.control_mode_name(node.mode)})")
            res["sonuc"] = "abort"; res["neden"] = "OFFBOARD kaybı (dönüş)"; return 2
        n = c.abort_nedeni()
        if n:
            return guvenli(f"abort (dönüş): {n}", 2)
        pp = node.pos
        s, _x = ly.hat_koordinat(pp.x, pp.y, T, yon_deg)
        if s >= sC - on:
            break
        node.stream((T[0] + (s + 200.0) * d[0], T[1] + (s + 200.0) * d[1], -alt_sp), (args.hiz * d[0], args.hiz * d[1], 0.0))
    # 1) bacaklar
    en_x = 0.0; nxt = time.monotonic()
    for i in range(len(rota) - 1):
        a, b = rota[i], rota[i + 1]
        Lb = math.dist(a, b)
        yb = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        db, rb = _birim(yb)
        son = i == len(rota) - 2
        if son:
            on = 0.0
        else:
            y2 = math.degrees(math.atan2(rota[i + 2][1] - b[1], rota[i + 2][0] - b[0]))
            on = min(120.0, Rt * math.tan(math.radians(min(abs(_aci(y2, yb)), 170.0)) / 2.0))
        log(f"DÖNÜŞ bacağı {i + 1}/{len(rota) - 1}: ({a[0]:.0f}, {a[1]:.0f}) → ({b[0]:.0f}, {b[1]:.0f}), yön {yb % 360:.0f}°, "
            f"{Lb:.0f} m, irtifa {alt_sp:.0f}" + (f" → {desen_alt:.0f} m (iniş deseni)" if son else " m"))
        hizalandi = False
        while True:
            rclpy.spin_once(node, timeout_sec=1.0 / c.LOOP_HZ)
            if not node.offboard():
                log(f"ABORT: OFFBOARD kaybı (dönüş, mod={c.control_mode_name(node.mode)})")
                res["sonuc"] = "abort"; res["neden"] = "OFFBOARD kaybı (dönüş)"; return 2
            n = c.abort_nedeni()
            if n:
                return guvenli(f"abort (dönüş): {n}", 2)
            pp = node.pos
            sb = (pp.x - a[0]) * db[0] + (pp.y - a[1]) * db[1]
            xb = (pp.x - a[0]) * rb[0] + (pp.y - a[1]) * rb[1]
            rh = _aci(math.degrees(math.atan2(pp.vy, pp.vx)), yb)
            if not hizalandi and abs(xb) < 15.0 and abs(rh) < 15.0:
                hizalandi = True
            if hizalandi:
                en_x = max(en_x, abs(xb))
            sn = max(0.0, min(Lb, sb))
            alt_k = alt_sp + (desen_alt - alt_sp) * min(1.0, max(0.0, sn / Lb)) if (son and Lb > 1.0) else alt_sp
            node.stream((a[0] + sn * db[0], a[1] + sn * db[1], -alt_k), (args.hiz * db[0], args.hiz * db[1], 0.0))
            if son and math.dist((pp.x, pp.y), b) <= args.gorev_mesafe:
                break
            if sb >= Lb - on:
                if son:
                    return guvenli("son dönüş bacağı bitti ama görev devri mesafesine girilmedi")
                break
            if time.monotonic() - t_bas > args.donus_asimi:
                return guvenli(f"dönüş zaman aşımı ({args.donus_asimi:.0f} sn)")
            if time.monotonic() >= nxt:
                nxt += 5.0
                log(f"dönüş: bacak {i + 1}  s={sb:6.0f}/{Lb:.0f} m  x={xb:+6.1f} m  rota hatası={rh:+6.1f}°  irtifa={-pp.z:5.1f} m")
    olc["bacak_yanal_maks_m"] = round(en_x, 1)
    olc["devir_irtifa_m"] = round(-node.pos.z, 1)
    olc["desen_irtifa_m"] = desen_alt
    olc["donus_sn"] = round(time.monotonic() - t_bas, 1)
    # 2) PX4 görevine devir (iniş deseni): akış mod değişene kadar sürer
    Lc = tuple(desen["Lc"]); Td = tuple(desen["Td"])
    log(f"GÖREV DEVRİ: iniş orbitine {math.dist((node.pos.x, node.pos.y), Lc):.0f} m; AUTO_MISSION isteniyor "
        f"(LOITER_TO_ALT ({Lc[0]:.0f}, {Lc[1]:.0f}) R {desen['yaricap']:.0f} → {desen['giris_irtifa_m']:.0f} m → LAND "
        f"({Td[0]:.0f}, {Td[1]:.0f}), iniş yönü {desen['inis_yonu_deg']:.0f}°)")
    t0 = time.monotonic(); son_k = 0.0
    while not (nav.nav == NAV_AUTO_MISSION and nav.fresh(1.5)):
        rclpy.spin_once(node, timeout_sec=1.0 / c.LOOP_HZ)
        pp = node.pos
        dn, de = Lc[0] - pp.x, Lc[1] - pp.y
        u = math.hypot(dn, de) or 1.0
        node.stream((Lc[0], Lc[1], -desen_alt), (args.hiz * dn / u, args.hiz * de / u, 0.0))
        if time.monotonic() - son_k >= 1.0:
            son_k = time.monotonic()
            node.pub_cmd.publish(c.make_vehicle_command(args.uav, node.ts(), VC.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 4.0))
        if time.monotonic() - t0 > 15.0:
            return guvenli("AUTO_MISSION moduna geçilemedi (15 sn)")
    olc["gorev_devri_saat"] = time.strftime("%H:%M:%S")
    log("AUTO_MISSION ✔ — iniş deseni PX4'te")
    # 3) inişi izle
    ins = {}
    res["inis"] = ins
    dl, rl = _birim(desen["inis_yonu_deg"])
    t0 = time.monotonic(); nxt = t0; son_vz = []; dusuk = False; t_yer = None; en_dusuk = math.inf
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        pp = node.pos
        alt = -pp.z
        en_dusuk = min(en_dusuk, alt)
        son_vz.append((time.monotonic(), -pp.vz, alt)); del son_vz[:-60]
        if alt < 10.0:
            dusuk = True
        if node.landed and t_yer is None and dusuk:
            t_yer = time.monotonic()
            boy = (pp.x - Td[0]) * dl[0] + (pp.y - Td[1]) * dl[1]
            yan = (pp.x - Td[0]) * rl[0] + (pp.y - Td[1]) * rl[1]
            ins.update({"yer_algilama_saat": time.strftime("%H:%M:%S"), "konum": [round(pp.x, 1), round(pp.y, 1)],
                        "boyuna_m": round(boy, 1), "yanal_m": round(yan, 1), "yer_hizi_m_s": round(math.hypot(pp.vx, pp.vy), 1),
                        "son_3sn_en_buyuk_alcalma_m_s": round(-min(v for _t, v, _a in son_vz), 2),
                        "gorevden_sn": round(t_yer - t0, 1)})
            log(f"YER ALGILANDI: Td'ye göre boyuna {boy:+.1f} m, yanal {yan:+.1f} m; yer hızı {ins['yer_hizi_m_s']} m/s")
        if t_yer is not None and node.mode is not None and not node.mode.flag_armed:
            ins["disarm_sn"] = round(time.monotonic() - t_yer, 1)
            ins["son_konum"] = [round(pp.x, 1), round(pp.y, 1)]
            ins["ustten_m"] = round(math.hypot(pp.x, pp.y), 1)
            log(f"DISARM ✔ (yer algılamadan {ins['disarm_sn']:.1f} sn sonra); son konum ({pp.x:.0f}, {pp.y:.0f}), üsse {ins['ustten_m']:.0f} m")
            res["sonuc"] = "indi"
            res["toplam_gorev_sn"] = round(time.monotonic() - c.tespit_t, 1)
            return 0
        if t_yer is not None and time.monotonic() - t_yer > 20.0:
            if "disarm_zorla_ack" in ins:                # komut verildi, yine de disarm olmadı
                res["sonuc"] = "indi_disarm_yok"; return 4
            r = arac.command(MAV_CMD_COMPONENT_ARM_DISARM, 0)
            ins["disarm_zorla_ack"] = r
            log(f"UYARI: yer algılandı ama 20 sn'de otomatik disarm olmadı → DISARM komutu (ack {r})")
            t_yer = time.monotonic() - 15.0              # 5 sn daha bekle
        if nav.fresh(2.0) and nav.nav not in (NAV_AUTO_MISSION, NAV_AUTO_LAND) and t_yer is None:
            ins["hata"] = f"görev modu bırakıldı ({nav.name()})"
            log(f"İNİŞ BAŞARISIZ: PX4 görev modundan çıktı (nav {nav.name()}); irtifa {alt:.1f} m")
            if nav.nav != NAV_AUTO_LOITER:
                node.pub_cmd.publish(c.make_vehicle_command(args.uav, node.ts(), VC.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0))
            res["sonuc"] = "inis_basarisiz"; res["neden"] = ins["hata"]
            return 4
        if time.monotonic() - t0 > args.inis_asimi and t_yer is None:
            ins["hata"] = f"iniş zaman aşımı ({args.inis_asimi:.0f} sn)"
            log(f"İNİŞ BAŞARISIZ: {ins['hata']}; irtifa {alt:.1f} m → Hold komutu")
            node.pub_cmd.publish(c.make_vehicle_command(args.uav, node.ts(), VC.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0))
            res["sonuc"] = "inis_basarisiz"; res["neden"] = ins["hata"]
            return 4
        if time.monotonic() >= nxt:
            nxt += 5.0
            log(f"iniş: nav={nav.name()}  ({pp.x:6.0f}, {pp.y:6.0f})  irtifa={alt:5.1f} m  yer_hızı={math.hypot(pp.vx, pp.vy):4.1f}  "
                f"vz={-pp.vz:+.1f}  Td'ye {math.dist((pp.x, pp.y), Td):5.0f} m")


def duz_plan(engeller, pist_yonu_deg, kalkis_mesafe, kalkis_irtifa, guvenli_irtifa, hiz=15.0, tirmanma_hizi=3.0,
             merkez=(0.0, 0.0), desen=None, iz_payi=25.0, desen_irtifa=60.0):
    """Hedefsiz deneme (Aşama 1): pist ekseninde kalkış → aynı doğrultuda güvenli irtifaya tırmanış (C) → dönüş
    rotası → iniş deseni. Tüm bacaklar denetlenir. Dönüş: plan sözlüğü ("uygun", "C", "donus", "yer")."""
    ly = _ly()
    dp, _ = _birim(pist_yonu_deg)
    desen = desen or inis_deseni(pist_yonu_deg)
    cikis = kalkis_mesafe + hiz * (max(guvenli_irtifa - kalkis_irtifa, 0.0) / max(tirmanma_hizi, 0.1) + 3.0)
    C = (cikis * dp[0], cikis * dp[1])
    plan = {"uygun": False, "giris": "yer", "yon_deg": pist_yonu_deg % 360.0, "C": [round(C[0], 1), round(C[1], 1)],
            "yer": {"pist_yonu_deg": pist_yonu_deg, "kalkis_mesafe_m": kalkis_mesafe, "kalkis_irtifa_m": kalkis_irtifa,
                    "inis": {k: ([round(x, 1) for x in v] if isinstance(v, tuple) else round(v, 2) if isinstance(v, float) else v)
                             for k, v in desen.items()}}}
    pts = ly._parca((0.0, 0.0), C, 0.0, 0.0, "pist/kalkış/tırmanış") + ly._parca(desen["G"], (0.0, 0.0), 0.0, 0.0, "iniş hattı")
    ihl, acik = ly.yol_ihlalleri(pts, _paylı(engeller, iz_payi))
    if ihl:
        plan["neden"] = f"{ihl[0][0]} ↔ {ihl[0][1]} ({ihl[0][2]} m)"
        return plan
    don = donus_rotasi(C, pist_yonu_deg, desen["Lc"], engeller, guvenli_irtifa, merkez, iz_payi,
                       lc_alt=desen["giris_irtifa_m"], son_bacak_alt=min(desen_irtifa, guvenli_irtifa))
    if don is None:
        plan["neden"] = "dönüş rotası yok (C → iniş orbiti)"
        return plan
    plan.setdefault("yer", {})["desen_irtifa_m"] = desen_irtifa
    plan.update({"uygun": True, "donus": don,
                 "en_kucuk_aciklik_m": {k_: round(v, 1) for k_, v in sorted(acik.items(), key=lambda kv: kv[1])[:3]}})
    return plan


def offboard_devral(args, c, hat_fn, log, asim=20.0):
    """Kalkış irtifası geçildikten sonra OFFBOARD devralma: hat_fn(pp) → (konum, hız) setpoint'i akıtılır, saniyede
    bir DO_SET_MODE(OFFBOARD). Dönüş: None (tamam) ya da neden."""
    node = c.node
    t0 = time.monotonic(); son = 0.0
    while not node.offboard():
        c.rclpy.spin_once(node, timeout_sec=1.0 / c.LOOP_HZ)
        nk = c.abort_nedeni_kritik()
        if nk:
            return f"abort (devralma): {nk}"
        node.stream(*hat_fn(node.pos))
        now = time.monotonic()
        if now - t0 >= 0.5 and now - son >= 1.0:
            son = now
            node.pub_cmd.publish(c.make_vehicle_command(args.uav, node.ts(), c.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0))
        if now - t0 > asim:
            return f"OFFBOARD'a geçilemedi ({asim:.0f} sn)"
    return None


def yalniz_kalkis_inis(args, c, arac, engeller, log):
    """Aşama 1 akışı: plan → görev yükle → kalkış → OFFBOARD, pist doğrultusunda güvenli irtifaya tırmanış →
    dönüş → iniş → disarm. Hedef / dalış yok. Dönüş: çıkış kodu."""
    node, res = c.node, c.res
    desen = inis_deseni(args.pist_yonu, args.td_uzaklik, args.inis_yaklasma, args.inis_giris_irtifa, args.loiter_yaricap)
    plan = duz_plan(engeller, args.pist_yonu, args.kalkis_mesafe, args.kalkis_irtifa, args.guvenli_irtifa, args.hiz,
                    args.tirmanma_hizi, (-args.kaynak_fark_n, -args.kaynak_fark_e), desen,
                    desen_irtifa=args.inis_desen_irtifa)
    res["plan"] = plan
    log(f"hedefsiz plan: {plan}")
    if not plan["uygun"]:
        log(f"NO-GO: {plan.get('neden')} — uav1 KALKMADI (yerde, disarmed)")
        res["sonuc"] = "no-go"; res["neden"] = plan.get("neden"); return 3
    ok, acik = gorev_yukle(arac, inis_gorevi(desen, c.ne_to_latlon, node.pos.ref_lat, node.pos.ref_lon))
    log(f"iniş görevi yükleme: {'✔' if ok else '✘'} ({acik})")
    if not ok:
        res["sonuc"] = "no-go"; res["neden"] = f"iniş görevi yüklenemedi ({acik})"; return 3
    ok, _olc = kalkis(args, c, arac, log)
    if not ok:
        res["sonuc"] = "kalkis_basarisiz"; return 4
    dp, _ = _birim(args.pist_yonu)

    def hat(pp):
        s = pp.x * dp[0] + pp.y * dp[1]
        return ((s + 200.0) * dp[0], (s + 200.0) * dp[1], -args.guvenli_irtifa), (args.hiz * dp[0], args.hiz * dp[1], 0.0)
    hata = offboard_devral(args, c, hat, log)
    if hata:
        log(f"KALKIŞ SONRASI: {hata} — araç PX4 Takeoff/Hold modunda bırakıldı")
        res["sonuc"] = "abort"; res["neden"] = hata; return 2
    pp = node.pos
    res["devralma"] = {"irtifa_m": round(-pp.z, 1), "ustten_m": round(math.hypot(pp.x, pp.y), 1),
                       "yer_hizi_m_s": round(math.hypot(pp.vx, pp.vy), 1)}
    log(f"OFFBOARD ✔: irtifa {-pp.z:.1f} m, üsten {math.hypot(pp.x, pp.y):.0f} m; pist doğrultusunda {args.guvenli_irtifa:.0f} m'ye tırmanış")
    t0 = time.monotonic(); nxt = t0
    while -node.pos.z < args.guvenli_irtifa - args.tirmanma_tol:
        c.rclpy.spin_once(node, timeout_sec=1.0 / c.LOOP_HZ)
        if not node.offboard():
            res["sonuc"] = "abort"; res["neden"] = "OFFBOARD kaybı (tırmanış)"; return 2
        node.stream(*hat(node.pos))
        if time.monotonic() - t0 > args.tirmanma_asimi:
            break
        if time.monotonic() >= nxt:
            nxt += 2.0
            pp = node.pos
            log(f"tırmanış: irtifa {-pp.z:5.1f} m  üsten {math.hypot(pp.x, pp.y):5.0f} m  yanal {(-pp.x * dp[1] + pp.y * dp[0]):+5.1f} m  "
                f"yer_hızı {math.hypot(pp.vx, pp.vy):4.1f}")
    return donus_inis(args, c, arac, plan, tuple(plan["C"]), args.pist_yonu, log)
