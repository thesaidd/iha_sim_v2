"""tasarim.md'deki sayıları üretir (uçuş yok). Kullanım: ROS ortamı + python3 tasarim_hesap.py"""
import math, os, sys
sys.path.insert(0, os.path.expanduser("~/iha_sim_v2"))
import qr_pass as q
import tarama_plan as tp
BAS = (-30.0, 290.0)
print("## Arena")
for ad, (n, e) in q.PYLONS.items():
    print(f"- direk {ad}: (N {n:+.0f}, E {e:+.1f}); kutu kenarına en yakın {150 - max(abs(n), abs(e)):.1f} m İÇERİDE; merkezden {math.hypot(n, e):.0f} m")
BINA = [("ev", 60, -200, 8), ("depo", -100, -190, 10), ("kule", 210, -80, 24), ("atolye", -230, 60, 9), ("silo", -220, -250, 16), ("hangar", 220, 200, 12)]
for ad, n, e, h in BINA:
    print(f"- bina {ad}: (N {n}, E {e}), {h} m → 60 m'de dikey boşluk {60 - h} m; kutu {'içinde' if max(abs(n), abs(e)) <= 150 else 'dışında'}")
print("\n## k")
for d in (16.0,):
    print(f"d={d}: k=ceil(2·80/d)={math.ceil(160 / d)} (D={math.ceil(160 / d) * d:.0f} m); paylı ceil(170/d)={math.ceil(170 / d)} (D={math.ceil(170 / d) * d:.0f} m); kutuya sığan hat {int(284 // d) + 1}; 2k={2 * math.ceil(170 / d)}")
print("\n## Yön ve direk payı")
print("| yön | direk payı | hat | geometrik kapsama | C | POD Koopman | toplam m | süre sn | stadyum | ampul | en küçük D | hat–direk | dönüş–direk |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
planlar = {}
for psi0 in (0.0, 90.0):
    for hp in (40.0, 25.0, 20.0, 15.0):
        pl = tp.planla(tp.Param(psi0=psi0, hat_payi=hp), BAS)
        if pl is None:
            hl, _ = tp.hat_yerlestir(tp.Param(psi0=psi0, hat_payi=hp))
            print(f"| {psi0:.0f}° | {hp:.0f} | {len(hl)} yerleşti | — | — | — | PLAN YOK (dönüşler koridor/alan dışı) | | | | | | |")
            continue
        planlar[(psi0, hp)] = pl
        o = pl.ozet(); assert pl.dogrula() == []
        print(f"| {psi0:.0f}° | {hp:.0f} | {o['hat_sayisi']} | %{100 * o['geometrik_kapsama']:.1f} | {o['C']:.2f} | %{100 * o['POD_koopman']:.0f} | {o['toplam_m']} | {o['sure_sn']} | {o['stadyum']} | {o['ampul']} | {o['en_kucuk_D_m']} | {o['hat_direk_min_m']} | {o['donus_direk_min_m']} |")
print("\n## Karşılaştırma")
print("| yöntem | direk payı | hat | hat aralığı | geometrik kapsama | C | POD Koopman | hat m | dönüş m | toplam m | süre sn | dönüşler |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|")
for hp in (40.0, 20.0):
    p = tp.Param(hat_payi=hp)
    for ad, pl in (("atlamalı (dar aralık, atlamalı sıra)", planlar[(0.0, hp)]), ("sıralı dar (aynı hatlar, numara sırası)", tp.planla(p, BAS, sirali=True)),
                   ("atlamasız seyrek (aralık 2R)", tp.planla_seyrek(p, BAS))):
        o = pl.ozet()
        ar = sorted(pl.hatlar); a = [y - x for x, y in zip(ar, ar[1:])]
        print(f"| {ad} | {hp:.0f} | {o['hat_sayisi']} | {min(a):.1f}–{max(a):.1f} | %{100 * o['geometrik_kapsama']:.1f} | {o['C']:.2f} | %{100 * o['POD_koopman']:.0f} | {o['hat_uzunlugu_m']} | {o['donus_uzunlugu_m']} | {o['toplam_m']} | {o['sure_sn']} | {o['stadyum']} stadyum + {o['ampul']} ampul |")
for hp in (40.0, 20.0):
    pl = planlar[(0.0, hp)]; p = pl.p
    print(f"\n## Sıra ve geçişler (direk payı {hp:.0f})")
    print("serbest yanal aralıklar:", [(round(a, 1), round(b, 1)) for a, b in pl.serbest], "bloklu:", [(round(a), round(b), n) for a, b, n in tp.bloklu_bantlar(p)])
    gi = pl.giris(); print(f"giriş Hold merkezi ({gi['H'][0]:.0f}, {gi['H'][1]:.0f}), çember–direk {q.pylon_dist(gi['H']) - 80:.0f} m")
    print("| # | hat c (E) | yön | sonraki hata D | ≥ 2R_min (160) | ≥ 170 (paylı) | bağlantı | uzunluk | dönüş–direk | çıkış Hold merkezi |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for k, i in enumerate(pl.sira):
        b = pl.baglar[k] if k < len(pl.baglar) else None
        c = pl.cikis(k)
        print(f"| {k + 1} | {pl.hatlar[i]:+.1f} | {'K' if pl.yon(k) > 0 else 'G'} | " + (f"{b['D']:.1f} | {'evet' if b['D'] >= 160 else 'HAYIR'} | {'evet' if b['D'] >= 170 else 'HAYIR'} | {b['tur']} | {b['uzunluk']:.0f} | {b['direk_m']:.0f}" if b else "— | | | | | ") + f" | ({c['M'][0]:.0f}, {c['M'][1]:.0f}) |")
    kitap = tp.kitap_sirasi(len(pl.hatlar), math.ceil(170 / 16))
    D = [abs(pl.hatlar[a] - pl.hatlar[b]) for a, b in zip(kitap, kitap[1:])]
    print(f"kitap sırası (k={math.ceil(170 / 16)}): {[round(pl.hatlar[i]) for i in kitap]}; geçiş D: {[round(x) for x in D]}; < 170 olan {sum(x < 170 for x in D)}/{len(D)}")
    uz = max(max(abs(n) for n, e in tp.ornekle(b['ilkeller'])) for b in pl.baglar)
    print(f"dönüşlerin en uzak noktası |N| = {uz:.0f} m; QR (−60,−60) en yakın hat: {min(pl.hatlar, key=lambda c: abs(c + 60)):+.1f} (sıra {[pl.hatlar[i] for i in pl.sira].index(min(pl.hatlar, key=lambda c: abs(c + 60))) + 1}/{len(pl.sira)})")
# uav1 kalkış Hold'u ile dikey ayrım
print("\nuav1 Hold (45, 334) @119 m, disk 100 m: en yakın tarama noktası yatay", round(min(math.dist((45, 334), x) for x in tp.ornekle(planlar[(0.0, 20.0)].ilkeller())) - 100), "m; dikey boşluk", 119 - 10 - 60, "m")
