#!/usr/bin/env python3
"""
lora_yaklasma_test.py — lora_yaklasma.py'nin KARAR MANTIĞINI (yalnızca hesaplama: çerçeve dönüşümü, 3B mesafe,
durma noktası, irtifa hedefi, süre) kayıtlı /hedef_konum_role verisiyle test eder. Uçuş/simülasyon YOK.

Girdi: kayıtlı hedef konumları (qr_online yayınları = /hedef_konum_role içeriği), varsayılan
test/qr_lora_2026-10-31/online/degerlendirme.csv (yayin=True: x_kuzey, y_dogu). Hedef dünya konumu bilinir
(QR −60,−60); kayıtlı konumlar bunun etrafında.

Çıktı (test/lora_yaklasma_<tarih>/): `kuru_sonuc.json` (özet + başlangıç başına plan) ve `per_mesaj.csv`
(kayıtlı her mesaj için uav1 NED hedefi ve referans başlangıçtan 3B mesafe) — önceki deneylerle aynı
CSV/JSON mantığı (başlık + satır).

Kullanım:
  python3 lora_yaklasma_test.py [--kayit test/qr_lora_2026-10-31/online/degerlendirme.csv]
      [--durma-mesafesi 20] [--yaklasma-irtifa 15] [--guvenli-irtifa 100] [--hiz 15]
"""
import argparse
import csv
import datetime
import json
import math
import os

from lora_yaklasma import mesafe_3b, uav1_hedef, yaklasma_plani


def kayit_oku(yol):
    """degerlendirme.csv yayin=True satırlarından (sim_s, x_kuzey, y_dogu)."""
    out = []
    for r in csv.DictReader(open(yol)):
        if r.get("yayin") == "True" and r.get("x_kuzey"):
            out.append((float(r["sim_s"]), float(r["x_kuzey"]), float(r["y_dogu"])))
    return out


def polar(tN, tE, mesafe, kerteriz_deg):
    a = math.radians(kerteriz_deg)
    return (tN + mesafe * math.cos(a), tE + mesafe * math.sin(a))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kayit", default="test/qr_lora_2026-10-31/online/degerlendirme.csv")
    ap.add_argument("--kaynak-fark-n", type=float, default=15.0)
    ap.add_argument("--kaynak-fark-e", type=float, default=-15.0)
    ap.add_argument("--durma-mesafesi", type=float, default=20.0)
    ap.add_argument("--yaklasma-irtifa", type=float, default=15.0)
    ap.add_argument("--guvenli-irtifa", type=float, default=100.0)
    ap.add_argument("--hiz", type=float, default=15.0)
    ap.add_argument("--baslangic-irtifa", type=float, default=120.0, help="varsayımsal uav1 Hold irtifası [m]")
    ap.add_argument("--cikti", default=None, help="çıktı klasörü (varsayılan test/lora_yaklasma_<bugün>/)")
    args = ap.parse_args()

    kayit = os.path.abspath(args.kayit)
    msgs = kayit_oku(kayit)
    if not msgs:
        print(f"HATA: {kayit} içinde yayın satırı yok"); return 1
    # kayıtlı hedeflerin uav0 NED ortalaması (gerçek hedef ~ −60,−60)
    mN = sum(m[1] for m in msgs) / len(msgs)
    mE = sum(m[2] for m in msgs) / len(msgs)
    t1N, t1E = uav1_hedef(mN, mE, args.kaynak_fark_n, args.kaynak_fark_e)

    test_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test")
    cikti = args.cikti or os.path.join(test_dir, f"lora_yaklasma_{datetime.date.today().isoformat()}")
    os.makedirs(cikti, exist_ok=True)

    # Başlangıçlar (uav1 NED): S0 = uav1'in GERÇEK spawn'ı (0,0) — hedefe ~88 m, kaçınılmaz dik iniş;
    # S1-S3 varsayımsal (hedef etrafında farklı mesafe/kerteriz).
    baslangiclar = {
        "S0_uav1_spawn_~88m": (0.0, 0.0, args.baslangic_irtifa),
        "S1_varsayim_KD_150m": (*polar(t1N, t1E, 150.0, 45.0), args.baslangic_irtifa),
        "S2_varsayim_GB_300m": (*polar(t1N, t1E, 300.0, 225.0), args.baslangic_irtifa),
        "S3_varsayim_D_500m": (*polar(t1N, t1E, 500.0, 90.0), args.baslangic_irtifa),
    }

    print(f"kayıt: {kayit}  ({len(msgs)} yayın)")
    print(f"kayıtlı hedef ort (uav0 NED): ({mN:.2f}, {mE:.2f}) → uav1 NED: ({t1N:.2f}, {t1E:.2f})")
    print(f"params: durma {args.durma_mesafesi} m (3B), yaklaşma irtifa {args.yaklasma_irtifa} m, "
          f"güvenli {args.guvenli_irtifa} m, hız {args.hiz} m/s\n")

    # per-mesaj CSV: referans başlangıç S2'den her kayıtlı hedefe 3B mesafe + uav1 NED hedefi
    refN, refE, refalt = baslangiclar["S2_varsayim_GB_300m"]
    with open(os.path.join(cikti, "per_mesaj.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sim_s", "x_kuzey_uav0", "y_dogu_uav0", "hedef_n_uav1", "hedef_e_uav1", "ref_3b_m"])
        for sim_s, xk, yd in msgs:
            hn, he = uav1_hedef(xk, yd, args.kaynak_fark_n, args.kaynak_fark_e)
            w.writerow([f"{sim_s:.1f}", f"{xk:.3f}", f"{yd:.3f}", f"{hn:.3f}", f"{he:.3f}",
                        f"{mesafe_3b(refN, refE, refalt, hn, he):.2f}"])

    planlar = {}
    for turu in ("3b", "yatay"):
        print(f"--- mesafe-turu = {turu} ---")
        print(f"{'başlangıç':>20} {'yön°':>6} {'yatay':>7} {'bşl3B':>7} {'durmaYty':>8} {'durma3B':>7} "
              f"{'durmaIrt':>8} {'inişAçı°':>8} {'yaklaş_sn':>9} {'tırman_sn':>9} {'toplam_sn':>9}")
        for ad, st in baslangiclar.items():
            p = yaklasma_plani(st, t1N, t1E, args.yaklasma_irtifa, args.durma_mesafesi,
                               args.guvenli_irtifa, args.hiz, mesafe_turu=turu)
            planlar.setdefault(ad, {"baslangic_uav1_NED": {"n": round(st[0], 1), "e": round(st[1], 1), "irtifa": st[2]}})
            planlar[ad][turu] = p
            print(f"{ad:>20} {p['yon_deg']:>6} {p['yatay_mesafe_m']:>7} {p['baslangic_3b_m']:>7} "
                  f"{p['durma_yatay_m']:>8} {p['durma_3b_m']:>7} {p['durma_irtifa_m']:>8} {p['inis_aci_deg']:>8} "
                  f"{p['yaklasma_sn']:>9} {p['tirmanma_sn']:>9} {p['toplam_sn']:>9}")
        print()

    ozet = {"kayit": kayit, "yayin_sayisi": len(msgs),
            "kayitli_hedef_ort_uav0": [round(mN, 3), round(mE, 3)],
            "hedef_uav1_NED": [round(t1N, 3), round(t1E, 3)],
            "params": {"durma_mesafesi_m": args.durma_mesafesi, "yaklasma_irtifa_m": args.yaklasma_irtifa,
                       "guvenli_irtifa_m": args.guvenli_irtifa, "hiz": args.hiz,
                       "baslangic_irtifa_m": args.baslangic_irtifa},
            "planlar": planlar}
    json.dump(ozet, open(os.path.join(cikti, "kuru_sonuc.json"), "w"), indent=1, ensure_ascii=False)
    print(f"\nçıktı: {cikti}/ (kuru_sonuc.json, per_mesaj.csv)")
    # makul mü? pull-up irtifası (= yaklaşma irtifası) güvenlik kontrolü
    if args.yaklasma_irtifa < 10:
        print(f"UYARI: yaklaşma/pull-up irtifası {args.yaklasma_irtifa} m < 10 m — uçuş için düşük sayılabilir")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
