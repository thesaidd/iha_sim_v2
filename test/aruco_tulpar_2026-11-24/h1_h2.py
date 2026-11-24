#!/usr/bin/env python3
"""h1_h2.py — bapp günlüklerinden çevrimdışı H1 (yöntem) ve H2 (geçiş deseni) karşılaştırması.

H1: aynı tespitli kareler (kareler.csv) bapp Geolocator'dan üç yöntemle geçirilir — yalnız ray-cast, yalnız PnP, ray-cast +
PnP füzyonu (bias tabanı yok: yöntem farkı ölçülür, σ değil). Ölçü: son tahminin ve kare başına gözlemin yatay hatası
(gerçek hedef: dünya ENU (−60, −60), bapp scripts/sim_replay.load_log ile aynı dönüşüm).
H2: kareler geçişlere bölünür (ardışık tespitler arası > --bosluk s); her geçişin ortalama yönü (PX4 yaw) yazılır. Aynı
sayıda geçişten iki küme karşılaştırılır: "tek yön" (yönü en yakın geçişler) ve "çok yön" (yönleri en farklı geçişler);
her küme ray-cast füzyonuyla tek tahmine indirilir. Geçiş sayısı az ise yalnız tablo yazılır.
Kullanım: ~/bapp/.venv-ros/bin/python h1_h2.py <aruco_log dizini> ... [--json çıktı.json]
"""
import argparse
import itertools
import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

BAPP = Path.home() / "bapp"
sys.path.insert(0, str(BAPP))
sys.path.insert(0, str(BAPP / "scripts"))
import sim_replay as sr  # noqa: E402
from bap_geo.pipeline import Geolocator  # noqa: E402


def calistir(L, cfg, frames):
    geo = Geolocator(cfg)
    obs_err = {"raycast": [], "pnp": []}
    mid = None
    for fr in frames:
        res = geo.on_pose_detections(fr["t_us"], fr["p"], fr["R"], {fr["mid"]: fr["corners"]})
        mid = fr["mid"]
        for o in res.observations:
            if o.method in obs_err:
                obs_err[o.method].append(float(np.hypot(*(o.p_ned[:2] - L.target[:2]))))
    e = geo.estimate(mid) if mid is not None else None
    son = float(np.hypot(*(e.p_ned[:2] - L.target[:2]))) if e is not None else float("nan")
    return son, obs_err


def ist(x):
    x = np.asarray(x, float)
    return {"medyan": round(float(np.median(x)), 3), "p95": round(float(np.percentile(x, 95)), 3), "n": int(x.size)} if x.size else None


def yaw_deg(R):
    return float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))) % 360


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="+")
    ap.add_argument("--dunya", default=str(sr.SIM_ROOT / "gz_worlds/iha_arena_aruco.sdf"))
    ap.add_argument("--bosluk", type=float, default=5.0, help="geçiş ayırma boşluğu [s]")
    ap.add_argument("--maks-roll", type=float, default=None,
                    help="H2 geçişlerine yalnız |roll| ≤ bu değer [°] kareler (Hold dönüşündeki tespitleri ayıklar)")
    ap.add_argument("--min-kare", type=int, default=3, help="geçiş sayılması için en az tespitli kare")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    out = []
    for d in a.log:
        L = sr.load_log(Path(d), Path(a.dunya))
        b = sr.base_config(L)
        cfgs = {"raycast": replace(b, methods=("raycast",), **sr.NO_BIAS),
                "pnp": replace(b, methods=("pnp",), **sr.NO_BIAS),
                "fuzyon": replace(b, methods=("raycast", "pnp"), **sr.NO_BIAS)}
        r = {"log": L.name, "kare": len(L.frames), "H1": {}}
        for ad, c in cfgs.items():
            son, oe = calistir(L, c, L.frames)
            r["H1"][ad] = {"son_hata_m": round(son, 3)}
            if ad == "fuzyon":
                r["H1"]["kare_basina"] = {k: ist(v) for k, v in oe.items()}
        # geçişler (isteğe bağlı: yalnız düz uçuş kareleri)
        secili = [i for i, fr in enumerate(L.frames)
                  if a.maks_roll is None or abs(np.degrees(np.arctan2(fr["R"][2, 1], fr["R"][2, 2]))) <= a.maks_roll]
        t = np.array([L.frames[i]["t_us"] * 1e-6 for i in secili])
        kes = np.where(np.diff(t) > a.bosluk)[0] + 1
        gec = [[secili[j] for j in g] for g in np.split(np.arange(len(t)), kes)] if len(t) else []
        gec = [g for g in gec if len(g) >= a.min_kare]
        r["H2_kare"] = sum(len(g) for g in gec)
        gecisler = []
        for g in gec:
            fr = [L.frames[i] for i in g]
            son, _ = calistir(L, cfgs["raycast"], fr)
            yaws = np.radians([yaw_deg(f["R"]) for f in fr])
            gecisler.append({"t0": round(float(fr[0]["t_us"] * 1e-6), 1), "kare": len(g), "yon_deg": round(float(np.degrees(
                np.arctan2(np.sin(yaws).mean(), np.cos(yaws).mean())) % 360), 1), "raycast_hata_m": round(son, 3)})
        r["gecisler"] = gecisler
        n = len(gec)
        if n >= 4:
            k = n // 2
            def fark(i, j):
                x = abs(gecisler[i]["yon_deg"] - gecisler[j]["yon_deg"]) % 360
                return min(x, 360 - x)
            kumeler = list(itertools.combinations(range(n), k))
            yay = [np.mean([fark(i, j) for i, j in itertools.combinations(c, 2)]) for c in kumeler]
            tek, cok = kumeler[int(np.argmin(yay))], kumeler[int(np.argmax(yay))]
            for ad, c in (("tek_yon", tek), ("cok_yon", cok)):
                fr = [L.frames[i] for gi in c for i in gec[gi]]
                son, _ = calistir(L, cfgs["raycast"], fr)
                r.setdefault("H2", {})[ad] = {"gecisler": list(c), "yon_yayilimi_deg": round(float(yay[kumeler.index(c)]), 1),
                                              "raycast_fuzyon_hata_m": round(son, 3)}
        # tüm ikili / üçlü geçiş kümeleri: yön yayılımına göre sınıf (aynı yön < 30°, farklı ≥ 90°), sınıf başına hata
        if n >= 2:
            def fark2(i, j):
                x = abs(gecisler[i]["yon_deg"] - gecisler[j]["yon_deg"]) % 360
                return min(x, 360 - x)
            for k in (2, 3):
                if n < k:
                    continue
                sinif = {"ayni_yon": [], "farkli_yon": []}
                for c in itertools.combinations(range(n), k):
                    yay_c = max(fark2(i, j) for i, j in itertools.combinations(c, 2))
                    ad = "ayni_yon" if yay_c < 30 else ("farkli_yon" if yay_c >= 90 else None)
                    if ad is None:
                        continue
                    fr = [L.frames[i] for gi in c for i in gec[gi]]
                    sinif[ad].append(calistir(L, cfgs["raycast"], fr)[0])
                r.setdefault("H2_kumeler", {})[f"k{k}"] = {
                    ad: ({"n_kume": len(v), "medyan_m": round(float(np.median(v)), 3), "ort_m": round(float(np.mean(v)), 3),
                          "maks_m": round(float(np.max(v)), 3)} if v else None) for ad, v in sinif.items()}
        out.append(r)
        print(json.dumps(r, ensure_ascii=False))
    if a.json:
        json.dump(out, open(a.json, "w"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
