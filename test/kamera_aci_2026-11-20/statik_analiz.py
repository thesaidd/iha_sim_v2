#!/usr/bin/env python3
"""statik/sonuc.json → ölçülen tespit bandı ↔ kamera_geometri teorisi (kenar_px 5 ve 15, min kenar 20 px). Markdown tablo."""
import json, math, os, sys
sys.path.insert(0, os.path.expanduser("~/sim/iha_sim_v2"))
import kamera_geometri as kg
D = os.path.dirname(os.path.abspath(__file__))
s = json.load(open(os.path.join(D, "statik", "sonuc.json")))
kam = kg.Kamera.json_oku(os.path.expanduser("~/sim/iha_sim_v2/kalibrasyon/w_nadir_10_sim.json"))
out = ["| deney | ölçülen bant [m] | W ölçülen | teori kenar 5 px | teori kenar 15 px | sınırda kenar px (ölç.) |", "|---|---|---|---|---|---|"]
def bant(rs, merkez):
    rs = sorted(rs, key=lambda r: r["dl_m"]); ok = [r["oran"] == 1.0 for r in rs]
    i = min(range(len(rs)), key=lambda k: abs(rs[k]["dl_m"] - merkez))
    lo = hi = i
    while lo > 0 and ok[lo - 1]: lo -= 1
    while hi < len(rs) - 1 and ok[hi + 1]: hi += 1
    # herhangi bir tespit (kararsız kenar) dahil en dış
    dis = [r["dl_m"] for r in rs if (r["oran"] or 0) > 0]
    return rs[lo]["dl_m"], rs[hi]["dl_m"], rs[lo], rs[hi], (min(dis), max(dis))
for egim in (0, 10, 20, 30):
    rs = [r for r in s["olcum"] if r["deney"] == "A" and r["egim_deg"] == egim]
    lo, hi, rlo, rhi, dis = bant(rs, 100 * math.tan(math.radians(egim)))
    t5 = kg.tespit_seridi(kam, 100, yan=egim, roll_pay=0, pitch_pay=0, yawlar=(0.0,), kenar_px=5)
    t15 = kg.tespit_seridi(kam, 100, yan=egim, roll_pay=0, pitch_pay=0, yawlar=(0.0,), kenar_px=15)
    out.append(f"| A yan {egim}° | {lo:+.1f} … {hi:+.1f} (ara tespit {dis[0]:+.1f} … {dis[1]:+.1f}) | {hi-lo:.1f} | "
               f"{t5['alt']:+.1f} … {t5['ust']:+.1f} ({t5['W']:.1f}) | {t15['alt']:+.1f} … {t15['ust']:+.1f} ({t15['W']:.1f}) | "
               f"{rlo['kenar_px']} / {rhi['kenar_px']} |")
rs = [r for r in s["olcum"] if r["deney"] == "B"]
det = [r["dl_m"] for r in rs if r["oran"] == 1.0]
t5 = kg.tespit_seridi(kam, 100, roll_pay=0, pitch_pay=0, yawlar=(45.0,), kenar_px=5)
t15 = kg.tespit_seridi(kam, 100, roll_pay=0, pitch_pay=0, yawlar=(45.0,), kenar_px=15)
neg = [d for d in det if d < 0]; pos = [d for d in det if d > 0]
out.append(f"| B ψ 45° (yalnız kenarlar) | {min(neg):+.1f} … {max(pos):+.1f} | {max(pos)-min(neg):.1f} | {t5['alt']:+.1f} … {t5['ust']:+.1f} ({t5['W']:.1f}) | {t15['alt']:+.1f} … {t15['ust']:+.1f} ({t15['W']:.1f}) | — |")
out.append("\n| irtifa [m] | beklenen kenar px | ölçülen kenar px | tespit |\n|---|---|---|---|")
for r in s["olcum"]:
    if r["deney"] == "C":
        out.append(f"| {r['h']:.0f} | {r['beklenen_kenar_px']} | {r['kenar_px']} | {r['tespit']}/{r['kare']} |")
# kenar payı: tam tespitli pozlardaki en küçük marj ↔ tespitsizlerin en büyük marjı (yalnız kenar yakını, kenar px ≥ 22)
yak = [r for r in s["olcum"] if r["deney"] in "AB" and r["beklenen_marj_px"] < 60 and r["beklenen_kenar_px"] >= 22]
ok_m = [r["beklenen_marj_px"] for r in yak if r["oran"] == 1.0]
no_m = [r["beklenen_marj_px"] for r in yak if not r["oran"]]
out.append(f"\nKenar payı (kenar px ≥ 22 olan kenar yakını pozlar): tam tespitli en küçük marj {min(ok_m):.1f} px; "
           f"hiç tespit olmayan en büyük marj {max(no_m):.1f} px; kararsız (0 < oran < 1): "
           f"{sorted(r['beklenen_marj_px'] for r in yak if r['oran'] and r['oran'] < 1)}")
k = [r for r in s["olcum"] if r["kose_rms_px"] is not None]
import statistics as st
out.append(f"Köşe RMS (tespitli {len(k)} poz): medyan {st.median(r['kose_rms_px'] for r in k):.2f} px; ölçülen/beklenen kenar oranı medyan "
           f"{st.median(r['kenar_px']/r['beklenen_kenar_px'] for r in k):.3f}")
print("\n".join(out)); open(os.path.join(D, "statik", "ozet.md"), "w").write("\n".join(out) + "\n")
