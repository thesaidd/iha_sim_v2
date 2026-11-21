#!/usr/bin/env python3
"""kamera_geometri_test.py — kamera_geometri.py kuru testi (saf numpy; ROS / Gazebo yok). Kapalı biçimli nadir
formülleri ve bağımsız hesapla (2026-11-20, alt ajan çapraz denetimi: W ±2° = 72,24 m) karşılaştırır.
Kullanım: python3 kamera_geometri_test.py"""
import math

import numpy as np

import kamera_geometri as kg

n = 0


def dogrula(k, ad):
    global n
    n += 1
    print(("✔ " if k else "✘ ") + ad)
    if not k:
        raise SystemExit(1)


kam = kg.Kamera()
R0 = kg.R_body_cam()
o = kg.ozet(kam, 100.0)
dogrula(abs(o["ayak_izi_yanal_m"] - 83.7) < 0.1 and abs(o["ayak_izi_boyuna_m"] - 62.6) < 0.1,
        f"100 m ayak izi {o['ayak_izi_yanal_m']} × {o['ayak_izi_boyuna_m']} m (2·h·tan(FOV/2))")
gu, gv = kg.gsd(kam, np.array([[0.0, 0.0], [728.0, 544.0], [1455.0, 1087.0]]), 100.0, R0)
dogrula(np.allclose(gu, 100 / kam.fx, rtol=1e-3) and np.allclose(gv, 100 / kam.fx, rtol=1e-3),
        "nadir + düz zemin: GSD görüntü boyunca sabit (h/f = 5,75 cm/px; cos² açısal GSD'dir, piksel değil)")
uv = kg.goruntuye(kam, kg.zemine(kam, np.array([[100.0, 200.0], [1400.0, 1000.0]]), 87.0,
                                  kg.R_ned_body(0.05, -0.03, 1.0) @ R0), 87.0, kg.R_ned_body(0.05, -0.03, 1.0) @ R0)[0]
dogrula(np.allclose(uv, [[100.0, 200.0], [1400.0, 1000.0]], atol=1e-6), "zemine ↔ goruntuye ters dönüşüm")
for h in (80.0, 100.0, 120.0):
    for m in (0.0, 1.0, 2.0, 3.0):
        s = kg.tespit_seridi(kam, h, roll_pay=m, pitch_pay=0.0, kenar_px=5.0, adim=0.05)
        kapali = 2 * (h * math.tan(math.atan((kam.w / 2 - 5) / kam.fx) - math.radians(m)) - math.sqrt(2))
        dogrula(abs(s["W"] - kapali) < 0.15, f"tespit şeridi {h:.0f} m, roll ±{m:.0f}°: {s['W']:.2f} m ≈ kapalı biçim {kapali:.2f}")
s = kg.tespit_seridi(kam, 100.0, roll_pay=2.0, pitch_pay=2.0)
dogrula(abs(s["W"] - 72.24) < 0.3, f"bağımsız hesapla uyum: roll ±2 pitch ±2 → {s['W']:.2f} m (ajan 72,24)")
s10 = kg.tespit_seridi(kam, 100.0, yan=10.0, roll_pay=0.0, pitch_pay=0.0, yawlar=(0.0,))
dogrula(s10["ust"] > 60 and s10["alt"] > -25, f"yan eğim +10° şeridi sağa kaydırır ({s10['alt']:+.1f} … {s10['ust']:+.1f})")
dogrula(kg.tespit_seridi(kam, 180.0)["W"] == 0.0, "180 m: marker < 20 px → şerit yok")
fo, nk, L = kg.on_ortusme(kam, 100.0, 14.0, 10.0)
dogrula(abs(fo - 0.9776) < 1e-3 and abs(nk - 44.7) < 0.1, f"10 Hz, 14 m/s: ön örtüşme %{100 * fo:.1f}, nokta {nk:.1f} karede")
d, Wg = kg.hat_araligi(kam, 100.0, 72.0, 0.6, 2.0)
dogrula(abs(d - 0.4 * Wg) < 1e-9 and d < 72.0, f"%60 yan örtüşme → hat aralığı {d:.1f} m (görüntü şeridi {Wg:.1f} m, roll ±2°)")
print(f"\n{n} doğrulama geçti")
