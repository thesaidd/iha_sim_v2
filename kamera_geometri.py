#!/usr/bin/env python3
"""
kamera_geometri.py — aşağı bakan kameranın zemin geometrisi: ayak izi, GSD, marker piksel boyu, TESPİT ŞERİDİ
(tarama hat aralığının dayanağı), görüntü birleştirme (stitching) için örtüşme ve montaj açısı karşılaştırması.

Saf numpy (cv2 / ROS yok → alan_tarama.py sistem python'unda da içe aktarır). Eksen kuralları bapp ile aynı
(bap_geo/frames.py): dünya NED, gövde FRD, kamera OpenCV (x sağ, y aşağı, z ileri); nadir montajda kamera x =
gövde +y, kamera y = gövde −x (görüntü üstü = burun), kamera z = gövde +z. Montaj: R_body_cam = Rz(yaw_m)·Ry(ileri)·
Rx(yan)·R_NADIR (bap_geo.frames.R_body_cam_mount ile aynı sıra). Zemin düz, distorsiyon yok sayılır (simde yok;
gerçek 6 mm lenste kenar distorsiyonu şerit kenarını birkaç piksel oynatır — kenar_px payı bunu da örter).

Tespit ölçütü (bir karede): marker'ın 4 köşesi görüntü kenarlarından ≥ kenar_px içeride VE en kısa kenarı
≥ min_kenar_px. Tespit şeridi: hat boyunca (along-track) herhangi bir konumda tespit edilebilir olan yanal
(cross-track) bant; en kötü durum = marker yaw'ı (0–45°) × gövde roll'u (±roll_pay) × pitch'i (±pitch_pay)
üzerinden kesişim. Hat aralığı ≤ W olursa kapsama boşluksuz.

Kullanım:
  python3 kamera_geometri.py [--json kalibrasyon/w_nadir_10_sim.json] [--alt 100] [--roll-pay 2]
  python3 kamera_geometri.py --aci-tarama           # yan / ileri montaj eğimi karşılaştırması (tablo)
"""
import argparse
import json
import math
from dataclasses import dataclass

import numpy as np

R_NADIR = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def R_ned_body(roll, pitch, yaw):
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def R_body_cam(ileri=0.0, yan=0.0, yaw_m=0.0):
    """Montaj [rad]: ileri = optik eksen buruna doğru, yan = gövde x etrafında (+ → sağ kanat tarafına bakar)."""
    return rot_z(yaw_m) @ rot_y(ileri) @ rot_x(-yan) @ R_NADIR


@dataclass
class Kamera:
    fx: float = 1739.13            # 6 mm / 3,45 µm (RPi GS IMX296 + 6 mm CS)
    fy: float = 1739.13
    cx: float = 727.5
    cy: float = 543.5
    w: int = 1456
    h: int = 1088

    @classmethod
    def json_oku(cls, yol):
        d = json.load(open(yol))
        K = d["K"]
        return cls(K[0][0], K[1][1], K[0][2], K[1][2], int(d["width"]), int(d["height"]))

    @property
    def hfov(self):
        return 2 * math.atan(self.w / 2 / self.fx)

    @property
    def vfov(self):
        return 2 * math.atan(self.h / 2 / self.fy)

    def isin(self, uv):
        """Piksel(ler) → kamera çerçevesinde ışın (z = 1)."""
        uv = np.asarray(uv, float)
        return np.stack([(uv[..., 0] - self.cx) / self.fx, (uv[..., 1] - self.cy) / self.fy, np.ones(uv.shape[:-1])], -1)

    def izdus(self, p_cam):
        z = p_cam[..., 2]
        return np.stack([self.fx * p_cam[..., 0] / z + self.cx, self.fy * p_cam[..., 1] / z + self.cy], -1), z


def zemine(kam, uv, irtifa, R_nc):
    """Piksel → zemin (N, E), uçak orijinde, zemin irtifa m aşağıda. Ufuk üstü ışınlar NaN."""
    d = kam.isin(uv) @ R_nc.T
    t = np.where(d[..., 2] > 1e-9, irtifa / np.maximum(d[..., 2], 1e-9), np.nan)
    return d[..., :2] * t[..., None]


def goruntuye(kam, ne, irtifa, R_nc):
    """Zemin (N, E) noktaları → (uv, derinlik)."""
    ne = np.asarray(ne, float)
    p = np.concatenate([ne, np.full(ne.shape[:-1] + (1,), irtifa)], -1)
    return kam.izdus(p @ R_nc)


def ayak_izi(kam, irtifa, R_nc, n=8):
    """Görüntü sınırının zemin izdüşümü: (4n, 2) N, E poligon."""
    u = np.linspace(0, kam.w, n, endpoint=False)
    v = np.linspace(0, kam.h, n, endpoint=False)
    kenar = np.concatenate([np.stack([u, np.zeros(n)], -1), np.stack([np.full(n, kam.w), v], -1),
                            np.stack([kam.w - u, np.full(n, kam.h)], -1), np.stack([np.zeros(n), kam.h - v], -1)])
    return zemine(kam, kenar, irtifa, R_nc)


def gsd(kam, uv, irtifa, R_nc):
    """Pikseldeki yer örnekleme mesafesi [m/px]: (u yönünde, v yönünde) zemin uzunluğu."""
    uv = np.asarray(uv, float)
    p0 = zemine(kam, uv, irtifa, R_nc)
    pu = zemine(kam, uv + [1.0, 0.0], irtifa, R_nc)
    pv = zemine(kam, uv + [0.0, 1.0], irtifa, R_nc)
    return np.linalg.norm(pu - p0, axis=-1), np.linalg.norm(pv - p0, axis=-1)


def _marker_koseleri(merkez_ne, boy, yaw):
    """(…, 2) merkezler → (…, 4, 2) köşeler."""
    a = boy / 2
    k = np.array([[-a, -a], [-a, a], [a, a], [a, -a]])
    c, s = math.cos(yaw), math.sin(yaw)
    k = k @ np.array([[c, s], [-s, c]])
    return np.asarray(merkez_ne)[..., None, :] + k


def marker_durumu(kam, merkez_ne, irtifa, R_nc, boy=2.0, yaw=0.0, kenar_px=5.0):
    """Zemin merkezlerindeki marker için (tamamen içeride mi, en kısa kenar px)."""
    uv, z = goruntuye(kam, _marker_koseleri(merkez_ne, boy, yaw), irtifa, R_nc)
    ic = (z > 0).all(-1) & (uv[..., 0] >= kenar_px).all(-1) & (uv[..., 0] <= kam.w - kenar_px).all(-1) \
        & (uv[..., 1] >= kenar_px).all(-1) & (uv[..., 1] <= kam.h - kenar_px).all(-1)
    kenar = np.linalg.norm(uv - np.roll(uv, -1, axis=-2), axis=-1).min(-1)
    return ic, kenar


def _en_uzun_aralik(x, ok, merkez=None):
    """ok dizisindeki en uzun ardışık True aralığı (merkez verilirse onu içeren) → (alt, üst) ya da None."""
    best, i, n = None, 0, len(ok)
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and ok[j + 1]:
            j += 1
        if merkez is None or x[i] <= merkez <= x[j]:
            if best is None or x[j] - x[i] > best[1] - best[0]:
                best = (float(x[i]), float(x[j]))
        i = j + 1
    return best


def tespit_seridi(kam, irtifa, ileri=0.0, yan=0.0, roll_pay=2.0, pitch_pay=2.0, boy=2.0, kenar_px=5.0,
                  min_kenar_px=20.0, adim=0.25, yawlar=(0.0, 15.0, 30.0, 45.0), merkezli=None):
    """Tespit şeridi (uçak kuzeye uçar, yanal = doğu): en kötü durumda tespit edilebilen yanal bant.
    Açılar derece. Dönüş: {"alt", "ust", "W", "min_kenar_px"} (bant yoksa W = 0).
    merkezli: None → nadir montajda bant 0'ı içermeli; eğik montajda en uzun bant."""
    Rbc = R_body_cam(math.radians(ileri), math.radians(yan))
    gen = irtifa * math.tan(kam.hfov / 2 + math.radians(abs(yan) + roll_pay)) + boy
    boy_ = irtifa * math.tan(kam.vfov / 2 + math.radians(abs(ileri) + pitch_pay)) + boy
    c = np.arange(-gen, gen + adim, adim)
    a = np.arange(-boy_, boy_ + 0.5, 0.5)
    if abs(ileri) > 1e-9:                     # ileri eğimde ayak izi öne kayar: hat boyunu ona göre aç
        a = a + irtifa * math.tan(math.radians(ileri))
    A, C = np.meshgrid(a, c, indexing="ij")    # (along, cross)
    ne = np.stack([A, C], -1)
    ok = np.ones(len(c), bool)
    kmin = np.full(len(c), np.inf)
    for r in (-roll_pay, 0.0, roll_pay) if roll_pay else (0.0,):
        for pt in (-pitch_pay, 0.0, pitch_pay) if pitch_pay else (0.0,):
            R_nc = R_ned_body(math.radians(r), math.radians(pt), 0.0) @ Rbc
            for y in yawlar:
                ic, k = marker_durumu(kam, ne, irtifa, R_nc, boy, math.radians(y), kenar_px)
                gec = ic & (k >= min_kenar_px)
                ok &= gec.any(0)
                kmin = np.minimum(kmin, np.where(gec, k, -np.inf).max(0))   # en iyi kare, en kötü durum
    if merkezli is None:
        merkezli = abs(yan) < 1e-9 or None
    bant = _en_uzun_aralik(c, ok, 0.0 if merkezli else None)
    if bant is None:
        return {"alt": None, "ust": None, "W": 0.0, "min_kenar_px": None}
    sec = (c >= bant[0]) & (c <= bant[1])
    return {"alt": bant[0], "ust": bant[1], "W": bant[1] - bant[0], "min_kenar_px": float(kmin[sec].min())}


def goruntu_seridi(kam, irtifa, roll_pay=2.0, yan=0.0):
    """Ayak izinin roll ±roll_pay altında HER ZAMAN görülen yanal bandı (görüntü birleştirme için) → (alt, üst)."""
    Rbc = R_body_cam(0.0, math.radians(yan))
    alt, ust = -math.inf, math.inf
    for r in (-roll_pay, 0.0, roll_pay):
        R_nc = R_ned_body(math.radians(r), 0.0, 0.0) @ Rbc
        p = zemine(kam, np.array([[0.0, kam.cy], [kam.w, kam.cy]]), irtifa, R_nc)
        alt, ust = max(alt, min(p[:, 1])), min(ust, max(p[:, 1]))
    return alt, ust


def hat_araligi(kam, irtifa, W, yan_ortusme=0.0, roll_pay=2.0):
    """Hat aralığı: tespit şeridi W'yi aşmaz; yan_ortusme > 0 ise komşu hatların görüntüleri en az bu oranda örtüşür
    (görüntü şeridi roll ±roll_pay altında). Dönüş (aralık, görüntü şeridi genişliği)."""
    a, u = goruntu_seridi(kam, irtifa, roll_pay)
    Wg = u - a
    return (min(W, Wg * (1.0 - yan_ortusme)) if yan_ortusme > 0 else W), Wg


def on_ortusme(kam, irtifa, hiz, kare_hz):
    """Ardışık iki karenin hat boyunca örtüşmesi ve bir zemin noktasını gören kare sayısı."""
    L = 2 * irtifa * math.tan(kam.vfov / 2)
    d = hiz / kare_hz
    return 1.0 - d / L, L / d, L


def konum_duyarliligi(kam, irtifa, uv, R_nc, d_deg=1.0):
    """Pikseldeki zemin noktasının roll / pitch / yaw'da d_deg hatayla yatay kayması [m] (en büyüğü)."""
    p0 = zemine(kam, np.asarray([uv], float), irtifa, R_nc)[0]
    out = []
    for ax in range(3):
        e = [0.0, 0.0, 0.0]
        e[ax] = math.radians(d_deg)
        R2 = R_ned_body(*e) @ R_nc
        p1 = zemine(kam, np.asarray([uv], float), irtifa, R2)[0]
        out.append(float(np.linalg.norm(p1 - p0)))
    return max(out)


def aci_tarama(kam, irtifa, roll_pay=2.0, yanlar=(0, 5, 10, 15, 20, 25, 30, 35), ileriler=(0, 10, 20, 30), **kw):
    """Montaj eğimi karşılaştırması → satır listesi."""
    out = []
    for tur, aciler in (("yan", yanlar), ("ileri", ileriler)):
        for t in aciler:
            yan, il = (t, 0.0) if tur == "yan" else (0.0, t)
            s = tespit_seridi(kam, irtifa, ileri=il, yan=yan, roll_pay=roll_pay, **kw)
            R_nc = R_body_cam(math.radians(il), math.radians(yan))
            satir = {"tur": tur, "egim_deg": t, **{k: (None if v is None else round(v, 2)) for k, v in s.items()}}
            if s["W"] > 0:
                uzak = s["ust"] if abs(s["ust"]) >= abs(s["alt"]) else s["alt"]
                a0 = irtifa * math.tan(math.radians(il))
                (uv,), _ = goruntuye(kam, np.array([[a0, uzak]]), irtifa, R_nc)
                gu, gv = gsd(kam, uv, irtifa, R_nc)
                satir.update(uzak_kenar_aci_deg=round(math.degrees(math.atan(math.hypot(uzak, a0) / irtifa)), 1),
                             uzak_gsd_cm=round(100 * max(float(gu), float(gv)), 2),
                             gsd_orani=round(max(float(gu), float(gv)) / (irtifa / kam.fx), 2),
                             duyarlilik_1deg_m=round(konum_duyarliligi(kam, irtifa, uv, R_nc), 2))
            out.append(satir)
    return out


def ozet(kam, irtifa, hiz=14.0, kare_hz=10.0, roll_pay=2.0, **kw):
    R_nc = R_body_cam()
    p = ayak_izi(kam, irtifa, R_nc, 2)
    s = tespit_seridi(kam, irtifa, roll_pay=roll_pay, **kw)
    gl, gu_ = goruntu_seridi(kam, irtifa, roll_pay)
    fo, nk, L = on_ortusme(kam, irtifa, hiz, kare_hz)
    ku, kv = gsd(kam, np.array([0.0, kam.cy]), irtifa, R_nc)
    _, k0 = marker_durumu(kam, np.array([[0.0, 0.0]]), irtifa, R_nc, kw.get("boy", 2.0))
    return {"irtifa_m": irtifa, "hfov_deg": round(math.degrees(kam.hfov), 2), "vfov_deg": round(math.degrees(kam.vfov), 2),
            "ayak_izi_yanal_m": round(float(np.ptp(p[:, 1])), 1), "ayak_izi_boyuna_m": round(float(np.ptp(p[:, 0])), 1),
            "gsd_nadir_cm": round(100 * irtifa / kam.fx, 2),
            "gsd_yanal_kenar_cm": [round(100 * float(ku), 2), round(100 * float(kv), 2)],
            "marker_nadir_px": round(float(k0[0]), 1),
            "tespit_seridi": {k: (None if v is None else round(v, 2)) for k, v in s.items()},
            "goruntu_seridi_m": round(gu_ - gl, 1), "on_ortusme": round(fo, 4), "noktayi_goren_kare": round(nk, 1),
            "tetik_80_on_ortusme_s": round(0.2 * L / hiz, 2),
            "hat_araligi_yan_ortusme": {f"%{int(100 * o)}": round(hat_araligi(kam, irtifa, s["W"], o, roll_pay)[0], 1)
                                        for o in (0.3, 0.6, 0.7)}}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default=None, help="kamera kalibrasyonu (K, width, height); yoksa 6 mm nominal")
    ap.add_argument("--alt", type=float, nargs="+", default=[100.0])
    ap.add_argument("--hiz", type=float, default=14.0)
    ap.add_argument("--kare-hz", type=float, default=10.0)
    ap.add_argument("--roll-pay", type=float, default=2.0)
    ap.add_argument("--pitch-pay", type=float, default=2.0)
    ap.add_argument("--kenar-px", type=float, default=5.0)
    ap.add_argument("--min-kenar-px", type=float, default=20.0)
    ap.add_argument("--aci-tarama", action="store_true")
    a = ap.parse_args()
    kam = Kamera.json_oku(a.json) if a.json else Kamera()
    kw = dict(pitch_pay=a.pitch_pay, kenar_px=a.kenar_px, min_kenar_px=a.min_kenar_px)
    for h in a.alt:
        if a.aci_tarama:
            print(f"── montaj eğimi, {h:g} m, roll payı ±{a.roll_pay:g}° ──")
            for s in aci_tarama(kam, h, a.roll_pay, **kw):
                print(json.dumps(s, ensure_ascii=False))
        else:
            print(json.dumps(ozet(kam, h, a.hiz, a.kare_hz, a.roll_pay, **kw), ensure_ascii=False, indent=1))


if __name__ == "__main__":
    main()
