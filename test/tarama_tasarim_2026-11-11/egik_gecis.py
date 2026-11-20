"""Eğik geçiş tespit karakterizasyonu (statik, uçuş yok) — 2026-11-11.

Sabit `kamera_test` (T_nadir kamerası, 1456×1088, dik aşağı, 10 Hz) 60 m'de; QR'a göre YANAL kaydırma × uçuş
YÖNÜ (ψ, pusula) ızgarası. Her (ψ, kaydırma) için kamera hat boyunca s = −9…+9 m, 1,5 m adımla (15 m/s × 10 Hz
= bir geçişteki kare aralığı) taşınır → o kaydırmada bir düz geçişin göreceği kareler.
Görüntü üstü = burun (model yaw = 90° − ψ); görüntü yatayı (1456 px, yarı 9,41 m) = yanal, dikeyi (1088 px,
yarı 7,04 m) = hat boyu.

Kullanım: egik_gecis.py <çıktı klasörü> <etiket>
"""
import json, math, os, subprocess, sys, threading, time
import cv2, numpy as np
sys.path.insert(0, os.path.expanduser("~/iha_sim_v2"))
from qr_online import QrIsleyici      # uçuşta kullanılan boru hattı: Aruco bul → ROI çöz (+ yedek tam kare)
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.world_stats_pb2 import WorldStatistics

MODEL = os.path.expanduser("~/iha_sim_v2/gz_models_cam_hw/kamera_test/model.sdf")
NAME, QR_NE, FX, W, H = "kamera_test", (-60.0, -60.0), 4637.81, 1456, 1088
ALT = 60.0
KOD = 1.0                                   # QR kod kenarı [m]
YONLER = [0.0, 45.0, 90.0, 135.0]
KAYDIRMA = [-15, -12.5, -10, -9, -8.5, -8, -7, -5, -2.5, 0, 2.5, 5, 7, 8, 8.5, 9, 10, 12.5, 15]
S_ADIM = 1.5
S_LISTE = [round(-9 + S_ADIM * i, 2) for i in range(13)]
OUT, TAG = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)


def svc(s, t, req):
    r = subprocess.run(["gz", "service", "-s", s, "--reqtype", t, "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                        "--req", req], capture_output=True, text=True)
    return "data: true" in r.stdout


def poz(psi, off, s):
    """Kamera konumu: QR + s·ileri + off·sağ (NE). → (gz x=E, gz y=N, yaw)."""
    p = math.radians(psi)
    ileri, sag = (math.cos(p), math.sin(p)), (-math.sin(p), math.cos(p))
    n = QR_NE[0] + s * ileri[0] + off * sag[0]
    e = QR_NE[1] + s * ileri[1] + off * sag[1]
    return e, n, math.radians(90.0 - psi)


def beklenen_piksel(off, s):
    """QR merkezi gövdeye göre: ileri −s, sağ −off → piksel (u sağa, v aşağı; üst = burun)."""
    return W / 2 + FX * (-off) / ALT, H / 2 - FX * (-s) / ALT


def geometri(psi, off, s):
    """tam / kismi / disarida (kod karesi 4 köşesinin kadraja düşmesi)."""
    p = math.radians(psi); c, sn = math.cos(p), math.sin(p)
    ic = 0
    for dn, de in ((-.5, -.5), (-.5, .5), (.5, .5), (.5, -.5)):
        dn *= KOD; de *= KOD
        f = dn * c + de * sn - s            # köşe, gövde ileri
        r = -dn * sn + de * c - off         # köşe, gövde sağ
        u, v = W / 2 + FX * r / ALT, H / 2 - FX * f / ALT
        ic += (0 <= u < W) and (0 <= v < H)
    return "tam" if ic == 4 else ("kismi" if ic else "disarida")


kare, sayac, kilit = [None], [0], threading.Lock()


def on_img(m):
    a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3).copy()
    with kilit:
        kare[0] = a; sayac[0] += 1


def taze_kare(n=3, zaman_asimi=10.0):
    with kilit: s0 = sayac[0]
    t = time.time()
    while time.time() - t < zaman_asimi:
        with kilit:
            if sayac[0] >= s0 + n: return kare[0]
        time.sleep(0.02)
    return None


st = []
node = Node()
node.subscribe(WorldStatistics, "/world/default/stats",
               lambda m: st.append((m.sim_time.sec + m.sim_time.nsec * 1e-9, m.real_time.sec + m.real_time.nsec * 1e-9)))
x, y, yaw = poz(0, 0, 0)
print("create:", svc("/world/default/create", "gz.msgs.EntityFactory",
                     f'sdf_filename: "{MODEL}", name: "{NAME}", pose: {{position: {{x: {x}, y: {y}, z: {ALT + 0.05}}}}}'), flush=True)
time.sleep(3)
node.subscribe(Image, f"/world/default/model/{NAME}/link/link/sensor/camera/image", on_img)
time.sleep(2)

qd, ar = cv2.QRCodeDetector(), cv2.QRCodeDetectorAruco()
satirlar = []
t_bas = time.time(); n0 = len(st)
for psi in YONLER:
    for off in KAYDIRMA:
        boru = QrIsleyici()               # her geçiş (ψ, kaydırma) için taze durum; kareler s sırasıyla
        for s in S_LISTE:
            geo = geometri(psi, off, s)
            ub, vb = beklenen_piksel(off, s)
            # tümüyle dışarıda ve kadraj kenarından > 2 m uzaksa yalnız s = 0 örneklenir (yanlış bulma denetimi)
            uzak = abs(off) > W / 2 * ALT / FX + 2.7 or abs(s) > H / 2 * ALT / FX + 2.7
            if uzak and s != 0: continue
            x, y, yaw = poz(psi, off, s)
            ok = svc("/world/default/set_pose", "gz.msgs.Pose",
                     f'name: "{NAME}", position: {{x: {x}, y: {y}, z: {ALT + 0.05}}}, '
                     f'orientation: {{w: {math.cos(yaw / 2)}, x: 0, y: 0, z: {math.sin(yaw / 2)}}}')
            rgb = taze_kare()
            if rgb is None:
                print("KARE YOK", psi, off, s, flush=True); continue
            g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            r = {"etiket": TAG, "psi": psi, "kaydirma": off, "s": s, "geometri": geo, "set_pose": ok,
                 "beklenen_px": [round(ub, 1), round(vb, 1)]}
            for ad, det in (("qr", qd), ("aruco", ar)):
                bul, hata, kenar = False, None, None
                try:
                    okd, pts = det.detect(g)
                    txt, _, _ = det.detectAndDecode(g)
                except cv2.error:
                    okd, pts, txt = False, None, ""
                if okd and pts is not None and len(pts):
                    q = np.asarray(pts).reshape(-1, 4, 2)[0]; c = q.mean(axis=0)
                    hata = float(math.hypot(c[0] - ub, c[1] - vb))
                    kenar = float(np.mean([np.linalg.norm(q[i] - q[(i + 1) % 4]) for i in range(4)]))
                    bul = hata <= 25.0
                r[ad] = {"bulma": bool(bul), "ham_bulma": bool(okd), "merkez_hata_px": None if hata is None else round(hata, 1),
                         "kenar_px": None if kenar is None else round(kenar, 1), "cozme": txt == "HEDEF-01",
                         "yanlis_icerik": bool(txt) and txt != "HEDEF-01"}
            o = boru.isle(g)
            oh = math.hypot(o["u"] - ub, o["v"] - vb) if o["bulundu"] else None
            r["online"] = {"bulma": bool(o["bulundu"] and oh <= 25.0), "ham_bulma": bool(o["bulundu"]),
                           "cozme": bool(o["cozuldu"]), "yontem": o["yontem"], "icerik": o["icerik"],
                           "merkez_hata_px": None if oh is None else round(oh, 1),
                           "ms": round(o["aruco_ms"] + o["roi_ms"] + o["yedek_ms"], 1)}
            satirlar.append(r)
            if s == 0 or (geo == "kismi"):
                if abs(off) in (0, 7, 8, 9, 10) and s in (0.0, 6.0, -6.0):
                    cv2.imwrite(os.path.join(OUT, f"{TAG}_psi{int(psi)}_k{off:+.1f}_s{s:+.1f}.jpg"),
                                cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), (728, 544)), [cv2.IMWRITE_JPEG_QUALITY, 85])
        ozet = [x for x in satirlar if x["psi"] == psi and x["kaydirma"] == off]
        print(f"ψ {psi:5.0f}  kaydırma {off:+6.1f}  kare {len(ozet):2d}  tam {sum(x['geometri'] == 'tam' for x in ozet):2d}  "
              f"kısmi {sum(x['geometri'] == 'kismi' for x in ozet):2d}  QR bul {sum(x['qr']['bulma'] for x in ozet):2d} "
              f"çöz {sum(x['qr']['cozme'] for x in ozet):2d}  Aruco bul {sum(x['aruco']['bulma'] for x in ozet):2d} "
              f"çöz {sum(x['aruco']['cozme'] for x in ozet):2d}  online bul {sum(x['online']['bulma'] for x in ozet):2d} "
              f"çöz {sum(x['online']['cozme'] for x in ozet):2d}", flush=True)
rtf = (st[-1][0] - st[n0][0]) / (st[-1][1] - st[n0][1]) if len(st) > n0 + 1 else float("nan")
print(f"süre {time.time() - t_bas:.0f} sn, etkin RTF {rtf:.3f}", flush=True)
print("remove:", svc("/world/default/remove", "gz.msgs.Entity", f'name: "{NAME}", type: MODEL'), flush=True)
with open(os.path.join(OUT, f"egik_{TAG}.json"), "w") as f:
    json.dump({"etiket": TAG, "irtifa": ALT, "rtf": round(rtf, 3), "s_liste": S_LISTE, "satirlar": satirlar}, f, ensure_ascii=False)
