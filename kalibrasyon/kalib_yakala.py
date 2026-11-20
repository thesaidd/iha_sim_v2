#!/usr/bin/env python3
"""
kalib_yakala.py — Kalibrasyon dünyasında (kalib.sdf) kamerayı ~48 farklı poza taşır, her pozda TAZE kare alır,
gs_kamera.isle (RGGB mozaik + demozaik; uçuş görüntüleriyle aynı hat) uygular ve PNG kaydeder.

Pozlar: tahtaya uzaklık 2,6–6,5 m, tahta normaline göre bakış eğimi 0–40°, optik eksen etrafında rastgele
dönüş; dört tür sırayla (make_poses): tahta merkezde / tahta merkezi 5×4 ızgarada / tahtanın
dış köşesi görüntü kenar-köşesine 8–40 px yakın (iki kez) — distorsiyon en çok kenar ve köşede belirir.
Her 6. poz ayrılmış test görünümüdür (kareler/test/), kalanlar eğitim (kareler/egitim/).

Kamera taşıma: /world/kalib/set_pose servisi `gz service` CLI ile (abonelik açıkken gz.transport13
Python request() yanıtı teslim alamıyor — viz_trails.py notu). Taze kare: set_pose yanıtından sonra gelen
2. kare alınır ve zaman damgası set_pose anındaki son kareden yeni olmalı (kare yalnız abonelik varken
render edilir; abonelik baştan sona açık).

Kullanım (sunucu ayrı, aynı bölümde):
  export GZ_CONFIG_PATH=/usr/share/gz:$GZ_CONFIG_PATH GZ_PARTITION=kalib GZ_SIM_RESOURCE_PATH=$PWD
  gz sim -s -r kalib.sdf &
  ~/bapp/.venv-ros/bin/python kalib_yakala.py
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))  # gs_kamera
from gs_kamera import isle  # noqa: E402

from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.transport13 import Node  # noqa: E402

WORLD = "kalib"
MODEL = "kamera"
TOPIC = f"/world/{WORLD}/model/{MODEL}/link/link/sensor/camera/image"
W, H = 1456, 1088
F_NOM, CX_NOM, CY_NOM = 1739.13, 728.0, 544.0   # yalnız poz planlaması için
BOARD_Z = 0.005
# İç ChArUco köşeleri (7×5 kare, 0,30 m → 6×4 = 24 köşe): x ∈ {±0,15, ±0,45, ±0,75}, y ∈ {±0,15, ±0,45}
# (tahta merkezli dünya koordinatı; doku yönü önemsiz, tahta merkeze göre simetrik)
INNER = np.array([[-0.75 + 0.3 * i, -0.45 + 0.3 * j, BOARD_Z] for i in range(6) for j in range(4)])


def rot_z_to(r):
    """e_z'yi birim r vektörüne döndüren en kısa dönüş (Rodrigues)."""
    ez = np.array([0.0, 0.0, 1.0])
    v = np.cross(ez, r)
    s, c = np.linalg.norm(v), float(ez @ r)
    if s < 1e-12:
        return np.eye(3)
    k = v / s
    Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + s * Kx + (1 - c) * Kx @ Kx


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def plan_pose(rng, dist, tilt_deg, u_t, v_t, target, az=None, roll=None):
    """Tahta noktası ``target`` (dünya) pikselde (u_t, v_t)'de görünecek poz. ``az``: bakış eğiminin yönü,
    ``roll``: optik eksen etrafında dönüş (None → rastgele).

    Döner: kamera merkezi C (dünya) ve R_world_cv (OpenCV kamera ekseni: x sağ, y aşağı, z ileri).
    """
    az = rng.uniform(0, 2 * np.pi) if az is None else az
    roll = rng.uniform(-np.pi, np.pi) if roll is None else roll
    th = np.radians(tilt_deg)
    C = target + dist * np.array([np.sin(th) * np.cos(az), np.sin(th) * np.sin(az), np.cos(th)])
    d = (target - C) / np.linalg.norm(target - C)
    # R_look: kamera z'si d boyunca; x ekseni yatay seçilir
    zc = d
    xc = np.cross(zc, [0, 0, 1.0]) if abs(zc[2]) < 0.999 else np.array([1.0, 0, 0])
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    R_look = np.column_stack([xc, yc, zc])
    r = np.array([(u_t - CX_NOM) / F_NOM, (v_t - CY_NOM) / F_NOM, 1.0])
    r /= np.linalg.norm(r)
    R = R_look @ rz(roll) @ rot_z_to(r).T   # R^T d = r
    return C, R


def project_nominal(C, R, P):
    pc = (P - C) @ R            # kamera ekseninde (R_world_cv^T (P - C))
    z = pc[:, 2]
    uv = np.column_stack([F_NOM * pc[:, 0] / z + CX_NOM, F_NOM * pc[:, 1] / z + CY_NOM])
    return uv, z


def quat_from_R(R):
    """Dönüş matrisi → (w, x, y, z)."""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = np.empty(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def gz_quat(R_world_cv):
    """gz kamera ekseni: x ileri, y sol, z yukarı. Sensör pozu 0 olduğundan model yönelimi = kamera yönelimi."""
    R_gz = np.column_stack([R_world_cv[:, 2], -R_world_cv[:, 0], -R_world_cv[:, 1]])
    return quat_from_R(R_gz)


def make_poses(n, seed):
    """Dört tür poz sırayla: (0) tahta merkezde, yakın; (1) tahta merkezi 5×4 ızgara noktasında;
    (2, 3) tahtanın bir dış köşesi (±1,05, ±0,75 m) görüntü kenarına/köşesine 8–40 px yakın (12 kenar noktası
    sırayla) — distorsiyonun en büyük olduğu kenar ve köşeleri köşe noktalarıyla doldurmak için.
    Not: iç köşe doğrudan kenara konunca dış sıradaki marker'lar kesiliyor ve o köşe tespit edilemiyor
    (ChArUco köşesi komşu marker'lardan kestirilir); bu yüzden tahtanın DIŞ köşesi kenara dayanır."""
    rng = np.random.default_rng(seed)
    us = np.linspace(0.12, 0.88, 5) * W
    vs = np.linspace(0.14, 0.86, 4) * H
    grid = [(u, v) for v in vs for u in us]
    kenar = [(fx, fy) for fx, fy in ((0, 0), (0.5, 0), (1, 0), (1, 0.5), (1, 1), (0.5, 1), (0, 1), (0, 0.5),
                                       (0.25, 0), (1, 0.25), (0.75, 1), (0, 0.75))]
    dis_kose = np.array([[sx * 1.05, sy * 0.75, BOARD_Z] for sx in (-1, 1) for sy in (-1, 1)])  # tahta dış köşeleri
    poses = []
    k = n_grid = n_kenar = 0
    while len(poses) < n:
        tur = k % 4
        k += 1
        target = np.array([rng.uniform(-0.2, 0.2), rng.uniform(-0.15, 0.15), BOARD_Z])
        if tur == 0:     # merkezde, yakın/orta uzaklık: büyük tahta, odak uzaklığını iyi belirler
            u_t, v_t = rng.uniform(0.4, 0.6) * W, rng.uniform(0.4, 0.6) * H
            dist = rng.uniform(2.6, 4.5)
        elif tur == 1:   # ızgara noktası
            u_t, v_t = grid[n_grid % len(grid)]
            u_t += rng.uniform(-40, 40)
            v_t += rng.uniform(-30, 30)
            dist = rng.uniform(4.0, 6.5)
        else:            # kenar/köşe: tahtanın dış köşesi görüntü sınırına yakın, tahta tümüyle içeride
            fx, fy = kenar[n_kenar % len(kenar)]
            m = rng.uniform(8, 40, 2)
            u_t = m[0] + fx * (W - 2 * m[0])
            v_t = m[1] + fy * (H - 2 * m[1])
            target = dis_kose[rng.integers(4)].copy()
            dist = rng.uniform(3.5, 6.0)
        tilt = rng.uniform(0, 40)
        if tur < 2:
            C, R = plan_pose(rng, dist, tilt, u_t, v_t, target)
            uv, z = project_nominal(C, R, INNER)
            inside = (z > 0) & (uv[:, 0] > 8) & (uv[:, 0] < W - 8) & (uv[:, 1] > 8) & (uv[:, 1] < H - 8)
            if inside.sum() < 12 or C[2] < 1.0:
                continue
        else:
            # Dış köşe kenarda; tahtanın TAMAMI görüntüde kalacak bir optik eksen dönüşü ara (36 aday).
            az, r0 = rng.uniform(0, 2 * np.pi), rng.uniform(0, 2 * np.pi)
            for roll in r0 + np.arange(36) * np.pi / 18:
                C, R = plan_pose(rng, dist, tilt, u_t, v_t, target, az, roll)
                uvo, zo = project_nominal(C, R, dis_kose)
                if (zo > 0).all() and (uvo[:, 0] > 2).all() and (uvo[:, 0] < W - 2).all() and \
                        (uvo[:, 1] > 2).all() and (uvo[:, 1] < H - 2).all():
                    break
            else:
                continue
            uv, z = project_nominal(C, R, INNER)
            inside = z > 0
        n_grid += tur == 1
        n_kenar += tur >= 2
        poses.append({"C": C.tolist(), "R_world_cv": R.tolist(), "tilt_deg": float(tilt), "dist_m": float(dist),
                      "tur": int(tur), "hedef_px": [float(u_t), float(v_t)], "gorunen_kose_nominal": int(inside.sum())})
    return poses


class Kare:
    def __init__(self):
        self.lock = threading.Lock()
        self.cv = threading.Condition(self.lock)
        self.n = 0
        self.msg = None

    def cb(self, msg):
        with self.cv:
            self.msg = msg
            self.n += 1
            self.cv.notify_all()

    def bekle(self, n_hedef, timeout=10.0):
        with self.cv:
            if not self.cv.wait_for(lambda: self.n >= n_hedef, timeout):
                raise TimeoutError("kare gelmedi")
            return self.msg, self.n


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9


def set_pose(C, q):
    req = (f'name: "{MODEL}", position: {{x: {C[0]:.6f}, y: {C[1]:.6f}, z: {C[2]:.6f}}}, '
           f'orientation: {{w: {q[0]:.9f}, x: {q[1]:.9f}, y: {q[2]:.9f}, z: {q[3]:.9f}}}')
    for _ in range(3):   # gz CLI ara sıra servis keşfinde zaman aşımına düşüyor; yeniden dene
        out = subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
                              "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                             capture_output=True, text=True, env=os.environ)
        if "data: true" in out.stdout:
            return
        print(f"  set_pose yeniden deneniyor: {out.stdout.strip()} {out.stderr.strip()}", flush=True)
    raise RuntimeError("set_pose başarısız")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=48)
    ap.add_argument("--test-every", type=int, default=6, help="her N. poz test görünümü")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default=str(HERE / "kareler"))
    a = ap.parse_args()
    if os.environ.get("GZ_PARTITION") != "kalib":
        sys.exit("GZ_PARTITION=kalib ayarla (başka simülasyonla karışmasın)")

    out = Path(a.out)
    for sub in ("egitim", "test"):
        (out / sub).mkdir(parents=True, exist_ok=True)
    poses = make_poses(a.n, a.seed)

    kare = Kare()
    node = Node()
    if not node.subscribe(Image, TOPIC, kare.cb):
        sys.exit(f"abone olunamadı: {TOPIC}")
    kare.bekle(1, 30.0)
    rng = np.random.default_rng(a.seed + 1)
    kayit = []
    for i, p in enumerate(poses):
        C, R = np.array(p["C"]), np.array(p["R_world_cv"])
        _, n0 = kare.bekle(1)
        t_once = stamp(kare.msg)
        set_pose(C, gz_quat(R))
        msg, _ = kare.bekle(n0 + 2)            # set_pose'dan sonra 2. kare (ilk kare taşımadan önce başlamış olabilir)
        if stamp(msg) <= t_once:
            raise RuntimeError("taze kare değil")
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, : 3 * msg.width]
        rgb = rgb.reshape(msg.height, msg.width, 3)
        bgr = isle(rgb, rng=rng)
        sub = "test" if (i % a.test_every) == a.test_every - 1 else "egitim"
        fn = out / sub / f"view_{i:03d}.png"
        cv2.imwrite(str(fn), bgr)
        kayit.append({**p, "dosya": str(fn.relative_to(out)), "stamp_s": stamp(msg)})
        print(f"{i:2d} {sub:6s} d={p['dist_m']:.2f} m eğim={p['tilt_deg']:4.1f}° hedef=({p['hedef_px'][0]:.0f},"
              f"{p['hedef_px'][1]:.0f}) t={stamp(msg):.2f}", flush=True)
    node.unsubscribe(TOPIC)
    (out / "pozlar.json").write_text(json.dumps(kayit, indent=1))
    print(f"{len(kayit)} kare → {out}")


if __name__ == "__main__":
    main()
