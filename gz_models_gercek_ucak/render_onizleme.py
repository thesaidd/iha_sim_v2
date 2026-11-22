#!/usr/bin/env python3
"""model.sdf'ten (box/cylinder görseller, link pozları) 3D önizleme PNG'leri üretir.
Çalıştır: ~/venv_iha/bin/python -I render_onizleme.py
"""
import os
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
SDF = os.path.join(HERE, "gercek_ucak", "model.sdf")
OUT = os.path.join(HERE, "onizleme")
os.makedirs(OUT, exist_ok=True)


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def parse_pose(el):
    if el is None or not el.text:
        return np.zeros(3), np.eye(3)
    v = [float(x) for x in el.text.split()]
    return np.array(v[:3]), rot(*v[3:6])


def box_faces(sx, sy, sz):
    h = np.array([sx, sy, sz]) / 2
    c = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * h
    idx = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [c[list(i)] for i in idx]


def cyl_faces(r, l, n=16):
    t = np.linspace(0, 2 * np.pi, n + 1)
    faces = []
    for a, b in zip(t[:-1], t[1:]):
        faces.append(np.array([[r * np.cos(a), r * np.sin(a), -l / 2], [r * np.cos(b), r * np.sin(b), -l / 2],
                               [r * np.cos(b), r * np.sin(b), l / 2], [r * np.cos(a), r * np.sin(a), l / 2]]))
    faces.append(np.array([[r * np.cos(a), r * np.sin(a), l / 2] for a in t[:-1]]))
    faces.append(np.array([[r * np.cos(a), r * np.sin(a), -l / 2] for a in t[:-1]]))
    return faces


root = ET.parse(SDF).getroot()
model = root.find("model")
polys, colors = [], []
for link in model.findall("link"):
    lp, lR = parse_pose(link.find("pose"))
    for vis in link.findall("visual"):
        vp, vR = parse_pose(vis.find("pose"))
        g = vis.find("geometry")
        if g.find("box") is not None:
            s = [float(x) for x in g.find("box/size").text.split()]
            faces = box_faces(*s)
        elif g.find("cylinder") is not None:
            faces = cyl_faces(float(g.find("cylinder/radius").text), float(g.find("cylinder/length").text))
        else:
            continue
        col = [float(x) for x in vis.find("material/diffuse").text.split()]
        R = lR @ vR
        t = lp + lR @ vp
        for f in faces:
            polys.append((R @ f.T).T + t)
            colors.append(col)

allp = np.concatenate(polys)


def sahne(ax, elev, azim, baslik, ort=True):
    pc = Poly3DCollection(polys, facecolors=colors, edgecolors=(0, 0, 0, 0.35), linewidths=0.3)
    ax.add_collection3d(pc)
    mid = (allp.max(0) + allp.min(0)) / 2
    rng = np.maximum((allp.max(0) - allp.min(0)) * 1.05, [0.1, 0.1, 0.5])
    ax.set_xlim(mid[0] - rng[0] / 2, mid[0] + rng[0] / 2)
    ax.set_ylim(mid[1] - rng[1] / 2, mid[1] + rng[1] / 2)
    ax.set_zlim(mid[2] - rng[2] / 2, mid[2] + rng[2] / 2)
    ax.set_box_aspect(tuple(rng))
    ax.view_init(elev, azim)
    ax.set_title(baslik)
    ax.set_xlabel("x (ileri) [m]")
    ax.set_ylabel("y (sol) [m]")
    ax.set_zlabel("z [m]")


gorunumler = [("persp_ust_arka", 25, 210, "3/4 perspektif (arka-üst)"),
              ("persp_on", 22, 35, "3/4 perspektif (ön)"),
              ("ust", 90, -90, "Üstten"),
              ("yan", 0, -90, "Yandan (burun sağda)"),
              ("arka", 0, 180, "Arkadan (V-kuyruk)"),
              ]
for ad, e, a, b in gorunumler:
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    sahne(ax, e, a, b)
    fig.savefig(os.path.join(OUT, f"{ad}.png"), dpi=140, bbox_inches="tight")
    plt.close(fig)

# 4'lü pano
fig = plt.figure(figsize=(16, 13))
for i, (ad, e, a, b) in enumerate([gorunumler[0], gorunumler[2], gorunumler[3], gorunumler[4]]):
    ax = fig.add_subplot(2, 2, i + 1, projection="3d")
    sahne(ax, e, a, b)
fig.suptitle("gercek_ucak — 1,70 m açıklık, kord 0,27 m, V-kuyruk 100° (yataya 40°), pusher twin-boom", fontsize=14)
fig.savefig(os.path.join(OUT, "pano.png"), dpi=120, bbox_inches="tight")
print("tamam", OUT, "genişlik(y)=%.3f uzunluk(x)=%.3f yükseklik(z)=%.3f" % tuple(allp.max(0)[[1, 0, 2]] - allp.min(0)[[1, 0, 2]]))
