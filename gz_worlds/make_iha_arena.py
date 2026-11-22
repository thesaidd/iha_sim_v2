#!/usr/bin/env python3
"""
make_iha_arena.py — PX4 default.sdf'nin kopyasından iha_arena.sdf üretir (PX4 ağacına dokunmaz).

Dünya adı ("default") ve spherical_coordinates aynı kalır → doğma noktası, GPS ve PX4/ros_gz
tarafındaki dünya adı değişmez. Eklenenler yalnızca görsel (çarpışma yok, doku yok, gölge yok);
Gazebo dünya çerçevesi ENU: x = Doğu, y = Kuzey (PX4 NED: N = y, E = x).

  - Halka: merkezde, zeminde R = 100 m, RING_SEGMENTS ince kutu
  - Merkez işareti: düz zemin diski (uav0 (0,0)'da doğuyor; çarpışma olmadığından etkilemez)
  - Direkler: merkezden 140 m, 140 m boy, 20 m'de bir bant; 100 m kırmızı, 120 m mavi.
    D direği tam doğuda değil: kalkış doğuya doğru ve (140, 0) noktası kalkış hattının üstünde
    (2026-10-29 uçuş izleri: uav0 N≈0…−15, uav1 N≈+15…+17, E=140'ta ~30–40 m irtifa).
    Bu yüzden doğudan 30° kuzeye, kalkış ve yaklaşma koridorunun dışına alındı.
  - Izgara: 50 m aralık, ±300 m
  - Binalar: en çok 6, merkezden > 186 m (Hold dönüşleri r ≤ 186 m), doğu koridorunun dışında
  - QR hedefi: (−60, −60) zeminde düz levha (plane), doku qr_hedef01.png (PBR albedo, make_qr_hedef.py);
    kod 1,00 m, kenarla 29/21 m. Kalkış koridoru (doğu), direkler ve binaların dışında. Tek dokulu varlık.
  - ArUco hedefi (--hedef aruco → iha_arena_aruco.sdf, make_aruco_hedef.py): DICT_4X4_50 id 0, siyah dış kenar
    2,00 m + 1 modül (1/3 m) beyaz boşluk → 2,667 m levha, QR ile aynı yer (−60, −60), z = ARUCO_Z = 0,02 m
    (ızgara çizgileri ≤ 0,022 m ama ±50 m katlarında; hedef çizgi üstünde değil). Yaw 0: dokunun üstü kuzey.
  - Doku yolları göreli yazılır (SDF'nin klasörüne göre; gz-sim malzeme URI'lerini SDF dosyasının yoluna göre
    çözer) → depo başka bir kullanıcı/klasörde de çalışır. --mutlak-yol eski davranış.
  - <gui>: kullanıcı gui.config eklentileri + güneybatıdan, 300 m'den merkeze bakan kamera
Zemin görseli 500 → 700 m (ızgara ±300 m'yi kapsasın); çarpışma düzlemi zaten sonsuz.

Kullanım: python3 make_iha_arena.py   → iha_arena.sdf (aynı klasör), varlık sayıları ekrana
          python3 make_iha_arena.py --zemin dokulu --hedef aruco   → iha_arena_aruco.sdf
PX4 kaynağı: $PX4_DIR (varsayılan ~/PX4-Autopilot)/Tools/simulation/gz/worlds/default.sdf
"""

import argparse
import math
import os
import re

PX4_DIR = os.path.expanduser(os.environ.get("PX4_DIR", "~/PX4-Autopilot"))
SRC = os.path.join(PX4_DIR, "Tools/simulation/gz/worlds/default.sdf")
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "iha_arena.sdf")

RING_RADIUS = 100.0
RING_SEGMENTS = 60                    # sınır 72
PYLON_DIST = 140.0
PYLON_HEIGHT = 140.0
PYLONS = {"K": 90.0, "G": -90.0, "B": 180.0, "D": 30.0}   # merkeze göre açı (°, doğudan, saat tersi)
BAND_COLORS = {100: (0.9, 0.1, 0.1), 120: (0.1, 0.35, 1.0)}   # diğer bantlar sarı
GRID_HALF, GRID_STEP = 300, 50
BUILDINGS = [   # (ad, x, y, boyut x, boyut y, yükseklik, renk)
    ("ev", -200.0, 60.0, 14.0, 10.0, 8.0, (0.62, 0.52, 0.42)),
    ("depo", -190.0, -100.0, 30.0, 16.0, 10.0, (0.50, 0.55, 0.60)),
    ("kule", -80.0, 210.0, 10.0, 10.0, 24.0, (0.55, 0.55, 0.50)),
    ("atolye", 60.0, -230.0, 18.0, 12.0, 9.0, (0.60, 0.48, 0.40)),
    ("silo", -250.0, -220.0, 12.0, 12.0, 16.0, (0.70, 0.70, 0.66)),
    ("hangar", 200.0, 220.0, 20.0, 20.0, 12.0, (0.45, 0.50, 0.45)),
]
QR_POS = (-60.0, -60.0)
QR_SIZE = 29.0 / 21.0                    # 21 modül kod = 1,00 m, 4 modül kenar → 1,381 m
QR_Z = 0.05                              # zemin (0) ve ızgara (≤ 0,022) üstünde, z-fighting olmasın
QR_TEXTURE = os.path.join(HERE, "qr_hedef01.png")
ARUCO_POS = (-60.0, -60.0)              # ENU (x = Doğu, y = Kuzey) → NED N = −60, E = −60
ARUCO_SIZE = 2.0 * 8.0 / 6.0            # 2,00 m marker + 1 modül beyaz boşluk (make_aruco_hedef.py varsayılanı)
ARUCO_Z = 0.02                          # zemin (0) üstünde, z-fighting olmasın
ARUCO_TEXTURE = os.path.join(HERE, "aruco_hedef.png")
# Dokulu zemin (--zemin dokulu → iha_arena_v2.sdf): zemin görseli GROUND_TILES × GROUND_TILES karoya bölünür,
# hepsi aynı dikişsiz dokuyu (make_zemin_doku.py, PBR albedo — QR levhasıyla aynı yöntem) paylaşır. Karolar
# ground_plane link'inin içinde, özgün tek görselin yerine geçer (üst üste binme yok, çarpışma düzlemi aynı).
GROUND_SIZE = 700.0
GROUND_TILES = 2                         # 2×2 → 350 m karo; 2048 px doku ile 17 cm/texel
GROUND_TEXTURE = os.path.join(HERE, "zemin_doku.png")
TEXTURE_ABS = False                      # main(): --mutlak-yol
# --gercekci (iha_arena_gercek.sdf, 2026-11-21): ortofoto benzeri 2×2 benzersiz karo (make_zemin_gercek.py),
# halka/ızgara/merkez diski yok, gerçek uçuş-engeli direkleri (beyaz gövde + kırmızı bant), sönük bina renkleri,
# 8 km'lik çevre zemini (ufuk), gök mavisi arka plan. Engel konumları/boyutları DEĞİŞMEZ (planlayıcı aynı).
GERCEKCI = False
EMISSIVE = 0.35                          # --gercekci: 0,06 (kendinden ışıklı görünüm azalır)
GERCEK_DIREK_BANT = (0.85, 0.08, 0.05)
GERCEK_ARKA = (0.53, 0.70, 0.90)         # gök mavisi
GERCEK_CEVRE = (0.26, 0.34, 0.17)        # 700 m zemin görselinin ötesi
CAMERA_POSE = (-300.0, -300.0, 300.0)   # güneybatı, 300 m
CAMERA_TARGET = (0.0, 0.0, 60.0)


def material(rgb, emissive=None):
    emissive = EMISSIVE if emissive is None else emissive
    r, g, b = rgb
    return (f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0 0 0 1</specular><emissive>{r*emissive:.3f} {g*emissive:.3f} {b*emissive:.3f} 1</emissive></material>")


def box_visual(name, x, y, z, yaw, sx, sy, sz, rgb):
    return (f"<visual name='{name}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.5f}</pose>"
            f"<cast_shadows>false</cast_shadows><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
            f"{material(rgb)}</visual>")


def cylinder_visual(name, x, y, z, radius, length, rgb):
    return (f"<visual name='{name}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
            f"<cast_shadows>false</cast_shadows><geometry><cylinder><radius>{radius}</radius><length>{length}</length>"
            f"</cylinder></geometry>{material(rgb)}</visual>")


def tex_uri(path):
    """Doku URI'si: varsayılan göreli (SDF ile aynı klasör), --mutlak-yol ile mutlak."""
    return path if TEXTURE_ABS else os.path.relpath(path, HERE)


def textured_plane_visual(name, x, y, z, size, texture):
    # Doku resmi kuzey yukarı olacak şekilde; yaw 0. Ayna/ters durumu pencereden doğrulanır.
    return (f"<visual name='{name}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
            f"<cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal>"
            f"<size>{size:.4f} {size:.4f}</size></plane></geometry>"
            "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>"
            f"<pbr><metal><albedo_map>{tex_uri(texture)}</albedo_map><metalness>0</metalness><roughness>1</roughness>"
            "</metal></pbr></material></visual>")


def color_plane_visual(name, x, y, z, size, rgb):
    r, g, b = rgb
    return (f"<visual name='{name}'><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
            f"<cast_shadows>false</cast_shadows><geometry><plane><normal>0 0 1</normal>"
            f"<size>{size:.1f} {size:.1f}</size></plane></geometry>"
            f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0 0 0 1</specular>"
            "</material></visual>")


def static_model(name, visuals):
    return (f"<model name='{name}'><static>true</static><link name='link'>"
            + "".join(visuals) + "</link></model>")


def build(hedef="qr"):
    models = {}
    if GERCEKCI:
        models["arena_cevre"] = [color_plane_visual("cevre", 0.0, 0.0, -0.06, 8000.0, GERCEK_CEVRE)]

    seg = 2.0 * RING_RADIUS * math.sin(math.pi / RING_SEGMENTS) + 0.2
    if not GERCEKCI:
        models["arena_halka"] = [
            box_visual(f"s{i}", RING_RADIUS * math.cos(a), RING_RADIUS * math.sin(a), 0.03, a + math.pi / 2,
                       seg, 1.2, 0.03, (1.0, 0.45, 0.0))
            for i, a in ((i, 2.0 * math.pi * i / RING_SEGMENTS) for i in range(RING_SEGMENTS))]

        models["arena_merkez"] = [cylinder_visual("disk", 0.0, 0.0, 0.04, 3.0, 0.03, (0.9, 0.1, 0.1))]

    pylon_visuals = []
    for label, ang in PYLONS.items():
        x, y = PYLON_DIST * math.cos(math.radians(ang)), PYLON_DIST * math.sin(math.radians(ang))
        pylon_visuals.append(cylinder_visual(f"{label}_govde", x, y, PYLON_HEIGHT / 2, 1.5, PYLON_HEIGHT, (0.92, 0.92, 0.92)))
        for h in range(20, int(PYLON_HEIGHT) + 1, 20):
            pylon_visuals.append(cylinder_visual(f"{label}_bant{h}", x, y, h, 2.2, 3.0,
                                                 GERCEK_DIREK_BANT if GERCEKCI else BAND_COLORS.get(h, (1.0, 0.85, 0.0))))
    models["arena_direkler"] = pylon_visuals

    grid = []
    for v in ([] if GERCEKCI else range(-GRID_HALF, GRID_HALF + 1, GRID_STEP)):
        grid.append(box_visual(f"x{v}", v, 0.0, 0.01, 0.0, 0.6, 2.0 * GRID_HALF, 0.02, (0.35, 0.35, 0.38)))
        grid.append(box_visual(f"y{v}", 0.0, v, 0.012, 0.0, 2.0 * GRID_HALF, 0.6, 0.02, (0.35, 0.35, 0.38)))
    if grid:
        models["arena_izgara"] = grid

    models["arena_binalar"] = [box_visual(n, x, y, h / 2, 0.0, sx, sy, h, rgb) for n, x, y, sx, sy, h, rgb in BUILDINGS]

    if hedef == "qr":
        if not os.path.isfile(QR_TEXTURE):
            raise SystemExit(f"QR dokusu yok: {QR_TEXTURE} (önce make_qr_hedef.py)")
        models["arena_qr_hedef01"] = [textured_plane_visual("levha", *QR_POS, QR_Z, QR_SIZE, QR_TEXTURE)]
    else:
        if not os.path.isfile(ARUCO_TEXTURE):
            raise SystemExit(f"ArUco dokusu yok: {ARUCO_TEXTURE} (önce make_aruco_hedef.py)")
        models["arena_aruco_hedef"] = [textured_plane_visual("levha", *ARUCO_POS, ARUCO_Z, ARUCO_SIZE, ARUCO_TEXTURE)]
    return models


def gui_block():
    cx, cy, cz = CAMERA_POSE
    tx, ty, tz = CAMERA_TARGET
    yaw = math.atan2(ty - cy, tx - cx)
    pitch = math.atan2(cz - tz, math.hypot(tx - cx, ty - cy))      # + = aşağı bakış
    floating = ("<gz-gui><property key='resizable' type='bool'>false</property>"
                "<property key='width' type='double'>5</property><property key='height' type='double'>5</property>"
                "<property key='state' type='string'>floating</property>"
                "<property key='showTitleBar' type='bool'>false</property></gz-gui>")
    helpers = "".join(f"<plugin filename='{f}' name='{n}'>{floating}</plugin>" for f, n in (
        ("EntityContextMenuPlugin", "Entity context menu"), ("GzSceneManager", "Scene Manager"),
        ("InteractiveViewControl", "Interactive view control"), ("CameraTracking", "Camera Tracking"),
        ("MarkerManager", "Marker manager"), ("SelectEntities", "Select Entities"),
        ("Spawn", "Spawn Entities"), ("VisualizationCapabilities", "Visualization Capabilities")))
    return (
        "<gui fullscreen='0'>"
        "<plugin filename='MinimalScene' name='3D View'><gz-gui><title>3D View</title>"
        "<property type='bool' key='showTitleBar'>false</property><property type='string' key='state'>docked</property>"
        f"</gz-gui><engine>ogre2</engine><scene>scene</scene><ambient_light>{'0.5 0.5 0.5' if GERCEKCI else '0.4 0.4 0.4'}</ambient_light>"
        f"<background_color>{' '.join(map(str, GERCEK_ARKA)) if GERCEKCI else '0.8 0.8 0.8'}</background_color>"
        f"<camera_pose>{cx} {cy} {cz} 0 {pitch:.4f} {yaw:.4f}</camera_pose></plugin>"
        + helpers +
        "<plugin filename='WorldControl' name='World control'><gz-gui><title>World control</title>"
        "<property type='bool' key='showTitleBar'>false</property><property type='bool' key='resizable'>false</property>"
        "<property type='double' key='height'>72</property><property type='double' key='z'>1</property>"
        "<property type='string' key='state'>floating</property>"
        "<anchors target='3D View'><line own='left' target='left'/><line own='bottom' target='bottom'/></anchors>"
        "</gz-gui><play_pause>true</play_pause><step>true</step><start_paused>false</start_paused>"
        "<use_event>true</use_event></plugin>"
        "<plugin filename='WorldStats' name='World stats'><gz-gui><title>World stats</title>"
        "<property type='bool' key='showTitleBar'>false</property><property type='bool' key='resizable'>false</property>"
        "<property type='double' key='height'>110</property><property type='double' key='width'>290</property>"
        "<property type='double' key='z'>1</property><property type='string' key='state'>floating</property>"
        "<anchors target='3D View'><line own='right' target='right'/><line own='bottom' target='bottom'/></anchors>"
        "</gz-gui><sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor>"
        "<iterations>true</iterations></plugin>"
        f"<plugin filename='Screenshot' name='Screenshot'>{floating}</plugin>"
        "<plugin filename='ComponentInspector' name='Component inspector'><gz-gui>"
        "<property type='bool' key='showTitleBar'>false</property><property type='string' key='state'>docked</property>"
        "</gz-gui></plugin>"
        "<plugin filename='EntityTree' name='Entity tree'><gz-gui>"
        "<property type='bool' key='showTitleBar'>false</property><property type='string' key='state'>docked</property>"
        "</gz-gui></plugin>"
        "</gui>"), math.degrees(pitch), math.degrees(yaw)


def ground_tiles():
    t = GROUND_SIZE / GROUND_TILES
    out = []
    for i in range(GROUND_TILES):
        for j in range(GROUND_TILES):
            x, y = -GROUND_SIZE / 2 + (i + 0.5) * t, -GROUND_SIZE / 2 + (j + 0.5) * t
            tex = os.path.join(HERE, f"zemin_gercek_{i}{j}.jpg") if GERCEKCI else GROUND_TEXTURE
            out.append(textured_plane_visual(f"zemin_{i}{j}", x, y, 0.0, t, tex))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--zemin", choices=["duz", "dokulu"], default="duz",
                    help="duz = özgün gri zemin (iha_arena.sdf); dokulu = çim/toprak dokulu karolar (iha_arena_v2.sdf)")
    ap.add_argument("--hedef", choices=["qr", "aruco"], default="qr",
                    help="qr = 1 m QR (HEDEF-01); aruco = 2 m DICT_4X4_50 marker (aruco_hedef.png)")
    ap.add_argument("--gercekci", action="store_true",
                    help="ortofoto benzeri zemin + halka/ızgara yok + gök mavisi (iha_arena_gercek.sdf); --zemin dokulu sayılır")
    ap.add_argument("--mutlak-yol", action="store_true", help="doku yollarını mutlak yaz (eski davranış)")
    ap.add_argument("--out", default=None,
                    help="çıktı dosyası (varsayılan: iha_arena.sdf / iha_arena_v2.sdf; --hedef aruco → iha_arena_aruco.sdf)")
    args = ap.parse_args()
    global TEXTURE_ABS, GERCEKCI, EMISSIVE
    TEXTURE_ABS = args.mutlak_yol
    if args.gercekci:
        GERCEKCI, EMISSIVE = True, 0.06
        args.zemin = "dokulu"
    if args.gercekci:
        out = args.out or OUT.replace("iha_arena.sdf", "iha_arena_gercek.sdf")
    elif args.hedef == "aruco":
        out = args.out or OUT.replace("iha_arena.sdf", "iha_arena_aruco.sdf")
    else:
        out = args.out or (OUT if args.zemin == "duz" else OUT.replace("iha_arena.sdf", "iha_arena_v2.sdf"))
    text = open(SRC, encoding="utf-8-sig").read()
    if "<world name=\"default\">" not in text:
        raise SystemExit("default.sdf beklenen <world name=\"default\"> içermiyor")
    # Zemin görseli 500 → 700 m (yalnızca visual; collision düzlemi sonsuz)
    text, n = re.subn(r"(<visual name=\"visual\">\s*<geometry>\s*<plane>\s*<normal>0 0 1</normal>\s*<size>)500 500(</size>)",
                      r"\g<1>700 700\g<2>", text)
    if n != 1:
        raise SystemExit("zemin görseli boyutu bulunamadı")

    extra_visuals = 0
    if args.zemin == "dokulu":
        for dosya in ([os.path.join(HERE, f"zemin_gercek_{i}{j}.jpg") for i in range(2) for j in range(2)]
                      if GERCEKCI else [GROUND_TEXTURE]):
            if not os.path.isfile(dosya):
                raise SystemExit(f"zemin dokusu yok: {dosya} (önce make_zemin_{'gercek' if GERCEKCI else 'doku'}.py)")
        tiles = ground_tiles()
        # ground_plane'in tek görseli (çarpışmadan sonraki <visual name="visual"> … </visual>) karolarla değişir
        text, n = re.subn(r"<visual name=\"visual\">.*?</visual>", lambda m: "\n        ".join(tiles), text, count=1, flags=re.S)
        if n != 1:
            raise SystemExit("zemin görseli bloğu bulunamadı")
        extra_visuals = len(tiles) - 1

    if GERCEKCI:   # sahne: gök mavisi arka plan, daha parlak ortam, hafif sıcak güneş (yalnız atmosfer; geometri aynı)
        r, g, b = GERCEK_ARKA
        text, n1 = re.subn(r"<background>[^<]*</background>", f"<background>{r} {g} {b} 1</background>", text, count=1)
        text, n2 = re.subn(r"<ambient>0\.4 0\.4 0\.4 1</ambient>", "<ambient>0.5 0.5 0.5 1</ambient>", text, count=1)
        text, n3 = re.subn(r"(<light name=\"sunUTC\".*?<diffuse>)[^<]*(</diffuse>)", r"\g<1>1.0 0.96 0.88 1\g<2>", text, count=1, flags=re.S)
        if (n1, n2, n3) != (1, 1, 1):
            raise SystemExit(f"sahne/güneş ayarı bulunamadı ({n1},{n2},{n3})")
    models = build(args.hedef)
    gui, pitch, yaw = gui_block()
    extra = "\n    ".join(static_model(name, vis) for name, vis in models.items())
    text = text.replace("  </world>", f"    {gui}\n    {extra}\n  </world>", 1)
    open(out, "w", encoding="utf-8").write(text)

    visuals = sum(len(v) for v in models.values())
    print(f"yazıldı: {out} (zemin: {args.zemin})")
    for name, vis in models.items():
        print(f"  {name:16s} {len(vis):3d} görsel")
    total = 2 * len(models) + visuals + extra_visuals
    print(f"toplam: {len(models)} model + {len(models)} link + {visuals} görsel"
          + (f" + {extra_visuals} ek zemin karosu" if extra_visuals else "") + f" = {total} varlık (sınır 150)")
    if total > 150:
        raise SystemExit("varlık sınırı (150) aşıldı")
    print(f"kamera: {CAMERA_POSE} → hedef {CAMERA_TARGET}, eğim {pitch:.1f}° aşağı, yön {yaw:.1f}°")
    for label, ang in PYLONS.items():
        print(f"  direk {label}: ({PYLON_DIST*math.cos(math.radians(ang)):+7.1f}, {PYLON_DIST*math.sin(math.radians(ang)):+7.1f})")


if __name__ == "__main__":
    main()
