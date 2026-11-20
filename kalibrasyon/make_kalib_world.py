#!/usr/bin/env python3
"""
make_kalib_world.py — Simüle kameranın ChArUco iç kalibrasyonu için dünya üretir.

Üretilenler (bu klasörde):
  tahta.png  : bapp ChArUco tahtası (bap_geo.calibration: 7×5 kare, DICT_5X5_100, marker/kare = 26/35),
               fiziksel ölçek büyütülmüş (kare SQUARE_M), çevresinde MARGIN_SQ kare genişliğinde beyaz kenar.
  kalib.sdf  : boş dünya (Physics, UserCommands, SceneBroadcaster, Sensors/ogre2) + gri zemin + dokulu tahta
               düzlemi (PBR albedo_map) + statik kamera modeli "kamera".

Kamera sensör bloğu W_nadir_10 modelinden (gz_models_cam_hw/W_nadir_10/advanced_plane/model.sdf)
AYNEN kopyalanır; yalnız <pose> (0 0 0 0 0 0: yönelim modelin pozuyla verilir) ve <update_rate> değişir.
W_nadir_10 modeline dokunulmaz.

Tahta koordinatı (dünya): merkez (0, 0, BOARD_Z), yüzey normali +z. Ölçek iç parametreleri etkilemez
(calibrateCamera ölçekten bağımsız K, D verir); yalnız dış parametreleri (t) ölçekler.

Kullanım:  ~/bapp/.venv-ros/bin/python make_kalib_world.py
"""
import re
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(Path.home() / "bapp"))
from bap_geo.calibration import BOARD_MARKER_M, BOARD_SQUARE_M, BOARD_SQUARES, charuco_board  # noqa: E402

KAYNAK_MODEL = HERE.parent / "gz_models_cam_hw/W_nadir_10/advanced_plane/model.sdf"
WORLD_NAME = "kalib"
SQUARE_M = 0.30                                    # sim tahtası: 7×5 × 0,30 m = 2,1 × 1,5 m
MARKER_M = SQUARE_M * BOARD_MARKER_M / BOARD_SQUARE_M
MARGIN_SQ = 0.5                                    # beyaz kenar, kare cinsinden (her yanda)
PX_PER_SQ = 240                                    # doku çözünürlüğü (OpenCV 5.0: 200 px ROI hatası veriyor, 240 sorunsuz)
BOARD_Z = 0.005
UPDATE_RATE = 5
FLIP_TEXTURE = False                               # doku aynalı görünürse True (tespitle doğrulandı: gerek yok)


def board():
    return charuco_board(square_m=SQUARE_M, marker_m=MARKER_M)


def plane_size_m():
    sx, sy = BOARD_SQUARES
    return (sx + 2 * MARGIN_SQ) * SQUARE_M, (sy + 2 * MARGIN_SQ) * SQUARE_M


def make_texture(path: Path):
    sx, sy = BOARD_SQUARES
    m = int(round(MARGIN_SQ * PX_PER_SQ))
    # OpenCV 5.0 generateImage(marginSize>0) ROI hatası veriyor; kenarsız üret, beyaz kenarı sonra ekle.
    img = board().generateImage((sx * PX_PER_SQ, sy * PX_PER_SQ), marginSize=0)
    img = cv2.copyMakeBorder(img, m, m, m, m, cv2.BORDER_CONSTANT, value=255)
    if FLIP_TEXTURE:
        img = cv2.flip(img, 1)
    cv2.imwrite(str(path), img)
    return img.shape


def kamera_sensoru() -> str:
    txt = KAYNAK_MODEL.read_text()
    m = re.search(r'<sensor name="camera" type="camera">.*?</sensor>', txt, re.S)
    if not m:
        raise SystemExit(f"kamera sensörü bulunamadı: {KAYNAK_MODEL}")
    blok = m.group(0)
    blok, n1 = re.subn(r"<pose>[^<]*</pose>", "<pose>0 0 0 0 0 0</pose>", blok, count=1)
    blok, n2 = re.subn(r"<update_rate>[^<]*</update_rate>", f"<update_rate>{UPDATE_RATE}</update_rate>", blok, count=1)
    assert n1 == 1 and n2 == 1
    return blok


def world_sdf(tex_name: str) -> str:
    pw, ph = plane_size_m()
    return f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- make_kalib_world.py ile üretildi; elle düzenleme -->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <grid>false</grid>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>false</shadows>
    </scene>
    <light name="gunes" type="directional">
      <pose>0 0 50 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>1</intensity>
      <direction>0.2 0.1 -1</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
    </light>
    <model name="zemin">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.45 0.45 0.45 1</ambient><diffuse>0.45 0.45 0.45 1</diffuse><specular>0 0 0 1</specular></material>
        </visual>
      </link>
    </model>
    <model name="tahta">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <pose>0 0 {BOARD_Z} 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{pw:.4f} {ph:.4f}</size></plane></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
            <pbr><metal><albedo_map>{tex_name}</albedo_map><metalness>0</metalness><roughness>1</roughness></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
    <model name="kamera">
      <static>true</static>
      <pose>0 0 4 0 1.5708 0</pose>
      <link name="link">
      {kamera_sensoru()}
      </link>
    </model>
  </world>
</sdf>
"""


def main():
    shp = make_texture(HERE / "tahta.png")
    (HERE / "kalib.sdf").write_text(world_sdf("tahta.png"))
    pw, ph = plane_size_m()
    print(f"tahta.png {shp[1]}x{shp[0]} px, düzlem {pw:.2f}×{ph:.2f} m, kare {SQUARE_M} m, marker {MARKER_M:.4f} m")
    print(f"kalib.sdf yazıldı (dünya '{WORLD_NAME}')")


if __name__ == "__main__":
    main()
