#!/usr/bin/env python3
"""Gerçek uçak (pusher, twin-boom, V-kuyruk) Gazebo modeli üreticisi.

Çıktı: gercek_ucak/model.sdf + model.config. Eski advanced_plane'e dokunmaz;
yalnızca sensör bloğunu (IMU, baro, mag, navsat) şablondan okur.

Çerçeve: x ileri, y sol, z yukarı. base_link orijini = CG ≈ kanat çeyrek kord.
Tüm ölçüler metre. Kullanıcı ölçülerini değiştirmek için aşağıdaki sabitleri düzenle.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SABLON = os.path.join(HERE, "..", "gz_models_nocam", "advanced_plane", "model.sdf")
CIKTI = os.path.join(HERE, "gercek_ucak")

# ---- ölçüler (kullanıcı) ----
SPAN = 1.70            # kanat açıklığı
CHORD = 0.27           # kanat kordu
FUS_UZUN = 0.55        # gövde (pod) uzunluğu
BURUN_KUYRUK = 1.25    # burun ucu - kuyruk arka kenarı
V_YATAY = math.radians(40.0)   # kuyruk paneli yataya açısı (iç açı 100°)
KUYRUK_YATAY_GENIS = 0.50      # iki panelin yatay iz düşümü
KUYRUK_KORD = 0.14
RUDDER_KORD = 0.05
AIL_KORD, AIL_ACIKLIK, AIL_Y = 0.07, 0.40, 0.60   # dış panelde, y merkezi
BOOM_ARALIK = 0.45
BOOM_R = 0.004
KUTLE = 2.3
PERVANE_R = 13.6 * 0.0254 / 2

# ---- türetilmiş geometri ----
X_QC = 0.0                     # CG = çeyrek kord
X_KANAT_ORTA = X_QC - CHORD / 4          # orta kord
X_LE = X_KANAT_ORTA + CHORD / 2
X_TE = X_KANAT_ORTA - CHORD / 2
X_GOVDE_ARKA = X_TE + 0.02               # motor kanat arka kenarında (fotoğraf)
X_BURUN = X_GOVDE_ARKA + FUS_UZUN        # burun ucu
X_KUYRUK_TE = X_BURUN - BURUN_KUYRUK
X_KUYRUK_LE = X_KUYRUK_TE + KUYRUK_KORD
Z_BOOM = 0.03
PANEL_UZUN = (KUTLE and (KUYRUK_YATAY_GENIS / 2) / math.cos(V_YATAY))
PANEL_DIKEY = PANEL_UZUN * math.sin(V_YATAY)
Z_APEX = Z_BOOM + (BOOM_ARALIK / 2) * math.tan(V_YATAY)   # TERS V (Λ): tepe yukarıda, panel boom y'sinde boom_z'den geçer
RUDDER_X = X_KUYRUK_TE + RUDDER_KORD / 2
FIX_KORD = KUYRUK_KORD - RUDDER_KORD
FIX_X = X_KUYRUK_LE - FIX_KORD / 2
X_MOTOR = X_GOVDE_ARKA - 0.03
X_PERVANE = X_MOTOR - 0.05


def f(v):
    return f"{v:.5f}"


def pose(x, y, z, r=0.0, p=0.0, yw=0.0):
    return f"{f(x)} {f(y)} {f(z)} {f(r)} {f(p)} {f(yw)}"


def box_inertia(m, sx, sy, sz):
    return (m / 12 * (sy**2 + sz**2), m / 12 * (sx**2 + sz**2), m / 12 * (sx**2 + sy**2))


# --- kütle modeli (nokta+kutu) ---
parcalar = [  # (kütle, x, y, z, sx, sy, sz)
    (0.40, X_GOVDE_ARKA + FUS_UZUN / 2, 0, 0.045, FUS_UZUN, 0.06, 0.07),          # gövde yapısı+aviyonik
    (0.45, X_KANAT_ORTA, 0, 0, CHORD, 0.5, 0.03),                                  # merkez köpük
    (0.25, X_KANAT_ORTA, 0.675, 0, CHORD, 0.35, 0.03),                             # sol dış panel
    (0.25, X_KANAT_ORTA, -0.675, 0, CHORD, 0.35, 0.03),                            # sağ dış panel
    (0.10, X_MOTOR, 0, 0.04, 0.05, 0.05, 0.05),                                    # motor
    (0.05, X_PERVANE, 0, 0.04, 0.02, 2 * PERVANE_R, 0.02),                         # pervane (kütle hub'da)
    (0.05, (X_TE + X_KUYRUK_LE) / 2, 0.225, Z_BOOM, X_TE - X_KUYRUK_LE, 0.01, 0.01),  # boom L
    (0.05, (X_TE + X_KUYRUK_LE) / 2, -0.225, Z_BOOM, X_TE - X_KUYRUK_LE, 0.01, 0.01),  # boom R
    (0.08, X_KUYRUK_TE + KUYRUK_KORD / 2, 0, Z_APEX - PANEL_DIKEY / 2, KUYRUK_KORD, KUYRUK_YATAY_GENIS, PANEL_DIKEY),
]
M_BAT = KUTLE - sum(p[0] for p in parcalar)
XB = (X_QC * KUTLE - sum(p[0] * p[1] for p in parcalar)) / M_BAT   # CG = çeyrek kord olacak batarya konumu
if not (X_GOVDE_ARKA <= XB <= X_BURUN):
    print(f"UYARI: batarya x={XB:.3f} gövde dışında")
parcalar.append((M_BAT, XB, 0, 0.04, 0.12, 0.04, 0.03))
cgx = sum(p[0] * p[1] for p in parcalar) / KUTLE
cgz = sum(p[0] * p[3] for p in parcalar) / KUTLE
Ixx = Iyy = Izz = 0.0
for m, x, y, z, sx, sy, sz in parcalar:
    a, b, c = box_inertia(m, sx, sy, sz)
    dx, dy, dz = x - cgx, y, z - cgz
    Ixx += a + m * (dy**2 + dz**2)
    Iyy += b + m * (dx**2 + dz**2)
    Izz += c + m * (dx**2 + dy**2)

RENK = {
    "beyaz": "0.95 0.95 0.95 1",
    "mavi": "0.62 0.78 0.92 1",
    "yesil": "0.22 0.27 0.12 1",
    "gri": "0.15 0.15 0.15 1",
    "siyah": "0.02 0.02 0.02 1",
    "mavimotor": "0.1 0.3 0.8 1",
}


def malz(k):
    c = RENK[k]
    return f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse><specular>0.1 0.1 0.1 1</specular></material>"


def visual(name, p, geom, renk):
    return f"""      <visual name='{name}'>
        <pose>{p}</pose>
        <geometry>{geom}</geometry>
        {malz(renk)}
      </visual>
"""


def kutu(sx, sy, sz):
    return f"<box><size>{f(sx)} {f(sy)} {f(sz)}</size></box>"


def silindir_x(r, l):
    return f"<cylinder><radius>{f(r)}</radius><length>{f(l)}</length></cylinder>"


# ---- base_link görselleri ----
vis = ""
vis += visual("govde", pose(X_GOVDE_ARKA + FUS_UZUN / 2, 0, 0.045), kutu(FUS_UZUN, 0.06, 0.07), "beyaz")
vis += visual("govde_burun", pose(X_BURUN - 0.03, 0, 0.045), kutu(0.06, 0.06, 0.07), "beyaz")
vis += visual("merkez_kanat", pose(X_KANAT_ORTA, 0, 0), kutu(CHORD, 0.50, 0.03), "mavi")
for s, ad in ((1, "sol"), (-1, "sag")):
    # dış panel: aileron bölgesinde (y 0,40-0,80) kord kısaltılır, aileron ayrı link
    y0, y1 = AIL_Y - AIL_ACIKLIK / 2, AIL_Y + AIL_ACIKLIK / 2
    kisa = CHORD - AIL_KORD
    vis += visual(f"dis_panel_ic_{ad}", pose(X_KANAT_ORTA, s * (0.25 + y0) / 2, 0), kutu(CHORD, y0 - 0.25, 0.03), "yesil")
    vis += visual(f"dis_panel_ail_{ad}", pose(X_KANAT_ORTA + AIL_KORD / 2, s * AIL_Y, 0), kutu(kisa, AIL_ACIKLIK, 0.03), "yesil")
    vis += visual(f"dis_panel_uc_{ad}", pose(X_KANAT_ORTA, s * (y1 + SPAN / 2) / 2, 0), kutu(CHORD, SPAN / 2 - y1, 0.03), "yesil")
# kanat: merkez 0.50 + iki dış panel; aileron cebi dış panelin arkasında ayrı link. Dış panel kalan açıklık:
# toplam = 0.50 + 2*0.60 = 1.70
vis += visual("motor", pose(X_MOTOR, 0, 0.045, 0, math.pi / 2, 0), silindir_x(0.0175, 0.05), "mavimotor")
vis += visual("batarya", pose(XB, 0, 0.04), kutu(0.12, 0.04, 0.03), "gri")
vis += visual("motor_mount", pose(X_GOVDE_ARKA - 0.005, 0, 0.045), kutu(0.02, 0.06, 0.07), "beyaz")
for s, ad in ((1, "sol"), (-1, "sag")):
    L = X_TE - X_KUYRUK_LE + 0.0
    vis += visual(f"boom_{ad}", pose((X_TE + X_KUYRUK_LE) / 2 + 0.0, s * BOOM_ARALIK / 2, Z_BOOM, 0, math.pi / 2, 0),
                  silindir_x(BOOM_R, abs(X_TE - X_KUYRUK_LE) + 0.0), "gri")
    # kanat altındaki ön çubuklar (geçmeli)
    vis += visual(f"kanat_cubuk_{ad}", pose(X_KANAT_ORTA, s * 0.125, 0.0, math.pi / 2, 0, 0), "<cylinder><radius>0.004</radius><length>0.55</length></cylinder>", "gri")
# sabit V-kuyruk panelleri
for s, ad in ((1, "sol"), (-1, "sag")):
    yc = s * KUYRUK_YATAY_GENIS / 4
    zc = Z_APEX - PANEL_DIKEY / 2
    vis += visual(f"vkuyruk_sabit_{ad}", pose(FIX_X, yc, zc, -s * V_YATAY, 0, 0), kutu(FIX_KORD, PANEL_UZUN, 0.004), "beyaz")

collision = f"""      <collision name='govde_col'>
        <pose>{pose(X_GOVDE_ARKA + FUS_UZUN / 2, 0, 0.045)}</pose>
        <geometry>{kutu(FUS_UZUN, 0.06, 0.07)}</geometry>
      </collision>
      <collision name='kanat_col'>
        <pose>{pose(X_KANAT_ORTA, 0, 0)}</pose>
        <geometry>{kutu(CHORD, SPAN, 0.03)}</geometry>
      </collision>
"""

# ---- şablondan sensörler ----
with open(SABLON) as fh:
    satirlar = fh.read().splitlines()
sensorler = "\n".join(satirlar[54:157])  # imu, baro, mag, navsat (satır 55-157)

# ---- kontrol yüzeyleri ----
linkler = ""
eklemler = ""
pluginler = ""


def yuzey_link(ad, p, geom, renk):
    return f"""    <link name='{ad}'>
      <pose>{p}</pose>
      <inertial><mass>1e-5</mass><inertia><ixx>1e-7</ixx><iyy>1e-7</iyy><izz>1e-7</izz></inertia></inertial>
      <visual name='{ad}_visual'>
        <geometry>{geom}</geometry>
        {malz(renk)}
      </visual>
    </link>
"""


def servo(n, child, ax, limit=0.78):
    return f"""    <joint name='servo_{n}' type='revolute'>
      <parent>base_link</parent>
      <child>{child}</child>
      <axis>
        <xyz expressed_in='__model__'>{ax}</xyz>
        <limit><lower>-{limit}</lower><upper>{limit}</upper></limit>
        <dynamics><damping>0.1</damping></dynamics>
      </axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>servo_{n}</joint_name>
      <sub_topic>servo_{n}</sub_topic>
      <p_gain>10</p_gain>
      <i_gain>0</i_gain>
      <d_gain>0</d_gain>
    </plugin>
"""


# aileronlar: servo_0 sol, servo_1 sağ. Menteşe = kanat arka kenarı - aileron kordu.
X_AIL = X_TE + AIL_KORD / 2
for n, s, ad in ((0, 1, "sol"), (1, -1, "sag")):
    linkler += yuzey_link(f"{ad}_aileron", pose(X_AIL, s * AIL_Y, 0), kutu(AIL_KORD, AIL_ACIKLIK, 0.012), "yesil")
    eklemler += servo(n, f"{ad}_aileron", "0 1 0")

# ruddervator: servo_2 sol, servo_3 sağ; menteşe ekseni panelin açıklık yönünde
for n, s, ad in ((2, 1, "sol"), (3, -1, "sag")):
    yc = s * KUYRUK_YATAY_GENIS / 4
    zc = Z_APEX - PANEL_DIKEY / 2
    linkler += yuzey_link(f"{ad}_ruddervator", pose(RUDDER_X, yc, zc, -s * V_YATAY, 0, 0), kutu(RUDDER_KORD, PANEL_UZUN, 0.004), "siyah")
    ax = f"0 {f(math.cos(V_YATAY))} {f(-s * math.sin(V_YATAY))}"  # + = TE yüzeyin üst-normal tarafına (TE yukarı) her iki panelde
    eklemler += servo(n, f"{ad}_ruddervator", ax)

# itici pervane (arkaya doğru iter): +x ileri itki için eksen +x, MulticopterMotorModel
rotor = f"""    <link name='rotor_puller'>
      <pose>{pose(X_PERVANE, 0, 0.045, 0, math.pi / 2, 0)}</pose>
      <inertial><mass>0.005</mass><inertia><ixx>9.75e-07</ixx><iyy>0.000166704</iyy><izz>0.000167604</izz></inertia></inertial>
      <visual name='pervane'>
        <geometry><box><size>0.004 {f(2 * PERVANE_R)} 0.02</size></box></geometry>
        {malz("gri")}
      </visual>
      <visual name='hub'>
        <geometry><cylinder><radius>0.015</radius><length>0.02</length></cylinder></geometry>
        {malz("mavimotor")}
      </visual>
    </link>
    <joint name='rotor_puller_joint' type='revolute'>
      <child>rotor_puller</child>
      <parent>base_link</parent>
      <axis>
        <xyz expressed_in='__model__'>1 0 0</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
      </axis>
    </joint>
"""

pitot = f"""    <link name='airspeed_link'>
      <pose>{pose(X_BURUN + 0.03, 0, 0.045)}</pose>
      <inertial><mass>0.005</mass><inertia><ixx>1e-06</ixx><iyy>1e-06</iyy><izz>1e-06</izz></inertia></inertial>
      <visual name='pitot_visual'>
        <pose>0 0 0 0 1.5708 0</pose>
        <geometry><cylinder><radius>0.003</radius><length>0.06</length></cylinder></geometry>
        {malz("gri")}
      </visual>
      <sensor name='air_speed' type='air_speed'>
        <gz_frame_id>airspeed_link</gz_frame_id>
        <always_on>1</always_on>
        <update_rate>20</update_rate>
        <enable_metrics>false</enable_metrics>
        <air_speed><pressure><noise type='gaussian'><mean>0</mean><stddev>0.5</stddev></noise></pressure></air_speed>
      </sensor>
    </link>
    <joint name='airspeed_joint' type='fixed'>
      <parent>base_link</parent>
      <child>airspeed_link</child>
    </joint>
"""

# ---- aerodinamik ----
S = SPAN * CHORD
AR = SPAN**2 / S
aero = f"""    <plugin filename="gz-sim-advanced-lift-drag-system" name="gz::sim::systems::AdvancedLiftDrag">
      <a0>0.0</a0>
      <CL0>0.15</CL0>
      <AR>{AR:.3f}</AR>
      <eff>0.95</eff>
      <CLa>4.8</CLa>
      <CD0>0.035</CD0>
      <Cem0>0.027</Cem0>
      <Cema>-0.64</Cema>
      <CYb>-0.25</CYb>
      <Cellb>-0.04</Cellb>
      <Cenb>0.10</Cenb>
      <CDp>0.0</CDp>
      <CYp>0.065861</CYp>
      <CLp>0.0</CLp>
      <Cellp>-0.487407</Cellp>
      <Cemp>0.0</Cemp>
      <Cenp>-0.040416</Cenp>
      <CDq>0.055166</CDq>
      <CYq>0.0</CYq>
      <CLq>5.0</CLq>
      <Cellq>0.0</Cellq>
      <Cemq>-6.0</Cemq>
      <Cenq>0.0</Cenq>
      <CDr>0.0</CDr>
      <CYr>0.230299</CYr>
      <CLr>0.0</CLr>
      <Cellr>0.078165</Cellr>
      <Cemr>0.0</Cemr>
      <Cenr>-0.089947</Cenr>
      <alpha_stall>0.30</alpha_stall>
      <CLa_stall>-3.85</CLa_stall>
      <CDa_stall>-0.9233984055</CDa_stall>
      <Cema_stall>0</Cema_stall>
      <cp>0 0 0</cp>
      <area>{S:.4f}</area>
      <mac>{CHORD}</mac>
      <air_density>1.2041</air_density>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <link_name>base_link</link_name>
      <num_ctrl_surfaces>4</num_ctrl_surfaces>
      <control_surface>
        <name>servo_0</name><index>0</index><direction>1</direction>
        <CD_ctrl>-0.000015</CD_ctrl><CY_ctrl>0.0</CY_ctrl><CL_ctrl>-0.0033</CL_ctrl>
        <Cell_ctrl>-0.0009</Cell_ctrl><Cem_ctrl>0.0004</Cem_ctrl><Cen_ctrl>-0.00002</Cen_ctrl>
      </control_surface>
      <control_surface>
        <name>servo_1</name><index>1</index><direction>1</direction>
        <CD_ctrl>-0.000015</CD_ctrl><CY_ctrl>0.0</CY_ctrl><CL_ctrl>-0.0033</CL_ctrl>
        <Cell_ctrl>0.0009</Cell_ctrl><Cem_ctrl>0.0004</Cem_ctrl><Cen_ctrl>0.00002</Cen_ctrl>
      </control_surface>
      <!-- V-kuyruk: her panel yarım elevatör (Cem) + yarım dümen (Cen/CY) etkisi; yaw işareti ZIT -->
      <control_surface>
        <name>servo_2</name><index>2</index><direction>1</direction>
        <CD_ctrl>0.00005</CD_ctrl><CY_ctrl>0.0019</CY_ctrl><CL_ctrl>-0.0020</CL_ctrl>
        <Cell_ctrl>0.0</Cell_ctrl><Cem_ctrl>0.0052</Cem_ctrl><Cen_ctrl>-0.0008</Cen_ctrl>
      </control_surface>
      <control_surface>
        <name>servo_3</name><index>3</index><direction>1</direction>
        <CD_ctrl>0.00005</CD_ctrl><CY_ctrl>-0.0019</CY_ctrl><CL_ctrl>-0.0020</CL_ctrl>
        <Cell_ctrl>0.0</Cell_ctrl><Cem_ctrl>0.0052</Cem_ctrl><Cen_ctrl>0.0008</Cen_ctrl>
      </control_surface>
    </plugin>
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_puller_joint</jointName>
      <linkName>rotor_puller</linkName>
      <turningDirection>cw</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>2000</maxRotVelocity>
      <motorConstant>8.9e-06</motorConstant>
      <momentConstant>0.01</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>0</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
"""

sdf = f"""<?xml version="1.0"?>
<!-- Otomatik üretildi: uret_model.py. Elle değiştirme, betiği düzenle. -->
<sdf version='1.9'>
  <model name='gercek_ucak'>
    <pose>0 0 0.12 0 0 0</pose>
    <link name='base_link'>
      <inertial>
        <pose>{pose(cgx, 0, cgz)}</pose>
        <mass>{KUTLE}</mass>
        <inertia>
          <ixx>{Ixx:.5f}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{Iyy:.5f}</iyy><iyz>0</iyz>
          <izz>{Izz:.5f}</izz>
        </inertia>
      </inertial>
{collision}{vis}      <gravity>1</gravity>
      <self_collide>0</self_collide>
{sensorler}
    </link>
{rotor}{pitot}{linkler}{eklemler}{aero}  </model>
</sdf>
"""

os.makedirs(CIKTI, exist_ok=True)
with open(os.path.join(CIKTI, "model.sdf"), "w") as fh:
    fh.write(sdf)
with open(os.path.join(CIKTI, "model.config"), "w") as fh:
    fh.write("""<?xml version="1.0"?>
<model>
  <name>Gercek Ucak</name>
  <version>1.0</version>
  <sdf version='1.9'>model.sdf</sdf>
  <author><name>iha_sim_v2</name></author>
  <description>Pusher, twin-boom, V-kuyruklu 1,70 m açıklıklı gerçek İHA (ailerron x2, ruddervator x2).</description>
</model>
""")
print(f"batarya x={XB:.3f} m={M_BAT:.2f} kg")
print(f"S={S:.3f} m2 AR={AR:.2f} CG=({cgx:.3f},{cgz:.3f}) I=({Ixx:.4f},{Iyy:.4f},{Izz:.4f})")
print(f"panel_uzun={PANEL_UZUN:.3f} dikey={PANEL_DIKEY:.3f} z_apex={Z_APEX:.3f} burun={X_BURUN:.3f} kuyruk_te={X_KUYRUK_TE:.3f}")

# ---- isteğe bağlı: kameralı varyant (video çekimi) — gercek_ucak_kam/model.sdf ----
def kamera_linki(ad, x, z, pitch, fov, w, h):
    return f"""    <link name='{ad}_link'>
      <pose>{pose(x, 0, z, 0, pitch, 0)}</pose>
      <inertial><mass>0.001</mass><inertia><ixx>1e-08</ixx><iyy>1e-08</iyy><izz>1e-08</izz></inertia></inertial>
      <sensor name='{ad}_cam' type='camera'>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <camera>
          <horizontal_fov>{fov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>3000</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name='{ad}_joint' type='fixed'><parent>base_link</parent><child>{ad}_link</child></joint>
"""
kam = kamera_linki("chase", -1.7, 0.55, 0.12, 1.25, 960, 540) + kamera_linki("burun", X_BURUN + 0.02, 0.06, 0.0, 1.4, 640, 360)
sdf_kam = sdf.replace("<model name='gercek_ucak'>", "<model name='gercek_ucak_kam'>").replace("  </model>\n</sdf>", kam + "  </model>\n</sdf>")
KAM = os.path.join(HERE, "gercek_ucak_kam")
os.makedirs(KAM, exist_ok=True)
with open(os.path.join(KAM, "model.sdf"), "w") as fh:
    fh.write(sdf_kam)
with open(os.path.join(KAM, "model.config"), "w") as fh:
    fh.write("""<?xml version="1.0"?>
<model><name>Gercek Ucak Kamerali</name><version>1.0</version><sdf version='1.9'>model.sdf</sdf><author><name>iha_sim_v2</name></author>
<description>gercek_ucak + arkadan takip kamerasi + burun kamerasi (video cekimi icin).</description></model>
""")
