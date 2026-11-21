"""Bagimsiz kaplama / tespit seridi / egik kamera hesabi (duz zemin, pinhole).

Eksenler: dunya NED (X ileri = rota, Y sag, Z asagi), zemin Z=0, ucak Z=-h.
Govde FRD. Kamera OpenCV: x_c = govde y (sag kanat), y_c = -govde x (goruntu ustu = burun),
z_c = govde z (asagi). Goruntu alani u in [0,1456], v in [0,1088], cx=728, cy=544.
Tespit: 4 kose 5 <= u <= 1451, 5 <= v <= 1083 ve en kisa kenar >= 20 px.
"""
import numpy as np

W_PX, H_PX = 1456, 1088
PIX = 3.45e-6
F_MM = 6.0e-3
F = F_MM / PIX                      # 1739.13 px
CX, CY = W_PX / 2, H_PX / 2
MARGIN = 5.0
MIN_SIDE = 20.0
SIDE_M = 2.0
V = 14.0
RATE = 10.0
YAWS = np.deg2rad(np.arange(0, 91, 5))

R_BC = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)  # kamera -> govde


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def R_wc(roll_deg=0.0, pitch_deg=0.0, side_tilt=0.0, fwd_tilt=0.0):
    """kamera->dunya. side_tilt>0: kamera saga bakar; fwd_tilt>0: ileri bakar.
    roll>0 PX4 anlaminda (sag kanat asagi) -> kamera sola doner; isaret simetrik."""
    R_wb = Ry(np.deg2rad(pitch_deg)) @ Rx(np.deg2rad(roll_deg))
    R_mount = Ry(np.deg2rad(fwd_tilt)) @ Rx(np.deg2rad(-side_tilt))
    return R_wb @ R_mount @ R_BC


def project(Rwc, h, P):
    """P: (...,3) dunya noktalari (Z=0). -> (...,2) piksel, z_c."""
    C = np.array([0.0, 0.0, -h])
    pc = (P - C) @ Rwc  # = (Rwc^T (P-C))^T
    z = pc[..., 2]
    u = F * pc[..., 0] / z + CX
    v = F * pc[..., 1] / z + CY
    return np.stack([u, v], -1), z


def ray_ground(Rwc, h, u, v):
    d_c = np.stack([(u - CX) / F, (v - CY) / F, np.ones_like(u)], -1)
    d_w = d_c @ Rwc.T
    t = h / d_w[..., 2]
    return d_w[..., 0] * t, d_w[..., 1] * t


def corners(X, Y, yaw):
    a = SIDE_M / 2
    loc = np.array([[a, a], [-a, a], [-a, -a], [a, -a]])
    c, s = np.cos(yaw), np.sin(yaw)
    rot = loc @ np.array([[c, s], [-s, c]])
    Xs = X[..., None] + rot[:, 0]
    Ys = Y[..., None] + rot[:, 1]
    return np.stack([Xs, Ys, np.zeros_like(Xs)], -1)


def detect_metrics(Rwc, h, X, Y, yaw):
    """X, Y ayni sekil. -> (ok mask, min kenar px)."""
    P = corners(X, Y, yaw)
    uv, z = project(Rwc, h, P)
    u, v = uv[..., 0], uv[..., 1]
    inside = np.all((z > 0) & (u >= MARGIN) & (u <= W_PX - MARGIN)
                    & (v >= MARGIN) & (v <= H_PX - MARGIN), -1)
    d = np.linalg.norm(uv - np.roll(uv, -1, axis=-2), axis=-1)
    mins = d.min(-1)
    return inside & (mins >= MIN_SIDE), mins


def x_range(Rwc, h):
    """Bu Y icin aranacak X araligi: goruntu kose isinlarinin zemin X'leri."""
    us = np.array([0, W_PX, 0, W_PX, CX, CX]); vs = np.array([0, 0, H_PX, H_PX, 0, H_PX])
    gx, gy = ray_ground(Rwc, h, us.astype(float), vs.astype(float))
    return gx.min() - 3, gx.max() + 3


def best_over_x(Rwc, h, Yv, yaw, xr, dx=0.05):
    Xg = np.arange(xr[0], xr[1], dx)
    Xm, Ym = np.meshgrid(Xg, np.atleast_1d(Yv), indexing="ij")
    ok, mins = detect_metrics(Rwc, h, Xm, Ym, yaw)
    any_ok = ok.any(0)
    best = np.where(ok, mins, -np.inf).max(0)
    return any_ok, best, Xg, ok, mins


def band(Rwc, h, yaw, ylim=(-400, 400)):
    """Tespit edilebilir Y araligi [y0, y1] (herhangi bir X'te)."""
    xr = x_range(Rwc, h)
    xr = (max(xr[0], -600), min(xr[1], 600))
    Yc = np.arange(ylim[0], ylim[1], 0.5)
    ok, _, _, _, _ = best_over_x(Rwc, h, Yc, yaw, xr, dx=0.25)
    if not ok.any():
        return None
    idx = np.where(ok)[0]
    lo_in, hi_in = Yc[idx[0]], Yc[idx[-1]]

    def pred(y):
        return best_over_x(Rwc, h, np.array([y]), yaw, xr, dx=0.02)[0][0]

    def bis(a_in, b_out):
        for _ in range(30):
            m = 0.5 * (a_in + b_out)
            if pred(m):
                a_in = m
            else:
                b_out = m
            if abs(a_in - b_out) < 0.005:
                break
        return a_in
    return bis(lo_in, lo_in - 0.5), bis(hi_in, hi_in + 0.5)


def guaranteed_band(h, rolls, pitches=(0.0,), side_tilt=0.0, fwd_tilt=0.0, yaws=YAWS):
    lo, hi = -np.inf, np.inf
    for r in rolls:
        for p in pitches:
            Rwc = R_wc(r, p, side_tilt, fwd_tilt)
            for yw in yaws:
                b = band(Rwc, h, yw)
                if b is None:
                    return None
                lo, hi = max(lo, b[0]), min(hi, b[1])
    return (lo, hi) if hi > lo else None


def gsd_at(Rwc, h, u, v, e=0.5):
    """Pikselin zemin boyutu: (m/px u yonunde, m/px v yonunde), Jacobian sutun normlari."""
    x1, y1 = ray_ground(Rwc, h, np.array(u + e), np.array(v))
    x0, y0 = ray_ground(Rwc, h, np.array(u - e), np.array(v))
    gu = np.hypot(x1 - x0, y1 - y0) / (2 * e)
    x1, y1 = ray_ground(Rwc, h, np.array(u), np.array(v + e))
    x0, y0 = ray_ground(Rwc, h, np.array(u), np.array(v - e))
    gv = np.hypot(x1 - x0, y1 - y0) / (2 * e)
    return float(gu), float(gv)


def band_quality(h, rolls, pitches, side_tilt, fwd_tilt, lo, hi, n=41):
    """Serit icinde: her Y icin en iyi karedeki min kenar px; en kotu durumu raporla.
    Ayrica serit uclarinda marker merkezinin pikselindeki GSD."""
    worst_side = np.inf
    worst_gsd = (0.0, 0.0)
    Ys = np.linspace(lo, hi, n)
    for r in rolls:
        for p in pitches:
            Rwc = R_wc(r, p, side_tilt, fwd_tilt)
            xr = x_range(Rwc, h)
            for yw in YAWS[::3]:
                ok, best, Xg, okm, mins = best_over_x(Rwc, h, Ys, yw, xr, dx=0.1)
                if ok.all():
                    worst_side = min(worst_side, best.min())
            # GSD: serit uclarinda, X = o Y'de goruntu merkez satirina dusen nokta
            for Y in (lo, hi):
                # en kotu GSD: o Y'yi goren tum goruntu satirlari arasinda (v tara)
                for vv in np.linspace(MARGIN, H_PX - MARGIN, 23):
                    # o v satirinda Y'ye karsilik gelen u'yu bul (1B arama)
                    us = np.linspace(0, W_PX, 2913)
                    gx, gy = ray_ground(Rwc, h, us, np.full_like(us, vv))
                    k = np.argmin(np.abs(gy - Y))
                    if abs(gy[k] - Y) > 0.2 or not (MARGIN <= us[k] <= W_PX - MARGIN):
                        continue
                    g = gsd_at(Rwc, h, us[k], vv)
                    if max(g) > max(worst_gsd):
                        worst_gsd = g
    return worst_side, worst_gsd


def fmt(x, nd=1):
    return f"{x:.{nd}f}"


def main():
    print(f"f = {F:.2f} px, HFOV = {np.rad2deg(2*np.arctan(CX/F)):.2f} deg, "
          f"VFOV = {np.rad2deg(2*np.arctan(CY/F)):.2f} deg, IFOV = {np.rad2deg(1/F):.4f} deg/px")

    # ---- 1
    print("\n== 1. Ayak izi (duz ucus) ==")
    for h in (80, 100, 120):
        Wg = 2 * h * CX / F; Lg = 2 * h * CY / F; gsd = h / F
        Rw = R_wc()
        gu_e, gv_e = gsd_at(Rw, h, W_PX - 0.5, CY)
        uv, _ = project(Rw, h, corners(np.array([0.0]), np.array([0.0]), 0.0))
        side_n = np.linalg.norm(uv[0, 0] - uv[0, 1])
        Yedge = Wg / 2 - 1.5
        uv, _ = project(Rw, h, corners(np.array([0.0]), np.array([Yedge]), 0.0))
        d = np.linalg.norm(uv[0] - np.roll(uv[0], -1, 0), axis=-1)
        theta = np.rad2deg(np.arctan(CX / F))
        print(f"h={h}: genislik {Wg:.2f} m, uzunluk {Lg:.2f} m, GSD nadir {100*gsd:.2f} cm/px, "
              f"kenar GSD radyal {100*gu_e:.2f} teget {100*gv_e:.2f} cm/px, "
              f"marker nadir {side_n:.1f} px, kenarda kenarlar {np.round(d,1)} px; "
              f"kenarda egik mesafe {h/np.cos(np.deg2rad(theta)):.1f} m, "
              f"aci-tabanli (IFOV*R) teget {100*h/np.cos(np.deg2rad(theta))/F:.2f}, "
              f"radyal {100*h/np.cos(np.deg2rad(theta))**2/F:.2f} cm/px (bkz. not)")

    # ---- 2
    print("\n== 2. Tespit seridi W (en kotu: yaw 0..90, roll +-m) ==")
    for h in (80, 100, 120):
        row = []
        for m in (0, 1, 2, 3):
            b = guaranteed_band(h, rolls=sorted({-m, 0, m}))
            row.append(b[1] - b[0])
        bp = guaranteed_band(h, rolls=[-2, 0, 2], pitches=[-2, 0, 2])
        print(f"h={h}: W(roll 0/1/2/3) = " + " / ".join(f"{w:.2f}" for w in row)
              + f" m ; roll+-2 & pitch+-2: {bp[1]-bp[0]:.2f} m ; ayak izi {2*h*CX/F:.2f} m")

    # ---- 3
    print("\n== 3. Ucus yonu ==")
    for h in (80, 100, 120):
        Lg = 2 * h * CY / F; ds = V / RATE
        diag = SIDE_M * np.sqrt(2); mpx = 2 * MARGIN * h / F
        Luse = Lg - diag - mpx
        Luse0 = Lg - SIDE_M - mpx
        print(f"h={h}: kare arasi {ds:.2f} m, nokta {Lg/ds:.1f} karede gorunur, "
              f"frontlap {100*(1-ds/Lg):.2f} %, marker tam tespit {Luse/ds:.1f} (yaw45) / "
              f"{Luse0/ds:.1f} (yaw0) kare, %80 frontlap icin tetik {0.2*Lg/V:.3f} s "
              f"({0.2*Lg:.2f} m), %70: {0.3*Lg/V:.3f} s")

    # ---- 4
    print("\n== 4. Hat araligi (h=100) ==")
    Wg = 2 * 100 * CX / F
    for s in (0.3, 0.6, 0.7):
        print(f"sidelap {int(s*100)}%: {Wg*(1-s):.2f} m")
    b = guaranteed_band(100, rolls=[-2, 0, 2])
    print(f"(marker arama icin hat araligi <= W(roll2) = {b[1]-b[0]:.2f} m)")

    # ---- 5
    print("\n== 5. Yana egim (h=100, roll +-2) ==")
    h = 100.0
    hdr = ("t", "yakin kenar", "uzak kenar", "serit lo", "serit hi", "W", "min kenar px",
           "en kotu GSD u/v cm", "uzak aci", "1deg roll", "1deg pitch", "1deg yaw")
    print(" | ".join(hdr))
    for t in (0, 5, 10, 15, 20, 25, 30, 35):
        hf = np.arctan(CX / F)
        near = h * np.tan(np.deg2rad(t) - hf); far = h * np.tan(np.deg2rad(t) + hf)
        b = guaranteed_band(h, rolls=[-2, 0, 2], side_tilt=t)
        lo, hi = b
        ws, wg = band_quality(h, [-2, 0, 2], [0.0], t, 0.0, lo, hi)
        # uzak uc: |Y| buyuk olan
        Yf = hi if abs(hi) >= abs(lo) else lo
        th = np.arctan(abs(Yf) / h)
        d = np.deg2rad(1.0)
        e_roll = h * (np.tan(th + d) - np.tan(th))
        e_pitch = h * np.tan(d)              # tam: X = h tan(d), Y degisimi kucuk
        e_pitch_exact = np.hypot(h * np.tan(d), h * np.tan(th) / np.cos(d) - h * np.tan(th))
        e_yaw = abs(Yf) * d
        print(f"{t:2d} | {near:7.1f} | {far:7.1f} | {lo:7.2f} | {hi:7.2f} | {hi-lo:6.2f} | "
              f"{ws:5.1f} | {100*wg[0]:.1f}/{100*wg[1]:.1f} | {np.rad2deg(th):5.1f} | "
              f"{e_roll:5.2f} | {e_pitch_exact:5.2f} | {e_yaw:5.2f}")

    # ---- 6
    print("\n== 6. Ileri egim (h=100), en kotu roll+-2 & pitch+-2 ==")
    for t in (0, 10, 20, 30):
        b = guaranteed_band(h, rolls=[-2, 0, 2], pitches=[-2, 0, 2], fwd_tilt=t)
        b0 = guaranteed_band(h, rolls=[-2, 0, 2], fwd_tilt=t)
        lo, hi = b
        ws, wg = band_quality(h, [-2, 0, 2], [-2, 0, 2], 0.0, t, lo, hi)
        Rw = R_wc(fwd_tilt=t)
        gc = gsd_at(Rw, h, CX, CY)
        gx_c, _ = ray_ground(Rw, h, np.array(CX), np.array(CY))
        # goruntu ust/alt kenarinda serit genisligi (ham ayak izi)
        xs_t, ys_t = ray_ground(Rw, h, np.array([0.0, W_PX]), np.array([0.0, 0.0]))
        xs_b, ys_b = ray_ground(Rw, h, np.array([0.0, W_PX]), np.array([H_PX, H_PX]))
        print(f"t={t:2d}: W(roll+-2) {b0[1]-b0[0]:.2f} m, W(roll&pitch+-2) {hi-lo:.2f} m, "
              f"ayak izi genisligi arka {ys_b[1]-ys_b[0]:.1f} / on {ys_t[1]-ys_t[0]:.1f} m, "
              f"ileri ofset merkez {gx_c:.1f} m, on kenar {xs_t[0]:.1f} m, "
              f"merkez GSD u/v {100*gc[0]:.2f}/{100*gc[1]:.2f} cm, "
              f"seritte min kenar {ws:.1f} px, en kotu GSD u/v {100*wg[0]:.1f}/{100*wg[1]:.1f} cm")


if __name__ == "__main__":
    main()
