import numpy as np, hesap as H
H.YAWS = np.deg2rad(np.arange(0, 91, 15))
for ms in (20, 25):
    H.MIN_SIDE = ms
    out=[]
    for t in (0, 15, 20, 22, 24, 25, 26, 28):
        b = H.guaranteed_band(100, rolls=[-2,0,2], side_tilt=t)
        out.append(f"t{t}:{b[1]-b[0]:.1f}[{b[0]:.1f},{b[1]:.1f}]")
    print(f"min_side {ms}px:", " ".join(out), flush=True)
