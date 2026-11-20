#!/usr/bin/env python3
"""gs_kamera.py kuru testi (ROS gerekmez): mozaik/demozaik renkleri ve kenar davranışı."""
import numpy as np

import gs_kamera as g


def test_duz_renk_korunur():
    # Düz renk alanında bilineer demozaik rengi aynen geri vermeli (BGR çıkış).
    for rgb in [(255, 0, 0), (0, 255, 0), (0, 0, 255), (200, 120, 40)]:
        img = np.zeros((32, 32, 3), np.uint8); img[:] = rgb
        bgr = g.isle(img)
        c = bgr[8:24, 8:24].reshape(-1, 3).mean(0)
        assert np.allclose(c[::-1], rgb, atol=1.0), (rgb, c)


def test_kenarda_hata_var_ama_sinirli():
    img = np.full((64, 64, 3), 255, np.uint8); img[:, 32:] = 0          # dikey siyah-beyaz kenar
    gray = g.isle(img).mean(2)
    assert np.abs(gray[:, :30] - 255).max() < 1 and np.abs(gray[:, 34:]).max() < 1
    assert 0 < gray[10, 31] < 255 or 0 < gray[10, 32] < 255             # kenarda karışım (demozaik etkisi)


def test_gurultu_istatistigi():
    rng = np.random.default_rng(1)
    raw = np.full((200, 200), 128, np.uint8)
    out = g.gurultu(raw, shot=0.0, read=3.0, rng=rng).astype(float)
    assert abs(out.std() - 3.0) < 0.2 and abs(out.mean() - 128) < 0.2


if __name__ == "__main__":
    for f in [test_duz_renk_korunur, test_kenarda_hata_var_ama_sinirli, test_gurultu_istatistigi]:
        f(); print("ok", f.__name__)
