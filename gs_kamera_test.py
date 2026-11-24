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


K_TEST = np.array([[1739.13, 0, 727.74], [0, 1739.13, 542.73], [0, 0, 1.0]])


def test_geometri_kimlik():
    # distorsiyon ve boresight yoksa harita birim dönüşüm olmalı
    geo = g.Geometri(K_TEST, None, 1456, 1088)
    assert np.abs(geo.mx - np.arange(1456)[None]).max() < 1e-6
    assert np.abs(geo.my - np.arange(1088)[:, None]).max() < 1e-6 and geo.bos_oran < 1e-3


def test_geometri_distorsiyon_projectpoints_ile_tutarli():
    # dünya ışını → cv2.projectPoints(K, D) pikseli; harita o pikselde gz ideal karesindeki K·ışın'ı göstermeli
    import cv2
    D = np.array(g.GERCEKCI_DISTORSIYON)
    geo = g.Geometri(K_TEST, D, 1456, 1088)
    X = np.array([[0.3, 0.2, 1.0], [-0.35, -0.25, 1.0], [0.1, -0.28, 1.0]])
    uv, _ = cv2.projectPoints(X, np.zeros(3), np.zeros(3), K_TEST, D)
    for (u, v), x in zip(uv.reshape(-1, 2), X):
        mx = cv2.getRectSubPix(geo.mx, (1, 1), (float(u), float(v)))[0, 0]
        my = cv2.getRectSubPix(geo.my, (1, 1), (float(u), float(v)))[0, 0]
        ideal = K_TEST @ x
        assert abs(mx - ideal[0]) < 0.05 and abs(my - ideal[1]) < 0.05, (u, v, mx, my, ideal)


def test_boresight_kaydirma_yonu():
    # rx > 0: merkez pikseli nominal karede f·θ yukarıdan örneklenir → içerik f·θ AŞAĞI kayar (gs_kamera notu)
    geo = g.Geometri(K_TEST, None, 1456, 1088, (0.5, 0.0, 0.0))
    dy = geo.my[543, 728] - 543
    assert abs(dy + 1739.13 * np.radians(0.5)) < 0.5, dy
    geo = g.Geometri(K_TEST, None, 1456, 1088, (0.0, 0.5, 0.0))   # ry > 0: sağdan örneklenir → içerik SOLA kayar
    dx = geo.mx[543, 728] - 728
    assert abs(dx - 1739.13 * np.radians(0.5)) < 0.5, dx


if __name__ == "__main__":
    for f in [test_duz_renk_korunur, test_kenarda_hata_var_ama_sinirli, test_gurultu_istatistigi, test_geometri_kimlik,
              test_geometri_distorsiyon_projectpoints_ile_tutarli, test_boresight_kaydirma_yonu]:
        f(); print("ok", f.__name__)
