#!/usr/bin/env python3
"""
qr_video.py — QR geçiş kayıtlarından (qr_recorder.py çıktısı) açıklamalı mp4 üretir. Simülasyon GEREKMEZ.

Girdi: <kayit>/gecis_NN/kare_*.png + <kayit>/analiz/gecis_NN_kareler.csv (qr_analyze.py çıktısı; QR bulma/
çözme, QR merkezi u,v ve kenar px, geometrik kadrajda bayrağı, sim zamanı, irtifa, yanal sapma x_gt).
Bias değerleri <kayit>/../gecis_sonuclari.jsonl'den (geçiş → bias_m).

Her kare 728×544'e (yarı) indirilir. Üstte bilgi şeridi: geçiş no, bias, sim zamanı, irtifa, hedefe yanal
sapma. QR bulunduysa merkez (u,v) çevresinde kenar kadar kare + metin (çözüldüyse "HEDEF-01" yeşil, çözülmediyse
"bulundu, cozulemedi" sarı); bulunmadıysa kadrajda (geometrik gerçek) ise "kadrajda, bulunamadi" turuncu, değilse
"QR yok" gri. (cv2 Hershey yazı tipi yalnızca ASCII; Türkçe karakter yok.)

Çıktı (<kayit>/video/):
  gecis_NN_tam.mp4    — tüm kareler, 10 fps
  gecis_NN_yavas.mp4  — QR kadrajdayken (ilk–son bulma/kadrajda aralığı), 2 fps
  karsilastirma.png   — 4 geçiş yan yana, her birinin QR'ın en büyük göründüğü (en yakın) karesi

Codec: önce mp4v (.mp4); VideoWriter açılmazsa avc1, sonra XVID (.avi) denenir, neden yazılır.

Kullanım: python3 qr_video.py [--kayit test/qr_kadraj_2026-10-31/kayit]
"""

import argparse
import csv
import json
import os
import cv2
import numpy as np

W, H = 728, 544          # yarı çözünürlük (1456×1088 / 2)
SC = 0.5                 # piksel ölçek


def fbool(s):
    return str(s).strip().lower() == "true"


def fnum(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def kareler_oku(csv_yol):
    rows = list(csv.DictReader(open(csv_yol)))
    rows.sort(key=lambda r: int(r["kare"]))
    return rows


def ciz(img, row, gecis, bias):
    """img: 728×544 BGR; row: analiz CSV satırı. Yerinde çizer, döndürür."""
    sim = fnum(row.get("sim_s"))
    alt = fnum(row.get("irtifa_gt"))
    x = fnum(row.get("x_gt"))
    bulma, cozme, kadraj = fbool(row["QR_bulma"]), fbool(row["QR_cozme"]), fbool(row["kadrajda"])

    # QR kutusu
    if bulma:
        u, v, kenar = fnum(row["QR_u"]), fnum(row["QR_v"]), fnum(row["QR_kenar"])
        if u is not None and v is not None and kenar:
            cu, cv, half = u * SC, v * SC, max(kenar * SC / 2, 4)
            renk = (0, 220, 0) if cozme else (0, 215, 255)       # yeşil / sarı (BGR)
            p1 = (int(cu - half), int(cv - half))
            p2 = (int(cu + half), int(cv + half))
            cv2.rectangle(img, p1, p2, renk, 2)
            etiket = "HEDEF-01" if cozme else "bulundu, cozulemedi"
            ty = p1[1] - 6 if p1[1] > 16 else p2[1] + 16
            cv2.putText(img, etiket, (max(p1[0], 2), ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, renk, 2, cv2.LINE_AA)
    else:
        if kadraj:
            durum, renk = "kadrajda, bulunamadi", (0, 140, 255)   # turuncu
        else:
            durum, renk = "QR yok", (180, 180, 180)               # gri
        cv2.putText(img, durum, (W - 230, H - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, renk, 2, cv2.LINE_AA)

    # üst bilgi şeridi (yarı saydam)
    bar = img.copy()
    cv2.rectangle(bar, (0, 0), (W, 30), (0, 0, 0), -1)
    cv2.addWeighted(bar, 0.55, img, 0.45, 0, img)
    txt = (f"G{gecis} bias {bias:+.1f}m  t={sim:.1f}s  alt={alt:.1f}m  "
           f"yanal={x:+.2f}m") if sim is not None else f"G{gecis} bias {bias:+.1f}m"
    cv2.putText(img, txt, (6, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def kare_yukle(gecis_dir, row):
    # analiz CSV'sinde png adı yok; kare numarasından kur (kare_0001.png)
    pngname = f"kare_{int(row['kare']):04d}.png"
    im = cv2.imread(os.path.join(gecis_dir, pngname))
    if im is None:
        return None
    if im.shape[1] != W:
        im = cv2.resize(im, (W, H), interpolation=cv2.INTER_AREA)
    return im


def writer_ac(yol_base, fps):
    """mp4v → avc1 → XVID(.avi). (writer, yol, codec, not)."""
    for fourcc, ext in (("mp4v", ".mp4"), ("avc1", ".mp4"), ("XVID", ".avi")):
        yol = yol_base + ext
        w = cv2.VideoWriter(yol, cv2.VideoWriter_fourcc(*fourcc), fps, (W, H))
        if w.isOpened():
            return w, yol, fourcc, ""
        w.release()
        if os.path.exists(yol):
            os.remove(yol)
    return None, None, None, "hiçbir codec açılmadı"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kayit", default="test/qr_kadraj_2026-10-31/kayit")
    args = ap.parse_args()
    kayit = os.path.abspath(args.kayit)
    vdir = os.path.join(os.path.dirname(kayit), "video")   # test klasörü altında (kayit'in değil)
    os.makedirs(vdir, exist_ok=True)

    sonuc_yol = os.path.join(os.path.dirname(kayit), "gecis_sonuclari.jsonl")
    bias = {}
    for l in open(sonuc_yol):
        d = json.loads(l)
        bias[d["gecis"]] = d["bias_m"]

    en_yakin = {}        # gecis → (img_cizili) QR en büyük kare
    for gecis in (1, 2, 3, 4):
        gdir = os.path.join(kayit, f"gecis_{gecis:02d}")
        csv_yol = os.path.join(kayit, "analiz", f"gecis_{gecis:02d}_kareler.csv")
        if not (os.path.isdir(gdir) and os.path.exists(csv_yol)):
            print(f"G{gecis}: kayıt/analiz yok, atlandı")
            continue
        rows = kareler_oku(csv_yol)
        b = bias.get(gecis, 0.0)

        # tam akış 10 fps
        wt, yt, ct, nt = writer_ac(os.path.join(vdir, f"gecis_{gecis:02d}_tam"), 10)
        if wt is None:
            print(f"G{gecis}: VideoWriter açılamadı ({nt})")
            return
        # QR penceresi (ilk–son bulma/kadrajda)
        idx = [i for i, r in enumerate(rows) if fbool(r["QR_bulma"]) or fbool(r["kadrajda"])]
        pen = range(min(idx), max(idx) + 1) if idx else range(0)
        wy, yy, cy, ny = writer_ac(os.path.join(vdir, f"gecis_{gecis:02d}_yavas"), 2)

        en_kenar, en_img = -1.0, None
        for i, r in enumerate(rows):
            im = kare_yukle(gdir, r)
            if im is None:
                continue
            im = ciz(im, r, gecis, b)
            wt.write(im)
            if i in pen:
                wy.write(im)
            k = fnum(r["QR_kenar"]) if fbool(r["QR_bulma"]) else None
            if k and k > en_kenar:
                en_kenar, en_img = k, im.copy()
        wt.release()
        wy.release()
        if en_img is None:                       # hiç bulunmadıysa orta kare
            mid = rows[len(rows) // 2]
            im = kare_yukle(gdir, mid)
            en_img = ciz(im, mid, gecis, b) if im is not None else np.zeros((H, W, 3), np.uint8)
        en_yakin[gecis] = en_img

        szt = os.path.getsize(yt) / 1e6
        szy = os.path.getsize(yy) / 1e6 if yy else 0
        print(f"G{gecis}: tam {os.path.basename(yt)} {ct} {len(rows)}k/{len(rows)/10:.1f}s {szt:.2f}MB | "
              f"yavas {os.path.basename(yy)} {len(list(pen))}k/{len(list(pen))/2:.1f}s {szy:.2f}MB | "
              f"en buyuk QR kenar {en_kenar:.1f}px")

    # karşılaştırma: 2×2 ızgara
    if len(en_yakin) == 4:
        ust = np.hstack([en_yakin[1], en_yakin[2]])
        alt = np.hstack([en_yakin[3], en_yakin[4]])
        grid = np.vstack([ust, alt])
        pyol = os.path.join(vdir, "karsilastirma.png")
        cv2.imwrite(pyol, grid)
        print(f"karsilastirma.png {grid.shape[1]}x{grid.shape[0]} {os.path.getsize(pyol)/1e6:.2f}MB")
    else:
        print(f"karsilastirma: 4 geçiş bulunamadı ({sorted(en_yakin)})")


if __name__ == "__main__":
    main()
