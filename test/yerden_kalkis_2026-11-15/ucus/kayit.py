#!/usr/bin/env python3
"""
kayit.py — Gazebo penceresinden periyodik ekran görüntüsüyle açıklamalı mp4 (yerden kalkış senaryosu, Aşama 3).
Yöntem 2026-11-01 belgeleme kaydıyla aynı: gz VideoRecorder servisi yok → `/gui/screenshot` + OpenCV.
Üst şeritte video zamanı, duvar saati ve AŞAMA ETİKETİ (izlenen log dosyalarındaki anahtar satırlardan).

Kullanım: python3 kayit.py --cikti DIR --sure 600 [--log a.log b.log] [--fps 10]
Durdurmak için SIGINT/SIGTERM (mp4 düzgün kapatılır) ya da --sure dolar.
"""
import argparse, os, signal, subprocess, time, glob
import cv2

ASAMA = [  # (log'da aranan metin, video etiketi)
    ("ÜSTE HAZIR", "uav1 uste hazir (yerde, disarmed)"),
    ("PLAN", "plan"), ("hedefsiz plan", "plan"),
    ("NO-GO", "NO-GO"),
    ("inis gorevi", "inis gorevi yuklendi"), ("iniş görevi", "inis gorevi yuklendi"),
    ("HAT 1/", "uav0 tarama"), ("TARAMA DURDU", "uav0 tarama bitti"),
    ("YAYIN kare", "QR TESPIT"),
    ("KALKIŞ KOMUTU", "KALKIS KOMUTU (role geldi)"),
    ("TEKER KESME", "uav1 havada"),
    ("KALKIŞ İRTİFASI", "kalkis irtifasi"),
    ("OFFBOARD ✔", "OFFBOARD - hatta katilma"),
    ("HAT GİRİŞİ", "yaklasma hatti"),
    ("DURDU:", "DALIS - durdu"),
    ("tırmanış tamam", "tirmanis tamam"),
    ("DÖNÜŞ bacağı", "usse donus"),
    ("GÖREV DEVRİ", "inis deseni (PX4 gorevi)"),
    ("YER ALGILANDI", "TEKER KOYMA"),
    ("DISARM ✔", "DISARM - gorev tamam"),
]
PIC = os.path.expanduser("~/.gz/gui/pictures")


def ss():
    subprocess.run(["gz", "service", "-s", "/gui/screenshot", "--reqtype", "gz.msgs.StringMsg",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", 'data: ""'],
                   capture_output=True, timeout=5)


def etiket(loglar, durum):
    for p in loglar:
        try:
            with open(p, errors="ignore") as f:
                f.seek(durum.get(p, 0))
                for satir in f:
                    for anahtar, ad in ASAMA:
                        if anahtar in satir:
                            durum["etiket"] = ad
                durum[p] = f.tell()
        except OSError:
            pass
    return durum.get("etiket", "hazirlik")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cikti", required=True)
    ap.add_argument("--ad", default="kayit")
    ap.add_argument("--sure", type=float, default=600.0)
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--log", nargs="*", default=[])
    ap.add_argument("--baslik", default="IHA Sim V2 - yerden kalkis gorevi")
    a = ap.parse_args()
    os.makedirs(a.cikti, exist_ok=True)
    os.makedirs(PIC, exist_ok=True)
    for f in glob.glob(f"{PIC}/*.png"):
        os.remove(f)
    dur, vw, t0, n, son_etiket = {}, None, time.time(), 0, ""
    boyut, uyarildi = None, False
    csv = open(os.path.join(a.cikti, f"{a.ad}.csv"), "w")
    csv.write("video_s,saat,asama\n")
    calisiyor = [True]
    signal.signal(signal.SIGTERM, lambda *_: calisiyor.clear())
    signal.signal(signal.SIGINT, lambda *_: calisiyor.clear())
    yol = os.path.join(a.cikti, f"{a.ad}.mp4")
    try:
        while calisiyor and time.time() - t0 < a.sure:
            t_kare = time.time()
            ss()
            pngler = sorted(glob.glob(f"{PIC}/*.png"), key=os.path.getmtime)
            if not pngler:
                time.sleep(0.2); continue
            img = cv2.imread(pngler[-1])
            for f in pngler:
                try: os.remove(f)
                except OSError: pass
            if img is None:
                continue
            h, w = img.shape[:2]
            e = etiket(a.log, dur)
            vt = time.time() - t0
            serit = 46
            tuval = cv2.copyMakeBorder(img, serit, 0, 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))
            cv2.putText(tuval, f"{a.baslik}  t={vt:6.1f}s  {time.strftime('%H:%M:%S')}", (8, 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
            cv2.putText(tuval, f"ASAMA: {e}", (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (90, 220, 255), 1, cv2.LINE_AA)
            if vw is None:
                boyut = (w, h + serit)
                vw = cv2.VideoWriter(yol, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, boyut)
            elif (tuval.shape[1], tuval.shape[0]) != boyut:
                # OpenCV VideoWriter farklı boyutlu kareyi SESSİZCE yok sayar (2026-11-16: takip kameralı
                # kayıtta 5791 kare yazıldı, dosyada 63 kare kaldı). Pencere boyutu değişirse ölçekle.
                if not uyarildi:
                    print(f"UYARI: kare boyutu {tuval.shape[1]}x{tuval.shape[0]} ≠ {boyut}, ölçekleniyor", flush=True)
                    uyarildi = True
                tuval = cv2.resize(tuval, boyut)
            # gerçek zamanlı oynatma: geçen süreye göre kare tekrarı
            hedef_n = int(vt * a.fps)
            for _ in range(max(1, hedef_n - n)):
                vw.write(tuval); n += 1
            if e != son_etiket:
                son_etiket = e
                csv.write(f"{vt:.1f},{time.strftime('%H:%M:%S')},{e}\n"); csv.flush()
            kal = 1.0 / 3.0 - (time.time() - t_kare)
            if kal > 0:
                time.sleep(kal)
    finally:
        if vw is not None:
            vw.release()
        csv.close()
        print(f"kayit: {yol} ({n} kare, {n / a.fps:.1f} sn)", flush=True)


if __name__ == "__main__":
    main()
