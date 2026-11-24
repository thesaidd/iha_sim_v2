#!/usr/bin/env python3
"""gercekci_model.py — model.sdf kopyasına gerçekçi GNSS ve magnetometre hatası ekler (GERCEKCI=1, launch_all.sh).

Simde GNSS ve pusula fazla iyi (2026-11-20: PX4 konum hatası 0,1–0,4 m, yaw 0,2°); bapp'in raporladığı σ ve uav1'in
gerçek geçişi bu yüzden iyimser. Bu betik kaynak model klasörünü (ör. gz_models_cam_hw/W_nadir_10) hedefe kopyalar ve
`navsat_sensor` / `magnetometer_sensor` bloklarına gz-sensors Gauss gürültüsü yazar. Kaynak dosyalara DOKUNULMAZ.

gz-sensors GaussianNoiseModel (8.x): çıktı = giriş + bias + beyaz(σ) [+ dinamik bias, birinci derece Gauss–Markov
(dynamic_bias_stddev, dynamic_bias_correlation_time)]. Sabit bias, yükleme anında N(bias_mean, bias_stddev) çekilir ve
İŞARETİ %50 olasılıkla ters çevrilir (her sensör/eksen ayrı) → iki aracın sabit biasları bağımsızdır (ortak kısım yok).
Birimler (2026-11-24 ölçümü, durağan model, bias_mean 50 / 0,05): NavSat YATAY gürültüsü DERECE cinsinden doğrudan
enlem ve boylama eklenir (ikisi de +50° kaydı → aynı gürültü modeli, aynı sabit bias); DİKEY metre; magnetometre çıktı
biriminde (gauss; x/y/z işaretleri ayrı ayrı rastgele). Bu yüzden yatay metre değerleri 1/111320 ile dereceye çevrilir:
kuzey kayması = b, doğu kayması = b·cos(enlem) ≈ 0,68·b (47,4°) — sabit bias kuzeydoğu/güneybatı köşegeninde, doğu
bileşeni %32 küçük. Ayrıntı: test/aruco_tulpar_2026-11-24/sonuc.md "Gerçekçilik".

Varsayılan değerler (VARSAYIM, gerçek donanımda ölçülecek): RTK'sız tek frekans GNSS (u-blox M8/M9 sınıfı) yatay sabit
bias ~1 m (σ 0,5), yavaş gezinme σ 0,7 m / τ 300 s, beyaz 0,3 m; dikey 1,5× . Magnetometre: kalibrasyon sonrası kalan
sert demir ~8 mG yatay (yatay alan ~216 mG @ Zürih → yöne bağlı ±2° civarı yaw), 5 mG düşey.

Kullanım: python3 gercekci_model.py KAYNAK_KLASOR HEDEF_KLASOR [--gnss-bias 1.0 ...] [--gnss-yok] [--mag-yok]
"""
import argparse
import os
import re
import shutil
import sys


def gauss(sigma, bias=0.0, bias_sigma=0.0, dyn=0.0, tau=0.0):
    s = f'<noise type="gaussian"><mean>0</mean><stddev>{sigma:g}</stddev>'
    if bias or bias_sigma:
        s += f"<bias_mean>{bias:g}</bias_mean><bias_stddev>{bias_sigma:g}</bias_stddev>"
    if dyn and tau:
        s += f"<dynamic_bias_stddev>{dyn:g}</dynamic_bias_stddev><dynamic_bias_correlation_time>{tau:g}</dynamic_bias_correlation_time>"
    return s + "</noise>"


M_PER_DEG = 111320.0   # enlem derecesi başına metre (yatay NavSat gürültüsü DERECE cinsinden, aşağıdaki not)


def navsat_blok(a):
    d = 1.0 / M_PER_DEG
    h = gauss(a.gnss_beyaz * d, a.gnss_bias * d, a.gnss_bias_sigma * d, a.gnss_gezinme * d, a.gnss_tau)
    v = gauss(1.5 * a.gnss_beyaz, 1.5 * a.gnss_bias, 1.5 * a.gnss_bias_sigma, 1.5 * a.gnss_gezinme, a.gnss_tau)  # metre
    return (f"<navsat><position_sensing><horizontal>{h}</horizontal><vertical>{v}</vertical></position_sensing>"
            f"<velocity_sensing><horizontal>{gauss(a.gnss_hiz)}</horizontal><vertical>{gauss(a.gnss_hiz)}</vertical>"
            f"</velocity_sensing></navsat>")


def isle_sdf(metin, a):
    n_gnss = n_mag = 0
    if not a.gnss_yok:
        # <sensor name="navsat_sensor" type="navsat"> … </sensor>: varsa eski <navsat> çıkarılır, yenisi eklenir
        def nav(m):
            govde = re.sub(r"<navsat>.*?</navsat>", "", m.group(2), flags=re.S)
            return m.group(1) + govde.rstrip() + "\n        " + navsat_blok(a) + "\n      " + m.group(3)
        metin, n_gnss = re.subn(r'(<sensor name="navsat_sensor" type="navsat">)(.*?)(</sensor>)', nav, metin, flags=re.S)
    if not a.mag_yok:
        def mag(m):
            blok = m.group(0)
            for eks, b in (("x", a.mag_bias_yatay), ("y", a.mag_bias_yatay), ("z", a.mag_bias_dusey)):
                blok = re.sub(rf"<{eks}>\s*<noise type=\"gaussian\">.*?</noise>\s*</{eks}>",
                              f"<{eks}>{gauss(a.mag_beyaz, b, 0.25 * b)}</{eks}>", blok, count=1, flags=re.S)
            return blok
        metin, n_mag = re.subn(r"<magnetometer>.*?</magnetometer>", mag, metin, flags=re.S)
    return metin, n_gnss, n_mag


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kaynak", help="model klasörlerini içeren dizin (PX4_GZ_MODELS gibi)")
    ap.add_argument("hedef")
    ap.add_argument("--gnss-bias", type=float, default=1.0, help="yatay sabit bias ortalaması [m] (işaret rastgele)")
    ap.add_argument("--gnss-bias-sigma", type=float, default=0.5)
    ap.add_argument("--gnss-gezinme", type=float, default=0.7, help="Gauss–Markov gezinme σ [m]")
    ap.add_argument("--gnss-tau", type=float, default=300.0, help="gezinme zaman sabiti [s]")
    ap.add_argument("--gnss-beyaz", type=float, default=0.3, help="beyaz gürültü σ [m]")
    ap.add_argument("--gnss-hiz", type=float, default=0.05, help="hız gürültüsü σ [m/s]")
    ap.add_argument("--mag-bias-yatay", type=float, default=0.008, help="x/y sabit bias [gauss] (işaret rastgele)")
    ap.add_argument("--mag-bias-dusey", type=float, default=0.005)
    ap.add_argument("--mag-beyaz", type=float, default=0.0001)
    ap.add_argument("--gnss-yok", action="store_true")
    ap.add_argument("--mag-yok", action="store_true")
    a = ap.parse_args(argv)
    if os.path.abspath(a.kaynak) == os.path.abspath(a.hedef):
        sys.exit("HATA: hedef kaynakla aynı olamaz")
    shutil.rmtree(a.hedef, ignore_errors=True)
    shutil.copytree(a.kaynak, a.hedef)
    for kok, _, dosyalar in os.walk(a.hedef):
        if "model.sdf" in dosyalar:
            p = os.path.join(kok, "model.sdf")
            metin, ng, nm = isle_sdf(open(p, encoding="utf-8").read(), a)
            open(p, "w", encoding="utf-8").write(metin)
            print(f"{p}: navsat {ng}, magnetometre {nm}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
