#!/usr/bin/env python3
"""arac_modelleri.py — araç (uav) → model eşlemesi, tek yer.

Neden: gz örnek adı (`advanced_plane_0`), kamera topic'i ve kameranın gövdedeki konumu betiklere sabit
gömülüydü (2026-11-22 denetimi: 18 dosya / 33 yer). Tulpar'a geçişte (yalnız uav0) bu yerler buradan okur.

Seçim: ortam değişkeni `UAV0_MODEL` / `UAV1_MODEL` (varsayılan `advanced_plane`; bugün kullanılan: `tulpar`).
VARSAYILAN DAVRANIŞ DEĞİŞMEZ: değişken verilmezse tüm değerler eski sabitlerle birebir aynıdır.

Kullanım (Python):  from arac_modelleri import gz_ad, kamera_kol_flu, kamera_kol_frd
Kullanım (komut):   python3 arac_modelleri.py gz_ad 0        → advanced_plane_0
                    python3 arac_modelleri.py yaml KAYNAK HEDEF   → kamera köprüsü yaml'ında uav0 adını değiştirir
"""
import os
import sys

MODELLER = {
    # airframe: PX4 SYS_AUTOSTART, sim_model: PX4_SIM_MODEL (gz_ + klasör adı)
    # kamera_kol_flu: kameranın gövde (base_link) çerçevesinde konumu [m], x ileri, y sol, z yukarı
    "advanced_plane": {"airframe": 4008, "sim_model": "gz_advanced_plane", "kamera_kol_flu": (0.07, 0.0, -0.1),
                       "kamera_kol_frd": (0.07, 0.0, 0.1)},
    "tulpar": {"airframe": 4030, "sim_model": "gz_tulpar", "kamera_kol_flu": (0.30, 0.0, 0.0),
               "kamera_kol_frd": (0.30, 0.0, 0.0)},
}
VARSAYILAN = "advanced_plane"


def model_adi(uav):
    """uav numarasının model adı (ortam değişkeni UAV<n>_MODEL, yoksa varsayılan)."""
    ad = os.environ.get(f"UAV{uav}_MODEL", VARSAYILAN)
    if ad not in MODELLER:
        raise ValueError(f"UAV{uav}_MODEL={ad!r} tanınmıyor; geçerli: {', '.join(MODELLER)}")
    return ad


def gz_ad(uav):
    """gz örnek adı: PX4 örnek numarası eklenir (advanced_plane_0, tulpar_0)."""
    return f"{model_adi(uav)}_{uav}"


def airframe(uav):
    return MODELLER[model_adi(uav)]["airframe"]


def sim_model(uav):
    return MODELLER[model_adi(uav)]["sim_model"]


def kamera_topic(uav, ad="image"):
    """Aşağı bakan kameranın gz topic'i (ad: image | camera_info). CAM=T* modellerinde sensör base_link'te."""
    return f"/world/default/model/{gz_ad(uav)}/link/base_link/sensor/camera/{ad}"


def kamera_kol_flu(uav=0):
    return MODELLER[model_adi(uav)]["kamera_kol_flu"]


def kamera_kol_frd(uav=0):
    return MODELLER[model_adi(uav)]["kamera_kol_frd"]


def _main(argv):
    if len(argv) >= 3 and argv[1] == "gz_ad":
        print(gz_ad(int(argv[2])))
        return 0
    if len(argv) >= 4 and argv[1] == "yaml":
        # camera_bridge.yaml'daki uav0 adını (varsayılan advanced_plane_0) seçili modelle değiştirir
        metin = open(argv[2], encoding="utf-8").read().replace(f"{VARSAYILAN}_0", gz_ad(0))
        open(argv[3], "w", encoding="utf-8").write(metin)
        return 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(_main(sys.argv))
