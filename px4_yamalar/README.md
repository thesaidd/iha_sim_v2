# PX4 yamaları (eski Ubuntu 22.04 ortamından çıkarıldı, 2026-11-21)

PX4 `v1.17.0-alpha1-959-g2ca42e5252`, `Tools/simulation/gz` alt modülü `3eb05f7` üzerindeki elle değişiklikler:

- `gz_advanced_plane_el_degisiklik.patch` — `models/advanced_plane/model.sdf`: kırmızı gövde rengi + 640×480 30 Hz
  ileri bakan kamera bloğu. **Günlük kullanımda GEREKMEZ:** `launch_all.sh` kendi modellerini (`gz_models_nocam`,
  `gz_models_cam_hw/*`) `PX4_GZ_MODELS` ile verir. Yalnız `start_sim.sh` / elle `make px4_sitl gz_advanced_plane` için.
- `advanced_plane_target/` — hiçbir betikte kullanılmıyor (yalnız yedek).
- `gz_bridge_magnetometre.patch` — Muhammet'in gz_bridge eksen eşleme yaması (PX4 kaynağına uygulanır).

Yeni PX4'e: `cd Tools/simulation/gz && git apply <depo>/px4_yamalar/gz_advanced_plane_el_degisiklik.patch` (isteğe bağlı).
