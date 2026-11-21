# bapp gerçek-uçuş iyileştirmeleri — sim doğrulaması (2026-11-20 gece)

bapp'e eklenenler (ayrıntı: `~/bapp/docs/gercek_ucus_hazirlik.md`, `~/bapp/docs/sim_test1.md` son bölüm): PnP füzyon dışı
(yalnız tanı), raporlanan σ'ya N ile küçülmeyen ortak bias tabanı, kamera↔PX4 saat kipleri (`--clock yerel|alis|timesync`,
`scripts/pi/saat_dogrula.py`), kenar payı süzgeci (`--border-margin-px`, varsayılan 0), yaw izleyicisi (`bap_geo.heading`,
yalnız tanı), kamera kapsama modülü (`bap_geo.coverage`). Bu depoda: `launch_all.sh` sim bias σ'ları + yan kayma β₀
(`SIM_BIAS_ARGS`).

Tarama tüm koşularda B1 ile aynı (`--serit-kamera … --kenar-px 15 --daire 100 --uzatma 60`, `tarama.sh`).

| koşu | bapp | tespit (beklenen) | son rapor hatası | σ95 | raporların σ95 içinde kalan payı | not |
|---|---|---|---|---|---|---|
| B1 (önce) | eski | 38/38 | 0,42 m | 0,39 m | %12 | |
| B2 (önce) | eski | 41/41 | 1,36 m | 0,38 m | %1 | |
| B3 | yeni (yaw izleyicisi yok) | 39/40 | 1,28 m | 1,70 m | %100 | bias σ'ları bu koşudan ayarlanmadı (örneklem dışı) |
| B4 | yeni + yaw izleyicisi | **0** | — | — | — | **düğüm çöktü**: rclpy günlükçüsü aynı satırda önem düzeyi değişince ValueError → düzeltildi |
| **B5** | son | **34/34** | **0,60 m** | 1,72 m | **%100** | hata izi yok; gerçek pozla ray-cast 0,05 m |

Yaw izleyicisi (B5, β₀ 2,7°): ilk ~1 dk `suspicious` (δ_yaw 3,7–4,0° ± 1,0°; gerçek yaw hatası 0,2–0,3°) → yeterli yön
çeşitliliğinden sonra `ok` (1,2–1,9°). Erken dönem yanlış alarmı bilinen sınır (yan kayma ↔ yaw ayrılamıyor); izleyici
geo-lokalizasyonu değiştirmiyor.

Hâlâ simde iyimser: GNSS 0,1–0,4 m, rüzgâr yok; gerçek uçak varsayılanlarıyla σ95 ≈ 5 m olur (temkinli).
