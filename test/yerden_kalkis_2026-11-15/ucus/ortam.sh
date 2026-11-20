# Yerden kalkış senaryosu ortamı (üs seçimi: ../tasarim.md). basla.sh / ros.sh bunu yükler.
# PİST YÖNÜ 90° (gz yaw 0) ZORUNLU: advanced_plane'in AdvancedLiftDrag aerodinamiği yalnız yaw 0'da doğru
# çalışıyor (2026-11-16 teşhisi: yaw −15° ile uçak gövdesi 105°'yi gösterirken tam doğuya kayıyor, hiç
# pitch almıyor ve kalkamıyor). Ayrıntı tasarim.md "Kök neden".
export UAV1_POSE="-950,200,0.5,0,0,0"            # Gazebo ENU: x=Doğu −950, y=Kuzey 200, yaw 0 → NED 90°
export IHA_UAV1_SPAWN_NE="200,-950"              # ayrim.py: uav1 doğma noktası, uav0 NED
US_ARGS="--kaynak-fark-n 200 --kaynak-fark-e -950 --pist-yonu 90"
