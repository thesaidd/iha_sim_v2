Tulpar — pusher, twin-boom, ters-V (Λ) kuyruklu 1,70 m açıklıklı İHA'nın Gazebo modeli (üretici: uret_model.py)
(Eski çalışma adı "gercek ucak"; 2026-11-22'de Tulpar olarak yeniden adlandırıldı. Yalnız uav0 (tarama+tespit) için.)

Klasörler: her varyantta model adı aynı → `<varyant>/tulpar/model.sdf`, gz örnek adı hep `tulpar_0`:
  nocam/tulpar  kamerasız    kam/tulpar  video kameraları (arkadan takip + burun)    (T10/tulpar: Adım 2'de eklenecek)
Yeniden üret : python3 -I uret_model.py
Doğrula      : gz sdf -k nocam/tulpar/model.sdf
Önizleme     : ~/venv_iha/bin/python -I render_onizleme.py   (onizleme/*.png)
Servo eşlemesi: servo_0 sol aileron, servo_1 sağ aileron, servo_2 sol ruddervator, servo_3 sağ ruddervator (+ komut = arka kenar yukarı)
PX4: 4030_gz_tulpar airframe'i ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ altına kopyalanır (+ CMakeLists satırı);
  yedek/yama: ../px4_yamalar/4030_gz_tulpar ve ../px4_yamalar/4030_airframes_CMakeLists.patch
Aero katsayıları ve V-kuyruk işaretleri TAHMİNİ; uçak hiç uçurulmadı.

Çalıştırma:  [VARIANT=nocam|kam] ./tulpar_ortam.sh start | stop     (Agent + gz sunucusu + PX4 instance 0)
El fırlatma: python3 test/tulpar_firlat.py 12 40                    (yaklaşık 12 m/s, 10° burun yukarı)
Video (2026-11-21): VARIANT=kam ./tulpar_ortam.sh start
  ~/venv_iha/bin/python video/kamera_kayit.py <sn> video/chase.mp4 video/burun.mp4   (gz kamera, 30 fps, sim zamanı)
  python3 test/tulpar_firlat.py 12 38
Çıktılar: video/chase.mp4 (arkadan takip, 960x540), video/burun.mp4 (burun kamerası, 640x360), video/tulpar_firlatma_gui.mp4 (eski, 2 fps GUI).
