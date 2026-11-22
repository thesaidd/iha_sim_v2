gercek_ucak — gerçek İHA'nın Gazebo modeli (üretici: uret_model.py)
Yeniden üret : python3 -I uret_model.py
Doğrula      : gz sdf -k gercek_ucak/model.sdf
Önizleme     : ~/venv_iha/bin/python -I render_onizleme.py   (onizleme/*.png)
Servo eşlemesi: servo_0 sol aileron, servo_1 sağ aileron, servo_2 sol ruddervator, servo_3 sağ ruddervator
PX4: 4030_gz_gercek_ucak dosyası PX4'e KOPYALANMADI (kullanıcı onayıyla kopyalanacak, bkz. CLAUDE.md).
Aero katsayıları ve V-kuyruk işaretleri TAHMİNİ; uçak hiç uçurulmadı.

Video (2026-11-21): MODEL_NAME=gercek_ucak_kam ./gercek_ortam.sh start (kameralı varyant: uret_model.py üretir, gercek_ucak_kam/);
  ~/venv_iha/bin/python video/kamera_kayit.py <sn> video/chase.mp4 video/burun.mp4   (gz kamera, 30 fps, sim zamanı)
  MODEL_NAME=gercek_ucak_kam python3 test/gercek_firlat.py 12 38
Çıktılar: video/chase.mp4 (arkadan takip, 960x540), video/burun.mp4 (burun kamerası, 640x360), video/gercek_ucak_firlatma.mp4 (eski, 2 fps GUI).
