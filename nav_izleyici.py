"""
nav_izleyici.py — px4_durum.py --izle alt sürecinden PX4 nav_state'i sürekli okur (qr_pass.py, qr_watch.py).

Bu modül ortam değişkenlerine dokunmaz: px4_durum.py'nin RMW_FASTRTPS_USE_QOS_FROM_XML=1 ayarı yalnızca alt
süreçte geçerlidir; setpoint/komut yayınlayan süreçlerin ROS QoS'u değişmez.
"""

import os
import subprocess
import sys
import threading
import time

from px4_durum_sabit import AUTO_LOITER, AUTO_RTL, NAV_NAMES, OFFBOARD   # noqa: F401  (dışa aktarım)

PX4_DURUM = os.path.join(os.path.dirname(os.path.abspath(__file__)), "px4_durum.py")


class NavIzleyici:
    def __init__(self, uav_id: int = 0):
        self.nav = None          # son nav_state
        self.t = 0.0             # son örneğin duvar saati
        self.history = []        # (duvar, nav_state) yalnızca değişimler
        self.proc = subprocess.Popen([sys.executable, PX4_DURUM, "--uav", str(uav_id), "--izle"],
                                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            if not line.startswith("NAV ") or line.startswith("NAV yok"):
                continue
            nav = int(line.split()[1])
            if nav != self.nav:
                self.history.append((time.time(), nav))
            self.nav, self.t = nav, time.time()

    def name(self, nav=None):
        nav = self.nav if nav is None else nav
        return "?" if nav is None else NAV_NAMES.get(nav, str(nav))

    def fresh(self, max_age: float = 2.0) -> bool:
        return self.nav is not None and time.time() - self.t <= max_age

    def wait_first(self, timeout: float = 8.0) -> bool:
        t0 = time.time()
        while self.nav is None and time.time() - t0 < timeout:
            time.sleep(0.05)
        return self.nav is not None

    def changes_text(self):
        return " → ".join(f"{time.strftime('%H:%M:%S', time.localtime(t))} {self.name(n)}" for t, n in self.history)

    def close(self):
        self.proc.terminate()
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill()
