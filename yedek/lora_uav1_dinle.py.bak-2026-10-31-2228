#!/usr/bin/env python3
"""
lora_uav1_dinle.py — uav1 tarafı: YALNIZCA /hedef_konum_role'yi dinleyip aldığını kanıtlayan basit kayıt
düğümü. Hareket/komut YOK. Her mesaj loglanır; SIGTERM/Ctrl+C'de özet (adet, gecikme ort/maks) yazılır.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 lora_uav1_dinle.py [--log /tmp/uav1_role.log]
"""
import argparse
import json
import signal
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Uav1Dinle(Node):
    def __init__(self):
        super().__init__("lora_uav1_dinle")
        self.n = 0
        self.gecikmeler = []
        self.create_subscription(String, "/hedef_konum_role", self.on_role, 50)

    def on_role(self, msg):
        self.n += 1
        now = time.time()
        try:
            d = json.loads(msg.data)
            role = d.get("role", {})
            g = role.get("gecikme_ms")
            if g is not None:
                self.gecikmeler.append(float(g))
            ee = 1000 * (now - role["role_duvar"]) if role.get("role_duvar") else None
            print(f"{time.strftime('%H:%M:%S')}  ALINDI #{self.n} kare={d.get('kare_no')} "
                  f"konum=({d.get('x_kuzey')}, {d.get('y_dogu')}) içerik={d.get('icerik')} "
                  f"röle_gecikme={g} ms varış_gecikme={f'{ee:.1f}' if ee is not None else '?'} ms", flush=True)
        except (ValueError, KeyError) as e:
            print(f"{time.strftime('%H:%M:%S')}  ALINDI #{self.n} (çözümlenemedi: {e})", flush=True)

    def ozet(self):
        g = self.gecikmeler
        return {"alinan": self.n, "gecikme_ort_ms": round(sum(g) / len(g), 2) if g else None,
                "gecikme_maks_ms": round(max(g), 2) if g else None,
                "gecikme_min_ms": round(min(g), 2) if g else None}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default=None, help="(bilgi) harici log dosyası; stdout zaten yönlendirilebilir")
    ap.parse_args()
    rclpy.init()
    node = Uav1Dinle()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    print(f"{time.strftime('%H:%M:%S')}  uav1 dinleyici hazır: /hedef_konum_role (yalnızca dinler, hareket yok)",
          flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"{time.strftime('%H:%M:%S')}  ÖZET: {json.dumps(node.ozet(), ensure_ascii=False)}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
