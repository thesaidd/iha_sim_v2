#!/usr/bin/env python3
"""
lora_relay_test.py — lora_relay.py'yi SİMÜLASYONSUZ, kayıtlı/sentetik /hedef_konum ile test eder.

Tek süreçte LoraRole (gerçek röle düğümü) + test koşumcusu (publisher /hedef_konum, subscriber
/hedef_konum_role) MultiThreadedExecutor ile çalışır. Her mesaja benzersiz "test_seq" eklenir; röle yükü
olduğu gibi aktardığından alıcıda eşleşir → uygulanan gecikme (role.gecikme_ms) ve gerçekleşen kayıp ölçülür.

Modlar:
  sentetik : N mesaj (varsayılan 1000), --kume (9) boyutunda kümeler halinde (geçiş benzeri), gerçekçi aralıkla.
  gercek   : test/qr_kadraj_2026-10-31/kayit/analiz/gecis_01..04 kayıtlarından (QR_bulma & kadrajda) = 9/9/9/6
             = 33 tespit; her geçiş kendi sim-zamanı aralıklarıyla beslenir, geçiş başına tablo.

Kullanım:
  source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 lora_relay_test.py --mod sentetik --n 1000 --seed 1 --gecikme-ort 50 --kayip-orani 3
  python3 lora_relay_test.py --mod gercek --seed 1
"""
import argparse
import csv
import json
import os
import statistics
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from lora_relay import LoraRole

KAYIT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test/qr_kadraj_2026-10-31/kayit/analiz")
HEDEF_N, HEDEF_E = -60.0, -60.0


class Kosumcu(Node):
    def __init__(self):
        super().__init__("lora_test")
        self.pub = self.create_publisher(String, "/hedef_konum", 10)
        self.alinan = []          # (test_seq, gecikme_ms, uctan_uca_ms, gecis)
        self.create_subscription(String, "/hedef_konum_role", self.on_role, 50)
        self.gonderim = {}        # test_seq → gonder_duvar

    def on_role(self, msg):
        now = time.time()
        d = json.loads(msg.data)
        seq = d.get("test_seq")
        role = d.get("role", {})
        g0 = self.gonderim.get(seq)
        self.alinan.append((seq, role.get("gecikme_ms"),
                            1000 * (now - g0) if g0 else None, d.get("test_gecis")))

    def gonder(self, seq, gecis, damga_sim, kenar_px, kare_no):
        m = String()
        m.data = json.dumps({
            "damga_sim": damga_sim, "x_kuzey": HEDEF_N, "y_dogu": HEDEF_E,
            "cerceve": "uav0 yerel NED (x=Kuzey, y=Doğu)", "icerik": "HEDEF-01",
            "kenar_px": kenar_px, "modul_px": round(kenar_px / 21, 2), "kare_no": kare_no,
            "bulma_no": seq, "ort_x_kuzey": HEDEF_N, "ort_y_dogu": HEDEF_E, "ort_n": seq,
            "poz_dt_ms": 4.0, "yontem": "test", "test_seq": seq, "test_gecis": gecis}, ensure_ascii=False)
        self.gonderim[seq] = time.time()
        self.pub.publish(m)


def gercek_veri():
    """4 geçişten (QR_bulma & kadrajda) satırları: geçiş → [(sim_s, kenar_px, kare_no), ...]."""
    out = {}
    for i, g in enumerate(("01", "02", "03", "04"), 1):
        rows = []
        for x in csv.DictReader(open(os.path.join(KAYIT, f"gecis_{g}_kareler.csv"))):
            if x["QR_bulma"] == "True" and x["kadrajda"] == "True":
                rows.append((float(x["sim_s"]), float(x["QR_kenar"] or 0), int(x["kare"])))
        rows.sort()
        out[i] = rows
    return out


def istatistik(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    vals.sort()
    return {"n": len(vals), "ort": round(statistics.mean(vals), 2),
            "std": round(statistics.pstdev(vals), 2) if len(vals) > 1 else 0.0,
            "min": round(min(vals), 2), "maks": round(max(vals), 2),
            "p50": round(vals[len(vals) // 2], 2), "p95": round(vals[min(len(vals) - 1, int(0.95 * len(vals)))], 2)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mod", choices=("sentetik", "gercek"), default="sentetik")
    ap.add_argument("--n", type=int, default=1000, help="sentetik mesaj sayısı")
    ap.add_argument("--kume", type=int, default=9, help="sentetik küme (geçiş) boyutu")
    ap.add_argument("--ara", type=float, default=0.012, help="küme içi aralık [sn]")
    ap.add_argument("--kume-ara", type=float, default=0.04, help="kümeler arası ek aralık [sn]")
    ap.add_argument("--gecikme-ort", type=float, default=50.0)
    ap.add_argument("--gecikme-yari", type=float, default=20.0)
    ap.add_argument("--kayip-orani", type=float, default=3.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    rclpy.init()
    role = LoraRole(args.gecikme_ort, args.gecikme_yari, args.kayip_orani, args.seed, stat_sn=1e6)
    test = Kosumcu()
    ex = MultiThreadedExecutor()
    ex.add_node(role)
    ex.add_node(test)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    time.sleep(0.6)                       # keşif

    seq = 0
    if args.mod == "sentetik":
        print(f"sentetik: {args.n} mesaj, küme {args.kume}, ara {args.ara*1000:.0f} ms, "
              f"kümeler arası +{args.kume_ara*1000:.0f} ms", flush=True)
        for i in range(args.n):
            test.gonder(seq, 1 + seq // args.kume, round(100.0 + seq * 0.1, 4), 75.0, seq + 1)
            seq += 1
            time.sleep(args.ara + (args.kume_ara if (i + 1) % args.kume == 0 else 0))
    else:
        veri = gercek_veri()
        print(f"gercek: geçiş başına tespit {[len(veri[g]) for g in (1,2,3,4)]} = {sum(len(veri[g]) for g in veri)}",
              flush=True)
        for g in (1, 2, 3, 4):
            rows = veri[g]
            prev = None
            for sim_s, kenar, kare in rows:
                if prev is not None:
                    time.sleep(min(max(sim_s - prev, 0.0), 0.5))     # gerçek sim-zamanı aralığı (≤0.5 s)
                test.gonder(seq, g, sim_s, kenar, kare)
                seq += 1
                prev = sim_s
            time.sleep(0.3)               # geçişler arası

    time.sleep(args.gecikme_ort / 1000.0 + args.gecikme_yari / 1000.0 + 0.8)   # bekleyen röleleri bekle
    ex.shutdown()

    gonderilen = seq
    alinan = test.alinan
    kayip = gonderilen - len(alinan)
    print("\n===== SONUÇ =====", flush=True)
    print(f"gönderilen (uav0 /hedef_konum): {gonderilen}")
    print(f"alınan (uav1 /hedef_konum_role): {len(alinan)}")
    print(f"kayıp: {kayip}  →  gerçekleşen kayıp oranı %{100.0*kayip/gonderilen:.2f} "
          f"(istenen %{args.kayip_orani:g}, başarı %{100-100.0*kayip/gonderilen:.2f})")
    print(f"röle sayaçları (iç): {json.dumps(role.stat_ozet(), ensure_ascii=False)}")
    gec = istatistik([a[1] for a in alinan])
    uu = istatistik([a[2] for a in alinan])
    print(f"uygulanan gecikme (role.gecikme_ms): {json.dumps(gec, ensure_ascii=False)}")
    print(f"uçtan uca (alıcı duvar − gönderici duvar) ms: {json.dumps(uu, ensure_ascii=False)}")

    if args.mod == "gercek":
        print("\n--- geçiş başına ---")
        print(f"{'geçiş':>5} {'bias':>5} {'gönderilen':>10} {'alınan':>7} {'kayıp':>5} {'gecikme ort/maks ms':>20}")
        bias = {1: 0.0, 2: 7.5, 3: -7.5, 4: 9.5}
        veri = gercek_veri()
        for g in (1, 2, 3, 4):
            gon = len(veri[g])
            al = [a for a in alinan if a[3] == g]
            gl = istatistik([a[1] for a in al])
            print(f"{g:>5} {bias[g]:>+5} {gon:>10} {len(al):>7} {gon-len(al):>5} "
                  f"{(str(gl['ort'])+' / '+str(gl['maks'])) if gl else '-':>20}")

    role.destroy_node()
    test.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
