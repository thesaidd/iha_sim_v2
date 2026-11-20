#!/usr/bin/env python3
"""
lora_relay.py — uav0'ın /hedef_konum yayınlarını LoRa bağını taklit ederek uav1'e (/hedef_konum_role) aktarır.

Formdaki/projede konuşulan LoRa bağı: kısa paket, düşük hız. Varsayılanlar:
  - gecikme ~50 ms (--gecikme-ort), sabit DEĞİL rastgele: Gauss(ort, σ), [ort−yarı, ort+yarı]'ya kırpılır
    (--gecikme-yari 20 → varsayılan [30, 70] ms). Gerekçe: gerçek LoRa gecikmesi = time-on-air (SF/BW/yük;
    kısa paket için onlarca ms) + MAC/duty-cycle + işleme/kuyruk jitter'ı; paket başına değişir, sabit değil.
    Gauss seçildi çünkü jitter tipik değer çevresinde kümelenir, nadir uçlar olur (düzgün dağılım değil).
  - paket başarı oranı ~%97 → kayıp oranı %3 (--kayip-orani, yüzde). Rastgele kayıp (Bernoulli).

ROS 2 topic adları yalnızca ASCII [a-zA-Z0-9_] kabul ettiğinden "röle" yerine ASCII "role" kullanıldı
(/hedef_konum_röle geçersiz olurdu).

Çıktı:
  /hedef_konum_role       (std_msgs/String, JSON): orijinal yük + "role" alt nesnesi:
        alinan_duvar (röle alım, uav0 yayın zamanının vekili), role_duvar (röle yeniden yayın),
        gecikme_ms (uygulanan), damga_sim (orijinal kare sim damgası), kayip_toplam, alinan_toplam,
        gonderilen_toplam.
  /hedef_konum_role_stat  (std_msgs/String, JSON): periyodik istatistik (alinan, gonderilen, kayip,
        kayip_orani_gerceklesen_%, gecikme ort/min/maks). Ayrıca aynı özet loglanır.

Paket modu (--mod paket; bapp aruco_geolocator ile, 2026-11-20):
  /bap/hedef_rapor (std_msgs/UInt8MultiArray, 24 bayt TargetReport) → /bap/hedef_rapor_role (aynı tip, aynı bayt).
  Gerçek LoRa'ya daha yakın: gecikme = time-on-air (Semtech AN1200.13; --sf/--bw/--cr/--preamble, yük = rapor +
  --cerceve-bayt adres/CRC çerçevesi) + Gauss işleme jitter'ı (--isleme-ms ± --isleme-sigma). Tek kanal, yarı
  çift yönlü: verici meşgulken gelen rapor bekleyen tek paketin yerine geçer (en yeni rapor gider; eskiler
  düşer, --mesgul-dus). Kayıp Bernoulli (--kayip-orani). Görev döngüsü (duty cycle) sınırı --duty (%) ile:
  bir paketten sonra kanal ToA·(100/duty − 1) kapalı. Yük baytları değiştirilmez.

Kullanım:
  source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
  python3 lora_relay.py [--gecikme-ort 50] [--gecikme-yari 20] [--kayip-orani 3] [--seed N] [--stat-sn 10]
  python3 lora_relay.py --mod paket [--sf 7 --bw 125 --cr 1] [--duty 10] [--kayip-orani 3]
"""
import argparse
import json
import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray


def log(msg):
    print(f"{time.strftime('%H:%M:%S')}  {msg}", flush=True)


class LoraRole(Node):
    def __init__(self, gecikme_ort=50.0, gecikme_yari=20.0, kayip_yuzde=3.0, seed=None, stat_sn=10.0):
        super().__init__("lora_relay")
        self.gecikme_ort = float(gecikme_ort)       # ms
        self.gecikme_yari = float(gecikme_yari)     # ms (yarı genişlik; [ort−yarı, ort+yarı]'ya kırpılır)
        self.kayip_orani = float(kayip_yuzde) / 100.0
        self.rng = random.Random(seed)
        self.alinan = self.gonderilen = self.kayip = 0
        self.gecikmeler = []
        self.lock = threading.Lock()
        self._role_timers = set()                        # bitmeden kapanmayı önlemek için referans tut
        # /hedef_konum qr_online'da varsayılan QoS (reliable, depth 10) ile yayınlanıyor → eşle.
        self.create_subscription(String, "/hedef_konum", self.on_msg, 10)
        self.pub = self.create_publisher(String, "/hedef_konum_role", 10)
        self.stat_pub = self.create_publisher(String, "/hedef_konum_role_stat", 10)
        self.create_timer(float(stat_sn), self.stat_yayinla)
        log(f"lora_relay hazır: /hedef_konum → /hedef_konum_role | gecikme {self.gecikme_ort}±{self.gecikme_yari} ms "
            f"(Gauss, kırpılı), kayıp %{kayip_yuzde:g}")

    def gecikme_ornek(self):
        """Gauss(ort, σ=yarı/2.5), [ort−yarı, ort+yarı]'ya kırpılı (σ seçimi → ~%99 bant içinde)."""
        d = self.rng.gauss(self.gecikme_ort, self.gecikme_yari / 2.5)
        return max(self.gecikme_ort - self.gecikme_yari, min(self.gecikme_ort + self.gecikme_yari, d))

    def on_msg(self, msg):
        alinan_duvar = time.time()
        with self.lock:
            self.alinan += 1
        if self.rng.random() < self.kayip_orani:
            with self.lock:
                self.kayip += 1
            return
        gecikme = self.gecikme_ornek()
        t = threading.Timer(gecikme / 1000.0, self.gonder, args=(msg.data, alinan_duvar, gecikme))
        t.daemon = True
        with self.lock:
            self._role_timers.add(t)
        t.start()

    def gonder(self, orig_data, alinan_duvar, gecikme):
        role_duvar = time.time()
        try:
            payload = json.loads(orig_data)
        except (ValueError, TypeError):
            payload = {"ham": orig_data}
        with self.lock:
            self.gonderilen += 1
            self.gecikmeler.append(gecikme)
            kayip, alinan, gonderilen = self.kayip, self.alinan, self.gonderilen
            self._role_timers = {t for t in self._role_timers if t.is_alive()}
        payload["role"] = {
            "alinan_duvar": round(alinan_duvar, 6), "role_duvar": round(role_duvar, 6),
            "gecikme_ms": round(gecikme, 2), "damga_sim": payload.get("damga_sim"),
            "kayip_toplam": kayip, "alinan_toplam": alinan, "gonderilen_toplam": gonderilen}
        out = String()
        out.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(out)

    def stat_ozet(self):
        with self.lock:
            g = list(self.gecikmeler)
            a, s, k = self.alinan, self.gonderilen, self.kayip
        return {"alinan": a, "gonderilen": s, "kayip": k,
                "kayip_orani_gerceklesen_%": round(100.0 * k / a, 2) if a else 0.0,
                "gecikme_ort_ms": round(sum(g) / len(g), 2) if g else None,
                "gecikme_min_ms": round(min(g), 2) if g else None,
                "gecikme_maks_ms": round(max(g), 2) if g else None}

    def stat_yayinla(self):
        ozet = self.stat_ozet()
        m = String()
        m.data = json.dumps(ozet, ensure_ascii=False)
        self.stat_pub.publish(m)
        if ozet["alinan"]:
            log(f"istatistik: {json.dumps(ozet, ensure_ascii=False)}")


def lora_toa_ms(payload_bayt, sf=7, bw_khz=125.0, cr=1, preamble=8, crc=True, acik_baslik=True, ldro=None):
    """LoRa time-on-air [ms] (Semtech AN1200.13). cr: 1..4 → 4/5..4/8. ldro None → T_sym > 16 ms ise açık."""
    t_sym = (2 ** sf) / (bw_khz * 1e3)
    de = (t_sym > 0.016) if ldro is None else bool(ldro)
    ih = 0 if acik_baslik else 1
    pay = 8 + max(math.ceil((8 * payload_bayt - 4 * sf + 28 + 16 * int(crc) - 20 * ih) / (4 * (sf - 2 * int(de))))
                  * (cr + 4), 0)
    return ((preamble + 4.25) + pay) * t_sym * 1e3


class LoraPaketRole(Node):
    """Bayt yükü taşıyan LoRa taklidi (bkz. modül notu, paket modu)."""

    def __init__(self, a):
        super().__init__("lora_relay")
        self.a = a
        self.rng = random.Random(a.seed)
        self.lock = threading.Lock()
        self.alinan = self.gonderilen = self.kayip = self.ezilen = 0
        self.gecikmeler = []
        self.bekleyen = None                 # (bayt, alinan_duvar) — verici meşgulken en yeni rapor
        self.kanal_serbest = 0.0             # monotonic: verici (ToA + duty beklemesi) bitişi
        self.create_subscription(UInt8MultiArray, a.giris, self.on_msg, 10)
        self.pub = self.create_publisher(UInt8MultiArray, a.cikis, 10)
        self.stat_pub = self.create_publisher(String, a.cikis + "_stat", 10)
        self.create_timer(float(a.stat_sn), self.stat_yayinla)
        self.create_timer(0.005, self.kanal_dongusu)
        toa = lora_toa_ms(24 + a.cerceve_bayt, a.sf, a.bw, a.cr, a.preamble)
        log(f"lora_relay (paket) hazır: {a.giris} → {a.cikis} | SF{a.sf} BW{a.bw:g} kHz CR4/{a.cr + 4} "
            f"→ 24+{a.cerceve_bayt} B için ToA {toa:.1f} ms | duty %{a.duty:g} | kayıp %{a.kayip_orani:g}")

    def on_msg(self, msg):
        with self.lock:
            self.alinan += 1
            if self.bekleyen is not None:
                self.ezilen += 1             # verici meşgul: eski bekleyen rapor yerini yenisine bırakır
            self.bekleyen = (bytes(msg.data), time.time())

    def kanal_dongusu(self):
        now = time.monotonic()
        with self.lock:
            if self.bekleyen is None or now < self.kanal_serbest:
                return
            data, alinan_duvar = self.bekleyen
            self.bekleyen = None
            toa = lora_toa_ms(len(data) + self.a.cerceve_bayt, self.a.sf, self.a.bw, self.a.cr, self.a.preamble)
            bekleme = toa * (100.0 / self.a.duty - 1.0) if self.a.duty < 100 else 0.0
            self.kanal_serbest = now + (toa + bekleme) / 1000.0
            kayip = self.rng.random() < self.a.kayip_orani / 100.0
            if kayip:
                self.kayip += 1
                return
        gecikme = toa + max(0.0, self.rng.gauss(self.a.isleme_ms, self.a.isleme_sigma))
        t = threading.Timer(gecikme / 1000.0, self.gonder, args=(data, gecikme))
        t.daemon = True
        t.start()

    def gonder(self, data, gecikme):
        self.pub.publish(UInt8MultiArray(data=list(data)))
        with self.lock:
            self.gonderilen += 1
            self.gecikmeler.append(gecikme)

    def stat_ozet(self):
        with self.lock:
            g = list(self.gecikmeler)
            a, s, k, e = self.alinan, self.gonderilen, self.kayip, self.ezilen
        return {"alinan": a, "gonderilen": s, "kayip": k, "ezilen_mesgul": e,
                "gecikme_ort_ms": round(sum(g) / len(g), 2) if g else None,
                "gecikme_min_ms": round(min(g), 2) if g else None,
                "gecikme_maks_ms": round(max(g), 2) if g else None}

    def stat_yayinla(self):
        ozet = self.stat_ozet()
        self.stat_pub.publish(String(data=json.dumps(ozet, ensure_ascii=False)))
        if ozet["alinan"]:
            log(f"istatistik: {json.dumps(ozet, ensure_ascii=False)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gecikme-ort", type=float, default=50.0, help="ortalama gecikme [ms] (varsayılan 50)")
    ap.add_argument("--gecikme-yari", type=float, default=20.0, help="gecikme yarı genişliği [ms] (varsayılan 20 → [30,70])")
    ap.add_argument("--kayip-orani", type=float, default=3.0, help="paket kayıp oranı [%%] (varsayılan 3 → %%97 başarı)")
    ap.add_argument("--seed", type=int, default=None, help="RNG tohumu (yinelenebilirlik)")
    ap.add_argument("--stat-sn", type=float, default=10.0, help="istatistik yayın/log periyodu [sn]")
    ap.add_argument("--mod", choices=("json", "paket"), default="json",
                    help="json: /hedef_konum String (eski); paket: bapp 24 bayt TargetReport")
    ap.add_argument("--giris", default="/bap/hedef_rapor", help="paket modu giriş konusu")
    ap.add_argument("--cikis", default="/bap/hedef_rapor_role", help="paket modu çıkış konusu")
    ap.add_argument("--sf", type=int, default=7, help="LoRa yayılma çarpanı (paket modu)")
    ap.add_argument("--bw", type=float, default=125.0, help="LoRa bant genişliği [kHz]")
    ap.add_argument("--cr", type=int, default=1, choices=(1, 2, 3, 4), help="kodlama oranı 4/(4+cr)")
    ap.add_argument("--preamble", type=int, default=8)
    ap.add_argument("--cerceve-bayt", type=int, default=4, help="adres + uygulama CRC'si [bayt]")
    ap.add_argument("--duty", type=float, default=10.0, help="görev döngüsü sınırı [%%] (433 MHz ISM tipik %%10)")
    ap.add_argument("--isleme-ms", type=float, default=8.0, help="UART/MCU işleme gecikmesi ort. [ms]")
    ap.add_argument("--isleme-sigma", type=float, default=3.0)
    args = ap.parse_args()
    rclpy.init()
    if args.mod == "paket":
        node = LoraPaketRole(args)
    else:
        node = LoraRole(args.gecikme_ort, args.gecikme_yari, args.kayip_orani, args.seed, args.stat_sn)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        log(f"ÖZET: {json.dumps(node.stat_ozet(), ensure_ascii=False)}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
