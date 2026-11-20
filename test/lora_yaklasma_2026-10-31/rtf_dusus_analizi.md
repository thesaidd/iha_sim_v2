# Adım 3 — kuru test: devralma RTF düşüşü analizi (2026-10-31)

Kaynak: `ucus_sonuc.jsonl`, `yaklasma2.log`/`yaklasma3.log`/`yaklasma5.log` (düğümün kendi 5 sn pencereli
RTF ortalaması), `rtf.log` (bağımsız dış izleyici, 5 sn ÖRNEKLEME — ortalama değil, anlık).

## Gözlenen abort değerleri (düğüm içi, devralma/hazırlık anı)
- Deneme 2 (18:16:05): anlık RTF **0,170** (5 sn ortalaması henüz devredeydi ama o an kodda yoktu — sonradan eklendi).
- Deneme 3 (18:21:05): RTF(5s) ort **0,677** < 0,7.
- Deneme 5 (18:32:20): RTF(5s) ort **0,585** < 0,6 (eşik bilerek düşürülmüş denemeydi).

## Dış izleyici (rtf.log) ile çapraz kontrol
Aynı dakikalarda (18:15–18:16, 18:20–18:22, 18:30–18:32) dış izleyicinin 5 sn'lik örnekleri **0,77–0,81**
bandında, hiç 0,7 altına inmiyor. Bu, iki ayrı gz-transport aboneliğinin (düğüm içi vs. bağımsız izleyici,
ayrı Python süreçleri) ağır yük altında (CPU ~%400) birbirinden farklı örnekler/kayıplar görmesiyle açıklanıyor;
dış log'un kaba 5 sn örneklemesi geçici çöküşü **güvenilir şekilde yakalayamıyor** (yalanlamıyor da).

## Çıkarım
- 5 sn'lik kayan ortalamanın 0,585'e kadar inmesi, düşüşün en az ~5 sn **pencereyi doldurduğunu** gösteriyor
  (yalnızca anlık bir sıçrama değil).
- Elimdeki veriyle düşüşün **gerçek üst süresi bilinmiyor** (5 sn'den uzun sürebilir mi, kesin değil);
  dış log bunu çözmüyor çünkü örnekleme çok kaba ve muhtemelen kayıplı.
- **Latch-and-wait (Adım 1) varsayılan 15 sn**, doğrulanan ~5 sn'lik minimuma göre ~3× pay bırakıyor ama bu
  veriyle "yeterli" kanıtlanamaz, "yetersiz" de kanıtlanamaz — yalnızca uçuşta ölçülebilir.
- **Karar: Adım 2'yi (kamera hız azaltma) ŞİMDİ UYGULAMIYORUM.** Kuru test yetersizliği kanıtlamadı (talimat:
  "yalnızca... göstermesi durumunda uygula"); üstüne RTF çöküşünün asıl nedeni muhtemelen uav0'ın arena
  merkezi üzerinden geçişte sahneyi render etmesi (CLAUDE.md: "iki PX4+kamera+arena merkezi"), yalnızca
  qr_online'ın kamera işleme hızı değil — bu yüzden Adım 2 sorunu kökten çözmeyebilir. Doğrudan Adım 4
  (uçuş, en fazla 2 deneme) ile gerçek telemetriden karar vermek daha güvenilir ve bütçeye uygun.
