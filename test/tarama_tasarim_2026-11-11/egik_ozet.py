"""egik/egik_<etiket>.json → Adım 1 tablosu (markdown). Hücre: tam kadraj kare / online bul / online çöz (geçiş başına, 13 kare)."""
import json, sys
for tag in sys.argv[1:]:
    d = json.load(open(f"egik/egik_{tag}.json")); S = d["satirlar"]
    print(f"\n### {tag} zemin — etkin RTF {d['rtf']}, {len(S)} poz\n")
    print("| yanal kaydırma (m) | geometri | ψ 0° | ψ 45° | ψ 90° | ψ 135° | en az çözme / geçiş |")
    print("|---|---|---|---|---|---|---|")
    for o in sorted({r["kaydirma"] for r in S}):
        huc, enaz, geo = [], 99, ""
        for p in (0, 45, 90, 135):
            z = [r for r in S if r["psi"] == p and r["kaydirma"] == o]
            tam, kis = sum(r["geometri"] == "tam" for r in z), sum(r["geometri"] == "kismi" for r in z)
            geo = "tam kadraj" if tam else ("yalnız kısmi" if kis else "kadraj dışı")
            b, c = sum(r["online"]["bulma"] for r in z), sum(r["online"]["cozme"] for r in z)
            hb = sum(r["online"]["ham_bulma"] for r in z)
            enaz = min(enaz, c)
            huc.append(f"{tam} / {b}{'' if hb == b else f' (+{hb - b} hatalı köşe)'} / {c}")
        print(f"| {o:+.1f} | {geo} | " + " | ".join(huc) + f" | {enaz} |")
    tam = [r for r in S if r["geometri"] == "tam"]
    print(f"\ntam kadraj kareleri: {len(tam)}; online bulma {sum(r['online']['bulma'] for r in tam)}, çözme {sum(r['online']['cozme'] for r in tam)} "
          f"(%{100 * sum(r['online']['cozme'] for r in tam) / len(tam):.1f}); tam kare QRCodeDetector çözme {sum(r['qr']['cozme'] for r in tam)}, "
          f"Aruco çözme {sum(r['aruco']['cozme'] for r in tam)}; yanlış içerik {sum(r['online']['icerik'] not in ('', 'HEDEF-01') for r in S)}; "
          f"kısmi kadrajda çözme {sum(r['online']['cozme'] for r in S if r['geometri'] == 'kismi')}/{sum(r['geometri'] == 'kismi' for r in S)}; "
          f"online işlem ort {sum(r['online']['ms'] for r in S) / len(S):.0f} ms")
    for p in (0, 45, 90, 135):
        z = [r for r in tam if r["psi"] == p]
        print(f"  ψ {p}: tam kadrajda kare başı çözme %{100 * sum(r['online']['cozme'] for r in z) / len(z):.1f} ({len(z)} kare)")
