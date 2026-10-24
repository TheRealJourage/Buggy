# Session-Anleitung: Schaltung am Buggy vermessen

Du hilfst dem User live am Fahrzeug, die Gangschaltung für die pneumatische Schaltbox zu vermessen.
Er steht am Buggy, hat nur das iPhone (kein Vault am Handy) und das Formular
**http://100.81.33.96:8112/messung.html** (Tailscale, Datei `viewer/messung.html`) offen.

## Kontext zuerst lesen (nur lesen, nicht zusammenfassen)
- Vault `/srv/data/obsidian/vault/Buggy/Druckluftanlage.md` — Schaltbild, Zylinder G/A/B/K, Poti-Fenster, Abschnitte „Offen" und Einkaufsliste (Pos. 1, 22)
- Vault `/srv/data/obsidian/vault/Buggy/Zylinderbox.md` — Konstruktion, Abschnitt „Offen"
- Vault `/srv/data/obsidian/vault/Buggy/DOKUMENTATION.md` → „Offene Aufgaben" → Todo „Am Buggy: Schaltung vermessen"
- Git: Repo gehört ftk → `git -c safe.directory=/srv/data/projects/buggy …`; Referenzstand Tag `schaltbox-v3`

## Festgelegt (nicht neu diskutieren)
- Schaltbild oben R 1 3 5, unten 2 4 → 4 Gassen: R, 1/2, 3/4, 5
- G = Gassen-/Wählseil (mit Poti KPZ 125 mm, nutzbar bis ~115 mm); A+B kaskadiert = Schaltseil
- Zylinder greifen an den Original-Seilenden an (Bolzen Ø 8 mm, Schlitz RD-20 = 8 mm)

## Annahmen, die die Messung ersetzt
| Größe | Annahme | Notiz vom User |
|---|---|---|
| Gassenabstand | 32 mm → G-Hub 96 mm | 3,4 / 3 cm (dritter Wert fehlt) |
| Reihenwege A / B | 14 / 34 mm | 3,4 / 1,4 cm — Zuordnung unklar |
| Zylinder | HM-20 (Ø 20), Ø 16/25 je nach Kraft | — |
| Kupplungspedal | K-Hub 50 mm | Weg 5 cm |

## Ablauf mit dem User
Kurz, Deutsch, **ein Messschritt pro Nachricht**, immer sagen „Schritt x von 7". Der User tippt Werte ins Formular; frag nach dem Wert, damit du live plausibilisieren kannst.

1. **Schaltbild** gegenprüfen (Häkchen im Formular)
2. **Gassenwege G**: jede Gasse in Neutral am Seilende markieren, Abstände R→1/2, 1/2→3/4, 3/4→5
3. **Reihenwege**: je Gasse Neutral→oben und Neutral→unten
4. **Kräfte** (Kofferwaage, Motor aus, Kupplung getreten, Spitzenwert, Zug in Seilrichtung): Gassenwechsel beide Richtungen, jeden Gang einlegen + rausnehmen
5. **Seilenden**: Dicke Seilauge (≤ 8 mm?), Clip-Form fotografieren, festes Teil 7/13 cm
6. **Pedal**: Kraft + Weg, vorhandenen Kupplungsschalter fotografieren
7. **Einbauort** Zylinderbox: Platz, Seilverlauf, Biegeradien; nimmt das Seil Bewegung auf (→ Gummilagerung ja/nein)?

## Live-Plausibilisierung
- Summe Gassenabstände > ~115 mm → Poti 125 mm reicht nicht, sofort melden
- Abstände sehr ungleich → nachmessen lassen (Markierung am Seilende, nicht am Schalthebel)
- Oben/unten-Wege stark unterschiedlich zwischen Gassen → nachfragen, ob richtig angesetzt
- Kraft-Ausreißer (ein Gang deutlich schwerer) → wiederholen lassen, Hinweis notieren
- Kraft in N = kg × 9,81. Zylinderkraft ≈ Druck × Kolbenfläche (Ø 20 ≈ 3,14 cm² → ~31 N pro bar, ausfahrend; einfahrend weniger wegen Kolbenstange). Schaltdruck ist **noch nicht festgelegt** (Filterregler FRY, Ventile/Zylinder max. 10 bar) — Reserve-Faktor mit dem User abstimmen, nicht als Fakt setzen

## Danach (erst nach OK des Users)
1. User schickt den Text aus „Ergebnis kopieren" + Fotos
2. Werte in `Druckluftanlage.md` eintragen (Maße-Abschnitt, „Offen" aktualisieren), Zylinderbox-Offenpunkte abhaken
3. Vorschlag Hub/Ø für G, A, B, K mit Begründung → User entscheidet → Airtec-Liste anpassen
4. Bei Logik-/Maßänderungen `viewer/sim.js` + Doku gemeinsam ändern, `node --test` in `viewer/`
5. Todo in `DOKUMENTATION.md` abhaken, Memory `pneumatic-shift-box` aktualisieren

## Regeln
- Keine Controller festlegen (siehe Memory `controller-inventory`)
- Vault: keine Massen-Edits, schreibt auf alle Geräte durch
- Unsichere Dinge als Hypothese kennzeichnen
