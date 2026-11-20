# Lochraster-Steckplatinen für die Nucleo-F446RE (Relaisbox und Front-Controller)

Anleitung und Regeln aus der Session 2026-11-19/20 (Relaisbox-Controller, linke Seite CN7).
Zweck: Der Front-Controller (`left.py`, `right.py`, Viewer `viewer/nucleo-front.html`) soll in einer neuen Session
genauso umgebaut werden. Alles hier ist **Vorschlag Claude**, außer was als **Festgelegt (User, Datum)** markiert ist.

## 1. Rahmenbedingungen (vom User)

| Punkt | Wert | Quelle |
|---|---|---|
| Platinenvorrat | **10 × 6 × 8 cm**, doppelseitig, **22 × 27 Löcher** (Zahlen 1–22 quer, Buchstaben A–AA längs; korrigiert User 2026-11-20, vorher A–AB angenommen), Raster 2,54 mm (Amazon, Heevhas) · **1 × 4 × 6 cm**, **14 × 20 Löcher** (Buchstaben A–T, ohne Zahlen) | User 2026-11-19, Vault `Lager/Lagerbestand.md` „Lochrasterplatinen“ |
| Kleine Platine | nur 1 Stück → nehmen, wo sie für ein Design reicht (für den Front-Controller freigehalten); sonst große | User 2026-11-19 |
| Aufbau | **eine Platine je Morpho-Leiste** (CN7 links, CN10 rechts), von hinten auf die Stifte gesteckt, beide auf derselben Höhe; CN7 ↔ CN10 sind 26 Raster auseinander, eine 22er-Platine trägt nie beide | User 2026-11-19, gerechnet |
| Freie Höhe zwischen Platine und Nucleo | **9,3 mm** (Rückbestätigung durch den User noch offen) | User 2026-11-20 |
| Kabel | **keine Stecker auf der Platine, wo es eng ist:** Kabel direkt einlöten (auch unter dem Nucleo, Ausgang nach unten), Stecker/Buchse am Kabelende. Weiße JST-XH-Buchsen nur dort, wo sie außerhalb des Nucleo Platz haben und nichts verdrängen | User 2026-11-20 |
| Module | Pegelwandler und CAN-Modul **gesteckt** (Buchsenleisten) | User 2026-11-19 |
| Brücken | möglichst keine; eine Brücke war ok. „Steckbar über Stifte“ gewünscht, geht aber nicht unter dem Nucleo (Stift + Dupont ≈ 14 mm) → flache isolierte Drahtbrücke | User 2026-11-19, gemessen 9,3 mm |
| 100 nF an Eingängen | an Kl. 15, Batterie, Tank weggelassen (Firmware mittelt); an Masse-Eingängen schon vorher weg | Festgelegt User 2026-11-19 |
| Ziel | möglichst kurze Bahnen; was unter den Nucleo passt, dorthin | User 2026-11-20 |
| Heck rechts | CN10-Seite (Bündel A/B/C) ist **gebaut**, eigene Platine 3 Löcher nach innen, 4 nach außen, oben/unten frei → die linke Platine darf innen höchstens bis Spalte 34 (genutzt bis 26) | User 2026-11-19 |

**Höhen (Vorschlag Claude, ca. 1 mm Reserve → ≤ 8,3 mm unter dem Nucleo):** passt darunter: liegende R/Dioden, 1N5819, TVS liegend (5,3 mm), TO-220 flach liegend (4,5 mm), BC547 mit kurzen Beinen (≈ 7 mm), Elko liegend (6,3 mm), SIP-Netzwerk, Keramik-C, eingelötete Kabel. Passt nicht: stehende Widerstände (≈ 10 mm), Buchsenleisten für Module (8,5 mm), JST-Buchse mit Stecker.

## 2. Koordinaten

- Routergitter wie `router.py`: Spalte/Reihe in Löchern. `hdr(h, n)`: CN7 ungerade = Spalte 13 (außen), gerade = 14 (innen); CN10 ungerade = 38 (innen), gerade = 39 (außen); Pin 1 in Reihe 12, Pin 37/38 in Reihe 30.
- Nucleo-Umriss ≈ Spalte 12,2–39,8, Reihe −0,5–31,9. Alles links von Spalte 12 bzw. unterhalb Reihe 32 ist **außerhalb** (Höhe frei).
- Eine 22 × 27-Platine längs zur Leiste: 19 Reihen belegt die Leiste, es bleiben 8 Reihen oben/unten, quer 20 Spalten.
- **JST-Gehäuse brauchen Platz:** ca. 5,75 mm tief (≈ 2,3 Raster, Hypothese) → sie dürfen nicht direkt neben Bauteilbeinen oder Buchsenleisten sitzen (beim Heck deshalb J3/J7/J8 wieder als Kabel).
- Viewer zeigt Löcher als **Zahl|Buchstabe** wie auf der Platine (oben links = 1|A), `rowName()` in `viewer/nucleo-heck.html`.

## 3. Werkzeug

| Datei | Inhalt |
|---|---|
| `router.py` | Labyrinth-Router: verbindet die Anschlüsse jedes Netzes mit Lötbahnen (orthogonal), notfalls mit Drahtbrücken (`jumpers`); 1 Netz je Loch. `Board(name, cols, rows, header)`, `part()`, `pad()`, `preroute()`, `take()`, `route()`, `check()`, `ascii()` |
| `left.py`, `right.py`, `export.py` | Front-Controller (Stand 2026-11-18, noch auf den alten großen Maßen) |
| `heck_left.py` | Relaisbox CN7 auf 6 × 8 cm (Stand 2026-11-20) – **Vorlage für den Front-Umbau** |
| `heck_export.py` | routet, prüft (`check`), schreibt JSON für den Viewer |

Ablauf:
1. Platzieren in `build()`: Teile mit Löchern und Netzen, Pads (`b.pad(h, net, label, 'W', card)` = eingelötetes Kabel), JST (`jst(...)`, `vertical=True` für senkrecht), wichtige Bahnen als `preroute`.
2. `python3 heck_left.py` → ASCII-Karte, `jumpers`, `bad`. Ziel: `bad []` und nur gewollte Brücken. Jede ungewollte Brücke = Platzierung ändern, nicht hinnehmen.
3. `python3 heck_export.py <datei.json>` und den JSON-Inhalt als `const DATA = …;` in den Viewer einsetzen (eine Zeile).
4. Viewer im Headless-Browser prüfen (Playwright aus `~/.npm/_npx/*/node_modules/playwright`, Screenshot + Konsolenfehler), dann `node --test` in `viewer/`.

## 4. Muster, die funktioniert haben

1. **Kreuzungen über Bauteile lösen:** Ein liegendes Bauteil (4 Raster) springt über Bahnen darunter. Beispiele: R8 (RX-Reihenwiderstand) liegt über der 3,3-V-Bahn; die Längsteile der Eingänge liegen über der GND- und 3,3-V-Schiene.
2. **Eingänge als Treppe:** Pins Reihe 25–30 gehen nach rechts, der unterste biegt zuerst ab; je Eingang ein senkrechter Streifen, darunter zwei Schienen (GND, 3,3 V) quer unter den Längsteilen, Querteile senkrecht von der Signalbahn auf die Schiene.
3. **Pins umsortieren erlaubt** (Pinbelegung ist Vorschlag), damit Bahnen geordnet laufen. Heck: analoge Eingänge unten, digitale oben; danach `Relaisbox.md` und `relaistreiber.html` nachziehen. Beim Front: Pinliste in `Kabelbaum-Übersicht.md` + `Front.harness` mitziehen (Regel: Notiz + .harness + Viewer gleichzeitig).
4. **Kanal am Pegelwandler wählen**, sodass 3,3 V, GND und TX in der richtigen Reihenfolge ankommen (Heck: Kanal 4, LV4 liegt direkt neben PC10). Nur eine GND-Seite angeschlossen → LV-GND = HV-GND am Modul messen.
5. **Pins der äußeren Reihe** (CN7 ungerade) kommen nur nach außen heraus, Pins der inneren Reihe nur nach innen. Über das Leistenende (oben Reihe ≤ 11, unten ≥ 31) passen nur wenige Bahnen → was außen gebraucht wird, an äußere Pins legen.
6. **Widerstandsnetz (SIP)** für mehrere Pull-ups an nebeneinanderliegenden Bahnen; **dünne Dioden** (1N4148, DO-35), wenn Dioden im 2,54-mm-Raster nebeneinander liegen (1N5819 ist 3 mm dick).
7. **Stehende Bauteile außerhalb** des Nucleo (1 Raster Abstand) sparen viel Platz; Körper über dem äußeren Loch.
8. **Einpass-Test:** Die Einblendung „beste Lage einer 6 × 8-Platine über dem Plan“ (Commit `190cd93`) zeigt schnell, was übersteht.
9. **Plan B**, falls es nicht passt: Teilschaltung auf eine Hilfsplatine mit kurzem Kabel (Commit `d7e947b`); beim Heck am Ende nicht nötig, weil Stecker → Kabel Platz frei machte.

## 5. Viewer

`viewer/nucleo-heck.html` ist aus `viewer/nucleo-front.html` entstanden und kann zurückportiert werden. Zusätzliche Teiltypen: `TVS`, `TO92`, `TO220`, `MOD` (gesteckte Module), senkrechte `JST`, Pads `side: 'W'` (Kabel mit K-Nummer, Tabelle „Kabel und Stecker“ nach Ziel gruppiert über `CABLE`/`GROUP_ORDER`), Drahtbrücke mit `plug: true`, gebaute Gegenseite als Umriss. Karten (`CARDS`) = Schaltbild + Stückliste je Leitung, Stückliste summiert sich daraus.

## 6. Nach der Arbeit nachziehen

- Themen-Notiz (Heck: Vault `Buggy/Relaisbox.md` „Pinbelegung Nucleo-F446RE“; Front: `Buggy/Kabelbaum-Übersicht.md` „Pinbelegung Front-Controller“), `To-Do.md`, Session-Log in `DOKUMENTATION.md`.
- Verbrauchte/benötigte Teile im Vault `Lager/Lagerbestand.md` (Platinen, SIP, 1N4148, Buchsenleisten).
- Front: `WiringDiagram/harness.design/Front.harness`, wenn sich Pins oder Steckergruppen ändern.
- Commit auf `main` mit `git -c safe.directory=/srv/data/projects/buggy`, danach User um `git pull core main` + `git push origin main` am PC bitten.

## 7. Arbeitsweise (Rückmeldung des Users in dieser Session)

- Vor langen Rechen-/Routing-Runden einen Zwischenstand melden; nicht 20 Minuten nur planen.
- Erst routen und im Viewer zeigen, dann behaupten, dass etwas nicht passt („zeig mal im Viewer, wo es hakt“).
- Zahlen aus Diktat/Foto zurücklesen, bevor sie in den Vault kommen.
