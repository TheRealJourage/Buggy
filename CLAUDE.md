# Buggy — Projektregeln für Claude-Sessions

Hobby-Fahrzeugprojekt mit sicherheitsrelevanter Elektrik (12 V, VR6 AUE, pneumatische Schaltung, CAN, LIN).
Der User muss sich auf Grundaussagen verlassen können. Deutsch im Gespräch, Englisch in Code und Commits.
Hintergrund und Begründung aller Regeln: Vault `Buggy/Audit-2026-10-24.md`.

## Doku-Quelle (eine Datei je Thema)

Vault `/srv/data/obsidian/vault/Buggy/` auf core, am Windows-PC `C:\Users\ft-kr\Documents\ObsidianVault\Buggy\`
(synct auf alle Geräte, keine Massen-Edits). Zahlen stehen nur in der
genannten Datei; andere Notizen und Memory verlinken, statt zu wiederholen.

| Thema | Datei |
|---|---|
| Einstieg, Teilprojekte, Session-Log | `DOKUMENTATION.md` |
| Offene Aufgaben (nur Überblick) | `To-Do.md` |
| Bordnetz-Ebenen, Freigabe-Kette, Panels, Lenkrad-Funktionen, Hauptschalter | `Bordnetz-Konzept.md` |
| Kabelbaum-Struktur, Stecker X1–X6, Controller-Knoten, harness.design | `Kabelbaum-Übersicht.md` + `WiringDiagram/harness.design/{Heck,Front,Gesamt}.harness` |
| Ströme, Sicherungen, Hauptkabel | `Strombilanz.md` |
| Gesamt-Funktionsliste, TÜV-Licht, Widersprüche | `Funktionsliste.md` |
| Pneumatik: Logik, Messwerte, Airtec-Liste | `Druckluftanlage.md`; Mechanik/Beschaffung `Zylinderbox.md` |
| Kühlung K1–K14, L1–L5 | `Kühlkreislauf.md` |
| Speeduino-Pins | `Speeduino/Verkabelung.md` + `viewer/verkabelung.html` |
| Lenkrad (LIN, Versorgung, Schlüssel) | `SteeringControl/SteeringControl.md` |
| Vorhandene Teile / Controller | `Teile-Bestand.md` / `Controller-Inventar.md` |
| Golf-3-Elektrik (verbaut) | `ABS.md`, `Aussenspiegel.md` |

## Session-Start (immer)

1. `To-Do.md` und die Notiz des Themas lesen, dazu `Teile-Bestand.md` und `Controller-Inventar.md`.
2. Erste Antwort beginnt mit „Gelesen: …, Stand …" und nennt die festgelegten Punkte des Themas.
3. Ein Thema pro Session. Kommt ein zweites: als To-Do eintragen, neue Session vorschlagen.
4. Läuft parallel eine andere Session (Datei in den letzten Minuten geändert)? Dann nur die eigene Notiz anfassen.

## Festgelegt vs. Vorschlag

- Alles, was der User entscheidet, wird mit **Festgelegt (User, Datum)** in die Themen-Notiz geschrieben.
- Festgelegtes wird nie wieder als Frage gestellt. Sperrliste (Stand 2026-10-24):
  - Gänge über die großen Plus/Minus-Wippen, Reihenfolge R-1-2-3-4-5, N auf Lenkrad Schwarz. **Nie** Hall-Paddel zum Schalten.
  - Freigabe-Kette hält den Zustand: Lenkrad-Code nur beim Einschalten, Kl. 15 bleibt bis „Zündung aus", Kontaktverlust warnt nur.
  - Hauptsicherung Batterie → Relaisbox fehlt noch und kommt; Relaisbox mit Einzelsicherungen existiert.
  - Winde direkt an der Batterie, am Hauptschalter vorbei. Kompressor direkt ab Batterie mit eigenem Relais.
  - Bremslicht-Not-Leitung mit Dioden-ODER in der Relaisbox.
  - Warnblinker ohne Zündung über Schalter an der Relaisbox; mit Zündung gelbe Lenkradtaste.
  - Lüfter RIDEX 508R0028, PWM vom Relaisbox-Controller (Speeduino-Mega kann kein PWM, D47 nur Ein/Aus).
  - Ausgleichsbehälter nur am Kühlernippel. Front-Lampen LED außer Zusatz-Fernlicht. X1-Belegung wie gecrimpt 2026-10-19.
  - Pi ist die Brücke Speeduino ↔ CAN (Variante A). Heck-Controller = Relaisbox-Controller.
  - ABS an Kl. 15 (X2). Hub vorn über die kleine 5-V-Leitung aus dem Heck (große Schiene nur per Dioden-ODER parallel), X1/8 nicht für den Hub.
  - Hupe = Drucklufthorn, Megafon zusätzlich; R28-Horn für die Abnahme offen. Hauptschalter-Seite (Plus/Minus) bewusst offen, nicht nachfragen.
- Eigene Vorschläge heißen **Vorschlag Claude** und kommen nicht in „Offen"-Listen, solange der User sie nicht aufgreift.
- Unsicheres heißt **Hypothese**. Sicherheitsrelevante Zahlen (Querschnitt, Sicherung, Druck, Kraft, Strom) nur mit Quelle.
- Beim Durchsprechen von Funktionen nur fragen, was Leitung, Stecker, Sicherung oder Controller ändert. Software-Belegungen nicht neu diskutieren.

## Eingaben des Users

- Zahlen aus Fotos oder Diktat vor dem Eintragen zurücklesen: „Ich lese: …, richtig?" Erst nach Bestätigung schreiben.
- Fotos mit Daten (Typenschild, Beleg, Tafel) sofort in Text übertragen (`Teile-Bestand.md` o. Ä.). Bilder überleben die Session nicht.
- Vor jedem Kauf- oder Messvorschlag `Teile-Bestand.md` und `Controller-Inventar.md` prüfen; Standort (Halle/zu Hause) steht dort.
- Diktatfehler kennen: „Haul-Pedal" = Hall-Paddel, „Ken/Kähnen" = CAN, „Speedoino" = Speeduino, „Wafel/Tafel" = Messtafel.
- Vom Handy aus (Halle): kurze Antworten, ein Schritt pro Nachricht, Ergebnisse sofort eintragen, Viewer statt Fließtext.

## Schreiben in den Vault

- Nur mit dem Edit-Werkzeug nach vorherigem Lesen. Keine `python … s.replace()` ohne Treffer-Prüfung.
- Jede Änderung an Steckern, Pins, Leitungen oder Farben gleichzeitig in Notiz, `.harness`-Datei und Viewer (Regel des Users).
- „Stand:"-Zeile mitpflegen oder weglassen.
- Session-Ende: Ergebnis in die Themen-Notiz, `To-Do.md` aktualisieren, eine Zeile im Session-Log von `DOKUMENTATION.md`, Memory nur als Wegweiser (keine Zahlen).

## Kontext-Hygiene

- Agent-Berichte und lange Recherchen nicht in den Hauptkontext kippen: in Datei schreiben, nur Kurzfassung lesen.
- Viele Bilder oder mehrere Stunden Laufzeit → Ergebnis sichern und dem User eine neue Session vorschlagen.

## Repo

- Git nur mit `git -c safe.directory=/srv/data/projects/buggy …` (Repo gehört ftk). Commits direkt auf `main`.
- **core kann nicht pushen.** Nach Commits auf core den User bitten, am PC `git pull core main` + `git push origin main` zu machen. Nach Arbeit am PC auf core `git fetch origin` + `git merge origin/main`.
- `srcTest/` (Firmware) wird auf dem Windows-PC entwickelt und getestet; die core-Kopie ist nur Spiegel. `lib/steering_key/key_secret.h` nie committen.
- Viewer: Port 8112 (`podman-compose`, `viewer/`), Tests `node --test` in `viewer/`. Schaltlogik in `sim.js` immer zusammen mit `Druckluftanlage.md` ändern.
- `tools/harness_front_v3/v4/v5.py` sind einmalige Migrationen und laufen nicht erneut; wiederholbar ist nur `harness_layout.py`. Quelle sind die `.harness`-Dateien.
- Keine Fremd-PDFs und keine Build-Ordner ins Git.
