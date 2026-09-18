# Wokwi-Simulation SteeringControl

Wokwi lässt nur **eine Firmware pro Simulation** zu. Deshalb zwei Projekte:

| Projekt | Echte Firmware | Gegenstelle (Custom Chip) |
|---|---|---|
| `simA-master/` | Uno-Master (Commit `65c6823`) | `lin-slave` – korrekter Slave, Werte per Schieberegler |
| `simB-slave/` | RP2040-Slave auf Pi Pico (Commit `65c6823`) | `lin-master` – korrekter Master, verwirft sein Echo |

Beide nutzen `chips/mcp2003.*` als Transceiver-Modell (POR unter 5,5 V, Echo auf RXD, Open-Drain-RXD,
Pull-up nur im Operation-Mode, RXD-Monitoring). Analoge Effekte (Kondensatoren, Pegel, Flanken) sind nicht simuliert.

## Einrichten auf wokwi.com (je Projekt einmal)

1. Neues Projekt anlegen:
   - Sim A: https://wokwi.com/projects/new/arduino-uno
   - Sim B: https://wokwi.com/projects/new/pi-pico
2. Custom Chips anlegen: im Diagramm auf das blaue **+** → **Custom Chip** → Name eingeben, Sprache **C** → *Create Chip*.
   - Sim A: `mcp2003` und `lin-slave`
   - Sim B: `mcp2003` und `lin-master`
3. In den neu entstandenen Dateien den Inhalt komplett ersetzen durch die Dateien aus `chips/`
   (`mcp2003.chip.json`, `mcp2003.chip.c`, usw.).
4. `sketch.ino` komplett ersetzen durch `simA-master/sketch.ino` bzw. `simB-slave/sketch.ino`.
5. **Zuletzt** `diagram.json` komplett ersetzen durch `simA-master/diagram.json` bzw. `simB-slave/diagram.json`.
6. Grün ▶ starten. Unter dem Diagramm erscheint der Tab **Chips Console** mit den Logzeilen.
7. Speichern (Account nötig), dann ist das Projekt per Link wieder erreichbar.

## Was man sehen sollte

### Sim A (echter Master)
- `[MCP2003 #1/#2] ... -> READY`, dann `-> OPERATION` sobald CS high geht.
- `[LIN slave] request #n answered: ...` → der Master fragt korrekt an, der Slave antwortet.
- **Rote LED D13 blinkt dauerhaft, grüne D12 toggelt nicht** → Verdacht *Echo-Bug* bestätigt
  (Master liest sein eigenes `55 3C` als Antwort-Bytes 0–1 → Checksumme falsch).

### Sim B (echter Slave)
- `[LIN master] after N polls: ok=… timeout=… checksum=…` im Sekundentakt.
- **timeout ≈ ok** (etwa jede zweite Anfrage) → Verdacht *blockierendes `delay(100)`* bestätigt.
- Taster drücken / Potis drehen → `[LIN master] BTN=1------ POT1=… POT2=…`.

### Gegenprobe Spannung
- Bei einem MCP2003 den Schieberegler **VBB** auf 5 V stellen (Chip anklicken) bzw. im `diagram.json`
  `"vbb": "5"` setzen → der Chip bleibt in `POR (VBB too low)`, keine Kommunikation.

## Grenzen
- Pi Pico statt RP2040-Tiny: gleicher Chip, gleiche GPIO-Nummern; die LED auf GP25 gibt es nur am Pico.
- Kein 12-V-Netz in Wokwi: `vbb_rail` ist ein logisches High, die Spannung kommt aus dem `vbb`-Attribut.
- Diode im Master-Pull-up und 220-pF-Kondensatoren sind weggelassen (digitale Simulation).
