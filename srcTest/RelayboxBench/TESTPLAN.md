# Bench-Test Relaisbox-Controller (Nucleo-F446RE)

Für den User und die Claude-Session am Windows-PC. Stand 2026-11-21. Firmware auf core gebaut und kompiliert
(`ststm32@20.0.0`, STM32-Arduino-Core 3.x), **noch nie auf Hardware gelaufen**. Alles hier ist **Vorschlag Claude**;
Werte gerechnet, nicht gemessen.

## 1. Worum es geht

Der Relaisbox-Controller ist ein Nucleo-F446RE mit zwei Steckplatinen von unten:

- **CN10 (rechts):** 21 Adern (Bündel A/B/C) zur Treiberplatine mit 3 × ULN2003AN → Relais. Gebaut 2026-11-19.
- **CN7 (links):** Lochrasterplatine 6 × 8 cm, gebaut 2026-11-20 nach `viewer/nucleo-heck.html`
  (http://dev.home/v/buggy/nucleo-heck.html, im Repo `viewer/nucleo-heck.html`). Darauf: Pegelwandler (Serial3 zum
  Speeduino), Lüfter-PWM-Stufe (BC547B), ZL180-Treiber (IRLZ44N), 6 geschützte Eingänge, CAN-Modul SN65HVD230.

Quellen (bei Fragen lesen, Zahlen nur dort):

| Was | Wo |
|---|---|
| Pinbelegung, Festlegungen, Abweichungen beim Bau | Vault `Buggy/Relaisbox.md` „Pinbelegung Nucleo-F446RE“ (PC: `C:\Users\ft-kr\Documents\ObsidianVault\Buggy\Relaisbox.md`) |
| Lötplan, Bauteile, Kabel K1–K17, Stecker wie gebaut | `viewer/nucleo-heck.html` (Tabelle „Kabel und Stecker“, Schaltbild-Karten) |
| Wie die Platine entworfen wurde | `tools/perfboard/README.md` |
| Offene Aufgaben | Vault `Buggy/To-Do.md` Punkt 1a |

Projektregeln: `CLAUDE.md` im Repo (Deutsch im Gespräch, Festgelegtes nicht neu fragen, Zahlen zurücklesen, Ergebnisse
in die Themen-Notiz). **Diese Firmware ist nur ein Test.** Pinbelegung nicht ändern, ohne `Relaisbox.md` mitzuziehen.

## 2. Pins (wie in `src/main.cpp`)

| Funktion | Pin | Stecker/Kabel wie gebaut |
|---|---|---|
| Relais A1–A7 (lila…grau) | PA9, PA8, PB10, PB4, PB5, PB3, PA10 | Bündel A an CN10 |
| Relais B1–B7 | PB12, PB2, PB1, PB15, PB14, PB13, PC4 | Bündel B |
| Relais C1–C7 | PB8, PB9, PC5, PA6, PA7, PB6, PA5 (C7 = grüne LED LD2 leuchtet mit) | Bündel C |
| Serial3 TX → Speeduino D15 | PC10 → Pegelwandler Kanal 4 | J3 (2-pol.) |
| Serial3 RX ← Speeduino D14 | PC11 ← 10 kΩ (R8) | J3 |
| Lüfter-PWM 25 Hz | PA15 → BC547B → Leitung mit 2 × 2 kΩ an Kl. 15 | J1 (2-pol.: Kl. 15, PWM) |
| ZL180-Spule Low-Side | PB7 → IRLZ44N | J2 (3-pol.): 1 Source, 2 Spule (−), 3 GND |
| Speeduino D47 / Zündschalter / Warnblinker | PA0 / PA1 / PA4, je Pull-up + Diode | J6 Pin 6 / 5 / 4 |
| Kl.-15-Rückmeldung / Batterie | PB0 / PC1, Teiler 10 kΩ / R14 bzw. R12 | J6 Pin 3 / 2 |
| Tankgeber | PC0, 1 kΩ Pull-up + 1 kΩ in Reihe | J6 Pin 1 (+ K10 GND) |
| CAN TX / RX | PA12 / PA11 (Drähte K16/K17 von CN10-12/-14) | J5 (2-pol.: CAN-H, CAN-L) |
| 5-V-Einspeisung | E5V (CN7-6) | K4 (+5 V) / K5 (GND), noch offen |

**Pin-Reihenfolge von J1, J3 und J5 ist nicht dokumentiert** → vor dem Test mit dem Durchgangsprüfer bestimmen und
in `Relaisbox.md` eintragen.

## 3. Sicherheit und Vorbedingungen (Pflicht)

1. **Ohne Strom durchmessen (Durchgangsprüfer):** 3,3 V–GND, 5 V(E5V)–GND, E5V–3,3 V dürfen nicht piepen.
   CN7-6 ↔ HV-Pin des Pegelwandlers muss piepen, CN7-6 ↔ LV / LV4 / HV4 nicht.
   LV-GND ↔ HV-GND am Pegelwandler muss piepen (nur HV-GND ist angeschlossen).
   Metallfahne Q2 (Drain) gegen GND: kein Durchgang.
2. **Keine Lasten an den Relais** beim ersten Test. 12 V nur über ein Labornetzteil mit Strombegrenzung (z. B. 0,5 A,
   für Test 7 mehr).
3. **Stromversorgung des Nucleo:** Für Test 1–3 und 5–8 reicht USB (Jumper JP5 in Werkseinstellung). Für **Test 4
   (Serial3)** braucht die HV-Seite des Pegelwandlers 5 V an E5V: kleine 5-V-Quelle an K4/K5 und JP5 auf E5V.
   Reihenfolge und Jumper **vorher in UM1724 (Kapitel Stromversorgung, JP5/E5V) nachlesen** — nicht geprüft, ob E5V bei
   USB-Versorgung Spannung führt.
4. Alle Massen verbinden: Labornetzteil-GND an die Platinen-GND (z. B. J2 Pin 3).
5. Firmware startet mit allen Ausgängen aus. Befehl `a` schaltet jederzeit alles ab.

## 4. Bauen, flashen, Monitor

```
cd srcTest/RelayboxBench
pio run                      # baut .pio/build/bench/firmware.bin
pio run -t upload            # oder firmware.bin auf das Laufwerk NUCLEO ziehen
pio device monitor           # 115200 Baud, Enter schickt die Zeile
```

Nach dem Start erscheint die Befehlsliste (`h`).

## 5. Tests

Ergebnisse je Test in die Tabelle in Abschnitt 6 eintragen.

| # | Test | Aufbau | Befehl | Erwartet (gerechnet) | Bestanden wenn |
|---|---|---|---|---|---|
| 1 | Start | USB | – | Befehlsliste erscheint, keine Relais an | Text kommt, nichts schaltet |
| 2 | Digitaleingänge | J6 Pin 4/5/6 offen, dann einzeln gegen GND | `w` | offen = HIGH(offen), gegen GND = LOW(Masse) | alle drei richtig |
| 2b | 12 V an einem Digitaleingang (optional) | 12 V über das Netzteil an J6 Pin 6 | `w` | bleibt HIGH; Spannung an PA0 ≤ 3,4 V | kein LOW, Pin ≤ 3,4 V |
| 3 | Analog Kl. 15 / Batterie | Netzteil 12,0 V an J6 Pin 3 bzw. Pin 2 | `i` | Pin 2,00 V → 12,0 V (Faktor 6, R12/R14 = 2 kΩ). Bei 1 kΩ: `k 11`, Pin 1,09 V | ± 3 % |
| 3b | Tankgeber | J6 Pin 1 offen, dann 100 Ω und 190 Ω gegen GND | `i` | offen → „offen“; 100 Ω → Pin 0,30 V ≈ 100 Ω; 190 Ω → 0,53 V ≈ 190 Ω | ± 10 % |
| 4 | Serial3-Schleife | 5 V an E5V (s. Abschnitt 3), Brücke D14 ↔ D15 an J3 | `s` | 9600 und 115200 Baud: 256/256 | beide 256/256 |
| 5 | Lüfter-PWM | Netzteil 12 V an J1 Kl. 15, GND verbunden, Multimeter an J1 PWM | `p off`, `p 100`, `p 50` | off ≈ 12 V; 100 ≈ 0–0,3 V; 50 ≈ 6 V (Mittelwert) | Werte ± 10 % |
| 6 | ZL180-Treiber | Last (12-V-Lampe oder Relaisspule) von +12 V an J2 Pin 2, J2 Pin 1 an Netzteil-GND | `z 1`, `z 0` | Last an/aus. U(Pin 2 → Pin 1) bei an < 0,1 V (Hypothese IRLZ44N an 3,3 V Gate) | Last schaltet; U_DS notieren |
| 7 | Relaiskanäle | Treiberplatine an, COM mit 12 V, Relais ohne Lasten | `rs` | Relais klicken einzeln A1 … C7 | jeder Kanal klickt; **Sockel-Nr. je Kanal notieren** |
| 7b | Wärme ULN (optional) | wie 7, alle an 5 min | `r all 1`, danach `a` | Chiptemperatur notieren; To-Do: > 50 °C → vierter Chip | Temperatur notiert |
| 8 | CAN intern | USB | `c l` | 5 Frames empfangen, TEC=0 | 5/5 |
| 8b | CAN Modul senden | USB, kein zweiter Knoten | `c n` | LEC=3 (ACK fehlt), TEC steigt → Modul sendet | LEC=3 |
| 8c | CAN Pegel | stromlos: Widerstand CAN-H ↔ CAN-L; eingeschaltet: Spannung je Leitung gegen GND | – | ≈ 120 Ω (R2 auf dem Modul); Ruhepegel je ca. 2–2,5 V (Hypothese) | notiert |

Wenn etwas nicht stimmt: messen, nicht raten. Typische Ursachen stehen in der Karte der jeweiligen Schaltung im Viewer.
Ursache nur nennen, wenn gemessen; sonst als Hypothese kennzeichnen.

## 6. Ergebnis-Tabelle (ausfüllen)

| # | Ergebnis | Messwerte | Bemerkung |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | Faktor: | R12/R14 = ? Ω |
| 3b | | | |
| 4 | | | |
| 5 | | | |
| 6 | | U_DS = | Last: |
| 7 | | Kanal → Sockel: | |
| 8 | | | |

## 7. Danach dokumentieren

1. Ergebnisse (Tabelle 6) und die Pin-Reihenfolge von J1/J3/J5 in Vault `Buggy/Relaisbox.md`, Abschnitt
   „Gebaut (User, 2026-11-20)“ ergänzen; Kanal → Sockel aus Test 7 ebenfalls dort.
2. `To-Do.md` Punkt 1a abhaken bzw. offene Punkte nennen; eine Zeile ins Session-Log `DOKUMENTATION.md`.
3. Teile, die ersetzt wurden (z. B. R12/R14), im Vault `Lager/Lagerbestand.md` nachziehen.
4. Commit auf `main` am PC, `git push origin main`; core holt per `git fetch origin` + `git merge origin/main`.

## 8. Offene Hypothesen, die der Test klären soll

- IRLZ44N schaltet mit 3,3 V am Gate voll durch (Test 6, U_DS).
- LV-GND und HV-GND am Pegelwandler sind auf dem Modul verbunden (Abschnitt 3).
- Pinfolge des Pegelwandlers (LV1 LV2 LV GND LV3 LV4) und des CAN-Moduls (3V3 GND CTX CRX CANH CANL) wie angenommen
  (Test 4 und 8b).
- Teilerfaktor hängt an R12/R14 (2 kΩ → 6, 1 kΩ → 11).
