"""Order list for the pneumatic gear shift: Airtec (cylinders, valves and their accessories) + Riegler (everything else).

Replaces tools/airtec_request.py (price request of 2026-10-24). Usage:
    python3 tools/pneumatik_bestellliste.py out.pdf
    python3 tools/pneumatik_bestellliste.py --md
"""
import sys
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, KeepTogether

STAND = "18.11.2026"

FD = "/usr/share/fonts/truetype/dejavu/"
pdfmetrics.registerFont(TTFont("DV", FD + "DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("DVB", FD + "DejaVuSans-Bold.ttf"))

base = ParagraphStyle("b", fontName="DV", fontSize=9.5, leading=12.5)
small = ParagraphStyle("s", parent=base, fontSize=7.5, leading=9.5)
cell = ParagraphStyle("c", parent=base, fontSize=8, leading=10)
cellb = ParagraphStyle("cb", parent=cell, fontName="DVB")
h1 = ParagraphStyle("h1", parent=base, fontName="DVB", fontSize=15, leading=19, spaceAfter=4)
h2 = ParagraphStyle("h2", parent=base, fontName="DVB", fontSize=11, leading=14, spaceBefore=8, spaceAfter=4, keepWithNext=1)

P = lambda t, s=cell: Paragraph(t, s)

# (order no., description, qty, use, note)
BLOCKS = [
    ("Airtec – Zylinder und Zylinderzubehör", [
        ("HM-20-040", "Rundzylinder ISO 6432, Ø 20, doppeltwirkend, Positionserkennung, Hub 40", "2", "Gasse G, Reihe B", "Standardhub"),
        ("HM-25-040", "Rundzylinder ISO 6432, Ø 25, doppeltwirkend, Positionserkennung, Hub 40", "1", "Reihe A", "Standardhub; Nutzweg 28 mm über eigene Anschlaghülse"),
        ("RM-25", "Befestigungsmutter M22 × 1,5", "3", "Böcke", "nur falls beim HM keine beiliegt"),
        ("FK-32", "Ausgleichskupplung M10 × 1,25", "1", "A → Schlitten B", "Muttern im Lieferumfang?"),
        ("RD-20", "Gabelkopf M8 mit Bolzen Ø 8 + Sicherung", "2", "G, B → Seilauge", "Bolzen + Sicherung im Lieferumfang?"),
        ("RL-20", "Kolbenstangenmutter M8", "2", "Konter RD-20", ""),
        ("RL-25", "Kolbenstangenmutter M10 × 1,25", "1", "Konter FK-32", "+1, falls FK-32 ohne Mutter"),
        ("ZS-6701", "Zylinderschalter induktiv, PNP, 5–30 V DC, 0,3 m mit M8-Stecker", "4", "A, B je 2", ""),
        ("NT-550", "Spannbandhalterung für Zylinderschalter, Baureihe HM", "4", "A (Ø 25), B (Ø 20)", "passt für Ø 20 und Ø 25?"),
        ("KA-10-01", "Anschlusskabel M8-Buchse gerade, Überwurfmutter, 3-polig, 1 m", "4", "Sensoren", "passend zum Stecker ZS-6701?"),
    ]),
    ("Airtec – Ventile", [
        ("M-04-533-HN-441", "5/3-Wegeventil, Mitte entlüftet, G1/8, 12 V DC 4,2 W, Form B Industrienorm", "3", "Ventilstapel", "ohne Steckdose geliefert"),
        ("28-ST-01", "Steckdose Form B Industrienorm (22 mm, 11 mm), 0–250 V", "6", "je Spule 1", "Flachdichtung dabei? Nicht 28-ST-01-G (10 mm)"),
    ]),
    ("Camozzi – Steckverschraubungen Serie 6000 (Messing vernickelt, 16 bar), Drosseln, Rückschlagventil", [
        ("S6520 6-1/8", "Winkel-Steckverschraubung drehbar, G1/8 – Schlauch 6, Sprint (PTFE-Gewindebeschichtung)", "4", "Ventile G, A Anschl. 2/4", "gerade/Winkel nach Einbauprobe"),
        ("S6510 6-1/8", "Gerade Steckverschraubung, G1/8 – Schlauch 6, Sprint", "4", "Ventil B Anschl. 2/4 + 2 Reserve", ""),
        ("S6510 8-1/8", "Gerade Steckverschraubung, G1/8 – Schlauch 8, Sprint", "3", "Ventile Anschl. 1", "Versorgung durchgehend 8 mm"),
        ("6540 8", "T-Steckverbindung 3 × 8 mm", "2", "Verteilung auf 3 Ventile", ""),
        ("S6510 8-1/4", "Gerade Steckverschraubung, G1/4 – Schlauch 8, Sprint", "2", "Filterregler ein/aus", ""),
        ("6590 8", "Schott-Steckverbindung 8 mm, M16 × 1 mit Mutter (Wand bis 11,5 mm)", "1", "Rückwand Zylinderbox (1,5 mm)", "Bohrungsmaß?"),
        ("SCU 604-1/8", "Drosselrückschlagventil Hohlschraube G1/8, Abluftdrosselung, Schlitzverstellung, Messing", "6", "direkt in alle Zylinderanschlüsse", "2 nötig (G), 4 für A/B statt Winkel; 1–10 bar; nicht MCU/GSCU (zu hoch)"),
        ("6610 6-1/8", "Ringstück (Banjo) mit Steckanschluss 6 für SCU 604-1/8", "6", "zu SCU 604-1/8", "wird getrennt geliefert"),
        ("VNR-238-3/8", "Rückschlagventil G3/8 Innengewinde, Messing, max. 25 bar, NBR 0…80 °C", "1", "Kompressor → Tank", "nah am Tank montieren (Luft kühler); NBR bis 80 °C"),
        ("S6510 8-3/8", "Gerade Steckverschraubung, G3/8 – Schlauch 8, Sprint", "2", "an VNR-238-3/8", ""),
    ]),
    ("Riegler – Schalldämpfer, Aufbereitung", [
        ("113338 / 567-1", "Schalldämpfer Sinterbronze G1/8, mit Sechskant", "6", "Ventile Anschl. 3/5", "max. 8 bar"),
        ("C 06 K-10-HA", "Filterregler multifix-mini Gr. 0, G1/4, 0,5–10 bar, 5 µm, halbautom. Ablass, mit Manometer", "1", "vor der Zylinderbox, 7 bar", "Umgebung max. 60 °C"),
        ("MV 30", "Haltewinkel mit Mutter R 11-55 für multifix-mini", "1", "Wandmontage Filterregler", ""),
    ]),
    ("Riegler – Schlauch PA 12", [
        ("259.11 S-V2", "PA-Schlauch 6 × 4, schwarz (29 bar bei 23 °C, Biegeradius min. 35 mm)", "5 m", "Arbeitsleitungen", "Rolle 25 m, falls nicht als Meterware"),
        ("259.12 S-V2", "PA-Schlauch 8 × 6, schwarz (21 bar bei 23 °C, Biegeradius min. 40 mm)", "5 m", "Tank → Regler → Box", "Rolle 25 m, falls nicht als Meterware"),
    ]),
]

DROPPED = [
    ("44-XSCF-60", "Inline-Drossel → Drosseln direkt am Zylinder (Camozzi SCU)"),
    ("44-XCVPU-80, 40-501-18, blue C-line, PU-Schlauch, FRY", "→ Riegler"),
    ("RA-25, RB-25", "Böcke Eigenbau (Edelstahl) mit RM-25"),
    ("Reihenleiste, Schottverschraubungen 6 mm", "Ventile seit 11.11. als Stapel in der Zylinderbox"),
    ("PE-14-01", "Pressostat vorhanden"),
    ("Riegler »metallica«, Drossel 343.018-6", "→ Camozzi (Rat Händler 18.11.: robuster bei mechanischer Belastung)"),
]

QUESTIONS = [
    "Sind HM-20-040 und HM-25-040 ab Lager lieferbar? Liegt eine Mutter RM-25 bei?",
    "RD-20: Bolzen und Sicherung dabei? FK-32: Muttern dabei?",
    "Passen ZS-6701 und KA-10-01 zusammen (Stecker verschraubt oder Schnapp)? NT-550 für Ø 20 und Ø 25?",
    "Ventilspule 12 V DC 4,2 W, Toleranz laut Datenblatt ±10 %: verträgt sie im Fahrzeug Dauerbetrieb bei 14,4 V?",
    "Camozzi: Bohrungsmaß für Schott 6590 8 (geplant 16,5 mm)?",
    "Riegler: PA-Schlauch als Meterware?",
]


def numbered():
    n = 1
    for title, rows in BLOCKS:
        out = []
        for r in rows:
            out.append((n,) + r)
            n += 1
        yield title, out


def table(title, rows):
    data = [[P(h, cellb) for h in ("#", "Bestell-Nr.", "Bezeichnung", "Anz.", "Einsatz", "Anmerkung")]]
    for n, nr, desc, qty, use, note in rows:
        data.append([P(str(n)), P(nr, cellb), P(desc), P(qty), P(use, small), P(note, small)])
    t = Table(data, colWidths=[8*mm, 34*mm, 52*mm, 14*mm, 31*mm, 35*mm], repeatRows=1)
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#999999")),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e6ecf2")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f7f7f7")]),
        ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    return KeepTogether([Paragraph(title, h2), t])


def footer(c, doc):
    c.saveState()
    c.setFont("DV", 7.5)
    c.setFillColor(colors.grey)
    c.drawString(18*mm, 10*mm, f"Bestellliste Pneumatik Gangschaltung – Stand {STAND}")
    c.drawRightString(A4[0] - 18*mm, 10*mm, f"Seite {doc.page}")
    c.restoreState()


def build(out):
    doc = SimpleDocTemplate(out, pagesize=A4, leftMargin=18*mm, rightMargin=18*mm, topMargin=16*mm, bottomMargin=18*mm,
                            title="Bestellliste Pneumatik", author="")
    s = [Paragraph("Bestellliste Pneumatik – Gangschaltung Buggy", h1),
         Paragraph(f"Stand {STAND}. Airtec für Zylinder und Ventile samt Zubehör, Camozzi für Steckverschraubungen, Drosseln und "
                   "Rückschlagventil, Riegler für Aufbereitung und PA-Schlauch. Versorgung 7 bar über Filterregler, Tank bis 10,3 bar.", base),
         Spacer(1, 2*mm)]
    for title, rows in numbered():
        s.append(table(title, rows))
    s.append(Paragraph("Entfällt gegenüber der Anfrage vom 24.10.2026", h2))
    for nr, why in DROPPED:
        s.append(Paragraph(f"• <b>{nr}</b>: {why}", base))
    s.append(Paragraph("Fragen", h2))
    for i, q in enumerate(QUESTIONS, 1):
        s.append(Paragraph(f"{i}. {q}", base))
        s.append(Spacer(1, 1.2*mm))
    doc.build(s, onFirstPage=footer, onLaterPages=footer)


def markdown():
    out = []
    for title, rows in numbered():
        out += [f"**{title}**", "", "| # | Bestell-Nr. | Teil | Menge | Einsatz | Anmerkung |", "|---|---|---|---|---|---|"]
        out += [f"| {n} | {nr} | {desc} | {qty} | {use} | {note} |" for n, nr, desc, qty, use, note in rows]
        out.append("")
    out += ["**Entfällt gegenüber 24.10.:** " + " · ".join(f"{nr} ({why})" for nr, why in DROPPED), "",
            "**Fragen:**", ""] + [f"{i}. {q}" for i, q in enumerate(QUESTIONS, 1)]
    return "\n".join(out)


if __name__ == "__main__":
    if sys.argv[1] == "--md":
        print(markdown())
    else:
        build(sys.argv[1])
