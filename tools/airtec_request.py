import sys
from reportlab.lib import colors
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, KeepTogether

FD = "/usr/share/fonts/truetype/dejavu/"
pdfmetrics.registerFont(TTFont("DV", FD + "DejaVuSans.ttf"))
pdfmetrics.registerFont(TTFont("DVB", FD + "DejaVuSans-Bold.ttf"))

base = ParagraphStyle("b", fontName="DV", fontSize=9.5, leading=12.5)
small = ParagraphStyle("s", parent=base, fontSize=8, leading=10)
cell = ParagraphStyle("c", parent=base, fontSize=8, leading=10)
cellb = ParagraphStyle("cb", parent=cell, fontName="DVB")
h1 = ParagraphStyle("h1", parent=base, fontName="DVB", fontSize=15, leading=19, spaceAfter=4)
h2 = ParagraphStyle("h2", parent=base, fontName="DVB", fontSize=11, leading=14, spaceBefore=8, spaceAfter=4)

P = lambda t, s=cell: Paragraph(t, s)

SECTIONS = [
    ("A – Zylinder und Befestigung", [
        ("HM-20-096*", "Rundzylinder ISO 6432, Ø 20, doppeltwirkend, Positionserkennung, Hub ca. 96 mm", "1", "Sonderhub, Hub wird nach Messung bestätigt"),
        ("HM-20-014*", "Rundzylinder ISO 6432, Ø 20, doppeltwirkend, Positionserkennung, Hub ca. 14 mm", "1", "Sonderhub, Hub wird nach Messung bestätigt"),
        ("HM-20-034*", "Rundzylinder ISO 6432, Ø 20, doppeltwirkend, Positionserkennung, Hub ca. 34 mm", "1", "Sonderhub, Hub wird nach Messung bestätigt"),
        ("RA-25", "Fußbefestigung für Ø 20/25", "4", ""),
        ("RB-25", "Flanschplatte für Ø 20/25", "1", ""),
        ("FK-20", "Ausgleichskupplung M8", "1", ""),
        ("RD-20", "Gabelkopf M8", "2", "Bitte angeben, ob Bolzen enthalten ist"),
        ("RL-20", "Kolbenstangenmutter M8", "3", ""),
        ("ZS-6701", "Zylinderschalter 3-Leiter PNP, 0,3 m mit M8-Stecker", "4", ""),
        ("NT-550", "Spannbandhalterung für Zylinderschalter (Baureihe HM)", "4", ""),
        ("KA-10-01", "Anschlusskabel M8, Buchse gerade, 3-polig, 1 m", "4", ""),
        ("44-XSCF-60", "Drosselrückschlagventil, 2 × Steckanschluss 6 mm", "2", ""),
    ]),
    ("B – Ventile", [
        ("M-04-533-HN-441", "5/3-Wegeventil, Mittelstellung entlüftet, G1/8, 12 V DC 4,2 W", "3", ""),
        ("28-ST-01", "Steckdose Form B Industrienorm", "6", ""),
        ("Reihenleiste M-04", "Reihenleiste für Baureihe M-04, 3-fach", "1", "Bitte passenden Typ angeben (Katalog S. 4-10)"),
        ("40-501-18", "Schalldämpfer G1/8", "6", "entfällt ggf. bei Reihenleiste mit Sammelabluft"),
    ]),
    ("C – Verschlauchung (blue C-line)", [
        ("blue C-line", "Steckverschraubung gerade, G1/8 auf Schlauch 6 mm", "12", "Bitte passenden Typ angeben"),
        ("blue C-line", "Schottverschraubung Schlauch 6 mm", "12", "Bitte passenden Typ angeben"),
        ("blue C-line", "Schottverschraubung Schlauch 8 mm", "1", "Bitte passenden Typ angeben"),
        ("—", "PU-Schlauch 6 × 4 mm", "6 m", "falls im Programm"),
        ("—", "PU-Schlauch 8 × 6 mm", "3 m", "falls im Programm"),
    ]),
    ("D – Druckluftaufbereitung und Versorgung", [
        ("FRY", "Filterregler mit Manometer, Anschluss G1/4, Regelbereich bis 10 bar", "1", "Bitte passenden Typ angeben"),
        ("44-XCVPU-80", "Rückschlagventil, Steckanschluss 8 mm", "1", ""),
        ("PE-14-01", "Druckschalter 1–12 bar, Wechsler, G1/4", "1", ""),
    ]),
]
OPTION = ("E – Option (bitte separat ausweisen)", [
    ("HM-20-050", "Rundzylinder ISO 6432, Ø 20, Hub 50 mm", "1", "Option Kupplungsbetätigung"),
    ("M-04-511-HN-441", "5/2-Wegeventil monostabil, mech. Feder, G1/8, 12 V DC 4,2 W", "1", ""),
    ("28-ST-01", "Steckdose Form B Industrienorm", "1", ""),
    ("RD-20", "Gabelkopf M8", "1", ""),
    ("RA-25", "Fußbefestigung für Ø 20/25", "2", ""),
])

QUESTIONS = [
    "Sind die Zylinder HM-20 mit Sonderhub (ca. 14 / 34 / 96 mm) lieferbar, und zu welchem Aufpreis? Alternativ bitte Preis für den nächstgrößeren Standardhub (25 / 40 / 100 mm).",
    "Ist bei RD-20 ein Bolzen (Ø 8 mm) im Lieferumfang?",
    "Welche Reihenleiste passt für 3 Ventile M-04 (gemeinsame Versorgung, ggf. Sammelabluft)?",
    "Welche Typen der blue C-line passen für die Positionen in Abschnitt C?",
    "Welcher Filterregler der Baureihe FRY mit Manometer passt für G1/4 und bis 10 bar?",
    "Lieferzeit, Versandkosten und Mindestbestellwert für Privat-/Einzelbestellung?",
]


def section_table(title, rows, start):
    data = [[P("Pos", cellb), P("Bestell-Nr.", cellb), P("Bezeichnung", cellb), P("Anz.", cellb),
             P("Anmerkung", cellb), P("Preis/St.", cellb), P("Gesamt", cellb)]]
    for i, (nr, desc, qty, note) in enumerate(rows):
        data.append([P(str(start + i)), P(nr, cellb), P(desc), P(qty), P(note, small), "", ""])
    widths = [10*mm, 34*mm, 48*mm, 12*mm, 34*mm, 18*mm, 18*mm]
    t = Table(data, colWidths=widths, repeatRows=1)
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#999999")),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e6ecf2")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f7f7f7")]),
        ("TOPPADDING", (0, 0), (-1, -1), 3), ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    return [Paragraph(title, h2), t], start + len(rows)


def footer(c, doc):
    c.saveState()
    c.setFont("DV", 7.5)
    c.setFillColor(colors.grey)
    c.drawString(18*mm, 10*mm, "Preisanfrage Pneumatikkomponenten – Stand 14.10.2026")
    c.drawRightString(A4[0] - 18*mm, 10*mm, f"Seite {doc.page}")
    c.restoreState()


def build(out):
    doc = SimpleDocTemplate(out, pagesize=A4, leftMargin=18*mm, rightMargin=18*mm, topMargin=16*mm, bottomMargin=18*mm,
                            title="Preisanfrage Airtec", author="")
    s = []
    sender = Table([[P("<b>Absender</b><br/>Name: ______________________________<br/>Anschrift: __________________________<br/>"
                       "______________________________________<br/>E-Mail: ____________________________<br/>Telefon: ___________________________", base),
                     P("<b>An</b><br/>AIRTEC Pneumatic GmbH<br/>Vertrieb / Angebotsabteilung<br/><br/>Datum: 14.10.2026", base)]],
                   colWidths=[95*mm, 79*mm])
    sender.setStyle(TableStyle([("VALIGN", (0, 0), (-1, -1), "TOP"), ("LEFTPADDING", (0, 0), (-1, -1), 0)]))
    s += [sender, Spacer(1, 8*mm)]
    s.append(Paragraph("Preisanfrage – Pneumatikkomponenten", h1))
    s.append(Paragraph(
        "Sehr geehrte Damen und Herren,<br/><br/>"
        "für ein Einzelprojekt (pneumatische Gangschaltung, Prototyp) bitte ich um ein Angebot für die unten aufgeführten "
        "Artikel aus Ihrem Katalog. Bitte tragen Sie Einzel- und Gesamtpreise ein bzw. senden Sie ein separates Angebot. "
        "Wo „Bitte passenden Typ angeben“ steht, bin ich für einen Vorschlag mit Bestellnummer dankbar. "
        "Die mit * markierten Zylinderhübe sind vorläufig und werden vor der Bestellung bestätigt.", base))
    s.append(Spacer(1, 3*mm))
    pos = 1
    for title, rows in SECTIONS:
        block, pos = section_table(title, rows, pos)
        s.append(KeepTogether(block))
    block, pos = section_table(*OPTION, pos)
    s.append(KeepTogether(block))

    s.append(Paragraph("Fragen zum Angebot", h2))
    for i, q in enumerate(QUESTIONS, 1):
        s.append(Paragraph(f"{i}. {q}", base))
        s.append(Spacer(1, 1.5*mm))
    tot = Table([[P("<b>Summe netto (A–D):</b>", base), "", P("<b>Option E netto:</b>", base), ""],
                 [P("Versandkosten:", base), "", P("Lieferzeit:", base), ""]],
                colWidths=[42*mm, 45*mm, 42*mm, 45*mm])
    tot.setStyle(TableStyle([("GRID", (0, 0), (-1, -1), 0.4, colors.HexColor("#999999")),
                             ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 6)]))
    s += [Spacer(1, 4*mm), tot, Spacer(1, 6*mm),
          Paragraph("Vielen Dank im Voraus. Mit freundlichen Grüßen<br/><br/><br/>______________________________", base)]
    doc.build(s, onFirstPage=footer, onLaterPages=footer)


if __name__ == "__main__":
    build(sys.argv[1])
