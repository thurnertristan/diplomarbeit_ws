# Sicherheit

Diese Übersicht fasst zusammen, welche Sicherheitsvorkehrungen für den
Biegeroboter (deckenmontierter Fairino FR20 + eigenkonstruierter Biegekopf
aus `diplomarbeit_dateien/Mechanik/Biegekopf`) nötig sind, um mit den in
Österreich geltenden Vorschriften übereinzustimmen. Ersetzt keine förmliche
Risikobeurteilung durch eine Fachkraft für Arbeitssicherheit - dient als
Ausgangspunkt/Checkliste dafür.

## Rechtsrahmen

- **Maschinenrichtlinie 2006/42/EG**, national umgesetzt - ab **20.1.2027**
  abgelöst durch die unmittelbar geltende **EU-Maschinenverordnung
  (EU) 2023/1230**.
- **ArbeitnehmerInnenschutzgesetz (ASchG)** + **Arbeitsmittelverordnung
  (AM-VO)**: gelten sobald mit dem Roboter an einer österreichischen
  Institution gearbeitet wird - Grundpflicht: dokumentierte
  Gefahrenermittlung/-evaluierung, 10 Jahre aufzubewahren.
- Wer Roboter + eigenkonstruierten Biegekopf zu einer Anlage zusammenbaut,
  gilt rechtlich als **Hersteller/Inverkehrbringer einer neuen Maschine**
  und muss selbst CE-kennzeichnen, technische Dokumentation +
  Betriebsanleitung erstellen. Die Cobot-Zertifizierung des FR20 allein
  deckt den Biegekopf **nicht** ab.

## Einschlägige Normen

| Norm | Zweck |
|---|---|
| ÖNORM EN ISO 12100 | Basisnorm Risikobeurteilung/-minderung |
| ÖNORM EN ISO 10218-1/-2 | Sicherheitsanforderungen Industrieroboter (Teil 1: Roboter, Teil 2: Integration/Anlage) |
| ISO/TS 15066 | Grenzwerte für Mensch-Roboter-Kollaboration (Kraft/Druck je Körperregion, 4 Kollaborationsarten) |
| ÖNORM EN ISO 13849-1 | Sicherheitsbezogene Steuerungsteile - für Kollaborationsbetrieb i. d. R. Kategorie 3 / PL d gefordert |
| ÖNORM EN 60204-1 | Elektrische Ausrüstung, Not-Halt-Kategorien 0/1 |
| ÖNORM EN ISO 13857 | Sicherheitsabstände gegen Erreichen von Gefahrstellen (Quetsch-/Schergefahren am Biegekopf) |
| ÖNORM EN ISO 13855 | Mindestabstand von Schutzeinrichtungen (Laserscanner, Lichtvorhang) zur Gefahrstelle |
| AUVA M 940 | Praxismerkblatt "Arbeiten mit kollaborativen Robotern" |

## Zentraler Punkt: der Biegekopf bricht die Kollaborationsfähigkeit

Die Fairino-FR20-Zertifizierung (10218/13849/15066) gilt für den Arm
allein, ohne Werkzeug, meist nur innerhalb bestimmter
Geschwindigkeits-/Nutzlastfenster. Der Biegekopf hat einen eigenen Motor,
Planetengetriebe und Kupplung - klassische Quetsch- und Scherstellen, die
ISO/TS 15066 nicht abdeckt. Außerdem liegt 20 kg Traglast nahe an oder über
den biomechanischen Schmerzgrenzwerten aus ISO/TS 15066 Anhang A, sodass
Power-and-Force-Limiting (PFL, "schutzzaunloser" Betrieb) bei voller
Last/Geschwindigkeit oft gar nicht mehr zulässig ist. Ob die Gesamtanlage
schutzzaunlos betrieben werden darf oder Geschwindigkeits-/
Abstandsüberwachung bzw. einen klassischen Schutzzaun braucht, entscheidet
**nur** die Risikobeurteilung der fertigen Anlage - nicht die Datenblätter
der Einzelkomponenten.

## Konkrete Sicherheitsvorkehrungen

1. **Risikobeurteilung nach EN ISO 12100** für die Gesamtanlage (Arm +
   Biegekopf + Werkstückaufnahme + Tisch), schriftlich, 10 Jahre
   aufzubewahren - Basis für alles Folgende.
2. **Not-Halt** (Kategorie 0 oder 1 nach EN 60204-1), hardwareseitig, gut
   erreichbar an Bedienstand und Gefahrenbereich - aktuell im Code
   (`fr20_ceiling_gazebo.py`, `robot_program.py`, `gui/my_gui.py`) nicht
   vorhanden, da bisher reine Gazebo-Simulation. Muss vor Realbetrieb
   nachgerüstet werden.
3. **Handbetrieb/Jog abgesichert**: Die Jog-Buttons in `gui/my_gui.py`
   entsprechen funktional einem Teach-Modus - dafür braucht es einen
   Zustimm-/Freigabetaster (3-Stufen-Schalter) und Geschwindigkeitsbegrenzung
   auf ≤ 250 mm/s im Handbetrieb (T1-Modus nach EN ISO 10218-1), nicht nur
   die aktuelle Software-Schrittweite (`JOG_STEP_M`).
4. **Schutz gegen Quetsch-/Scherstellen am Biegekopf** (Getriebe/Kupplung):
   feste Verdeckung oder Sicherheitsabstände nach EN ISO 13857.
5. **Absicherung des Wirkbereichs**, falls PFL nicht nachweisbar ist:
   Schutzzaun, Lichtvorhang oder Sicherheits-Laserscanner mit
   Geschwindigkeits-/Abstandsüberwachung (Kat. 3/PL d nach
   EN ISO 13849-1), gekoppelt an die Robotersteuerung (siehe Abschnitt
   unten).
6. **Sicherheitsbezogene Steuerung PL d/Kat. 3**: Die Software-Grenzen im
   aktuellen Code (Tischkollisionsprüfung `_is_transition_safe`,
   Gelenkgrenzen in `fr20_kinematics.inverse_kinematics`) zählen **nicht**
   als normkonforme Schutzmaßnahme - dafür braucht's sicherheitsbewertete
   E/A (z. B. safety-rated I/O des FR20-Controllers) für Not-Halt,
   Schutztürüberwachung, Geschwindigkeitsbegrenzung.
7. **Deckenmontage-spezifisch**: mechanische Absturzsicherung/
   Bruchsicherung der Halterung, Bremsen an den Gelenken die bei
   Stromausfall halten, Risikobewertung des Bereichs unterhalb des
   Arbeitsraums.
8. **Betriebsanleitung + Unterweisung** (ASchG): Bedienpersonal muss
   eingewiesen sein, Betriebsanweisung am Gerät.
9. **CE-Kennzeichnung + Technische Dokumentation** für die Gesamtanlage,
   sobald sie über den reinen Laboraufbau hinaus betrieben/übergeben wird.

## Laserscanner (Geschwindigkeits-/Abstandsüberwachung)

Falls PFL nicht nachweisbar ist (siehe oben), ist die gängige Alternative
zum vollen Schutzzaun ein sicherheitszertifizierter **2D-Laserscanner**
(Typ 3 nach EN 61496-3, PL d/Kategorie 3 nach EN ISO 13849-1), der
konzentrische **Warn-** und **Schutzfelder** um die Zelle überwacht:
Warnfeld-Eintritt reduziert das Robotertempo, Schutzfeld-Eintritt löst
einen sicherheitsgerichteten Halt (Kategorie 0/1 nach EN 60204-1) aus.

### Konkrete Kandidaten für den 4×4 m Raum (`ROOM_SIZE_XY` in `fr20_ceiling_gazebo.py`)

Die Raumdiagonale beträgt ca. 5,7 m - je nach Montageposition des Scanners
(mittig an einer Wand vs. in einer Ecke) muss das Schutzfeld das
abdecken können. Alle drei Kandidaten erfüllen PL d/Kategorie 3, Typ 3:

| Gerät | Schutzfeld | Warnfeld | Besonderheit für diese Anwendung |
|---|---|---|---|
| **SICK microScan3 Core** (Empfehlung) | bis 4 m (Standard-Auflösung), bis 5,5 m (gröbere Auflösung) | bis 40 m, 275° | Breite Robotik-Integration (z. B. SICK Flexi Soft), gute Doku/Marktverfügbarkeit |
| **Pilz PSENscan** | bis 5,5 m (bei 70/150 mm Auflösung) | bis 22 m | Bis zu 3 Zonen gleichzeitig, bis zu 4 Geräte kaskadierbar - relevant, falls der deckenmontierte Arm aus mehreren Raumrichtungen erreichbar ist und mehrere Seiten mit einem System abgedeckt werden sollen |
| **Keyence SZ-V** | bis 8,4 m | bis 26 m | Mehr Reichweiten-Marge als nötig für 4×4 m - sinnvoll bei Eckmontage oder falls Keyence-Ökosystem ohnehin genutzt wird |

Quellen für die Werte: [SICK microScan3 Datenblatt](https://files.kempstoncontrols.com/files/a864a3d60080ba01089d14e97697fb0f/MICS3-AAAZ55AZ1P01.pdf), [Pilz PSENscan Produktseite](https://www.pilz.com/en-US/products/sensor-technology/safety-laser-scanner), [Keyence SZ-V Specs](https://www.keyence.com/products/safety/laser-scanner/sz-v/specs/).

Der Mindestabstand vom Schutzfeldrand zur Gefahrstelle wird nach
**EN ISO 13855** berechnet:

```
S = K × T + C
```

- S = Mindestabstand [mm]
- K = Annäherungsgeschwindigkeit: 1600 mm/s (Gehen) bzw. 2000 mm/s (Greifen)
- T = Gesamt-Ansprechzeit: Scanner-Erkennung + Signallaufzeit + Bremszeit
  des Roboters bis zum sicheren Halt
- C = Zuschlag je nach Scanner-Auflösung (z. B. 850 mm bei grober Auflösung)

**Offener Punkt - Bremszeit/-distanz des FR20 (für T):** Das offizielle
[FR-Handbuch](https://fairino-doc-en.readthedocs.io/latest/CobotsManual/safety.html)
(Kap. 7 "Safety") bestätigt die Stopp-Architektur (Type 0 = sofortige
Leistungsabschaltung, Type 1a/1b = Bremsrampe dann Antrieb aus/deaktiviert,
Type 2 = Bremsrampe mit Enable-Erhalt; TCP-Handbetrieb ≤ 250 mm/s; 8
konfigurierbare Sicherheitsebenen), enthält aber **keine numerische
Stopp-Zeit-/Distanz-Tabelle** für den FR20. Eine Wiederverkäufer-Seite
(top3dshop.com, keine Fairino-eigene Quelle) nennt "Category-0 stop times
range from 170-700 ms and distances from 0.22-0.80 rad, measured with the
arm fully extended at full speed (180°/s) and max load" - plausibel, aber
**nicht gegen eine offizielle Fairino-Quelle verifizierbar** (das dafür
vorgesehene "Functional Safety Certificate" ist auf der Fairino-
Downloadseite gelistet, aber nicht frei als PDF verlinkt).

Für die tatsächliche Feldauslegung braucht ihr die zertifizierten
Stopp-Zeit-/Distanz-Werte für **eure konkrete Konfiguration** (Nutzlast
inkl. Biegekopf, gefahrene Geschwindigkeitsstufe) - entweder von Fairino
anfordern (Functional Safety Certificate) oder nach EN ISO 10218-1/-2
selbst messen, bevor Schutzfeldgrößen und Scanner-Position festgelegt
werden.

## Empfehlung

Diese Einordnung ersetzt keine förmliche Risikobeurteilung. Vor Inbetriebnahme
mit echter Hardware: mit Betreuer bzw. Arbeitssicherheitsfachkraft der Uni
durchgehen, insbesondere die Frage, ob der Biegekopf-Aufbau PFL-tauglich
gemacht werden kann oder ob auf Schutzzaun/Absperrung ausgewichen werden muss.

## Quellen

- [FAIRINO FR20 Cobot Specs](https://top3dshop.com/product/fairino-fr20-robotic-arm)
- [Arbeitsschutz in Österreich – Arbeitsinspektion](https://www.arbeitsinspektion.gv.at/Uebergreifendes/Arbeitsschutz_in_Oesterreich.html)
- [Arbeitsmittelverordnung (AM-VO)](https://gesetzefinden.at/bundesrecht/verordnungen/am-vo)
- [ISO/TS 15066 Explained – Robotiq](https://www.automate.org/robotics/tech-papers/iso-ts-15066-explained)
- [ISO/TS 15066 Volltext](https://www.diag.uniroma1.it/deluca/pHRI_elective/ISO_TS_15066_2016_en.pdf)
- [EU-Maschinenverordnung 2023/1230 – TÜV SÜD Österreich](https://www.tuvsud.com/de-at/branchen/produzierende-industrie/maschinen-geraete-ausruestung/maschinenbau/maschinenverordnung)
- [Neue Maschinenverordnung – WKO](https://www.wko.at/ce-kennzeichnung-normen/neue-maschinenverordnung-eu)
- [AUVA M 940 – Arbeiten mit kollaborativen Robotern](https://auva.at/praevention/medien-und-publikationen/publikationen-us/m-940-arbeiten-mit-kollaborativen-robotern/)
- [Kollaborierende Roboter – TÜV SÜD](https://www.tuvsud.com/de-de/indust-re/maschinensicherheit-info/sondermaschinen/roboter/kollaborierende-roboter-sicherheit-arbeitsschutz)
- [Absicherung von Gefahrstellen (Mindestabstand) – SICK](https://www.sick.com/medias/Mindestabstand-Safety-13855-TS149-Thol.pdf)
- [Sicherheits-Laserscanner microScan3 – SICK](https://www.sick.com/de/de/sicherheits-laserscanner-als-enabling-technology-fuer-die-robotik-und-mobile-applikationen-microscan3-pro-fuer-profinet-und-ethernetipTM-meistern-bis-zu-128-ueberwachungsszenarien/w/press-2018-hm-microscan3pro)
- [SICK microScan3 Core Datenblatt](https://files.kempstoncontrols.com/files/a864a3d60080ba01089d14e97697fb0f/MICS3-AAAZ55AZ1P01.pdf)
- [Pilz PSENscan – Produktseite](https://www.pilz.com/en-US/products/sensor-technology/safety-laser-scanner)
- [Keyence SZ-V Serie – Specs](https://www.keyence.com/products/safety/laser-scanner/sz-v/specs/)
- [FR-Series Handbuch (offiziell), Kap. 7 Safety – Fairino](https://fairino-doc-en.readthedocs.io/latest/CobotsManual/safety.html)
- [FR20 Stopp-Zeit/-Distanz-Angabe (unverifizierte Drittquelle) – top3dshop.com](https://top3dshop.com/product/fairino-fr20-robotic-arm)
