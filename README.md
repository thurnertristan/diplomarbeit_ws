# diplomarbeit_ws
Programmierung des Roboters

## GUI starten

Alle Befehle im Repo-Ordner (`diplomarbeit_ws`) ausführen. `start_gui.sh`
erledigt alles selbst (ROS sourcen, venv anlegen, fehlende Pakete
installieren) - kein manuelles `source` nötig.

### Normalbetrieb (10"-Touchscreen, Vollbild)

```bash
./start_gui.sh
```

Alle Fenster laufen randlos im Vollbild auf dem Touchscreen. Ist zusätzlich
ein normaler Monitor angeschlossen, wird automatisch der **kleinste**
Bildschirm genommen (= der Touchscreen). Einen bestimmten Monitor erzwingen
(Name laut `xrandr`):

```bash
FR20_MONITOR=HDMI-1 ./start_gui.sh
```

### Am Entwickler-Bildschirm (normale Fenster)

```bash
FR20_KIOSK=0 ./start_gui.sh
```

Die Fenster öffnen dann als normale, verschiebbare Fenster statt im Vollbild.

### Simulation: RViz2

Die Simulation läuft in RViz2 (nur 3D-Anzeige, keine Physik). Gazebo wird nicht mehr
verwendet.

```bash
python3 robot_program.py programs/beispiel_tisch.json            # Programm in RViz abfahren
python3 robot_program.py programs/beispiel_tisch.json --dry-run  # nur Kollisionsprüfung, ohne RViz
python3 fr20_scene.py                                            # nur Szene + Beispielpose
```

Umgesetzt in `rviz_backend.py` (URDF mit Tisch/Baugruppe/Biegekopf-Testachse,
`robot_state_publisher`, eigener `/joint_states`-Publisher), RViz-Ansicht in
`rviz/fr20.rviz`. Alle sechs Armgelenke fahren synchron (wie PTP/MoveJ am echten Roboter).
Die ROS-Kommunikation bleibt auf diesem Rechner (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`).

### Kollisionsschutz (Tisch/Baugruppe)

RViz hat keine Physik. Damit der Arm nicht durch Tisch oder Baugruppe fährt, prüft
`fr20_collision.py` jede Bahn vorher mit der echten Geometrie der Armglieder (STL-Meshes) gegen
Tisch, Baugruppe, Biegekopf und Boden, mit mindestens 30 mm Abstand (`CLEARANCE_MARGIN_M`).

- `robot_program.plan_motion()` sucht für jeden Punkt eine kollisionsfreie Bahn: direkt,
  sonst senkrecht hoch (in 5-cm-Schritten, so hoch wie sicher möglich, höchstens
  `SAFE_TRAVEL_Z`), oben verfahren und absenken, sonst zusätzlich über die Parkpose. IK-Lösungen mit dem Ellbogen zur Decke werden bevorzugt.
- Vor dem Start wird das **ganze** Programm durchgeplant (bei Figur 1 ca. 10 s), danach
  werden genau diese geprüften Bahnen gefahren. Ist ein Punkt nicht sicher
  erreichbar, startet es gar nicht (GUI: Warnfenster „Kollisionsgefahr“).
- Jog-Schritte in Richtung Tisch werden verweigert.
- x/y/z in den Programmen meinen die **Mitte der Flanschfläche** (`tool0`). Die Oberkante der
  Baugruppe liegt bei z = 0.795. Eine sichere Arbeitshöhe ist z ≥ 0.845 (`APPROACH_Z`).
- Ein Werkzeug/Greifer ist noch nicht modelliert. Sobald es feststeht, in
  `fr20_collision.py` und beim `tool0`-Versatz ergänzen.

Die Software-Prüfung ersetzt keine sicherheitsbewertete Überwachung (siehe `SICHERHEIT.md`).

### Autostart beim Hochfahren

```bash
./autostart.sh ein      # GUI startet ab dem nächsten Anmelden automatisch
./autostart.sh aus      # Autostart wieder abschalten
./autostart.sh status   # zeigt, ob der Autostart aktiv ist
```

- Der Autostart greift beim **Anmelden**. Damit die GUI direkt nach dem
  Hochfahren ohne Passworteingabe erscheint: Ubuntu-Einstellungen →
  Benutzer → **Automatische Anmeldung** einschalten.
- Ausgaben der automatisch gestarteten GUI (zur Fehlersuche):
  `~/.cache/fr20_gui_autostart.log`
- Technisch ist das der Eintrag `~/.config/autostart/fr20-gui.desktop`.

### Zugriff auf den RS485-Converter

Für die echte Motoransteuerung (`/dev/ttyUSB0`) muss der Benutzer einmalig in
der Gruppe `dialout` sein, danach ab- und wieder anmelden:

```bash
sudo usermod -aG dialout $USER
```

## Fairino FR20-Szene

`fr20_scene.py` enthält die Zelle (Raum, kopfüber an der Decke montierter Fairino FR20,
Tisch, Baugruppe, Biegekopf-Testachse) und startet die RViz2-Simulation. Das URDF stammt aus
dem vendorten `fairino_description`-Paket (`fairino_description/urdf/fairino20_v6.urdf`, von
https://github.com/FAIR-INNOVATION/frcobot_ros2). Zur Laufzeit kommen ein fixer
Decken-Verankerungslink ("world") und ein "tool0"-Frame auf der Flanschfläche dazu.

## Punktprogramme abfahren

`robot_program.py` fährt eine Liste von Punkten aus einer JSON-Datei
nacheinander ab (siehe `programs/*.json` für Beispiele). Ein Punkt ist
entweder eine kartesische Zielposition (`x`/`y`/`z`, optional `rpy` in Grad
für die Werkzeugorientierung) oder eine direkte Gelenkpose (`joints` in
Grad); pro Punkt lässt sich mit `dwell_s` die Wartezeit einstellen, bevor
der nächste Punkt angefahren wird.

Nach dem letzten Punkt eines Programms fährt der Roboter automatisch noch
den Nullpunkt (`fr20_scene.CALIBRATION_REFERENCE_XYZ`, derselbe
Punkt wie bei der Nullpunkt-Kalibrierung) an, bevor das Programm endet
(`robot_program.run_program(..., home_at_end=True)`, Standard). Wird ein
Programm durch ein anderes abgelöst (z.B. Klick auf einen anderen
Programm-Button in der GUI, während eins läuft), entfällt dieser
Rückfahrtschritt.

Eigene Programme = eigene JSON-Dateien im `programs/`-Ordner nach diesem
Schema, ohne Code-Änderung nutzbar:

```
source /opt/ros/jazzy/setup.bash
python3 robot_program.py                              # verfügbare Programme auflisten
python3 robot_program.py programs/beispiel_tisch.json
python3 robot_program.py programs/beispiel_gelenkpose.json --loop
```

## Nullpunkt kalibrieren (falls sich der Tisch/die Baugruppe verschoben hat)

Alle `x`/`y`/`z`-Koordinaten in den Programmdateien sind fest relativ zur
Deckenhalterung des Roboters gemeint, nicht relativ zum Tisch. Verrutscht
der Tisch etwas, landen die Programmpunkte an der falschen Stelle - dafür
gibt es die Nullpunkt-Kalibrierung:

1. Die GUI zeigt das Kalibrierungs-Fenster automatisch beim Start (bevor
   das Hauptfenster mit den Programm-Buttons erscheint) - später ist es
   über den Button "Kalibrierung" im Hauptfenster jederzeit erneut
   erreichbar, falls sich der Tisch nochmal verschiebt.
2. "Nullpunkt kalibrieren" drücken - der Roboter fährt automatisch auf den
   markanten Referenzpunkt auf der Baugruppe (`CALIBRATION_REFERENCE_XYZ`
   in `fr20_scene.py` - dort auch änderbar, falls eine andere
   Stelle besser geeignet ist).
3. Im Bestätigungsdialog per Jog (X/Y/Z) feinjustieren, falls die Baugruppe
   nicht exakt an der erwarteten Stelle steht, dann bestätigen.
4. "Weiter" führt (mit oder ohne vorheriges Kalibrieren) zum Hauptfenster.

Der neu berechnete Tisch-Offset wird in `table_offset.json` gespeichert
(nicht Teil des Repos, siehe `.gitignore`) und ab sofort automatisch auf
alle kartesischen Programmpunkte angewendet (`robot_program.run_program`).
Gelenkpose-Punkte (`joints`) sind davon nicht betroffen, ebenso wenig die
Jog-Buttons selbst.

## Biegekopf-Motor und Biegekraft-Abschätzung

Der Biegekopf wird von genau einem realen Servo angetrieben (Panasonic
MSMF082L1U1 + Verstärker Panasonic MCDLN35SG), angesteuert über
Modbus-RTU/RS485 (115200 Baud, 8N1, USB-RS485-Konverter). `bending_head_motor.py`
übernimmt diese Ansteuerung - wie bei anderer Motor-Hardware in diesem
Repo ist die Kommunikation von der Steuerlogik getrennt (austauschbares
"Backend"):

```
python3 bending_head_motor.py 45 --simulate            # nur loggen, ohne Hardware
python3 bending_head_motor.py 45 --rviz                 # simulierte Testachse in RViz drehen
python3 bending_head_motor.py 45 --port /dev/ttyUSB0   # echter Servo über Modbus-RTU
```

Die `--rviz`-Testachse (siehe `RvizBendingBackend`) ist kein Nachbau
des realen Biegekopfs, sondern ein einfacher Platzhalter (Ständer +
drehbare Welle, wie zuvor die Rotationsachsen-Testachse) - sie dient nur
dazu, Kinematik/Zeitablauf im Zusammenspiel mit dem FR20-Arm zu prüfen,
solange die Modbus-Register des realen Servos noch nicht final sind. In
der GUI startet der Button "Biegekopf testen" denselben Testlauf.

Die exakten Modbus-Register (Sollposition, Solldrehzahl, Drehmoment-Limit,
Control-/Status-Word) sind noch nicht final und stehen als TODO-Platzhalter
in `ModbusRTUBendingBackend` - ohne sie wirft der reale Betrieb bewusst
`NotImplementedError`, statt mit geratenen Registern auf den Verstärker zu
schreiben. Das Drehmoment-Limit MUSS beim Verbindungsaufbau gesetzt werden:
laut `biegekraft_rechner.py`-Abschätzung liegt das gemessene Biegemoment
(~230 Nm) sehr nah an der angenommenen Drivetrain-Kapazität (~225 Nm) - die
Sicherheitsmarge ist praktisch null.

`biegekraft_rechner.py` schätzt aus Rohr-Außendurchmesser, Wandstärke und
Werkstoff das nötige Biegemoment und daraus (mit Getriebeübersetzung,
Wirkungsgrad, Sicherheitsfaktor) das nötige Motor-Drehmoment ab - als
Startpunkt für die Motor-/Getriebeauswahl, keine FEM-Simulation:

```
python3 biegekraft_rechner.py --od 20 --wall 1.5 --material S235 --gear-ratio 50
```
