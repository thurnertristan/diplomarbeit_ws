# diplomarbeit_ws
Programmierung des Roboters

## Programm start befehl
source /opt/ros/jazzy/setup.bash
source gui/venv/bin/activate
python3 gui/my_gui.py

## Fairino FR20-Simulation und GUI-Steuerung

`fr20_ceiling_gazebo.py` startet eine Gazebo-Simulation mit einem
kopfüber an der Decke montierten Fairino FR20, der über gz-transport-Topics
pro Gelenk (j1..j6) angesteuert wird (siehe Docstring der Datei für
Voraussetzungen). Das URDF stammt aus dem vendorten `fairino_description`-
Paket (`fairino_description/urdf/fairino20_v6.urdf`, von
https://github.com/FAIR-INNOVATION/frcobot_ros2), dem zur Laufzeit ein
fixer Decken-Verankerungslink ("world") sowie ein "tool0"-Werkzeugframe
angehängt werden.

## Punktprogramme abfahren

`robot_program.py` fährt eine Liste von Punkten aus einer JSON-Datei
nacheinander ab (siehe `programs/*.json` für Beispiele). Ein Punkt ist
entweder eine kartesische Zielposition (`x`/`y`/`z`, optional `rpy` in Grad
für die Werkzeugorientierung) oder eine direkte Gelenkpose (`joints` in
Grad); pro Punkt lässt sich mit `dwell_s` die Wartezeit einstellen, bevor
der nächste Punkt angefahren wird.

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
   in `fr20_ceiling_gazebo.py` - dort auch änderbar, falls eine andere
   Stelle besser geeignet ist).
3. Im Bestätigungsdialog per Jog (X/Y/Z) feinjustieren, falls die Baugruppe
   nicht exakt an der erwarteten Stelle steht, dann bestätigen.
4. "Weiter" führt (mit oder ohne vorheriges Kalibrieren) zum Hauptfenster.

Der neu berechnete Tisch-Offset wird in `table_offset.json` gespeichert
(nicht Teil des Repos, siehe `.gitignore`) und ab sofort automatisch auf
alle kartesischen Programmpunkte angewendet (`robot_program.run_program`).
Gelenkpose-Punkte (`joints`) sind davon nicht betroffen, ebenso wenig die
Jog-Buttons selbst.

## Rohr-Rotationsachse und Biegekraft-Abschätzung

`tube_rotation.py` steuert den Motor an, der das eingespannte Rohr
zwischen zwei Biegungen um einen Drehwinkel weiterdreht - die Hardware
steht noch nicht fest, deshalb ist die Ansteuerung von der Kommunikation
mit dem Motor getrennt (austauschbares "Backend"):

```
python3 tube_rotation.py 45 --simulate            # nur loggen, ohne Hardware
python3 tube_rotation.py 45 --gazebo --headless   # in der Simulation drehen
python3 tube_rotation.py 45 --port /dev/ttyUSB0   # echter Schrittmotor an einem Mikrocontroller
```

`biegekraft_rechner.py` schätzt aus Rohr-Außendurchmesser, Wandstärke und
Werkstoff das nötige Biegemoment und daraus (mit Getriebeübersetzung,
Wirkungsgrad, Sicherheitsfaktor) das nötige Motor-Drehmoment ab - als
Startpunkt für die Motor-/Getriebeauswahl, keine FEM-Simulation:

```
python3 biegekraft_rechner.py --od 20 --wall 1.5 --material S235 --gear-ratio 50
```
