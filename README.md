# diplomarbeit_ws
Programmierung des Roboters

## Beispiel für die Steuerung

In diesem Workspace befindet sich eine kleine Python-Basis für die Robotersteuerung.

### Ausführung

- `python3 robot_control.py move 1.5`
- `python3 robot_control.py turn 90`
- `python3 robot_control.py stop`
- `python3 robot_control.py status`

## Programm start befehl
source /opt/ros/jazzy/setup.bash
source gui/venv/bin/activate
python3 gui/my_gui.py

Die Steuerung ist als leicht erweiterbares Beispiel gedacht und kann später
an echte Hardware-Interfaces oder eine Simulation angepasst werden.

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

## Nullpunkt kalibrieren (falls sich der Tisch verschoben hat)

Alle `x`/`y`/`z`-Koordinaten in den Programmdateien sind fest relativ zur
Deckenhalterung des Roboters gemeint, nicht relativ zum Tisch. Verrutscht
der Tisch etwas, landen die Programmpunkte an der falschen Stelle - dafür
gibt es die Nullpunkt-Kalibrierung:

1. In der GUI ein beliebiges Programm starten und sofort "Pause" drücken
   (button16) - das aktiviert die Jog-Buttons (X/Y/Z).
2. Den Roboter per Jog exakt auf die Referenzstelle fahren: die vordere
   linke Ecke der Tischplatte, auf Tischhöhe (siehe
   `CALIBRATION_REFERENCE_XYZ` in `fr20_ceiling_gazebo.py` - dort auch
   änderbar, falls eine andere Stelle besser geeignet ist).
3. "Nullpunkt kalibrieren" (button22) drücken und bestätigen.

Der neu berechnete Tisch-Offset wird in `table_offset.json` gespeichert
(nicht Teil des Repos, siehe `.gitignore`) und ab sofort automatisch auf
alle kartesischen Programmpunkte angewendet (`robot_program.run_program`).
Gelenkpose-Punkte (`joints`) sind davon nicht betroffen, ebenso wenig die
Jog-Buttons selbst.
