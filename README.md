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

Für eine einfache manuelle Steuerung gibt es zusätzlich `robot_gui.py`:
Es startet dieselbe Simulation und öffnet ein Fenster mit einem
Schieberegler pro Gelenk (±180°) sowie Buttons für "Blick auf Tisch" und
"Home". Zusätzlich lässt sich unten in der GUI eine Zielposition (X/Y/Z in
Metern, Raumkoordinaten wie bei `TABLE_CENTER`) eingeben - "Fahre zu
Position" berechnet per inverser Kinematik (`fr20_kinematics.py`, liest
die tatsächliche Gelenkkette aus dem generierten URDF) die passenden
Gelenkwinkel und fährt den Arm dorthin, mit nach unten gerichtetem
Werkzeug.

```
source /opt/ros/jazzy/setup.bash
python3 robot_gui.py
```

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
