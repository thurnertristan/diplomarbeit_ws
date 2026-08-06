# diplomarbeit_ws
Programmierung des Roboters

## Beispiel für die Steuerung

In diesem Workspace befindet sich eine kleine Python-Basis für die Robotersteuerung.

### Ausführung

- `python3 robot_control.py move 1.5`
- `python3 robot_control.py turn 90`
- `python3 robot_control.py stop`
- `python3 robot_control.py status`

Die Steuerung ist als leicht erweiterbares Beispiel gedacht und kann später
an echte Hardware-Interfaces oder eine Simulation angepasst werden.

## UR10e-Simulation und GUI-Steuerung

`ur10e_ceiling_gazebo.py` startet eine Gazebo-Simulation mit einem
kopfüber an der Decke montierten UR10e, der über gz-transport-Topics pro
Gelenk angesteuert wird (siehe Docstring der Datei für Voraussetzungen).

Für eine einfache manuelle Steuerung gibt es zusätzlich `robot_gui.py`:
Es startet dieselbe Simulation und öffnet ein Fenster mit einem
Schieberegler pro Gelenk (±180°) sowie Buttons für "Blick auf Tisch" und
"Home". Zusätzlich lässt sich unten in der GUI eine Zielposition (X/Y/Z in
Metern, Raumkoordinaten wie bei `TABLE_CENTER`) eingeben - "Fahre zu
Position" berechnet per inverser Kinematik (`ur10e_kinematics.py`, liest
die tatsächliche Gelenkkette aus dem generierten URDF) die passenden
Gelenkwinkel und fährt den Arm dorthin, mit nach unten gerichtetem
Werkzeug.

```
source /opt/ros/jazzy/setup.bash
python3 robot_gui.py
```
