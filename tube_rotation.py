#!/usr/bin/env python3
"""Ansteuerung der Rohr-Rotationsachse (Motor + Getriebe, dreht das
eingespannte Rohr zwischen zwei Biegungen um einen bestimmten Drehwinkel
weiter, damit die nächste Biegung in der richtigen Ebene liegt).

Die konkrete Motor-Hardware steht noch nicht fest - deshalb ist die
Ansteuerung hier bewusst in zwei Teile getrennt:

  - `TubeRotationAxis` kennt nur "drehe um X Grad weiter" und hält den
    aktuell angenommenen Winkel nach - unabhängig davon, WIE die Drehung
    physisch passiert.
  - Ein "Backend" (siehe `SimulatedRotationBackend`/`SerialStepperBackend`
    unten) übernimmt die eigentliche Kommunikation mit dem Motor. Steht die
    Hardware fest, reicht es, ein neues Backend zu schreiben (gleiche
    kleine Schnittstelle: `move_by_degrees(delta_deg)`) - der Rest des
    Programms (robot_program.py, GUI) bleibt unverändert.

`SimulatedRotationBackend` loggt nur auf der Konsole (kein echter Motor
nötig) - genau wie `robot_program.py --dry-run` beim FR20-Arm, zum Testen
des Ablaufs ohne Hardware. `SerialStepperBackend` ist die häufigste reale
Variante (Schrittmotor an einer Treiberplatine, gesteuert von einem
Arduino/ESP32, das über USB-Seriell einfache Textbefehle entgegennimmt) -
als Startpunkt, sobald die Hardware feststeht. `GazeboRotationBackend`
steuert stattdessen die simulierte Rotationsachse aus
fr20_ceiling_gazebo.build_tube_rotation_model_sdf() an - zum Testen von
Kinematik/Zeitablauf im Zusammenspiel mit dem FR20-Arm, bevor überhaupt
reale Hardware existiert.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Protocol


class RotationBackend(Protocol):
    """Schnittstelle, die jedes Motor-Backend erfüllen muss - eine einzige
    Methode, damit ein Hardware-Wechsel später nur diese eine Klasse
    betrifft, nicht die Aufrufer."""

    def move_by_degrees(self, delta_deg: float) -> None: ...


class SimulatedRotationBackend:
    """Backend ohne echte Hardware - loggt nur, was gesendet würde.

    Zum Testen der Programmlogik (z.B. neue Punkte mit Rotationsschritten
    in robot_program.py), solange der reale Motor noch nicht angeschlossen
    ist, oder um am PC ohne Werkstatt zu entwickeln."""

    def move_by_degrees(self, delta_deg: float) -> None:
        print(f"[Simulation] Rotationsachse dreht um {delta_deg:+.2f}°")


@dataclass
class SerialStepperBackend:
    """Backend für einen Schrittmotor an einem Mikrocontroller (Arduino/
    ESP32), der über USB-Seriell ein einfaches Textprotokoll spricht:

        PC  -> Controller:  "R<grad>\\n"      z.B. "R45.0\\n"
        Controller -> PC:   "OK\\n"           sobald die Bewegung fertig ist

    Der Mikrocontroller rechnet Grad in Schritte um (steps_per_revolution *
    gear_ratio * microstepping / 360) und gibt STEP/DIR-Pulse an die
    Treiberplatine (z.B. A4988/DRV8825/TB6600) aus - dieser Teil ist
    Firmware, nicht Teil dieser Datei. `steps_per_revolution`/`gear_ratio`
    sind hier trotzdem als Parameter vorgesehen, falls stattdessen die
    Umrechnung PC-seitig passieren und direkt eine Schrittzahl gesendet
    werden soll (dann `move_by_degrees` entsprechend anpassen).

    Bewusst ein simples Text-Protokoll statt z.B. Firmata/ROS-Serial: für
    eine einzelne Achse mit "dreh um X Grad, warte auf fertig" reicht das,
    ohne zusätzliche Abhängigkeiten auf Controller-Seite.
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout_s: float = 30.0

    def __post_init__(self) -> None:
        import serial  # pyserial - nur importieren, wenn dieses Backend genutzt wird

        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)

    def move_by_degrees(self, delta_deg: float) -> None:
        command = f"R{delta_deg:.3f}\n".encode("ascii")
        self._serial.write(command)
        response = self._serial.readline().decode("ascii", errors="replace").strip()
        if response != "OK":
            raise RuntimeError(
                f"Rotationsachse hat den Befehl nicht bestätigt (erwartet 'OK', bekommen {response!r}) - "
                f"Verbindung/Firmware prüfen."
            )

    def close(self) -> None:
        self._serial.close()


class GazeboRotationBackend:
    """Backend für die simulierte Rotationsachse in Gazebo (siehe
    fr20_ceiling_gazebo.build_tube_rotation_model_sdf).

    Anders als der reale Schrittmotor (SerialStepperBackend, der pro Aufruf
    nur den relativen Winkel als Impuls bekommt) ist der gz-sim
    JointPositionController ein Regler, der auf ein ABSOLUTES Zielgelenk
    (in Radiant) hält - dieses Backend führt den absoluten Zielwinkel daher
    selbst mit (`_current_rad`) und veröffentlicht bei jedem Aufruf die neue
    Summe, statt nur das Delta durchzureichen.

    `publishers` ist das dict aus fr20_ceiling_gazebo.launch_simulation() -
    das Veröffentlichen selbst (inkl. Wiederholungen gegen die noch nicht
    vollständig aufgebaute gz-transport-Discovery) übernimmt
    sim.send_joint_pose(), genau wie bei den FR20-Armgelenken."""

    def __init__(self, publishers: dict, start_deg: float = 0.0) -> None:
        import fr20_ceiling_gazebo as sim

        self._sim = sim
        self._publishers = publishers
        self._current_rad = math.radians(start_deg)

    def move_by_degrees(self, delta_deg: float) -> None:
        self._current_rad += math.radians(delta_deg)
        self._sim.send_joint_pose({"tube_rotation": self._current_rad}, self._publishers)


class TubeRotationAxis:
    """Hält den aktuell angenommenen Drehwinkel des Rohrs nach und reicht
    Bewegungen an ein `RotationBackend` weiter (siehe oben).

    Der Winkel wird rein rechnerisch mitgeführt (kein Feedback von einem
    Encoder) - bei einem Schrittmotor ohne Rückmeldung ist das die übliche
    Annahme (open-loop), solange kein Schritt verloren geht.
    """

    def __init__(self, backend: RotationBackend, start_deg: float = 0.0) -> None:
        self._backend = backend
        self.current_deg = start_deg

    def rotate_by(self, delta_deg: float) -> float:
        """Dreht das Rohr um `delta_deg` relativ zur aktuellen Stellung
        weiter (z.B. zwischen zwei Biegungen, um die nächste Biegeebene
        einzustellen). Gibt den neuen absoluten Winkel zurück."""
        self._backend.move_by_degrees(delta_deg)
        self.current_deg += delta_deg
        return self.current_deg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("degrees", type=float, help="Drehwinkel in Grad (relativ, z.B. 45 oder -90)")
    parser.add_argument(
        "--port", default="/dev/ttyUSB0", help="Serieller Port des Mikrocontrollers (Standard: /dev/ttyUSB0)"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--simulate",
        action="store_true",
        help="Ohne echte Hardware testen - loggt nur, was gesendet würde (siehe SimulatedRotationBackend).",
    )
    mode.add_argument(
        "--gazebo",
        action="store_true",
        help="Startet Gazebo mit der simulierten Rotationsachse (siehe GazeboRotationBackend) und dreht dort "
        "das Rohr - zum Testen von Kinematik/Zeitablauf im Zusammenspiel mit dem FR20-Arm, bevor reale "
        "Hardware existiert.",
    )
    parser.add_argument("--headless", action="store_true", help="Bei --gazebo: gz-sim ohne GUI starten")
    args = parser.parse_args()

    if args.gazebo:
        import fr20_ceiling_gazebo as sim

        print("Starte Gazebo mit der Rohr-Rotationsachse ...")
        gz_proc, publishers = sim.launch_simulation(headless=args.headless)
        try:
            axis = TubeRotationAxis(GazeboRotationBackend(publishers))
            new_angle = axis.rotate_by(args.degrees)
            print(f"Rohr auf {new_angle:.2f}° gedreht.")
            print("\nGazebo-Fenster schließen oder Strg+C hier drücken, um zu beenden.")
            gz_proc.wait()
        except KeyboardInterrupt:
            print("\nBeende Gazebo ...")
        finally:
            sim.shutdown_simulation(gz_proc)
        return

    backend: RotationBackend
    if args.simulate:
        backend = SimulatedRotationBackend()
    else:
        backend = SerialStepperBackend(port=args.port)

    axis = TubeRotationAxis(backend)
    new_angle = axis.rotate_by(args.degrees)
    print(f"Rohr auf {new_angle:.2f}° gedreht.")


if __name__ == "__main__":
    main()
