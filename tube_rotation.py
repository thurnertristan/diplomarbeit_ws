#!/usr/bin/env python3
"""Ansteuerung der Rohr-Rotationsachse: dreht das eingespannte Rohr zwischen
zwei Biegungen um einen Winkel weiter, damit die nächste Biegung in der
richtigen Ebene liegt.

Die Motor-Hardware steht noch nicht fest, daher zweigeteilt:
  - `TubeRotationAxis` kennt nur "drehe um X Grad weiter" und hält den
    Winkel nach, unabhängig von der physischen Umsetzung.
  - Ein "Backend" übernimmt die Kommunikation mit dem Motor. Neue Hardware
    braucht nur ein neues Backend (Schnittstelle: `move_by_degrees`), der
    Rest (robot_program.py, GUI) bleibt unverändert.

Backends: `SimulatedRotationBackend` (Konsolen-Log, keine Hardware nötig),
`SerialStepperBackend` (Schrittmotor an Arduino/ESP32 über USB-Seriell),
`GazeboRotationBackend` (simulierte Achse in Gazebo).
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from typing import Protocol


class RotationBackend(Protocol):
    """Schnittstelle für jedes Motor-Backend - ein Hardware-Wechsel betrifft
    dadurch nur die Backend-Klasse, nicht die Aufrufer."""

    def move_by_degrees(self, delta_deg: float) -> None: ...


class SimulatedRotationBackend:
    """Backend ohne echte Hardware - loggt nur, was gesendet würde."""

    def move_by_degrees(self, delta_deg: float) -> None:
        print(f"[Simulation] Rotationsachse dreht um {delta_deg:+.2f}°")


@dataclass
class SerialStepperBackend:
    """Backend für einen Schrittmotor an einem Mikrocontroller (Arduino/
    ESP32) über ein einfaches USB-Seriell-Textprotokoll:

        PC  -> Controller:  "R<grad>\\n"      z.B. "R45.0\\n"
        Controller -> PC:   "OK\\n"           sobald die Bewegung fertig ist

    Der Mikrocontroller (Firmware, nicht Teil dieser Datei) rechnet Grad in
    Schritte um (steps_per_revolution * gear_ratio * microstepping / 360)
    und gibt STEP/DIR-Pulse an die Treiberplatine aus.
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    timeout_s: float = 30.0

    def __post_init__(self) -> None:
        import serial  # nur importieren, wenn dieses Backend genutzt wird

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

    Der gz-sim JointPositionController hält ein ABSOLUTES Zielgelenk (rad),
    anders als der reale Schrittmotor, der nur den relativen Winkel als
    Impuls bekommt - dieses Backend führt den absoluten Zielwinkel daher
    selbst mit (`_current_rad`).

    Discovery-Warmup (bis gz-transport den Publisher sicher gefunden hat)
    passiert nur einmal in __init__, danach sendet move_by_degrees() direkt
    eine einzelne Nachricht - wichtig, da run_rotation_sequence eine Drehung
    in viele kleine Zwischenschritte zerlegt (siehe dort); mit vollem
    Wiederholungs-Overhead pro Schritt wäre allein das Senden bei größeren
    Sequenzen spürbar langsam."""

    def __init__(self, publishers: dict, start_deg: float = 0.0) -> None:
        import fr20_ceiling_gazebo as sim

        self._publishers = publishers
        self._current_rad = math.radians(start_deg)
        self._Double = sim.import_gz("msgs", "double_pb2").Double
        sim.send_joint_pose({"tube_rotation": self._current_rad}, self._publishers)

    def move_by_degrees(self, delta_deg: float) -> None:
        self._current_rad += math.radians(delta_deg)
        msg = self._Double()
        msg.data = self._current_rad
        self._publishers["tube_rotation"].publish(msg)


class TubeRotationAxis:
    """Hält den aktuell angenommenen Drehwinkel des Rohrs nach und reicht
    Bewegungen an ein `RotationBackend` weiter.

    Der Winkel wird rein rechnerisch mitgeführt (kein Encoder-Feedback) -
    bei einem Schrittmotor ohne Rückmeldung die übliche open-loop-Annahme.

    `rotate_by` ist per Lock serialisiert, da gui/my_gui.py sie sowohl aus
    dem Programm-Worker-Thread als auch aus dem Jog (Tkinter-Hauptthread,
    während einer Pause) aufruft - ohne Lock könnten sich beide beim
    `current_deg`-Update überschreiben."""

    def __init__(self, backend: RotationBackend, start_deg: float = 0.0) -> None:
        self._backend = backend
        self.current_deg = start_deg
        self._lock = threading.Lock()

    def rotate_by(self, delta_deg: float) -> float:
        """Dreht das Rohr um `delta_deg` relativ zur aktuellen Stellung
        weiter (z.B. zwischen zwei Biegungen, um die nächste Biegeebene
        einzustellen). Gibt den neuen absoluten Winkel zurück."""
        with self._lock:
            self._backend.move_by_degrees(delta_deg)
            self.current_deg += delta_deg
            return self.current_deg


def run_rotation_sequence(
    axis: TubeRotationAxis,
    steps: list[float],
    stop_event: "threading.Event | None" = None,
    pause_event: "threading.Event | None" = None,
    on_angle_changed=None,
    settle_s=None,
    dwell_s: float = 0.5,
    step_deg: float = 20.0,
) -> float:
    """Fährt eine Liste relativer Drehschritte (Grad) nacheinander ab -
    dasselbe stop_event/pause_event-Muster wie robot_program.run_program,
    nur für die eine Rotationsachse statt den ganzen Arm.

    Jeder Eintrag aus `steps` wird in `step_deg`-große Zwischenschritte
    zerlegt (z.B. 180° in neun 20°-Schritten), nicht als ein Sprung
    gesendet - so werden pause_event/stop_event/on_angle_changed alle
    `step_deg` statt erst am Ende der ganzen Bewegung geprüft/ausgelöst.

    `stop_event`: sofortiger, endgültiger Abbruch vor dem nächsten
    Zwischenschritt. `pause_event`: hält vor jedem Zwischenschritt an,
    macht danach normal weiter - während der Pause darf der Aufrufer
    selbst `axis.rotate_by()` aufrufen (z.B. Jog-Buttons), ohne dass sich
    das mit dieser Funktion in die Quere kommt (siehe TubeRotationAxis-Lock).
    `on_angle_changed`: Hook für die Live-Anzeige, pro Zwischenschritt
    aufgerufen. `settle_s`: Callable[[float], float], liefert die
    Wartezeit bis eine Bewegung abgeschlossen ist (z.B. für einen
    nicht-blockierenden gz-sim-Regler; bei einem blockierenden Backend wie
    SerialStepperBackend genügt `settle_s=None`). `dwell_s` gilt nur nach
    dem letzten Zwischenschritt eines `steps`-Eintrags.

    Gibt den zuletzt erreichten Winkel (`axis.current_deg`) zurück.
    """

    def stopped() -> bool:
        return stop_event is not None and stop_event.is_set()

    def wait_paused_or_stopped() -> bool:
        """Wartet, solange pause_event gesetzt ist. Gibt True zurück, wenn
        währenddessen (oder schon vorher) stop_event gesetzt wurde."""
        while pause_event is not None and pause_event.is_set() and not stopped():
            time.sleep(0.1)
        return stopped()

    for delta_deg in steps:
        if delta_deg == 0:
            continue
        n = max(1, math.ceil(abs(delta_deg) / step_deg))
        increment = delta_deg / n

        for i in range(n):
            if wait_paused_or_stopped():
                print("Rohrdrehung abgebrochen.")
                return axis.current_deg

            axis.rotate_by(increment)
            if on_angle_changed is not None:
                on_angle_changed(axis.current_deg)

            is_last = i == n - 1
            wait_s = settle_s(increment) if settle_s is not None else 0.0
            deadline = time.time() + (max(wait_s, dwell_s) if is_last else wait_s)
            while time.time() < deadline and not stopped():
                time.sleep(min(0.05, max(0.0, deadline - time.time())))
            if stopped():
                print("Rohrdrehung abgebrochen.")
                return axis.current_deg

    return axis.current_deg


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
