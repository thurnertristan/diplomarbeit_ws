#!/usr/bin/env python3
"""Ansteuerung des Biegekopf-Motors: der EINE reale Servo im System (Panasonic
MSMF082L1U1 + Verstärker Panasonic MCDLN35SG), der den Biegekopf antreibt.

Es gibt in diesem Projekt keine separate Rohr-Rotationsachse mit eigenem
Schrittmotor - die wurde nur evaluiert und nicht gebaut. Der Biegekopf-Motor
ist der einzige anzusteuernde Motor.

Wie bei jeder Motor-Hardware in diesem Repo zweigeteilt (siehe z.B. die
frühere tube_rotation.py-Struktur):
  - `BendingHeadAxis` kennt nur "fahre um X weiter" und hält die Position
    nach, unabhängig von der physischen Umsetzung.
  - Ein "Backend" übernimmt die Kommunikation mit dem Verstärker. Neue
    Hardware braucht nur ein neues Backend (Schnittstelle: `connect`,
    `move_by`), der Rest (robot_program.py, GUI) bleibt unverändert.

Backends: `SimulatedBendingBackend` (Konsolen-Log, keine Hardware nötig),
`ModbusRTUBendingBackend` (echte Kommunikation über Modbus-RTU/RS485 mit
dem MCDLN35SG, 115200 Baud, 8N1, über einen USB-RS485-Konverter),
`GazeboBendingBackend` (simulierte Testachse in Gazebo, siehe
fr20_ceiling_gazebo.build_bending_head_model_sdf - rein für Kinematik-/
Zeitablauf-Tests BEVOR die Modbus-Register feststehen, kein Nachbau der
realen Baugruppe).

WICHTIG - Modbus-Register: die exakten Register (Sollposition, Solldrehzahl,
Drehmoment-Limit, Control-/Status-Word) sind NOCH NICHT final - sie stehen
als klar markierte TODO-Platzhalter unten und dürfen nicht geraten werden.
Vor echtem Hardwarebetrieb im MCDLN35SG-Handbuch (Modbus-RTU-Registerliste)
nachschlagen und eintragen.

WICHTIG - Drehmoment-Limit: laut biegekraft_rechner.py-Abschätzung liegt das
gemessene Biegemoment (~230 Nm) SEHR NAH an bzw. über der angenommenen
Drivetrain-Kapazität (~225 Nm) - die Sicherheitsmarge ist praktisch null.
Das Drehmoment-Limit MUSS deshalb beim Verbindungsaufbau gesetzt werden
(siehe `connect()`), bevor irgendeine Bewegung gesendet wird. Diese Werte
sind vorläufig und müssen vor Inbetriebnahme an einer echten Messung
verifiziert werden.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from typing import Protocol

# Drivetrain-Kapazität und gemessenes Biegemoment (Nm) - siehe Warnung im
# Modul-Docstring. DRIVETRAIN_CAPACITY_NM < MEASURED_BENDING_MOMENT_NM wäre
# ein Auslegungsfehler (Antriebsstrang unterdimensioniert) - beide Werte hier
# sind vorläufige Platzhalter aus biegekraft_rechner.py-Abschätzungen und
# müssen vor Inbetriebnahme an echten Messungen verifiziert werden.
DRIVETRAIN_CAPACITY_NM = 225.0
MEASURED_BENDING_MOMENT_NM = 230.0

# Sicherheitsmarge zwischen Drivetrain-Kapazität und gemessenem Biegemoment
# ist minimal (hier sogar negativ) - das Drehmoment-Limit wird deshalb
# bewusst NICHT auf MEASURED_BENDING_MOMENT_NM, sondern auf die kleinere
# Drivetrain-Kapazität gesetzt, damit der Antriebsstrang nicht überlastet
# wird. Das bedeutet: der Biegekopf könnte am realen Motor-Torque-Limit
# hängenbleiben, statt den vollen Biegemoment aufzubringen - das muss vor
# Inbetriebnahme durch eine echte Auslegung (stärkeres Getriebe, größerer
# Motor) aufgelöst werden, nicht durch dieses Limit.
TORQUE_LIMIT_NM = min(DRIVETRAIN_CAPACITY_NM, MEASURED_BENDING_MOMENT_NM)


class BendingBackend(Protocol):
    """Schnittstelle für jedes Motor-Backend - ein Hardware-Wechsel betrifft
    dadurch nur die Backend-Klasse, nicht die Aufrufer."""

    def connect(self, torque_limit_nm: float) -> None: ...

    def move_by(self, delta: float) -> None: ...


class SimulatedBendingBackend:
    """Backend ohne echte Hardware - loggt nur, was gesendet würde."""

    def connect(self, torque_limit_nm: float) -> None:
        print(f"[Simulation] Verbindung 'aufgebaut', Drehmoment-Limit={torque_limit_nm:.1f} Nm")

    def move_by(self, delta: float) -> None:
        print(f"[Simulation] Biegekopf-Motor fährt um {delta:+.2f} weiter")


@dataclass
class ModbusRTUBendingBackend:
    """Backend für den Panasonic MCDLN35SG-Verstärker über Modbus-RTU/RS485
    (USB-RS485-Konverter, 115200 Baud, 8N1).

    Die tatsächlichen Register (TODO unten) sind noch nicht aus dem
    MCDLN35SG-Handbuch übernommen - NICHT raten. Bis dahin wirft `connect`/
    `move_by` bewusst NotImplementedError, statt mit falschen Registern
    stillschweigend etwas Falsches auf den Verstärker zu schreiben.
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    parity: str = "N"   # 8N1
    stopbits: int = 1
    bytesize: int = 8
    slave_id: int = 1
    timeout_s: float = 1.0

    # TODO: Register-Adressen aus dem MCDLN35SG Modbus-RTU-Handbuch
    # übernehmen, sobald final - NICHT raten/erfinden.
    REGISTER_TARGET_POSITION: int | None = None
    REGISTER_TARGET_VELOCITY: int | None = None
    REGISTER_TORQUE_LIMIT: int | None = None
    REGISTER_CONTROL_WORD: int | None = None
    REGISTER_STATUS_WORD: int | None = None

    def __post_init__(self) -> None:
        from pymodbus.client import ModbusSerialClient  # nur importieren, wenn dieses Backend genutzt wird

        self._client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=self.bytesize,
            timeout=self.timeout_s,
        )
        if not self._client.connect():
            raise RuntimeError(f"Modbus-RTU-Verbindung zu {self.port} fehlgeschlagen.")

    def connect(self, torque_limit_nm: float) -> None:
        """MUSS das Drehmoment-Limit setzen, BEVOR irgendeine Bewegung
        gesendet wird (siehe Warnung im Modul-Docstring: minimale
        Sicherheitsmarge zwischen Drivetrain-Kapazität und Biegemoment)."""
        if self.REGISTER_TORQUE_LIMIT is None or self.REGISTER_CONTROL_WORD is None:
            raise NotImplementedError(
                "Modbus-Register für den MCDLN35SG sind noch nicht eingetragen "
                "(REGISTER_TORQUE_LIMIT/REGISTER_CONTROL_WORD u.a.) - siehe TODO "
                "in ModbusRTUBendingBackend. Erst aus dem Handbuch übernehmen, "
                "dann echte Hardware ansteuern."
            )
        # TODO: torque_limit_nm in die vom Verstärker erwartete Einheit/Skalierung
        # umrechnen und in REGISTER_TORQUE_LIMIT schreiben, danach Verstärker
        # über REGISTER_CONTROL_WORD freischalten (Servo-ON).
        raise NotImplementedError

    def move_by(self, delta: float) -> None:
        if self.REGISTER_TARGET_POSITION is None:
            raise NotImplementedError(
                "Modbus-Register für den MCDLN35SG sind noch nicht eingetragen "
                "(REGISTER_TARGET_POSITION u.a.) - siehe TODO in "
                "ModbusRTUBendingBackend."
            )
        # TODO: delta in die vom Verstärker erwartete Einheit (z.B. Encoder-
        # Inkremente) umrechnen, in REGISTER_TARGET_POSITION schreiben, dann
        # über REGISTER_STATUS_WORD auf "Bewegung abgeschlossen" pollen.
        raise NotImplementedError

    def close(self) -> None:
        self._client.close()


class GazeboBendingBackend:
    """Backend für eine simulierte Test-Achse in Gazebo (siehe
    fr20_ceiling_gazebo.build_bending_head_model_sdf) - NICHT der reale
    Biegekopf, sondern ein einfacher Platzhalter (Ständer + drehbare Welle,
    wie bei der früheren tube_rotation.py-Testachse), um Kinematik/Zeitablauf
    zu prüfen, solange die Modbus-Register des MCDLN35SG noch nicht final
    sind (siehe ModbusRTUBendingBackend).

    Der gz-sim JointPositionController hält ein ABSOLUTES Zielgelenk (rad),
    anders als der reale Servo, der über Modbus i.d.R. eine relative/absolute
    Sollposition in eigenen Einheiten bekommt - dieses Backend führt den
    absoluten Zielwinkel daher selbst mit (`_current_rad`). `delta`/Position
    werden hier in Grad interpretiert (die Testachse ist rotatorisch).

    Discovery-Warmup (bis gz-transport den Publisher sicher gefunden hat)
    passiert nur einmal in __init__, danach sendet move_by() direkt eine
    einzelne Nachricht - wichtig, da run_bending_test_sequence eine Bewegung
    in viele kleine Zwischenschritte zerlegt (siehe dort)."""

    def __init__(self, publishers: dict, start_position_deg: float = 0.0) -> None:
        import fr20_ceiling_gazebo as sim

        self._publishers = publishers
        self._current_rad = math.radians(start_position_deg)
        self._Double = sim.import_gz("msgs", "double_pb2").Double
        sim.send_joint_pose({"bending_head": self._current_rad}, self._publishers)

    def connect(self, torque_limit_nm: float) -> None:
        # der Gazebo-JointPositionController kennt kein Drehmoment-Limit -
        # rein informativ, damit dieselbe BendingBackend-Schnittstelle gilt.
        print(f"[Gazebo] Testachse verbunden (Drehmoment-Limit {torque_limit_nm:.1f} Nm gilt hier nur real, nicht in der Simulation).")

    def move_by(self, delta: float) -> None:
        self._current_rad += math.radians(delta)
        msg = self._Double()
        msg.data = self._current_rad
        self._publishers["bending_head"].publish(msg)


class BendingHeadAxis:
    """Hält die aktuell angenommene Position des Biegekopf-Motors nach und
    reicht Bewegungen an ein `BendingBackend` weiter.

    Die Position wird rein rechnerisch mitgeführt (kein Encoder-Feedback in
    dieser Klasse) - die übliche open-loop-Annahme, solange kein Ist-Wert
    aus dem Verstärker zurückgelesen wird.

    `move_by` ist per Lock serialisiert, analog zum bisherigen Muster in
    diesem Repo (Programm-Worker-Thread und Jog aus dem GUI-Hauptthread
    können sonst beim `current_position`-Update kollidieren)."""

    def __init__(self, backend: BendingBackend, torque_limit_nm: float = TORQUE_LIMIT_NM, start_position: float = 0.0) -> None:
        self._backend = backend
        backend.connect(torque_limit_nm)
        self.current_position = start_position
        self._lock = threading.Lock()

    def move_by(self, delta: float) -> float:
        """Fährt den Biegekopf-Motor um `delta` relativ zur aktuellen
        Stellung weiter. Gibt die neue absolute Position zurück."""
        with self._lock:
            self._backend.move_by(delta)
            self.current_position += delta
            return self.current_position


def run_bending_test_sequence(
    axis: BendingHeadAxis,
    steps: list[float],
    stop_event: "threading.Event | None" = None,
    pause_event: "threading.Event | None" = None,
    on_position_changed=None,
    settle_s=None,
    dwell_s: float = 0.5,
    step_size: float = 20.0,
) -> float:
    """Fährt eine Liste relativer Testschritte nacheinander ab - dasselbe
    stop_event/pause_event-Muster wie robot_program.run_program.

    Jeder Eintrag aus `steps` wird in `step_size`-große Zwischenschritte
    zerlegt (z.B. 180 in neun 20er-Schritten), nicht als ein Sprung
    gesendet - so werden pause_event/stop_event/on_position_changed alle
    `step_size` statt erst am Ende der ganzen Bewegung geprüft/ausgelöst.

    `stop_event`: sofortiger, endgültiger Abbruch vor dem nächsten
    Zwischenschritt. `pause_event`: hält vor jedem Zwischenschritt an, macht
    danach normal weiter - während der Pause darf der Aufrufer selbst
    `axis.move_by()` aufrufen (z.B. Jog-Buttons), ohne dass sich das mit
    dieser Funktion in die Quere kommt (siehe BendingHeadAxis-Lock).
    `on_position_changed`: Hook für die Live-Anzeige, pro Zwischenschritt
    aufgerufen. `settle_s`: Callable[[float], float], liefert die Wartezeit
    bis eine Bewegung abgeschlossen ist (z.B. für einen nicht-blockierenden
    gz-sim-Regler wie GazeboBendingBackend; bei einem blockierenden Backend
    wie ModbusRTUBendingBackend genügt `settle_s=None`). `dwell_s` gilt nur
    nach dem letzten Zwischenschritt eines `steps`-Eintrags.

    Das Ziel jedes `steps`-Eintrags wird EINMAL als absolute Position
    berechnet (`start + delta`), bevor der erste Zwischenschritt gesendet
    wird. Wird während einer Pause per Jog manuell nachgefahren
    (`axis.move_by()` von außen, siehe `pause_event`), wirkt sich das nicht
    auf dieses feste Ziel aus: die Restdistanz für jeden weiteren
    Zwischenschritt wird aus der tatsächlichen `axis.current_position` neu
    berechnet, nicht einfach mit dem ursprünglich geplanten festen
    Increment fortgesetzt - sonst würde ein Jog während der Pause das
    Endergebnis um genau den Jog-Betrag verschieben, statt ihn
    auszugleichen.

    Gibt die zuletzt erreichte Position (`axis.current_position`) zurück.
    """

    def stopped() -> bool:
        return stop_event is not None and stop_event.is_set()

    def wait_paused_or_stopped() -> bool:
        while pause_event is not None and pause_event.is_set() and not stopped():
            time.sleep(0.1)
        return stopped()

    for delta in steps:
        if delta == 0:
            continue
        n = max(1, math.ceil(abs(delta) / step_size))
        target = axis.current_position + delta

        for i in range(n):
            if wait_paused_or_stopped():
                print("Biegekopf-Testlauf abgebrochen.")
                return axis.current_position

            # Restdistanz zum festen Ziel neu aus der aktuellen Position
            # bestimmen - kompensiert einen manuellen Jog während der Pause,
            # statt mit dem ursprünglich geplanten Increment weiterzumachen.
            remaining_steps = n - i
            increment = (target - axis.current_position) / remaining_steps

            axis.move_by(increment)
            if on_position_changed is not None:
                on_position_changed(axis.current_position)

            is_last = i == n - 1
            wait_s = settle_s(increment) if settle_s is not None else 0.0
            deadline = time.time() + (max(wait_s, dwell_s) if is_last else wait_s)
            while time.time() < deadline and not stopped():
                time.sleep(min(0.05, max(0.0, deadline - time.time())))
            if stopped():
                print("Biegekopf-Testlauf abgebrochen.")
                return axis.current_position

    return axis.current_position


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("delta", type=float, help="Relative Sollbewegung (Grad in --gazebo, sonst Einheit noch offen, siehe TODOs)")
    parser.add_argument(
        "--port", default="/dev/ttyUSB0", help="Serieller Port des USB-RS485-Konverters (Standard: /dev/ttyUSB0)"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--simulate",
        action="store_true",
        help="Ohne echte Hardware testen - loggt nur, was gesendet würde (siehe SimulatedBendingBackend).",
    )
    mode.add_argument(
        "--gazebo",
        action="store_true",
        help="Startet Gazebo mit einer simulierten Testachse (siehe GazeboBendingBackend) und fährt dort "
        "den Biegekopf-Platzhalter - zum Testen von Kinematik/Zeitablauf, bevor die Modbus-Register final sind.",
    )
    parser.add_argument("--headless", action="store_true", help="Bei --gazebo: gz-sim ohne GUI starten")
    args = parser.parse_args()

    if args.gazebo:
        import fr20_ceiling_gazebo as sim

        print("Starte Gazebo mit der Biegekopf-Testachse ...")
        gz_proc, publishers = sim.launch_simulation(headless=args.headless)
        try:
            axis = BendingHeadAxis(GazeboBendingBackend(publishers))
            new_position = axis.move_by(args.delta)
            print(f"Biegekopf-Testachse auf {new_position:+.2f}° gefahren.")
            print("\nGazebo-Fenster schließen oder Strg+C hier drücken, um zu beenden.")
            gz_proc.wait()
        except KeyboardInterrupt:
            print("\nBeende Gazebo ...")
        finally:
            sim.shutdown_simulation(gz_proc)
        return

    backend: BendingBackend
    if args.simulate:
        backend = SimulatedBendingBackend()
    else:
        backend = ModbusRTUBendingBackend(port=args.port)

    axis = BendingHeadAxis(backend)
    new_position = axis.move_by(args.delta)
    print(f"Biegekopf-Motor auf {new_position:+.2f} gefahren.")


if __name__ == "__main__":
    main()
