#!/usr/bin/env python3
"""Ansteuerung des Biegekopf-Motors: der EINE reale Servo im System (Panasonic
MSMF082L1U1 + Verstärker Panasonic MCDLN35SG), der den Biegekopf antreibt -
der einzige anzusteuernde Motor in diesem Projekt.

Backend-Pattern: `BendingHeadAxis` kennt nur "fahre um X weiter" und hält die
Position nach, unabhängig von der physischen Umsetzung. Ein "Backend"
übernimmt die Kommunikation mit dem Verstärker - neue Hardware braucht nur
ein neues Backend (Schnittstelle: `connect`, `move_by`), der Rest
(robot_program.py, GUI) bleibt unverändert.

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


def _encode_int32(value: int) -> tuple[int, int]:
    """Zerlegt einen (ggf. negativen) 32-bit-Wert in zwei 16-bit-Register,
    NIEDERWERTIGES Register zuerst - so layoutet das MCDLN35SG-Handbuch
    (SX-DSV03042) durchgängig 32-bit-Werte über zwei Register (z.B.
    "Diagnosis message": Adresse N = untere 16 bit, N+1 = obere 16 bit;
    ebenso "Block[n].data": byte0/1 = LL/LH, byte2/3 = HL/HH)."""
    raw = value & 0xFFFFFFFF
    return raw & 0xFFFF, (raw >> 16) & 0xFFFF


@dataclass
class ModbusRTUBendingBackend:
    """Backend für den Panasonic MCDLN35SG-Verstärker (MINAS-A6-Serie) über
    Modbus-RTU/RS485 (USB-RS485-Konverter, 115200 Baud, 8N1).

    Register-Adressen stammen aus dem offiziellen Handbuch "Technical
    Reference - Modbus communication and Block operation Specification"
    (Panasonic Industry, Dok.-Nr. SX-DSV03042, Rev. R3.1) - siehe
    Seitenangaben in den Kommentaren unten.

    WICHTIG - Block-Betrieb statt einfachem Sollwert-Register: die
    MINAS-A6-Serie hat KEIN einzelnes "Sollposition schreiben"-Register.
    Stattdessen konfiguriert man einen "Block" (vorprogrammierter
    Bewegungssatz: Befehl + Zielwert + Geschwindigkeits-/Beschleunigungs-
    Referenz), wählt ihn per Blocknummer-Register aus und löst ihn per
    Strobe-Coil aus (SX-DSV03042 Kapitel 3/4, S.42-64). `move_by()` nutzt
    dafür immer denselben Block 0 mit dem Befehl "Relative positioning"
    (01h), da `move_by` selbst eine relative Bewegung ist.

    WICHTIG - noch offene Werte (bewusst NICHT geraten, siehe
    NotImplementedError unten):
      - `command_units_per_step`: die Umrechnung von `delta`
        (Biegekopf-Bewegungseinheit) in "command units" hängt vom
        elektronischen Getriebeverhältnis (Pr0.09/Pr0.10) ab, das nicht in
        diesem Modbus-Dokument steht, sondern in der separaten "Technical
        Reference (Functional Specification)" (SX-DSV03031) bzw. per
        Panaterm ausgelesen werden muss.
      - `rated_torque_nm`: um `torque_limit_nm` in den vom Verstärker
        erwarteten Drehmoment-Limit-Wert umzurechnen, wird das
        Nenndrehmoment des MSMF082L1U1 aus dem Motor-Datenblatt benötigt
        (nicht Teil des Modbus-Dokuments).
    Erst eintragen, dann echte Hardware ansteuern.
    """

    port: str = "/dev/ttyUSB0"
    baudrate: int = 115200
    parity: str = "N"   # 8N1
    stopbits: int = 1
    bytesize: int = 8
    slave_id: int = 1
    timeout_s: float = 1.0

    # --- Register-/Coil-Adressen (SX-DSV03042) -----------------------------
    # Modbus-Kommunikationsrechte holen/freigeben (S.24): 0055h = holen, 00AAh = freigeben.
    REGISTER_MODBUS_COMM_RIGHTS = 0x4300
    # Pr6.28 "Special function select" - Block-Betrieb per Modbus aktivieren
    # (Wert 1). Adresse nach der generischen Parameterformel
    # 3000h + Klasse*100h + Nr.*2h (S.22): 3000h + 6*100h + 28*2h.
    # ACHTUNG: Attribut "Reclose power supply" (S.73) - wirkt erst nach
    # einem Aus-/Wiedereinschalten des Verstärkers, muss also einmalig bei
    # der Inbetriebnahme (z.B. über Panaterm) gesetzt und dort auch mit
    # Spannungszyklus bestätigt werden; das Schreiben hier hält den
    # Parameter nur konsistent, aktiviert Block-Betrieb nicht "live".
    REGISTER_BLOCK_OPERATION_SELECT = 0x3000 + 6 * 0x100 + 28 * 2
    # Pr0.13 "1st torque limit setting" (Klasse 0, Nr. 13) und Pr5.22 "2nd
    # torque limit setting" (Klasse 5, Nr. 22) - erwähnt bei Alarm 29.2
    # "Counter overflow abnormal protection 2" als die Parameter, die das
    # Drehmoment-Limit setzen (S.69). Einheit/Skalierung NICHT im
    # Modbus-Dokument spezifiziert - siehe rated_torque_nm-TODO oben.
    REGISTER_TORQUE_LIMIT_1 = 0x3000 + 0 * 0x100 + 13 * 2
    REGISTER_TORQUE_LIMIT_2 = 0x3000 + 5 * 0x100 + 22 * 2
    # Block-Geschwindigkeit/-Beschleunigung/-Verzögerung, Slot 0 von 16 (S.26).
    REGISTER_BLOCK_VELOCITY_0 = 0x4600
    REGISTER_BLOCK_ACCELERATION_0 = 0x4610
    REGISTER_BLOCK_DECELERATION_0 = 0x4620
    # Block[0].command (2 Register) direkt gefolgt von Block[0].data (2
    # Register) - zusammen 4 Register ab 4800h (S.27, S.44-45).
    REGISTER_BLOCK_0_COMMAND = 0x4800
    REGISTER_BLOCK_0_DATA = 0x4802
    # "Block number" - wählt den auszuführenden Block (S.25).
    REGISTER_BLOCK_NUMBER = 0x4414
    # Coils: Servo-ON (S.19), Strobe/Start-Trigger (S.20), Action
    # implementation state BUSY (S.20, ReadOnly).
    COIL_SERVO_ON = 0x0060
    COIL_STROBE = 0x0120
    COIL_BUSY = 0x0140

    COMMAND_CODE_RELATIVE_POSITIONING = 0x01

    # TODO: elektronisches Getriebeverhältnis am Verstärker (Panaterm)
    # nachschlagen und hier die command-units pro Bewegungseinheit
    # eintragen - siehe Klassen-Docstring. Bis dahin NICHT raten.
    command_units_per_step: float | None = None
    # TODO: Nenndrehmoment (Nm) des MSMF082L1U1 aus dem Motor-Datenblatt
    # eintragen, um torque_limit_nm in den Verstärker-Sollwert umzurechnen.
    rated_torque_nm: float | None = None

    # Konservative Platzhalter-Bewegungsvorgabe für Block 0 (Slot 0) - vor
    # echtem Betrieb an die reale Mechanik/Traglast anpassen, keine
    # Handbuch-Vorgabe.
    velocity_rpm: float = 10.0
    acceleration_ms: int = 500
    deceleration_ms: int = 500
    move_timeout_s: float = 30.0

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

    def _write_registers(self, address: int, values: list[int]) -> None:
        result = self._client.write_registers(address, values, slave=self.slave_id)
        if result.isError():
            raise RuntimeError(f"Modbus-Schreibfehler auf Register {address:#06x}: {result}")

    def _write_coil(self, address: int, value: bool) -> None:
        result = self._client.write_coil(address, value, slave=self.slave_id)
        if result.isError():
            raise RuntimeError(f"Modbus-Schreibfehler auf Coil {address:#06x}: {result}")

    def _read_busy(self) -> bool:
        result = self._client.read_coils(self.COIL_BUSY, count=1, slave=self.slave_id)
        if result.isError():
            raise RuntimeError(f"Modbus-Lesefehler auf Coil {self.COIL_BUSY:#06x}: {result}")
        return bool(result.bits[0])

    def connect(self, torque_limit_nm: float) -> None:
        """MUSS das Drehmoment-Limit setzen, BEVOR irgendeine Bewegung
        gesendet wird (siehe Warnung im Modul-Docstring: minimale
        Sicherheitsmarge zwischen Drivetrain-Kapazität und Biegemoment)."""
        if self.rated_torque_nm is None:
            raise NotImplementedError(
                "rated_torque_nm ist nicht gesetzt - Nenndrehmoment des MSMF082L1U1 "
                "aus dem Motor-Datenblatt eintragen, um torque_limit_nm in den "
                "Verstärker-Sollwert umzurechnen (siehe Klassen-Docstring)."
            )
        # Modbus-Kommunikationsrechte holen (S.24) - danach sind Panaterm/
        # Frontpanel-Bedienung bis auf Monitor-Modus gesperrt.
        self._write_registers(self.REGISTER_MODBUS_COMM_RIGHTS, [0x0055])

        # Block-Betrieb per Modbus aktivieren (Pr6.28 = 1) - siehe Kommentar
        # bei REGISTER_BLOCK_OPERATION_SELECT: wirkt erst nach Neustart des
        # Verstärkers, hier nur zur Konsistenz mitgeschrieben.
        self._write_registers(self.REGISTER_BLOCK_OPERATION_SELECT, list(_encode_int32(1)))

        # Drehmoment-Limit: Prozent des Nenndrehmoments in 0.1%-Schritten
        # (Panasonic-übliche Skalierung für MINAS-Torque-Limit-Parameter -
        # siehe rated_torque_nm-TODO, falls die reale Skalierung abweicht).
        torque_limit_pct_x10 = round(torque_limit_nm / self.rated_torque_nm * 1000)
        self._write_registers(self.REGISTER_TORQUE_LIMIT_1, list(_encode_int32(torque_limit_pct_x10)))
        self._write_registers(self.REGISTER_TORQUE_LIMIT_2, list(_encode_int32(torque_limit_pct_x10)))

        # Servo erst NACH gesetztem Drehmoment-Limit freischalten.
        self._write_coil(self.COIL_SERVO_ON, True)
        print(
            f"[Modbus] Verbindung aufgebaut, Drehmoment-Limit={torque_limit_nm:.1f} Nm "
            f"({torque_limit_pct_x10 / 10:.1f}% des Nenndrehmoments) gesetzt, Servo ON."
        )

    def move_by(self, delta: float) -> None:
        if self.command_units_per_step is None:
            raise NotImplementedError(
                "command_units_per_step ist nicht gesetzt - elektronisches "
                "Getriebeverhältnis (Pr0.09/Pr0.10, per Panaterm auslesen) "
                "eintragen, um delta in Modbus-'command units' umzurechnen "
                "(siehe Klassen-Docstring)."
            )
        travel_command_units = round(delta * self.command_units_per_step)

        # Geschwindigkeit/Beschleunigung/Verzögerung für Slot 0 setzen -
        # siehe velocity_rpm/acceleration_ms/deceleration_ms-Kommentar
        # (konservative Platzhalter, vor Betrieb anpassen).
        self._write_registers(self.REGISTER_BLOCK_VELOCITY_0, [round(self.velocity_rpm)])
        self._write_registers(self.REGISTER_BLOCK_ACCELERATION_0, [self.acceleration_ms])
        self._write_registers(self.REGISTER_BLOCK_DECELERATION_0, [self.deceleration_ms])

        # Block[0] = "Relative positioning" (01h), Velocity-/Accel-/
        # Decel-Nummer je 0 (Slot 0), Block transition condition = 0
        # (endet nach Abschluss der Positionierung, siehe SX-DSV03042 S.50).
        command_code = self.COMMAND_CODE_RELATIVE_POSITIONING
        velocity_number = accel_number = decel_number = 0
        block_transition_condition = 0
        byte2 = (velocity_number << 4) | accel_number
        byte1 = (decel_number << 4) | block_transition_condition
        command_word = (command_code << 24) | (byte2 << 16) | (byte1 << 8)
        command_low, command_high = _encode_int32(command_word)
        data_low, data_high = _encode_int32(travel_command_units)
        self._write_registers(
            self.REGISTER_BLOCK_0_COMMAND,
            [command_low, command_high, data_low, data_high],
        )

        # Block 0 auswählen und per Strobe-Flanke (OFF->ON) auslösen (S.63).
        self._write_registers(self.REGISTER_BLOCK_NUMBER, [0])
        self._write_coil(self.COIL_STROBE, False)
        time.sleep(0.02)  # Mindestabstand vor der auslösenden Flanke (siehe S.65, *1)
        self._write_coil(self.COIL_STROBE, True)

        deadline = time.time() + self.move_timeout_s
        while time.time() < deadline:
            if self._read_busy():
                time.sleep(0.05)
                continue
            self._write_coil(self.COIL_STROBE, False)
            return
        self._write_coil(self.COIL_STROBE, False)
        raise TimeoutError(
            f"Biegekopf-Motor hat die Bewegung nicht innerhalb von {self.move_timeout_s:.0f}s "
            f"abgeschlossen (BUSY-Coil {self.COIL_BUSY:#06x} bleibt gesetzt)."
        )

    def close(self) -> None:
        self._client.close()


class GazeboBendingBackend:
    """Backend für eine simulierte Test-Achse in Gazebo (siehe
    fr20_ceiling_gazebo.build_bending_head_model_sdf) - NICHT der reale
    Biegekopf, sondern ein einfacher Platzhalter, um Kinematik/Zeitablauf zu
    prüfen, solange die Modbus-Register des MCDLN35SG noch nicht final sind.

    Der gz-sim JointPositionController hält ein ABSOLUTES Zielgelenk (rad) -
    dieses Backend führt den absoluten Zielwinkel daher selbst mit
    (`_current_rad`). `delta`/Position werden hier in Grad interpretiert
    (die Testachse ist rotatorisch)."""

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
    reicht Bewegungen an ein `BendingBackend` weiter. Rein rechnerisch
    mitgeführt (kein Encoder-Feedback), solange kein Ist-Wert zurückgelesen
    wird. `move_by` ist per Lock serialisiert - Programm-Worker-Thread und
    Jog aus dem GUI-Hauptthread könnten sonst beim `current_position`-Update
    kollidieren."""

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
    zerlegt (nicht als ein Sprung), damit pause_event/stop_event/
    on_position_changed regelmäßig statt erst am Ende geprüft/ausgelöst werden.

    `stop_event`: sofortiger, endgültiger Abbruch vor dem nächsten
    Zwischenschritt. `pause_event`: hält vor jedem Zwischenschritt an -
    während der Pause darf der Aufrufer selbst `axis.move_by()` aufrufen
    (z.B. Jog-Buttons), ohne mit dieser Funktion zu kollidieren (siehe
    BendingHeadAxis-Lock). `on_position_changed`: Hook pro Zwischenschritt.
    `settle_s`: Callable[[float], float], Wartezeit bis eine Bewegung
    abgeschlossen ist (für einen nicht-blockierenden Regler wie
    GazeboBendingBackend; bei blockierenden Backends genügt `None`).
    `dwell_s` gilt nur nach dem letzten Zwischenschritt eines Eintrags.

    Das Ziel jedes Eintrags wird EINMAL als absolute Position berechnet
    (`start + delta`). Ein Jog während einer Pause wirkt sich nicht auf
    dieses feste Ziel aus: die Restdistanz für jeden weiteren Zwischenschritt
    wird aus der tatsächlichen `axis.current_position` neu berechnet, statt
    mit dem ursprünglich geplanten Increment fortzufahren - sonst würde der
    Jog das Endergebnis um genau seinen Betrag verschieben.

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
