"""Jog-Steuerung des FR20-Arms (X/Y/Z/Werkzeugrotation) - Drücken-halten-loslassen über die
Jog-Kreuze in den Fenstern Kalibrierung/Programm/Manuell. Für den Biegekopf-Motor-Jog siehe
bending_mixin.py (eigener Antrieb, eigene Achse)."""

import math

import fr20_ceiling_gazebo as sim
import fr20_kinematics as kin
import robot_program as rp

from constants import JOG_PAD_BUTTONS, JOG_REPEAT_MS


class JogMixin:
    def _jog(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, drot_deg: float = 0.0) -> bool:
        """Bewegt den Arm um einen kleinen Schritt in X/Y/Z oder dreht das Werkzeug (j6) um
        drot_deg, ausgehend von der zuletzt bekannten Pose. dx/dy/dz und drot_deg schließen
        sich gegenseitig aus. Gibt True zurück, wenn der Schritt gesendet wurde.

        Läuft bewusst nicht über die Rückzug/Verfahren/Absenken-Logik aus robot_program.py -
        Jog-Schritte sind klein genug für direkte kontinuitätsbasierte IK ohne Höhenumweg.

        Läuft ein Programm pausiert, wird mitbewegt; läuft es unpausiert, wird der Jog
        verweigert (gleichzeitiges Schreiben auf dieselben Publisher würde kollidieren)."""
        if (
            self._program_thread is not None
            and self._program_thread.is_alive()
            and not self._pause_event.is_set()
        ):
            print("Jog ignoriert: Programm läuft gerade - erst 'Pause' drücken.")
            return False

        with self._pose_lock:
            current = self._current_joints

        if current is None or self._publishers is None:
            print("Jog ignoriert: Simulation ist noch nicht bereit.")
            return False

        Double = sim.import_gz("msgs", "double_pb2").Double

        if drot_deg:
            solution = dict(current)
            solution["j6"] = current["j6"] + math.radians(drot_deg)
            msg = Double()
            msg.data = solution["j6"]
            self._publishers["j6"].publish(msg)
            with self._pose_lock:
                self._current_joints = solution
            return True

        pose = kin.forward_kinematics(current)
        x, y, z = pose[:3, 3]
        target = rp.Point(name="Jog", xyz=(x + dx, y + dy, z + dz))
        try:
            solution = rp.resolve_point(target, current, allow_fallback=False)
        except (ValueError, RuntimeError) as exc:
            print(f"Jog fehlgeschlagen: {exc}")
            return False

        for joint, angle in solution.items():
            msg = Double()
            msg.data = angle
            self._publishers[joint].publish(msg)

        with self._pose_lock:
            self._current_joints = solution
        return True

    def _hold_jog(self, axis: str, sign: float) -> None:
        """Fährt beim Drücken eines Jog-Buttons sofort einen Schritt und plant sich per after()
        alle JOG_REPEAT_MS erneut ein, solange der Button gehalten wird - _release_jog bricht
        die Kette beim Loslassen ab, ebenso ein fehlschlagender _jog() (z.B. Reichweitenende)."""
        kwargs = {}
        if axis == "x":
            kwargs["dx"] = sign * self._jog_step_m
        elif axis == "y":
            kwargs["dy"] = sign * self._jog_step_m
        elif axis == "z":
            kwargs["dz"] = sign * self._jog_step_m
        elif axis == "rot":
            kwargs["drot_deg"] = sign * self._jog_rotation_step_deg
        if not self._jog(**kwargs):
            self._jog_after_id = None
            return
        self._jog_after_id = self.mainwindow.after(JOG_REPEAT_MS, lambda: self._hold_jog(axis, sign))

    def _release_jog(self) -> None:
        """Beendet die per _hold_jog gestartete Wiederholung (Loslassen des Jog-Buttons)."""
        if self._jog_after_id is not None:
            self.mainwindow.after_cancel(self._jog_after_id)
            self._jog_after_id = None

    def _bind_jog_hold(self, widget_id: str, axis: str, sign: float, symbol: str) -> None:
        """Verbindet einen Jog-Button mit Drücken-halten-loslassen statt Einzelklick und
        stylt ihn mit einem Pfeilsymbol. Wird beim ersten Öffnen eines Jog-Fensters aufgerufen."""
        widget = self.builder.get_object(widget_id, self.mainwindow)
        widget.bind("<ButtonPress-1>", lambda event: self._hold_jog(axis, sign))
        widget.bind("<ButtonRelease-1>", lambda event: self._release_jog())
        widget.configure(style="Jog.TButton", text=symbol)

    def _bind_jog_pad(self, suffix: str) -> None:
        """Bindet das komplette Jog-Kreuz (X/Y/Z/Rot, je +/-) für ein Fenster - z.B.
        `_bind_jog_pad("kalibrierung")` bindet button_jog_xplus_kalibrierung usw. (siehe
        JOG_PAD_BUTTONS). Gemeinsamer Helfer für die drei identisch aufgebauten Jog-Fenster
        (Kalibrierung/Programm/Manuell)."""
        for axis, sign, symbol in JOG_PAD_BUTTONS:
            direction = "plus" if sign > 0 else "minus"
            self._bind_jog_hold(f"button_jog_{axis}{direction}_{suffix}", axis=axis, sign=sign, symbol=symbol)

    def _bind_jog_toggle(self, toggle_button_id: str, controls_frame_id: str) -> None:
        """Macht die Kopfzeile eines Jog-Bereichs klickbar: blendet die Pfeil-Buttons darunter
        per grid()/grid_remove() ein/aus, Pfeil im Button-Text (▾/▸) zeigt den Zustand."""
        button = self.builder.get_object(toggle_button_id, self.mainwindow)
        controls = self.builder.get_object(controls_frame_id, self.mainwindow)
        window = button.winfo_toplevel()

        def toggle():
            if controls.winfo_ismapped():
                controls.grid_remove()
                button.configure(text=button.cget("text").replace("▾", "▸"))
            else:
                controls.grid()
                button.configure(text=button.cget("text").replace("▸", "▾"))
            # Fenster hat feste Größe (siehe _fit_window_to_screen) - Neuberechnung nötig,
            # sonst bleibt nach dem Einklappen eine leere Fläche stehen.
            # Im Vollbild (Kiosk-Modus) bleibt das Fenster bildschirmfüllend, Inhalt zentriert.
            if not self._kiosk:
                window.update_idletasks()
                window.geometry(f"{window.winfo_reqwidth()}x{window.winfo_reqheight()}")

        button.configure(command=toggle, style="JogToggle.TButton")

    def _update_position_readout(self) -> None:
        """Aktualisiert die Live-Positionsanzeige in allen bereits geöffneten Jog-Fenstern und
        plant sich selbst alle 300ms erneut - ein zentraler Loop statt einem pro Fenster."""
        with self._pose_lock:
            current = self._current_joints
        if current is not None:
            pose = kin.forward_kinematics(current)
            x, y, z = pose[:3, 3]
            text = f"TCP  x={x:+.3f}  y={y:+.3f}  z={z:+.3f} m   j6={math.degrees(current['j6']):+.1f}°"
            for window_attr, label_id in (
                ("window_programm", "label_position_readout_programm"),
                ("window_kalibrierung", "label_position_readout_kalibrierung"),
                ("window_manuell", "label_position_readout_manuell"),
            ):
                if getattr(self, window_attr, None) is not None:
                    self.builder.get_object(label_id, self.mainwindow).configure(text=text)

        if getattr(self, "window_biegekopf", None) is not None:
            with self._bending_pose_lock:
                bending_position = self._bending_current_position
            self.builder.get_object("label_biegekopf_position", self.mainwindow).configure(
                text=f"Winkel: {bending_position:+.1f}°"
            )

        self.mainwindow.after(300, self._update_position_readout)
