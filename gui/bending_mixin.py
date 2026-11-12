"""Biegekopf-Testlauf-Fenster (window_biegekopf) - unabhängig vom FR20-Arm-Programm, eigener
Thread und eigenes Jog-Kreuz. Siehe bending_head_motor.py für das Backend-Pattern."""

import math
import threading
import traceback

import fr20_ceiling_gazebo as sim
import bending_head_motor as bhm

from constants import JOG_REPEAT_MS


class BendingMixin:
    def on_biegekopf_pressed(self):
        """Button 'Biegekopf testen' (vierte Hauptfenster-Kachel): Achsen-Testlauf für die
        Biegekopf-Testachse in Gazebo (180°, dann -360°), unabhängig vom FR20-Arm.

        Nutzt bewusst GazeboBendingBackend statt ModbusRTUBendingBackend: die echten
        Modbus-Register sind noch nicht final verdrahtet (siehe bending_head_motor.py) - ein
        Testen-Button darf nicht versehentlich den echten Servo ansteuern. Die simulierte
        Testachse ist kein Nachbau des realen Biegekopfs.

        Läuft in self._bending_thread (nicht self._program_thread), da der Biegekopf-Motor
        ein anderer Antrieb als der Arm ist und parallel zu einem Arm-Programm laufen darf."""
        if self._bending_thread is not None and self._bending_thread.is_alive():
            print("Biegekopf-Testlauf läuft bereits.")
            return

        self.mainwindow.withdraw()
        if getattr(self, "window_biegekopf", None) is None:
            self.window_biegekopf = self.builder.get_object("window_biegekopf", self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_biegekopf.configure(background=self._palette()["bg"])
            self._style_widget("label_biegekopf_status", "Title.TLabel")
            self._style_widget("label_biegekopf_position", "Readout.TLabel")
            self._style_widget("frame_jog_biegekopf", "Card.TLabelframe")
            self._style_widget("frame_ablauf_biegekopf", "Card.TLabelframe")
            self._style_widget("button_biegekopf_pause", "Warning.TButton")
            self._style_widget("button_biegekopf_weiter", "Success.TButton")
            self._style_widget("button_biegekopf_abbruch", "Danger.TButton")
            for widget_id, sign in (
                ("button_biegekopf_jog_minus", -1),
                ("button_biegekopf_jog_plus", 1),
            ):
                widget = self.builder.get_object(widget_id, self.mainwindow)
                widget.bind("<ButtonPress-1>", lambda event, s=sign: self._hold_bending_jog(s))
                widget.bind("<ButtonRelease-1>", lambda event: self._release_bending_jog())
                widget.configure(style="Jog.TButton")
            self._make_responsive(self.window_biegekopf, self.builder.get_object("frame_biegekopf"))
            self._fit_window_to_screen(self.window_biegekopf)
        else:
            self.window_biegekopf.deiconify()

        self._bending_pause_event.clear()
        with self._bending_pose_lock:
            self._bending_current_position = 0.0
        stop_event = threading.Event()
        self._bending_stop_event = stop_event

        def on_position_changed(position: float) -> None:
            with self._bending_pose_lock:
                self._bending_current_position = position

        def settle_s(delta_deg: float) -> float:
            # dieselben Sicherheitsfaktoren wie robot_program._min_settle_time
            delta_rad = abs(math.radians(delta_deg))
            return delta_rad / (sim.BENDING_HEAD_VELOCITY_RAD_S * 0.6) + 0.4

        def back_to_main() -> None:
            self.window_biegekopf.withdraw()
            self.mainwindow.deiconify()

        def worker() -> None:
            try:
                publishers = self._ensure_simulation_running()
                axis = bhm.BendingHeadAxis(bhm.GazeboBendingBackend(publishers))
                self._bending_axis = axis
                print("Biegekopf-Testlauf gestartet: 180°, dann -360°.")
                bhm.run_bending_test_sequence(
                    axis,
                    steps=[180.0, -360.0],
                    stop_event=stop_event,
                    pause_event=self._bending_pause_event,
                    on_position_changed=on_position_changed,
                    settle_s=settle_s,
                )
                if not stop_event.is_set():
                    print("Biegekopf-Testlauf beendet.")
            except Exception as exc:  # noqa: BLE001 - im Hintergrund-Thread sichtbar loggen
                print(f"Fehler beim Biegekopf-Testlauf: {exc}")
                traceback.print_exc()
            finally:
                self._bending_axis = None
                if not stop_event.is_set():
                    self.mainwindow.after(0, back_to_main)

        self._bending_thread = threading.Thread(target=worker, daemon=True)
        self._bending_thread.start()

    def on_biegekopf_pause_pressed(self):
        """Button 'Pause' in window_biegekopf - analog zu on_pause_pressed beim Arm."""
        if self._bending_thread is None or not self._bending_thread.is_alive():
            print("Kein Biegekopf-Testlauf läuft gerade - Pause hat nichts zu tun.")
            return
        self._bending_pause_event.set()
        print("Biegekopf-Testlauf pausiert - Jog-Pfeile sind jetzt aktiv.")

    def on_biegekopf_weiter_pressed(self):
        """Button 'Weiter' in window_biegekopf: hebt die Pause auf."""
        if not self._bending_pause_event.is_set():
            print("Biegekopf-Testlauf ist nicht pausiert - nichts zu tun.")
            return
        self._bending_pause_event.clear()
        print("Biegekopf-Testlauf wird fortgesetzt.")

    def on_biegekopf_abbruch_pressed(self):
        """Button 'Abbruch' in window_biegekopf: bricht den Testlauf sofort ab."""
        if self._bending_thread is None or not self._bending_thread.is_alive():
            print("Kein Biegekopf-Testlauf läuft gerade - Abbruch hat nichts zu tun.")
            return
        self._bending_pause_event.clear()  # sonst würde run_bending_test_sequence auf die Pause warten
        self._bending_stop_event.set()
        self._bending_thread.join(timeout=5.0)
        print("Biegekopf-Testlauf abgebrochen.")
        self.window_biegekopf.withdraw()
        self.mainwindow.deiconify()

    def _do_bending_jog_step(self, sign: float) -> bool:
        """Fährt den Biegekopf-Motor manuell einen Schritt weiter - nur erlaubt, während ein
        laufender Testlauf pausiert ist (dasselbe Prinzip wie beim Arm-Jog, siehe _jog)."""
        if (
            self._bending_thread is not None
            and self._bending_thread.is_alive()
            and not self._bending_pause_event.is_set()
        ):
            print("Biegekopf-Jog ignoriert: Testlauf läuft gerade - erst 'Pause' drücken.")
            return False
        if self._bending_axis is None:
            print("Biegekopf-Jog ignoriert: keine Achse aktiv.")
            return False
        new_position = self._bending_axis.move_by(sign * self._bending_jog_step_size)
        with self._bending_pose_lock:
            self._bending_current_position = new_position
        return True

    def _hold_bending_jog(self, sign: float) -> None:
        """Wie _hold_jog, aber für die Biegekopf-Jog-Pfeile."""
        if not self._do_bending_jog_step(sign):
            self._bending_jog_after_id = None
            return
        self._bending_jog_after_id = self.mainwindow.after(
            JOG_REPEAT_MS, lambda: self._hold_bending_jog(sign)
        )

    def _release_bending_jog(self) -> None:
        """Beendet die per _hold_bending_jog gestartete Wiederholung."""
        if self._bending_jog_after_id is not None:
            self.mainwindow.after_cancel(self._bending_jog_after_id)
            self._bending_jog_after_id = None
