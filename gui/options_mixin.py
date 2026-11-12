"""Nebenfenster ohne eigenen Ausführungs-Thread: Testen-Auswahl, Optionen (Comboboxen,
Dark Mode, Entwicklermodus) und die freie Manuelle Steuerung."""

from tkinter import messagebox, simpledialog

from constants import (
    BENDING_JOG_STEP_OPTIONS,
    DEV_MODE_PASSWORD,
    FONT_SIZE_OPTIONS,
    JOG_STEP_OPTIONS,
    ZOOM_OPTIONS,
    BASE_TK_SCALING,
)


class OptionsMixin:
    def on_testen_pressed(self):
        """Button 'Testen': öffnet window_testen mit den Start-Buttons für Gelenkpose/Tisch."""
        self.mainwindow.withdraw()
        if getattr(self, 'window_testen', None) is None:
            self.window_testen = self.builder.get_object('window_testen', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_testen.configure(background=self._palette()["bg"])
            self._style_widget("label_testen_title", "Title.TLabel")
            self._style_widget("card_gelenkpose", "Card.TLabelframe")
            self._style_widget("card_tisch", "Card.TLabelframe")
            self._style_widget("button_start_gelenkpose", "Primary.TButton")
            self._style_widget("button_start_tisch", "Primary.TButton")
            self._style_widget("button_testen_zurueck", "Secondary.TButton")
            self._make_responsive(self.window_testen, self.builder.get_object("frame_testen"))
            self._fit_window_to_screen(self.window_testen)
        else:
            self.window_testen.deiconify()

    def on_testen_zurueck_pressed(self):
        """Button '◂ Zurück' in window_testen."""
        self.window_testen.withdraw()
        self.mainwindow.deiconify()

    def on_optionen_pressed(self):
        """Button 'Optionen': öffnet window_optionen - Hauptfenster bleibt bewusst sichtbar,
        da Kalibrierung/Manuelle Steuerung parallel dazu nutzbar bleiben sollen."""
        if getattr(self, 'window_optionen', None) is None:
            self.window_optionen = self.builder.get_object('window_optionen', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_optionen.configure(background=self._palette()["bg"])
            self._style_widget("label_optionen_title", "Title.TLabel")
            self._style_widget("button_optionen_kalibrierung", "Primary.TButton")
            self._style_widget("button_optionen_manuell", "Primary.TButton")
            self._style_widget("button_optionen_log", "Primary.TButton")
            self._style_widget("frame_einstellungen", "Card.TLabelframe")
            self._style_widget("button_optionen_schliessen", "Secondary.TButton")

            # "readonly" verhindert Freitext-Eingabe; aktueller Wert wird als Startauswahl gesetzt.
            combo_jogstep = self.builder.get_object("combo_jogstep", self.mainwindow)
            combo_jogstep["values"] = list(JOG_STEP_OPTIONS.keys())
            combo_jogstep.state(["readonly"])
            combo_jogstep.set(next(k for k, v in JOG_STEP_OPTIONS.items() if v == self._jog_step_m))
            combo_jogstep.bind("<<ComboboxSelected>>", lambda e: self._on_jogstep_selected(combo_jogstep.get()))

            combo_schrift = self.builder.get_object("combo_schriftgroesse", self.mainwindow)
            combo_schrift["values"] = list(FONT_SIZE_OPTIONS.keys())
            combo_schrift.state(["readonly"])
            combo_schrift.set(next(k for k, v in FONT_SIZE_OPTIONS.items() if v == self._font_size))
            combo_schrift.bind("<<ComboboxSelected>>", lambda e: self._on_schriftgroesse_selected(combo_schrift.get()))

            combo_zoom = self.builder.get_object("combo_zoom", self.mainwindow)
            combo_zoom["values"] = list(ZOOM_OPTIONS.keys())
            combo_zoom.state(["readonly"])
            combo_zoom.set(next(k for k, v in ZOOM_OPTIONS.items() if v == self._zoom))
            combo_zoom.bind("<<ComboboxSelected>>", lambda e: self._on_zoom_selected(combo_zoom.get()))

            combo_biegekopf_jogstep = self.builder.get_object("combo_biegekopf_jogstep", self.mainwindow)
            combo_biegekopf_jogstep["values"] = list(BENDING_JOG_STEP_OPTIONS.keys())
            combo_biegekopf_jogstep.state(["readonly"])
            combo_biegekopf_jogstep.set(
                next(k for k, v in BENDING_JOG_STEP_OPTIONS.items() if v == self._bending_jog_step_size)
            )
            combo_biegekopf_jogstep.bind(
                "<<ComboboxSelected>>", lambda e: self._on_biegekopf_jogstep_selected(combo_biegekopf_jogstep.get())
            )

            self._style_widget(
                "button_darkmode_toggle",
                "Secondary.TButton",
                text="☀ Helles Design" if self._dark_mode else "🌙 Dunkles Design",
            )

            self._style_widget("frame_entwicklermodus", "Card.TLabelframe")
            self._style_widget(
                "button_entwicklermodus_toggle",
                "Secondary.TButton",
                text="🔧 Entwicklermodus deaktivieren" if self._dev_mode else "🔧 Entwicklermodus aktivieren",
            )
            checkbutton_einzelschritt = self.builder.get_object("checkbutton_einzelschritt", self.mainwindow)
            checkbutton_einzelschritt.configure(variable=self._single_step_var)
            checkbutton_einzelschritt.state(["!disabled"] if self._dev_mode else ["disabled"])

            self._make_responsive(self.window_optionen, self.builder.get_object("frame_optionen"))
            self._fit_window_to_screen(self.window_optionen)
        else:
            self.window_optionen.deiconify()
        self.window_optionen.lift()
        self.window_optionen.focus_force()

    def on_optionen_schliessen_pressed(self):
        """Button '◂ Schließen' in window_optionen - Hauptfenster war nie versteckt."""
        self.window_optionen.withdraw()

    def _on_jogstep_selected(self, label: str) -> None:
        self._jog_step_m = JOG_STEP_OPTIONS[label]
        print(f"Jog-Schrittweite: {label}")

    def _on_biegekopf_jogstep_selected(self, label: str) -> None:
        self._bending_jog_step_size = BENDING_JOG_STEP_OPTIONS[label]
        print(f"Biegekopf-Schrittweite: {label}")

    def _on_schriftgroesse_selected(self, label: str) -> None:
        self._font_size = FONT_SIZE_OPTIONS[label]
        self._apply_style()
        print(f"Schriftgröße: {label}")

    def _on_zoom_selected(self, label: str) -> None:
        """Setzt den Tcl/Tk-Skalierungsfaktor - wirkt sofort auf neue/neu geöffnete Fenster;
        bereits offene Fenster zeigen den Zoom erst nach Schließen/Wiederöffnen vollständig."""
        self._zoom = ZOOM_OPTIONS[label]
        self.mainwindow.tk.call("tk", "scaling", self._zoom * BASE_TK_SCALING)
        print(f"Zoom: {label}")

    def on_darkmode_toggle_pressed(self):
        """Button 'Dunkles Design'/'Helles Design': wechselt Palette und wendet sie auf
        ttk-Styles sowie alle offenen Toplevel-Fenster an."""
        self._dark_mode = not self._dark_mode
        self._apply_style()
        self._refresh_toplevel_backgrounds()
        self._style_widget(
            "button_darkmode_toggle",
            "Secondary.TButton",
            text="☀ Helles Design" if self._dark_mode else "🌙 Dunkles Design",
        )

    def on_entwicklermodus_toggle_pressed(self):
        """Aktivieren braucht DEV_MODE_PASSWORD (kein echter Sicherheitsmechanismus, nur
        Schutz vor Versehen). Deaktivieren schaltet Einzelschrittbetrieb mit ab und gibt ein
        evtl. wartendes Programm sofort frei, sonst bliebe es hängen."""
        if not self._dev_mode:
            pw = simpledialog.askstring(
                "Entwicklermodus",
                "Passwort eingeben:",
                show="*",
                parent=self.window_optionen,
            )
            if pw != DEV_MODE_PASSWORD:
                if pw is not None:  # None = Dialog abgebrochen, keine Meldung nötig
                    messagebox.showerror("Entwicklermodus", "Falsches Passwort.")
                return
            self._dev_mode = True
            print("Entwicklermodus aktiviert.")
        else:
            self._dev_mode = False
            self._single_step_var.set(False)
            self._step_event.set()  # evtl. wartendes Programm freigeben
            print("Entwicklermodus deaktiviert.")

        self._style_widget(
            "button_entwicklermodus_toggle",
            "Secondary.TButton",
            text="🔧 Entwicklermodus deaktivieren" if self._dev_mode else "🔧 Entwicklermodus aktivieren",
        )
        checkbutton = self.builder.get_object("checkbutton_einzelschritt", self.mainwindow)
        checkbutton.state(["!disabled"] if self._dev_mode else ["disabled"])

    def on_einzelschritt_toggled(self):
        """Wird ausgeschaltet, während ein Programm bereits wartet, gibt das Warten sofort
        frei, statt den Roboter stehen zu lassen."""
        if not self._single_step_var.get():
            self._step_event.set()
        print(f"Einzelschrittbetrieb: {'ein' if self._single_step_var.get() else 'aus'}")

    def on_manuell_pressed(self):
        """Button 'Manuelle Steuerung': freie Jog-Steuerung ohne laufendes Punktprogramm."""
        if getattr(self, 'window_manuell', None) is None:
            self.window_manuell = self.builder.get_object('window_manuell', self.mainwindow)
            self.builder.connect_callbacks(self)
            self.window_manuell.configure(background=self._palette()["bg"])
            self._bind_jog_pad("manuell")
            self._bind_jog_toggle('button_toggle_jog_manuell', 'frame_jog_controls_manuell')
            self._style_widget("label_manuell_title", "Title.TLabel")
            self._style_widget("frame_jog_manuell", "Card.TLabelframe")
            self._style_widget("frame_jog_controls_manuell", "Card.TFrame")
            self._style_widget("label_jog_xy_header_manuell", "JogAxis.TLabel")
            self._style_widget("label_jog_z_header_manuell", "JogAxis.TLabel")
            self._style_widget("label_jog_rot_header_manuell", "JogAxis.TLabel")
            self._style_widget("label_jog_center_manuell", "JogCenter.TLabel")
            self._style_widget("label_position_readout_manuell", "Readout.TLabel")
            self._style_widget("button_manuell_schliessen", "Secondary.TButton")
            self._make_responsive(self.window_manuell, self.builder.get_object("frame_manuell"))
            self._fit_window_to_screen(self.window_manuell)
        else:
            self.window_manuell.deiconify()
        self.window_manuell.lift()
        self.window_manuell.focus_force()

    def on_manuell_schliessen_pressed(self):
        self._release_jog()
        self.window_manuell.withdraw()
