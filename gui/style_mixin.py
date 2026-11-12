"""Theming/Layout-Helfer für MyApp - Farbpalette, ttk-Styles, Fenstergrößen. Wird von MyApp
(siehe my_gui.py) über Mehrfachvererbung eingebunden, damit `self` weiterhin die eine
pygubu-Callback-Instanz bleibt."""

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

from constants import COMPACT_LAYOUT_MAX_HEIGHT, KIOSK_MODE, KIOSK_MONITOR, LOGO_FILE, PROJECT_PATH
from display import find_target_monitor


class StyleMixin:
    def set_image(self, widget_id, filename, size=(512, 512)):
        """Lädt ein Bild, skaliert es und setzt es auf ein Widget."""
        img = Image.open(PROJECT_PATH / filename).resize(size)
        photo = ImageTk.PhotoImage(img)
        self._images[widget_id] = photo  # Referenz behalten!
        self.builder.get_object(widget_id).configure(image=photo)

    def _palette(self) -> dict[str, str]:
        """Farbpalette abhängig von self._dark_mode - zentral, damit _apply_style() und
        _refresh_toplevel_backgrounds() dieselben Farben verwenden."""
        if self._dark_mode:
            return {
                "bg": "#11141b",
                "card_bg": "#1c212c",
                "text": "#f1f3f9",
                "muted": "#9aa4b8",
                "accent": "#7c8bff",
                "accent_active": "#98a5ff",
                "border": "#333c4d",
                "jog_hover": "#2a3140",
                "success": "#34d399",
                "success_active": "#5fe3b2",
                "warning": "#fbbf24",
                "warning_active": "#fdd35a",
                "danger": "#f87171",
                "danger_active": "#fb9d9d",
                "secondary": "#5c6577",
                "secondary_active": "#707b90",
                "jog": "#3d4658",
                "jog_active": "#4b5670",
            }
        return {
            "bg": "#eef1f6",
            "card_bg": "#ffffff",
            "text": "#1b1f27",
            "muted": "#5b6472",
            "accent": "#3654d1",
            "accent_active": "#2842a8",
            "border": "#d7dbe3",
            "jog_hover": "#e7ecf7",
            "success": "#2f9e57",
            "success_active": "#22803f",
            "warning": "#d98c00",
            "warning_active": "#b57400",
            "danger": "#d9453f",
            "danger_active": "#b7332e",
            "secondary": "#8a93a0",
            "secondary_active": "#6f7784",
            "jog": "#586470",
            "jog_active": "#414a54",
        }

    def _apply_style(self) -> None:
        """Einheitliches Erscheinungsbild - in Python statt in der .ui-Datei, da pygubu-Designer
        die .ui beim Speichern komplett neu schreibt und reine XML-Anpassungen verwerfen würde.
        Wiederholt aufrufbar (ttk-Styles aktualisieren live); Toplevel-Fenster selbst brauchen
        zusätzlich _refresh_toplevel_backgrounds()."""
        style = ttk.Style(self.mainwindow)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        p = self._palette()
        bg, card_bg, text, muted, accent = p["bg"], p["card_bg"], p["text"], p["muted"], p["accent"]
        fs = self._font_size
        base_font = ("Helvetica", fs)

        self.mainwindow.configure(background=bg)
        style.configure("TFrame", background=bg)
        style.configure("TLabel", background=bg, foreground=text, font=base_font)
        style.configure("Header.TLabel", background=bg, foreground=muted, font=("Helvetica", fs + 1, "bold"))
        style.configure(
            "Title.TLabel", background=bg, foreground=accent, font=("Helvetica", fs + 9, "bold")
        )
        style.configure("TButton", font=base_font, padding=(14, 10))
        style.configure(
            "Card.TButton",
            background=card_bg,
            bordercolor=p["border"],
            relief="flat",
            padding=6,
        )
        style.map("Card.TButton", background=[("active", p["jog_hover"])])

        style.configure(
            "Card.TLabelframe",
            background=card_bg,
            bordercolor=p["border"],
            relief="flat",
            padding=14,
        )
        style.configure(
            "Card.TLabelframe.Label",
            background=card_bg,
            foreground=muted,
            font=("Helvetica", fs, "bold"),
        )
        style.configure("Card.TFrame", background=card_bg)
        style.configure(
            "JogToggle.TButton",
            background=card_bg,
            foreground=muted,
            font=("Helvetica", fs, "bold"),
            anchor="w",
            relief="flat",
            padding=(4, 6),
        )
        style.map("JogToggle.TButton", background=[("active", p["jog_hover"])])
        style.configure(
            "Readout.TLabel", background=card_bg, foreground=muted, font=("Courier", fs - 1)
        )

        def colored_button(name: str, color: str, active: str, fg: str = "white") -> None:
            style.configure(name, background=color, foreground=fg, font=base_font, padding=(14, 10))
            style.map(name, background=[("active", active), ("disabled", "#c8ccd2")])

        # Im Dark Mode ist dunkler statt weißer Text auf den hellen Status-Akzentfarben lesbarer.
        dark_fg = "#12151c"
        colored_button("Primary.TButton", accent, p["accent_active"])
        colored_button("Success.TButton", p["success"], p["success_active"], fg=dark_fg if self._dark_mode else "white")
        colored_button("Warning.TButton", p["warning"], p["warning_active"], fg=dark_fg if self._dark_mode else "white")
        colored_button("Danger.TButton", p["danger"], p["danger_active"])
        colored_button("Secondary.TButton", p["secondary"], p["secondary_active"])
        colored_button("Jog.TButton", p["jog"], p["jog_active"])
        style.configure("Jog.TButton", width=3, font=("Helvetica", fs + 5, "bold"), padding=(4, 10))
        style.configure("JogAxis.TLabel", background=card_bg, foreground=muted, font=("Helvetica", fs - 1, "bold"))
        style.configure("JogCenter.TLabel", background=card_bg, foreground=p["border"], font=("Helvetica", fs + 3))

    def _refresh_toplevel_backgrounds(self) -> None:
        """Zieht die Hintergrundfarbe nach einem Dark-Mode-Wechsel bei allen bereits gebauten
        Toplevel-Fenstern nach - deren Hintergrund ist per .configure() gesetzt, nicht per
        ttk-Style, und aktualisiert sich daher nicht von selbst."""
        bg = self._palette()["bg"]
        for attr in (
            "mainwindow", "window_testen", "window_programm",
            "window_kalibrierung", "window_optionen", "window_manuell", "window_log",
            "window_biegekopf",
        ):
            window = getattr(self, attr, None)
            if window is not None:
                window.configure(background=bg)
        for label in getattr(self, "_logo_labels", []):
            label.configure(background=bg)

    def _make_responsive(self, window: tk.Toplevel, container: ttk.Frame) -> None:
        """Hält den Inhalt beim Verkleinern/Vergrößern zentriert statt ihn zu verzerren:
        nur `window`s Zelle bekommt Gewicht, überschüssiger Platz wird Rand statt Streckung."""
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

    def _init_display(self) -> None:
        """Bestimmt einmalig den Ziel-Monitor (10"-Touchscreen) - muss vor dem ersten
        _fit_window_to_screen() laufen. Siehe display.py und KIOSK_* in constants.py."""
        self._kiosk = KIOSK_MODE
        self._monitor = find_target_monitor(
            KIOSK_MONITOR, self.mainwindow.winfo_screenwidth(), self.mainwindow.winfo_screenheight()
        )
        self._compact = self._monitor.height <= COMPACT_LAYOUT_MAX_HEIGHT
        mode = "Vollbild" if self._kiosk else "Fenster"
        print(f"Anzeige: {mode} auf Monitor {self._monitor.name} ({self._monitor.geometry})")

    def _fit_window_to_screen(self, window: tk.Toplevel, margin: int = 80) -> None:
        """Kiosk-Modus: Fenster randlos im Vollbild auf den Ziel-Monitor legen (Inhalt bleibt
        über _make_responsive zentriert). Sonst: Fenster auf dem Ziel-Monitor zentrieren und
        ggf. verkleinern, damit unten keine Buttons abgeschnitten werden."""
        mon = self._monitor
        self._add_logo(window)
        window.update_idletasks()
        req_w, req_h = window.winfo_reqwidth(), window.winfo_reqheight()
        if req_w > mon.width or req_h > mon.height:
            print(
                f"Warnung: Fenster '{window.title()}' braucht {req_w}x{req_h}, Bildschirm hat nur "
                f"{mon.width}x{mon.height} - Schriftgröße/Zoom in den Optionen verkleinern."
            )
        if self._kiosk:
            window.geometry(mon.geometry)
            window.attributes("-fullscreen", True)
            # Manche Fenstermanager platzieren ein wieder eingeblendetes (deiconify) Fenster
            # neu - beim Einblenden daher erneut auf den Touchscreen + Vollbild zwingen.
            window.bind("<Map>", lambda e, w=window: self._reassert_fullscreen(w, e), add="+")
            return
        w = min(req_w, mon.width - margin)
        h = min(req_h, mon.height - margin)
        x = mon.x + max(0, (mon.width - w) // 2)
        y = mon.y + max(0, (mon.height - h) // 2)
        window.geometry(f"{w}x{h}+{x}+{y}")

    def _add_logo(self, window: tk.Toplevel, size: int = 70) -> None:
        """Blendet das Dry-Expert-Logo unten rechts in der Ecke ein - per place() über dem
        Fenster, damit das grid-Layout aus der .ui unverändert bleibt. Weißer Hintergrund
        des Logos wird transparent gemacht, damit es auch im Dark Mode passt."""
        if getattr(self, "_logo_photo", None) is None:
            img = Image.open(PROJECT_PATH / LOGO_FILE).convert("RGBA")
            img.putdata([(r, g, b, 0) if min(r, g, b) > 240 else (r, g, b, a) for r, g, b, a in img.getdata()])
            self._logo_photo = ImageTk.PhotoImage(img.resize((size, size), Image.LANCZOS))
            self._logo_labels: list[tk.Label] = []
        label = tk.Label(window, image=self._logo_photo, borderwidth=0, background=self._palette()["bg"])
        label.place(relx=1.0, rely=1.0, x=-12, y=-12, anchor="se")
        self._logo_labels.append(label)

    def _reassert_fullscreen(self, window: tk.Toplevel, event: tk.Event) -> None:
        if event.widget is not window:  # <Map> feuert auch für jedes Kind-Widget
            return
        # Erst prüfen, nachdem der Fenstermanager das Fenster platziert hat: GNOME/mutter legt
        # ein Fenster, das zum ersten Mal sichtbar wird, auf den Monitor mit dem Mauszeiger
        # und macht es DORT zum Vollbild - dann zurück auf den Touchscreen holen.
        window.after(150, lambda: self._move_to_target_monitor(window))

    def _move_to_target_monitor(self, window: tk.Toplevel, tries: int = 5) -> None:
        mon = self._monitor
        if not window.winfo_viewable():
            return
        if (window.winfo_rootx(), window.winfo_rooty()) == (mon.x, mon.y):
            return
        window.attributes("-fullscreen", False)
        window.update_idletasks()
        window.geometry(mon.geometry)
        window.update_idletasks()
        window.attributes("-fullscreen", True)
        if tries > 1:
            window.after(150, lambda: self._move_to_target_monitor(window, tries - 1))

    def _dialog_parent(self) -> tk.Toplevel:
        """Das gerade oben liegende sichtbare Fenster - als `parent` für messagebox/simpledialog,
        damit Dialoge auf dem Touchscreen über diesem Fenster erscheinen (ohne parent landen
        sie mittig auf dem gesamten Desktop, also ggf. auf dem anderen Monitor)."""
        for attr in (
            "window_log", "window_manuell", "window_kalibrierung", "window_optionen",
            "window_programm", "window_biegekopf", "window_testen",
        ):
            window = getattr(self, attr, None)
            if window is not None and window.winfo_viewable():
                return window
        return self.mainwindow

    def _apply_compact_layout_main(self) -> int:
        """Flacher Bildschirm (z.B. 1280x800): die vier Kacheln nebeneinander statt 2x2 - in
        Python statt in der .ui, da pygubu-Designer die .ui beim Speichern neu schreibt.
        Gibt die passende Kachel-Bildhöhe zurück."""
        get = self.builder.get_object
        get("label_title").grid_configure(columnspan=4)
        tiles = (
            ("label_figur1_caption", "button_figur1"),
            ("label_testen_caption", "button_testen"),
            ("label_tischecken_caption", "button_tischecken"),
            ("label_platzhalter_caption", "button_platzhalter"),
        )
        for col, (caption, button) in enumerate(tiles):
            get(caption).grid_configure(row=1, column=col, padx=10)
            get(button).grid_configure(row=2, column=col, padx=10)
        get("button_optionen").grid_configure(row=3, column=0, columnspan=2, padx=10, pady=(20, 0))
        get("button_beenden").grid_configure(row=3, column=2, columnspan=2, padx=10, pady=(20, 0))
        # Bildgröße aus dem verfügbaren Platz: Höhe minus Titel/Beschriftung/Buttons (~300px),
        # Breite pro Kachel minus Ränder/Button-Padding (~45px) - Seitenverhältnis 270:420.
        max_h_by_height = self._monitor.height - 300
        max_h_by_width = (self._monitor.width - 40) // 4 - 45
        max_h_by_width = max_h_by_width * 420 // 270
        return max(150, min(420, max_h_by_height, max_h_by_width))

    def _apply_compact_layout_options(self) -> None:
        """Optionen zweispaltig: links Navigation, rechts Einstellungen/Entwicklermodus."""
        get = self.builder.get_object
        get("label_optionen_title").grid_configure(columnspan=2)
        get("frame_einstellungen").grid_configure(row=1, column=1, rowspan=3, padx=(20, 0), pady=0)
        get("frame_entwicklermodus").grid_configure(row=4, column=1, padx=(20, 0))
        get("button_optionen_schliessen").grid_configure(row=4, column=0, sticky="sew")

    def _style_widget(self, widget_id: str, style_name: str | None = None, text: str | None = None) -> None:
        """Setzt Style und/oder Text eines bereits gebauten Widgets."""
        widget = self.builder.get_object(widget_id)
        if style_name is not None:
            widget.configure(style=style_name)
        if text is not None:
            widget.configure(text=text)
