"""Theming/Layout-Helfer für MyApp - Farbpalette, ttk-Styles, Fenstergrößen. Wird von MyApp
(siehe my_gui.py) über Mehrfachvererbung eingebunden, damit `self` weiterhin die eine
pygubu-Callback-Instanz bleibt."""

import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

from constants import PROJECT_PATH


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

    def _make_responsive(self, window: tk.Toplevel, container: ttk.Frame) -> None:
        """Hält den Inhalt beim Verkleinern/Vergrößern zentriert statt ihn zu verzerren:
        nur `window`s Zelle bekommt Gewicht, überschüssiger Platz wird Rand statt Streckung."""
        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

    def _fit_window_to_screen(self, window: tk.Toplevel, margin: int = 80) -> None:
        """Verkleinert/zentriert ein Fenster, falls es größer als der Bildschirm ist - sonst
        könnten Buttons am unteren Rand auf kleinen Bildschirmen unsichtbar werden."""
        window.update_idletasks()
        req_w, req_h = window.winfo_reqwidth(), window.winfo_reqheight()
        screen_w, screen_h = window.winfo_screenwidth(), window.winfo_screenheight()
        w = min(req_w, screen_w - margin)
        h = min(req_h, screen_h - margin)
        x = max(0, (screen_w - w) // 2)
        y = max(0, (screen_h - h) // 2)
        window.geometry(f"{w}x{h}+{x}+{y}")

    def _style_widget(self, widget_id: str, style_name: str | None = None, text: str | None = None) -> None:
        """Setzt Style und/oder Text eines bereits gebauten Widgets."""
        widget = self.builder.get_object(widget_id)
        if style_name is not None:
            widget.configure(style=style_name)
        if text is not None:
            widget.configure(text=text)
