"""Ermittelt den Bildschirm, auf dem die GUI im Vollbild laufen soll (10"-Touchscreen).

Tk kennt nur den gesamten virtuellen Desktop (bei zwei Monitoren z.B. 3200x1080) - nicht die
einzelnen Monitore. Deshalb wird `xrandr` abgefragt (funktioniert unter X11 und unter Wayland
über XWayland, auf dem Tk ohnehin läuft).

Auswahl des Ziel-Monitors (siehe constants.KIOSK_MONITOR / Umgebungsvariable FR20_MONITOR):
  - Name gesetzt (z.B. "HDMI-1") und angeschlossen -> genau dieser Monitor
  - sonst: der KLEINSTE angeschlossene Monitor (= der 10"-Touchscreen, falls zusätzlich ein
    normaler Monitor angeschlossen ist; ist nur der Touchscreen dran, ist es eben der)
  - xrandr nicht verfügbar -> gesamter Bildschirm laut Tk
"""

import re
import subprocess
from dataclasses import dataclass

_XRANDR_LINE = re.compile(r"^(\S+) connected (?:primary )?(\d+)x(\d+)\+(\d+)\+(\d+)")


@dataclass(frozen=True)
class Monitor:
    name: str
    x: int
    y: int
    width: int
    height: int

    @property
    def geometry(self) -> str:
        """Tk-Geometriestring "BxH+X+Y"."""
        return f"{self.width}x{self.height}+{self.x}+{self.y}"


def list_monitors() -> list[Monitor]:
    """Alle angeschlossenen UND aktiven Monitore laut xrandr (leere Liste, falls xrandr fehlt)."""
    try:
        out = subprocess.run(
            ["xrandr", "--query"], capture_output=True, text=True, timeout=3, check=False
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    monitors = []
    for line in out.splitlines():
        m = _XRANDR_LINE.match(line)
        if m:
            name, w, h, x, y = m.group(1), *map(int, m.groups()[1:])
            monitors.append(Monitor(name, x, y, w, h))
    return monitors


def find_target_monitor(preferred_name: str, fallback_width: int, fallback_height: int) -> Monitor:
    monitors = list_monitors()
    if preferred_name:
        for mon in monitors:
            if mon.name == preferred_name:
                return mon
        print(f"Monitor '{preferred_name}' nicht gefunden - nehme automatisch den kleinsten.")
    if monitors:
        return min(monitors, key=lambda mon: mon.width * mon.height)
    return Monitor("gesamt", 0, 0, fallback_width, fallback_height)
