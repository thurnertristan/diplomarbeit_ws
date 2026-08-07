#!/usr/bin/env bash
# Startet die FR20-Steuerungs-GUI (gui/my_gui.py) mit EINEM Befehl, egal ob
# schon ein venv existiert, ob es aktiviert ist, oder ob pygubu/Pillow/numpy
# schon installiert sind:
#   - sourced ROS 2 (für Gazebo/gz-transport), falls vorhanden
#   - legt gui/venv an, falls es fehlt (inkl. Zugriff auf System-Pakete für
#     die gz-Bindings, siehe gui/venv/pyvenv.cfg)
#   - installiert fehlende Python-Pakete (pygubu, Pillow, numpy) automatisch
#     nach - dafür ist beim allerersten Start Internetzugang nötig
#   - startet die GUI mit dem venv-Python, ganz ohne manuelles "source
#     gui/venv/bin/activate"
#
# Aufruf (von überall, nicht nur aus diesem Ordner):
#   ./start_gui.sh

set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

if [ -f /opt/ros/jazzy/setup.bash ]; then
    # ROS-Setup-Skripte selbst sind nicht "nounset"-sicher (nutzen intern
    # unbound variables) - "set -u" nur dafür kurz deaktivieren.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
else
    echo "Warnung: /opt/ros/jazzy/setup.bash nicht gefunden - gz/ros2 evtl. nicht verfügbar." >&2
fi

set -u

VENV_DIR="gui/venv"
VENV_PY="$VENV_DIR/bin/python3"

if [ ! -x "$VENV_PY" ]; then
    echo "Kein venv gefunden, lege es unter $VENV_DIR an ..."
    # --system-site-packages direkt bei der Erstellung, statt hinterher per
    # sed in pyvenv.cfg zu fummeln (siehe Fix weiter unten für bereits
    # bestehende venvs, die das noch nicht haben).
    VENV_ERR=$(mktemp)
    if ! python3 -m venv --system-site-packages "$VENV_DIR" 2>"$VENV_ERR"; then
        cat "$VENV_ERR" >&2
        rm -f "$VENV_ERR"
        PYVER=$(python3 -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")')
        echo "" >&2
        echo "Fehler: venv-Erstellung fehlgeschlagen - vermutlich fehlt das Systempaket" >&2
        echo "für Python-venv-Unterstützung. Einmalig nachinstallieren mit:" >&2
        echo "" >&2
        echo "    sudo apt install python${PYVER}-venv" >&2
        echo "" >&2
        echo "Danach dieses Skript erneut ausführen: ./start_gui.sh" >&2
        rm -rf "$VENV_DIR"
        exit 1
    fi
    rm -f "$VENV_ERR"
fi

# Damit das venv zusätzlich zu seinen eigenen Paketen auch die apt-
# installierten gz-Python-Bindings sieht (/usr/lib/python3/dist-packages) -
# ohne das wäre "import gz.transport" im venv nicht auffindbar.
if grep -q "include-system-site-packages = false" "$VENV_DIR/pyvenv.cfg" 2>/dev/null; then
    sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' "$VENV_DIR/pyvenv.cfg"
fi

MISSING=""
for pkg in pygubu PIL numpy; do
    "$VENV_PY" -c "import $pkg" >/dev/null 2>&1 || MISSING="$MISSING $pkg"
done

if [ -n "$MISSING" ]; then
    echo "Installiere fehlende Pakete:$MISSING ..."
    "$VENV_DIR/bin/pip" install --quiet pygubu Pillow numpy
fi

exec "$VENV_PY" gui/my_gui.py
