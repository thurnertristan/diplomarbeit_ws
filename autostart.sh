#!/usr/bin/env bash
# Schaltet den automatischen Start der FR20-GUI (start_gui.sh) beim Anmelden ein/aus.
#
#   ./autostart.sh ein      -> GUI startet ab jetzt automatisch nach dem Hochfahren/Anmelden
#   ./autostart.sh aus      -> Autostart wieder abschalten
#   ./autostart.sh status   -> zeigt, ob der Autostart gerade aktiv ist
#
# Technisch: legt einen Eintrag in ~/.config/autostart/ an (Standard-Mechanismus von
# GNOME/Ubuntu) - daher auch unter "Startprogramme" bzw. in GNOME Tweaks sichtbar.
# Die Ausgabe der GUI landet in ~/.cache/fr20_gui_autostart.log (zur Fehlersuche).
#
# Hinweis: Autostart greift beim ANMELDEN. Damit die GUI direkt nach dem Hochfahren ohne
# Passworteingabe erscheint, in den Ubuntu-Einstellungen unter "Benutzer" die
# "Automatische Anmeldung" aktivieren.

set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AUTOSTART_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/autostart"
DESKTOP_FILE="$AUTOSTART_DIR/fr20-gui.desktop"
LOG_FILE="${XDG_CACHE_HOME:-$HOME/.cache}/fr20_gui_autostart.log"

case "${1:-status}" in
    ein|an|on)
        mkdir -p "$AUTOSTART_DIR" "$(dirname "$LOG_FILE")"
        cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=FR20 Robotersteuerung
Comment=Startet die FR20-GUI im Vollbild auf dem Touchscreen (abschalten: $REPO_DIR/autostart.sh aus)
Exec=$REPO_DIR/autostart.sh starten
Path=$REPO_DIR
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
        echo "Autostart EIN - die GUI startet ab dem nächsten Anmelden automatisch."
        echo "Abschalten mit: $REPO_DIR/autostart.sh aus"
        ;;
    aus|off)
        if [ -f "$DESKTOP_FILE" ]; then
            rm -f "$DESKTOP_FILE"
            echo "Autostart AUS."
        else
            echo "Autostart war bereits aus."
        fi
        echo "Wieder einschalten mit: $REPO_DIR/autostart.sh ein"
        ;;
    starten)
        # Wird vom Autostart-Eintrag aufgerufen: kurz warten, bis Desktop und Monitore
        # bereit sind, dann GUI starten und Ausgabe ins Log schreiben.
        mkdir -p "$(dirname "$LOG_FILE")"
        sleep 3
        exec "$REPO_DIR/start_gui.sh" >"$LOG_FILE" 2>&1
        ;;
    status)
        if [ -f "$DESKTOP_FILE" ]; then
            echo "Autostart ist EIN ($DESKTOP_FILE)."
        else
            echo "Autostart ist AUS."
        fi
        ;;
    *)
        echo "Aufruf: $0 ein|aus|status" >&2
        exit 1
        ;;
esac
