#!/bin/bash
# Fast start for Linux / the Raspberry Pi: runs the unified web app straight away.
#
#   ./start_server.sh
#
# "Fast" means it does NOT install anything or build a virtual environment (use
# ../setup_pi.sh once on a Pi, or start_server.command on a Mac, for that); it
# just picks an interpreter that already has the packages, picks the serial
# port, and starts app.py in this terminal (Ctrl+C stops it). The browser is not
# opened. Run it from anywhere; it changes into its own folder first.
#
# What starting the app does to the drive (same as `python app.py`): it opens the
# serial port, drives GPIO4 (RS485_ENABLE) and the LED pins if RPi.GPIO exists,
# reads a few registers, and -- if PA23 does not already inhibit EEPROM writes --
# writes PA23 (EEPROM protection). It never moves the motor by itself.
#
# Settings (environment variables, all optional):
#   SERVO_PYTHON       interpreter to use (default: first of ../.venv, ./.venv, python3)
#   SERVO_SERIAL_PORT  serial port (default: the first /dev/ttyUSB* or /dev/ttyACM*;
#                      if there is none the app auto-detects, and if it still finds
#                      no port it starts anyway and shows "No RS-485 serial port
#                      connection" with a Reconnect button)
#   SERVO_WEB_HOST     address to listen on   (default 0.0.0.0 = the whole network)
#   SERVO_WEB_PORT     web port               (default 5000)
#   SERVO_WEB_PASSWORD turns on the web password (user SERVO_WEB_USER, default "servo")
#   SERVO_WEB_DEBUG=1  Flask debugger (never on a shared network)
# Example:  SERVO_WEB_HOST=127.0.0.1 SERVO_SERIAL_PORT=/dev/ttyUSB0 ./start_server.sh

cd "$(dirname "$0")" || exit 1
HERE="$(pwd)"

# 1. interpreter
PYTHON=""
if [ -n "${SERVO_PYTHON:-}" ]; then
    PYTHON="$SERVO_PYTHON"
else
    for candidate in "$HERE/../.venv/bin/python" "$HERE/.venv/bin/python" "$(command -v python3)"; do
        if [ -n "$candidate" ] && [ -x "$candidate" ]; then
            PYTHON="$candidate"
            break
        fi
    done
fi
if [ -z "$PYTHON" ]; then
    echo "ERROR: no Python interpreter found. Install python3, or set SERVO_PYTHON." >&2
    exit 1
fi

# 2. packages (fail early with a useful message instead of a traceback)
if ! "$PYTHON" -c "import flask, serial, pythonosc" 2>/dev/null; then
    echo "ERROR: $PYTHON is missing flask, pyserial or python-osc." >&2
    echo "  On the Pi run ../setup_pi.sh once (creates ../.venv with everything)." >&2
    echo "  Elsewhere:  $PYTHON -m pip install -r requirements_pc.txt" >&2
    exit 1
fi

# 3. serial port
if [ -z "${SERVO_SERIAL_PORT:-}" ]; then
    for candidate in /dev/ttyUSB* /dev/ttyACM*; do
        if [ -e "$candidate" ]; then
            SERVO_SERIAL_PORT="$candidate"
            break
        fi
    done
fi
if [ -n "${SERVO_SERIAL_PORT:-}" ]; then
    export SERVO_SERIAL_PORT
    echo "Serial port : $SERVO_SERIAL_PORT"
else
    echo "Serial port : none found (/dev/ttyUSB*, /dev/ttyACM*) -- the app will auto-detect."
    echo "              Using the RS485 CAN HAT? set SERVO_SERIAL_PORT=/dev/ttyAMA0"
fi

export SERVO_WEB_PORT="${SERVO_WEB_PORT:-5000}"
echo "Interpreter : $PYTHON"
echo "Web UI      : http://${SERVO_WEB_HOST:-0.0.0.0}:$SERVO_WEB_PORT   (Ctrl+C to stop)"
echo

# exec: this process becomes the app, so Ctrl+C, PM2 and systemd signals reach it directly.
exec "$PYTHON" app.py
