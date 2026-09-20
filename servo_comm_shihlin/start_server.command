#!/bin/bash
# macOS launcher -- the counterpart of start_server.bat. Double-click it in
# Finder (opens in Terminal) or run ./start_server.command from a terminal.
#
# First run only: Finder refuses files it doesn't consider executable, and a
# downloaded copy is quarantined. If double-clicking says it cannot be opened:
#     chmod +x start_server.command
#     xattr -d com.apple.quarantine start_server.command
# (or right-click -> Open once).
#
# Override the port with SERVO_WEB_PORT, or the serial port with
# SERVO_SERIAL_PORT=/dev/cu.usbserial-XXXX ./start_server.command

cd "$(dirname "$0")" || exit 1

pause_and_exit() {
    echo
    read -r -p "Press Return to close this window..." _
    exit "$1"
}

echo "============================================"
echo " Servo Control Server (servo_comm_shihlin, Modbus ASCII)"
echo "============================================"
echo

if ! command -v python3 >/dev/null 2>&1; then
    echo "ERROR: python3 was not found."
    echo "Install Python 3.9+ from https://www.python.org/downloads/macos/"
    echo "(or run: brew install python) and try again."
    pause_and_exit 1
fi

# Dependencies go into a private virtual environment: Homebrew's and recent
# macOS Pythons refuse a global "pip install" (PEP 668), and this keeps the
# Mac's own Python untouched.
if [ ! -x .venv/bin/python ]; then
    echo "Creating a virtual environment in .venv ..."
    if ! python3 -m venv .venv; then
        echo "ERROR: could not create the virtual environment."
        pause_and_exit 1
    fi
fi

echo "Checking/installing dependencies (pyserial, flask, python-osc)..."
if ! .venv/bin/python -m pip install -q -r requirements_pc.txt; then
    echo "ERROR: Failed to install dependencies. See the message above."
    pause_and_exit 1
fi

# macOS lists every USB-RS485 adapter as /dev/cu.* (use cu, not tty: tty.* waits
# for a carrier signal). Pick it here, because the server would otherwise try
# ports in alphabetical order and /dev/cu.Bluetooth-Incoming-Port comes first.
if [ -z "${SERVO_SERIAL_PORT:-}" ]; then
    for candidate in /dev/cu.usbserial* /dev/cu.usbmodem* /dev/cu.wchusbserial* /dev/cu.SLAB_USBtoUART*; do
        if [ -e "$candidate" ]; then
            SERVO_SERIAL_PORT="$candidate"
            break
        fi
    done
fi
if [ -n "${SERVO_SERIAL_PORT:-}" ]; then
    export SERVO_SERIAL_PORT
    echo "Using serial port: $SERVO_SERIAL_PORT"
else
    echo "No USB serial adapter found (/dev/cu.usbserial*, usbmodem*, ...)."
    echo "The web UI will still start and show 'No RS-485 serial port connection';"
    echo "plug the adapter in and press Reconnect there."
fi

WEB_PORT="${SERVO_WEB_PORT:-5000}"
export SERVO_WEB_PORT="$WEB_PORT"

echo
echo "Opening http://localhost:$WEB_PORT in your browser in a few seconds..."
(sleep 3; open "http://localhost:$WEB_PORT") &

echo "Starting the server. Press Ctrl+C in this window to stop it."
echo "============================================"
echo
.venv/bin/python app.py

echo
echo "Server stopped."
pause_and_exit 0
