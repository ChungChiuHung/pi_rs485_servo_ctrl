#!/bin/bash
# One-time setup on a Raspberry Pi 3 B (Raspberry Pi OS): installs every package
# the project needs. Run from the repository root:  ./setup_pi.sh
#
#   1. apt: Python venv support, a compiler and python3-dev (so pip CAN build
#      anything that has no ready-made wheel), and the ready-made GPIO package
#      python3-rpi.gpio (pip's own RPi.GPIO build fails on some setups).
#   2. A virtual environment in ./.venv that can also see the apt packages
#      (--system-site-packages). Recent Raspberry Pi OS refuses a global
#      "pip install" (PEP 668), and this keeps the system Python untouched.
#   3. pip install -r requirements.txt into that venv.
#
# It asks for sudo once for step 1. Safe to run again.
set -e
cd "$(dirname "$0")"

echo "== 1/3 apt packages (sudo) =="
sudo apt-get update
sudo apt-get install -y python3-venv python3-dev build-essential python3-rpi.gpio

echo "== 2/3 virtual environment (.venv) =="
python3 -m venv --system-site-packages .venv

echo "== 3/3 Python packages =="
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

echo
echo "Check:"
.venv/bin/python - <<'PY'
import flask, serial, pythonosc
print("  flask", flask.__version__, "| pyserial", serial.VERSION, "| python-osc OK")
try:
    import RPi.GPIO as GPIO
    print("  RPi.GPIO", GPIO.VERSION)
except Exception as e:
    print("  RPi.GPIO NOT importable (", e, ") -- the Type 2 apps still start without it")
PY
echo
echo "Done. Run the apps with:  $(pwd)/.venv/bin/python app.py   (from inside the app's folder)"
echo "For PM2 / systemd use that same interpreter path instead of python3."
