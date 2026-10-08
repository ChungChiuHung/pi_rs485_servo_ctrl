# Running on a Windows PC (not the Raspberry Pi)

This project normally deploys to a Raspberry Pi (see the root `README.md`),
but `servo_comm_shihlin_unified` runs fine directly on a Windows PC with a
USB-RS485 adapter — that's how it was developed and tested. This guide is
for that setup.

## Recommendation: use `start_server.bat`

Double-click `start_server.bat` in this folder — or `start_servo_unified.bat`
in the repository root, which runs the same file so you don't have to open
the folder. On every start it:

1. Finds Python (`python`, or the `py -3` launcher; the Microsoft Store
   placeholder that only opens the Store is ignored) and checks it's 3.8+.
2. Prints a **setup report** (`check_pc_setup.py`, no network): each package
   from `requirements_pc.txt` as `OK` / `MISSING` / `TOO OLD`, the serial
   ports Windows can see, and whether `motor_profiles.json` is present.
   **RPi.GPIO is not on the list on purpose** — it is the Raspberry Pi's GPIO
   package and a PC never needs it.
3. Runs `pip install -r requirements_pc.txt` **only if** something is missing
   or too old (pip's details also go to `pip_install.log`), then checks again.
   After the first run it starts without touching pip or the internet.
4. Stops with an explanation if `motor_profiles.json` is missing — that file
   is tuned per rig and not stored in git, so a fresh clone or pull won't
   have it.
5. Opens the browser and runs the server in the same window (`Ctrl+C` stops
   it). If the server exits with an error it says so instead of just closing.

Options (from a terminal, or a shortcut):

| Command | What it does |
|---|---|
| `start_server.bat --check` | Only the setup report: installs and starts nothing. Use this first on a new PC, or to see what's wrong. |
| `start_server.bat --install` | Runs pip even if the check passes (e.g. after a broken install), then starts. |
| `start_server.bat --help` | Lists the options. |

`SERVO_WEB_PORT` (default 5000) and `SERVO_SERIAL_PORT` (e.g. `COM7`) can be
set before running it.

On a PC the server logs `Running on a PC: Raspberry Pi GPIO is not used
(normal).` at start — that's expected, not an error.

A `.bat` file rather than a shell script because this is a plain Windows
PC target — it double-clicks and runs with no extra tooling (no Git Bash
or WSL required), unlike a `.sh` file.

## On a Mac: use `start_server.command`

The same launcher for macOS. Double-click `start_server.command` in Finder
(it opens in Terminal), or run `./start_server.command`. It needs `python3`
(python.org installer or `brew install python`), creates a private
`.venv` next to it on first run (a global `pip install` is refused by
Homebrew/recent macOS Pythons), installs `requirements_pc.txt`, picks the
USB-RS485 adapter (`/dev/cu.usbserial*`, `usbmodem*`, ...), opens the browser and
runs the server in that window (`Ctrl+C` to stop).

* If Finder says it can't be opened: `chmod +x start_server.command`, then
  `xattr -d com.apple.quarantine start_server.command` (a downloaded copy is
  quarantined), or right-click → Open once.
* No adapter plugged in? The web UI still starts, shows a red "No RS-485
  serial port connection" bar, and **Reconnect** retries once it is plugged in.
* To force a port: `SERVO_SERIAL_PORT=/dev/cu.usbserial-XXXX ./start_server.command`.
  Use the `cu.*` name, not `tty.*`. Most adapters need no driver on recent
  macOS; CH340/CP210x based ones may need the vendor driver.
* macOS's AirPlay Receiver can occupy port 5000 (the page then fails to
  load or shows a 403): turn it off in System Settings → General → AirDrop &
  Handoff, or start with `SERVO_WEB_PORT=5001 ./start_server.command`.

Not yet run on a real Mac — the script is written from the Windows launcher and
syntax-checked only.

## On Linux / the Raspberry Pi: use `start_server.sh`

`./start_server.sh` (in this folder) starts the web app immediately: it installs
nothing, uses the first interpreter that already has the packages (`../.venv`
from `../setup_pi.sh`, then `./.venv`, then `python3`), picks the first
`/dev/ttyUSB*` or `/dev/ttyACM*` adapter, and runs `app.py` in the terminal
(`Ctrl+C` stops it; no browser is opened). Settings are environment variables,
e.g. `SERVO_WEB_HOST=127.0.0.1 SERVO_SERIAL_PORT=/dev/ttyAMA0 ./start_server.sh`
(the RS485 CAN HAT is `/dev/ttyAMA0`); the list is at the top of the script.
Starting the app is not passive: like `python app.py` it opens the port, drives
GPIO4, and writes PA23 if EEPROM writes are not yet inhibited (it never moves
the motor by itself). With no adapter plugged in it still starts and shows the
"No RS-485 serial port connection" bar.

## Prerequisites

1. **Python 3.9+**, on PATH (`python --version` works from any terminal).
   [python.org/downloads](https://www.python.org/downloads/) — check
   "Add python.exe to PATH" during install.
2. **USB-RS485 adapter** plugged in, with its driver installed (Windows
   usually installs it automatically; check Device Manager → Ports
   (COM & LPT) if it doesn't show up). You don't need to know which COM
   port it landed on — the server auto-detects it.
3. The servo drive wired to the adapter and powered on.

> Use `servo_comm_shihlin_unified/requirements_pc.txt` (the launchers already
> do). The repo-root `requirements.txt` is the Raspberry Pi list (the same
> packages plus `RPi.GPIO`, which is skipped off ARM Linux).

## Quick start

1. New PC? Run `start_server.bat --check` once and read the report: every
   package `OK` (or let step 2 install them), your adapter listed under
   **Serial ports**, and `motor_profiles.json found`.
2. Double-click `start_server.bat` in this folder (or
   `start_servo_unified.bat` in the repository root).
3. Wait for `Running on http://0.0.0.0:5000` to appear — your browser
   should open to the control panel automatically a few seconds later.
4. To stop the server, click the console window and press `Ctrl+C`.

## Manual step-by-step

If you'd rather not use the batch file, or it doesn't work and you want
to see each step:

```bash
cd servo_comm_shihlin_unified
python -m pip install -r requirements_pc.txt
python app.py
```

Then open `http://localhost:5000` in a browser. Stop with `Ctrl+C` in
that terminal.

## Starting OSC / Art-Net

Not started automatically — use the web UI's "Continuous Motion Input"
section, or the HTTP API. Full reference: `OSC_ARTNET_GUIDE.md`.

## Troubleshooting

- **"Python was not found" / `python` not recognized** — Python isn't on
  PATH. Reinstall from python.org with "Add to PATH" checked, or use the
  `py` launcher (`py app.py`) if that's what your install provides.
- **Anything to do with `RPi.GPIO`** — you don't need it on a PC: it's the
  Raspberry Pi's GPIO package. `start_server.bat` never installs it, and the
  server just logs "Running on a PC: Raspberry Pi GPIO is not used (normal)."
  If `pip install` fails on it, you installed the repo-root
  `requirements.txt` (the Pi list) by hand — use this folder's
  `requirements_pc.txt` instead.
- **The package install fails** — read the error above it and
  `pip_install.log` in this folder. Usually there is no internet connection,
  or a proxy/firewall blocks pypi.org. Fix that and run `start_server.bat`
  again; `start_server.bat --install` forces a fresh attempt.
- **No serial ports in the report** — the adapter isn't plugged in, or its
  driver (CH340 / FTDI / CP210x) isn't installed: check Device Manager →
  Ports (COM & LPT).
- **Serial port / COM port errors, or `/status` shows
  `"connected_port": "Not connected"`** — most often another copy of
  `app.py` is already running and holding the port (check other terminal
  windows, or `python.exe` in Task Manager) — only one process can hold
  the port at a time. Otherwise, confirm the adapter shows up under
  Device Manager → Ports.
- **Port 5000 already in use** — same cause as above: an existing
  `app.py` instance. Stop it (`Ctrl+C` in its window, or end the
  `python.exe` process) before starting a new one.
