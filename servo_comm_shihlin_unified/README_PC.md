# Running on a Windows PC (not the Raspberry Pi)

This project normally deploys to a Raspberry Pi (see the root `README.md`),
but `servo_comm_shihlin_unified` runs fine directly on a Windows PC with a
USB-RS485 adapter — that's how it was developed and tested. This guide is
for that setup.

## Recommendation: use `start_server.bat`

You can either run a few commands by hand every time (see
[Manual step-by-step](#manual-step-by-step) below), or double-click
`start_server.bat`. **The batch file is what I'd recommend** — it checks
Python is installed, installs the three PC-safe dependencies, opens your
browser automatically, and runs the server in the same window so you can
watch the logs and stop it with `Ctrl+C`. It's the same commands either
way; the script just saves you retyping them and gets the dependency list
right (see the gotcha below).

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
> do). The repo-root `requirements.txt` is the same list for the Raspberry Pi;
> neither installs `RPi.GPIO` (on a Pi it comes from apt).

## Quick start

1. Double-click `start_server.bat` in this folder.
2. Wait for `Running on http://0.0.0.0:5000` to appear — your browser
   should open to the control panel automatically a few seconds later.
3. To stop the server, click the console window and press `Ctrl+C`.

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
- **`pip install` fails on `RPi.GPIO`** — an older checkout of the root
  `requirements.txt`, which listed it. Update, or use this folder's
  `requirements_pc.txt`. The app doesn't need the package off-Pi; it
  degrades gracefully without it (see `app.py`'s GPIO import).
- **Serial port / COM port errors, or `/status` shows
  `"connected_port": "Not connected"`** — most often another copy of
  `app.py` is already running and holding the port (check other terminal
  windows, or `python.exe` in Task Manager) — only one process can hold
  the port at a time. Otherwise, confirm the adapter shows up under
  Device Manager → Ports.
- **Port 5000 already in use** — same cause as above: an existing
  `app.py` instance. Stop it (`Ctrl+C` in its window, or end the
  `python.exe` process) before starting a new one.
