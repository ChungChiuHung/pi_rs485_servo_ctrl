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

## Prerequisites

1. **Python 3.9+**, on PATH (`python --version` works from any terminal).
   [python.org/downloads](https://www.python.org/downloads/) — check
   "Add python.exe to PATH" during install.
2. **USB-RS485 adapter** plugged in, with its driver installed (Windows
   usually installs it automatically; check Device Manager → Ports
   (COM & LPT) if it doesn't show up). You don't need to know which COM
   port it landed on — the server auto-detects it.
3. The servo drive wired to the adapter and powered on.

> **⚠️ Do not run `pip install -r requirements.txt` from the repo root.**
> That file includes `RPi.GPIO` and `gpiozero`, which only build on a Pi
> and will fail to install on Windows. Use
> `servo_comm_shihlin_unified/requirements_pc.txt` instead (the batch
> file already does this for you).

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
- **`pip install` fails on `RPi.GPIO` or `gpiozero`** — you used the root
  `requirements.txt` instead of this folder's `requirements_pc.txt`. The
  app doesn't need those packages off-Pi; it degrades gracefully without
  them (see `app.py`'s GPIO import).
- **Serial port / COM port errors, or `/status` shows
  `"connected_port": "Not connected"`** — most often another copy of
  `app.py` is already running and holding the port (check other terminal
  windows, or `python.exe` in Task Manager) — only one process can hold
  the port at a time. Otherwise, confirm the adapter shows up under
  Device Manager → Ports.
- **Port 5000 already in use** — same cause as above: an existing
  `app.py` instance. Stop it (`Ctrl+C` in its window, or end the
  `python.exe` process) before starting a new one.
