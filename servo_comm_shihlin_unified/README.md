# servo_comm_shihlin_unified

A merged, config-driven replacement for `servo_comm_shihlin/` and
`servo_comm_shihlin_50W/`: one codebase, a JSON motor-profile switch
instead of two hand-synced folders. Controls a Shihlin SDE-series servo
drive over RS-485 **Modbus RTU** (confirmed against real hardware:
115200 baud, station 1 — not the ASCII protocol the two legacy folders
assume).

> **Safety:** this code sends real commands to a physical servo motor.
> Make sure the motor and its load are clear of people and obstacles
> before starting any action that can move it. See "Hardware safety"
> below.

Full design history: `docs/servo_comm_shihlin_merge_design.md` (the
original planning doc — mostly of historical interest now; this README
and `OSC_ARTNET_GUIDE.md` describe the current, working system).

## Status

**Functional and tested.** Flask web UI, OSC server, and Art-Net server
all drive the same `ServoController` instance; motor profile switching,
positioning moves, continuous JOG rotation, and alarm handling have all
been verified against real hardware (not just unit tests). 687 unit
tests, all passing (2026-10-09).

## Quick start

- **Windows PC:** double-click `start_server.bat` (details: `README_PC.md`).
  It installs what's missing, creates `motor_profiles.json` if needed, opens
  the browser and starts the server; `start_server.bat --check` only reports.
- **Raspberry Pi / Linux:** `../setup_pi.sh` once, then `./start_server.sh`.
- **macOS:** `./start_server.command`.
- **By hand:**

  ```bash
  cd servo_comm_shihlin_unified
  pip3 install -r requirements_pc.txt      # on the Pi: -r ../requirements.txt
  python3 create_motor_profiles.py         # only if motor_profiles.json is missing
  python3 app.py
  ```

`start_server.sh` and `start_server.command` don't create
`motor_profiles.json` — on a fresh clone run `python3 create_motor_profiles.py`
once first, or `app.py` stops with `FileNotFoundError: motor_profiles.json`.

Then open `http://<HOST>:5000` in a browser. `app.py` opens the serial
port for the active motor profile at startup — only one process can hold
it at a time.

Run the test suite (no hardware required — everything is mocked):
```bash
python3 -m unittest discover -p "test_*.py"
```

## Motor profiles

`motor_profiles.json` holds the two known motors' settings; switch
between them from the web UI's "Motor Profile" dropdown or `POST
/profile` (that switch lasts until restart: `active_profile` in the file is
the motor used at every start). The file is tuned per rig and not in git;
`motor_profiles.example.json` is the committed template (the current rig's
configuration, default motor `shihlin_400W`), and on Windows
`start_server.bat` creates the file from it on first start (or run
`python create_motor_profiles.py [--profile shihlin_50W]`):

```json
{
  "active_profile": "shihlin_400W",
  "encoder_pulses_per_rev": 4194304,
  "modbus_device_number": 1,
  "profiles": {
    "shihlin_400W": {"baud_rate": 115200, "gear_ratio": 30, "abs_home_pos": 62369153},
    "shihlin_50W":  {"baud_rate": 9600,   "gear_ratio": 10, "abs_home_pos": 1184347}
  }
}
```

`base_pulse_per_degree` is always computed from `encoder_pulses_per_rev
* gear_ratio / 360` at load time — never hardcoded. Switching profiles
closes the current serial connection and reopens it at the new baud
rate; it's rejected while an OSC/Art-Net server is running (stop it
first). Each profile persists its own calibrated home position to
`servo_config_<profile>.json` (created at runtime, not checked into
git — it's this specific installation's calibration, not source code).

## Web UI

The single-page control panel (`templates/index.html`, English / 中文) covers:
- Live status panel (port, baud, alarm, Servo-on, current angle/encoder,
  drive test-mode state, home, electronic gear, EEPROM protection, encoder
  mode) — polled read-only, never sends a command. A red bar with a
  **Reconnect** button appears when there is no serial connection.
- Motor profile selection (kept in sync with the backend; locked while
  OSC/Art-Net runs).
- Continuous Motion Input: start/stop OSC or Art-Net, optional feedback and
  autostart (see `OSC_ARTNET_GUIDE.md`); the Art-Net Channel Monitor while
  Art-Net runs.
- Commands: Servo on/off, CLEAR ALARM 12 (click twice to confirm), SET HOME
  (confirmation dialog), encoder mode (PA28, click twice to confirm),
  Position Mode (arm, then ←/→ arrow keys nudge 0.5°; Set Point 1/2; HOME)
  and Speed Control JOG (arm, then **hold** ←/→ to rotate, release to stop,
  ↑/↓ = ±1 rpm). Numeric inputs are validated both client- and server-side.
  JOG start/stop results and errors appear in the JOG section's status line.
- Read-only extras: GET STATE VALUE (PA/PD registers), DO1-DO6 status,
  Activity Log (including what OSC/Art-Net did).
- While OSC or Art-Net runs, the page shows "External control: ..." and
  disables every control the backend would refuse (see "External control"
  below); a failed status poll shows "Unknown (status unavailable)" and keeps
  the last confirmed lock.

## HTTP API

| Endpoint | Method | Purpose |
|---|---|---|
| `/` | GET | Redirects to `/index`. |
| `/index` | GET | The control panel. |
| `/status` | GET | Read-only live status snapshot (including the active `profile`). Never sends a write/motion command — safe to poll on an interval. |
| `/log` | GET | Activity log entries newer than `?since=<id>`. |
| `/profile` | GET | Active profile + available profiles. |
| `/profile` | POST | Switch motor profile (until restart). 409 while an input server is running. |
| `/reconnect` | POST | Retry opening the serial port after starting without one. |
| `/encoder_mode` | GET | PA28 as configured in the drive, the mode in use, absolute-position health. Read-only. |
| `/encoder_mode` | POST | Write PA28. Requires `{"absolute": bool, "confirm": true}`; 409 while moving or while an input server runs. Takes effect after a drive power cycle. |
| `/encoder_mode/adopt` | POST | Re-read PA28 after the power cycle and use it. 409 while an input server runs. |
| `/io/do` | GET | DO1-DO6 state and assigned functions (three serial reads, read-only). |
| `/server/status` | GET | Which input server (if any) is active. |
| `/server/start` | POST | Start OSC or Art-Net — see `OSC_ARTNET_GUIDE.md`. Invalid options answer 400 with the reason; 409 if one is already running. `"autostart": true` also saves the request for the next app start. |
| `/server/autostart` | GET / DELETE | Show / remove the saved autostart request. |
| `/server/artnet_channels` | GET | Art-Net Channel Monitor data (last frame, receive statistics). |
| `/server/stop` | POST | Stop the active input server — this is also how the web UI gets control back. |
| `/alarm/clear` | POST | Clear Alarm 12 only. Requires `{"confirm": true}`; 409 while an input server runs. See safety note below. |
| `/action` | POST | Web UI button actions (`{"action": "..."}`, see `app.py`'s `handle_action()` for the full list). Most are refused with 409 while an input server runs — see below. |

Without a serial connection, every endpoint that would talk to the drive
answers 503 and sends nothing.

### External control: who owns the motor

While OSC or Art-Net is running, that sender controls the motor. The backend
then refuses (HTTP 409, `"<OSC|Art-Net> is controlling the motor. Stop the
input server first."`, nothing sent to the drive) every `/action` except this
allowlist (`WEB_ACTIONS_ALLOWED_DURING_EXTERNAL_CONTROL` in `app.py`):

| Still allowed | Why |
|---|---|
| `getMsg`, `setPoint_1`, `setPoint_2` | Reads, and Web-local set-point recording (the matching moves stay blocked) |
| `motionPause` | Stops **JOG** rotation only (`0x0904 = 0`) — does not stop a positioning move |
| `motionCancel`, `disablePosMode` | Leave test mode; the drive drops Servo ON as a side effect |
| `servoOff` | Servo off (raises AL.12, see the guide's Gotcha 2) |

None of these is an emergency stop — use the drive's hardware E-Stop for
that. An unknown action name is also refused while a server runs.
`/alarm/clear`, `/encoder_mode/adopt`, `/profile` and `/encoder_mode` POST are
refused too; `/server/stop` restores web control. The web UI mirrors these
rules (disabled controls, and its arrow keys don't arm because OSC/Art-Net put
the drive into JOG/position mode), but the backend is the enforcement.

### `POST /alarm/clear` safety precondition

Per the driver manual (`docs/en_manual.txt`, AL.12 entry), AL.12 means
the EMG (Emergency Stop) signal is active, and the manual's own remedy
is to release the trigger only *after* the emergency condition is
actually resolved. This endpoint's primary mechanism
(`clear_alarm_12()`) switches the drive's DI input source to
communication-control and writes the virtual EMG DI bit to "released" —
**if a physical E-Stop circuit is still engaged, the drive will report
EMG as released anyway**; the software cannot verify the physical
condition is actually gone. Confirming that is the caller's
responsibility, not something this endpoint can check.

```bash
curl -X POST http://<HOST>:5000/alarm/clear \
     -H "Content-Type: application/json" \
     -d '{"confirm": true}'
```

Response includes the alarm code before/after, which mechanism actually
cleared it (falls back to the official `0x0130` register write if the
primary mechanism doesn't work), caller IP, and timestamp. Every call is
logged regardless of outcome. Note this driver reports `0xFF` (255), not
`0`, for "no alarm" — the response's `after_alarm_code` reflects the raw
register value; `status: "success"` is what tells you whether it actually
cleared.

## Continuous-motion input: OSC and Art-Net

See **`OSC_ARTNET_GUIDE.md`** for the full address/channel reference,
example packets, and — importantly — the "Gotchas" section covering
non-obvious real-hardware behavior (direction value conventions, the
Alarm-12 side effect of servo-off, absolute vs. cumulative angle
tracking, the drive's pulse-range check, and the continuous-motion
direction-reversal fail-safe).

## Web UI access

The web UI can move the motor, set home and write drive parameters, and has
no login of its own. Configure it with environment variables before starting
`app.py`:

| Variable | Default | Meaning |
|---|---|---|
| `SERVO_WEB_PASSWORD` | *(unset = no login)* | Turns on HTTP Basic auth for every route |
| `SERVO_WEB_USER` | `servo` | User name for that login |
| `SERVO_WEB_HOST` | `0.0.0.0` | Interface to listen on; use `127.0.0.1` for local-only |
| `SERVO_WEB_PORT` | `5000` | Port |
| `SERVO_WEB_DEBUG` | *(unset)* | `1` enables Flask's debugger — never on a shared network (it allows arbitrary code execution) |

When the UI is reachable from the network without a password, startup logs a
warning (visible in the Activity Log). Flask's built-in server is still a
development server; see the deployment notes for the Pi.

## Hardware safety

Any code path that can enable/move/write live state to the motor has
been treated as requiring explicit confirmation before being considered
"done" throughout this project's development — this isn't just a
documentation convention, it shaped how features here were built and
tested (e.g. the direction-reversal fail-safe in
`speed_ctrl_action()`, the command-pulse range check in `pos_step_motion_by()`/
`post_step_motion_by()` (no 180° limit any more), the confirm-gated `/alarm/clear`,
and the web-vs-OSC/Art-Net ownership rule above). If
you're extending this code with a new path that can move the motor,
follow the same pattern: real-hardware verification before calling it
done, not just passing unit tests against mocks.

## Testing conventions

Tests mock the serial transport (`SerialPortManager`)/`ModbusRTUClient`
so the full suite runs without hardware attached. Real-hardware
verification (register read-backs, actual rotation) has been done
separately and is documented in commit messages and the design doc, not
re-run automatically — there is no CI hardware rig.
