# CLAUDE.md — pi_rs485_servo_ctrl

Project-specific rules. Inherits the general AI-agent engineering rules
(environment tiering, TDD rhythm, verification discipline, automation
guardrails, etc.) — this file only covers what's specific to *this* repo.
Deep memory: `memory/lessons.md` (see §8).

## 1. Project Overview
Controls an AC servo motor (Shihlin SDE-series driver) over RS485 from a
**Raspberry Pi 3 Model B**. Flask web app + serial/Modbus (RTU and ASCII)
protocol handling + GPIO. `main.py` boots `web/app.py`.

<<<<<<< HEAD
## 2. Hardware Platform — Raspberry Pi 3 B (CRITICAL for serial work)
The Pi 3 B has known quirks that directly affect RS485/Modbus reliability.
**Verify these against the actual `config.txt` and wiring before assuming
either state — don't guess which one this repo relies on.**

* **Two UARTs, only one is "real."** Unlike the Pi 2 and earlier boards,
  the Pi 3 B's full hardware UART (PL011) is wired to the onboard
  Bluetooth chip, not to the GPIO header. GPIO pins 8/10 (TXD0/RXD0) get
  the **mini-UART** (ttyS0) instead, which is a simpler, lower-featured
  peripheral.
* **Mini-UART baud rate is tied to the ARM core clock.** Unless
  `core_freq=250` (or equivalent fixed-clock setting) is set in
  `config.txt`, the core's dynamic frequency scaling will drift the
  mini-UART's actual baud rate under load — this shows up as intermittent
  CRC/Modbus framing errors that look like a wiring or timing bug in the
  RS485 transceiver code, but are actually a clock config issue. If you
  hit sporadic checksum failures, check `config.txt` before touching
  `servo_comm_shihlin/` or `servo_communication/` protocol code.
* **If RS485 needs the good UART**, the standard fix is
  `dtoverlay=disable-bt` (or `dtoverlay=pi3-disable-bt`) in `config.txt`
  to free the PL011 for GPIO use, plus disabling/reassigning the
  `hciuart` service — otherwise Bluetooth and the RS485 link fight over
  the same UART. This project doesn't use Bluetooth, so this is a
  reasonable thing to propose, but it's a boot-config change — confirm
  with the user before editing `/boot/config.txt` or systemd units on
  the actual Pi.
* **RS485 direction control (DE/RE)**: if a MAX485-style transceiver is
  used rather than a USB-RS485 dongle, driver-enable/receiver-enable is
  usually toggled via a spare GPIO pin, timed around each write. Treat
  any code doing this GPIO toggle as part of the "sends real commands to
  the motor" hardware-safety surface in §4 — a stuck DE/RE line can wedge
  the bus for both directions.
* **Power budget**: Pi 3 B wants a stable 5V/2.5A supply. Never assume
  the servo driver, RS485 transceiver board, or any peripheral can be
  bus-powered off the Pi's 5V/3.3V rails without checking current draw
  first — brownouts under motor-driver load manifest as random Pi
  reboots or SD card corruption, not obvious wiring faults.
* **Modest CPU (4-core Cortex-A53 @ 1.2GHz, 1GB RAM)**: don't add
  polling loops, busy-waits, or synchronous blocking I/O tighter than
  necessary in the Flask routes — this board does not have Pi 4/5-level
  headroom, and CPU contention can itself perturb serial timing.
* **SD card wear**: avoid adding verbose/high-frequency logging to the
  SD card by default (e.g. per-Modbus-frame debug logs). Pi 3 B has no
  eMMC/SSD fallback — the SD card is the only storage, and it's the
  most common hardware failure point on long-running Pi deployments.

## 3. Canonical Module — CONFIRMED 2026-07-22, UPDATED 2026-09-17
Repo has several parallel module folders. None are duplicates or dead
code — each serves a distinct, confirmed purpose:

| Folder | Purpose | Status |
|---|---|---|
| `servo_comm_shihlin/` | Type 2 motor (Shihlin SDE-series driver). See README "AC Servo Motor Type 2 Info." | **Actively developed** (404 commits). Self-contained Flask UI in its own `app.py` (full action-route set). Also run via `python osc_2.py` (OSC server), per README. |
| `servo_comm_shihlin_50W/` | Hardware-variant fork of `servo_comm_shihlin/`, for the SDE-010A2 driver + SME-L00530 (50W) motor specifically. | Active. Only motor-specific files differ (`servo_config.json`, `servo_control.py`, `servo_p_register.py`, `serial_port_manager.py`, `osc_2.py`) — **but `app.py` is now identical between the two folders too** (both have the full Shihlin action-route set), so `app.py` joins the "kept in sync manually" list below. |
| `servo_comm_shihlin_unified/` | In-progress merge of `servo_comm_shihlin/` + `servo_comm_shihlin_50W/` into one codebase, switching motors via a JSON config (`motor_profiles.json`) instead of manual copy-sync. Design doc: `docs/servo_comm_shihlin_merge_design.md`. | **Design/build phase, not runnable — no `app.py`/`osc_2.py`/`servo_control.py` yet.** Only the communication layer (copied from `servo_comm_shihlin_50W`, the decided baseline) plus a read-only PA28 (absolute-mode) fail-safe check (`absolute_mode_check.py`, `check_pa28.py`) exist so far; tests pass (verified 2026-09-17). Blocked on the user running `check_pa28.py` on real Pi hardware to confirm PA28==1 before `servo_control.py` is migrated/rewritten. Not referenced by `main.py` or any entrypoint. Per its own README, `servo_comm_shihlin/`/`servo_comm_shihlin_50W/` stay in place — not deleted or deprecated — until this passes real-hardware validation on both motor profiles. |
| `servo_communication/` | Type 1 motor (different brand). See README "AC Servo Motor Type 1 Info." | **Active, and now the real entrypoint** — `app.py` is self-contained with its own full action-route set and local imports; it can run standalone without `web/`. |
| `web/` | Formerly a separate Flask UI. `main.py` still boots it (`from web.app import app`). | **Superseded by `servo_communication/app.py` (confirmed 2026-07-22).** `web/app.py` was rewritten to import directly from the `servo_communication` package rather than duplicating its helpers, and its routes now just mirror `servo_communication/app.py`. Treat `web/` + `main.py` as the legacy path — don't build new features here; extend `servo_communication/app.py` instead. `main.py` has not been updated to reflect this yet (see §6). |
| `examples/` | Scratch/dev sandbox. | **Not deployed, not canonical.** Reference-only. |

Practical routing: Shihlin/Type 2 work → `servo_comm_shihlin*` (mind the
manual-sync list above; `servo_comm_shihlin_unified/` is the in-progress
merge target but isn't runnable yet — don't route active feature work
there until it's past the PA28 blocker). Type 1 work →
`servo_communication/app.py` directly. Exploratory/reference only →
`examples/`. If a task still doesn't map cleanly onto this table, ask
before editing rather than guessing.

Because `servo_comm_shihlin/` and `servo_comm_shihlin_50W/` share logic
by copy, not by import, a bug fix or behavior change in one that isn't
in the motor-specific diff list above (including `app.py` now) should
generally be mirrored in the other — confirm with the user which scope
a change should cover.

## 4. Hardware Safety (CRITICAL)
This code sends real commands over RS485 to a physical AC servo motor
attached to a physical Raspberry Pi 3 B. Any code path that can enable,
move, or write live state to the motor (e.g. SVON, jog mode, homing,
`write_*` parameter calls, or any GPIO DE/RE toggle per §2) requires
**explicit user confirmation before considering the change complete** —
treat this the same as a production deploy gate. Read-only/status-query
paths don't need this gate.

## 5. Testing
* No pytest config. Tests use `unittest` with **relative imports**
  (`from test_x import Y`), so they only work when run from *inside*
  their own directory (e.g. `cd servo_comm_shihlin/ && python -m
  unittest test_servo_control.py`), not via top-level pytest discovery.
  This is a known repo quirk — don't "fix" the imports unless asked.
* Some test files are smoke/print scripts (print output, no assertions),
  especially ones exercising serial/GPIO hardware — they can't be
  asserted without the physical Pi 3 B + RS485 transceiver + servo
  driver attached. Don't treat "no assertions" as a bug to silently
  patch.
* `tests/test_crc.py` is misnamed — it's actually a `CRC16CCITT`
  implementation, not a test file.
* This is ARMv8 (Cortex-A53) hardware — if a dependency ever needs to be
  pinned or built from source, don't assume x86 wheels are available;
  check for `armv7l`/`aarch64` compatibility first.

## 6. Known Issues (documented, not auto-fixed)
* `main.py` still boots `web/app.py`, but `web/` is now superseded by
  `servo_communication/app.py` (see §3). **This is a paused/idled
  feature, not an oversight:** the original plan (see first commit
  touching `main.py`) was for `main.py` + a JSON config file to select
  *which motor/scenario module to run* — it briefly imported both
  `web` and `servo_communication` behind a `main()` dispatcher before
  being simplified to directly boot `web.app`. That config-driven
  selector was never built out; the task is idle, not abandoned.
  **Don't "fix" this by just repointing the one-line import to
  `servo_communication.app`** — that would miss the actual intent
  (config-driven selection across motor variants). Confirm with the
  user whether to revive the original multi-module dispatch plan or
  formally commit to a single hardcoded entrypoint before touching it.
* `web/app.py` / `servo_communication/app.py`: `app.secret_key` falls
  back to a hardcoded default string if `FLASK_SECRET_KEY` isn't set in
  the environment. Known — don't silently patch; flag it if relevant to
  a task.
* No linter/formatter configured. Match the existing style of the file
  you're editing rather than imposing one.
* `requirements.txt` pins only lower bounds (`>=`), no lockfile — don't
  assume a specific version is required unless told.
* UART/Bluetooth conflict on Pi 3 B (§2) is a standing config question,
  not resolved in this doc — check `/boot/config.txt` on the actual
  device rather than assuming either state.

## 7. Deployment Context
No dev/staging/prod tiers here. Deployment is PM2 or a systemd service
running directly on the Raspberry Pi 3 B (see README). Treat "the Pi"
as production — changes there affect a live, physically-attached motor
immediately, not just data. Given the board's modest CPU/RAM (§2), avoid
recommending the Flask dev server for long-running production use;
prefer whatever WSGI setup is already configured rather than introducing
a new one.

## 8. Continuous Learning
Same two-tier pattern as the general rules doc: verified, dated lessons
go into `memory/lessons.md`; if one turns out recurring/load-bearing,
distill and promote it into this file.