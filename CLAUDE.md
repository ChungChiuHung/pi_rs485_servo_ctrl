# CLAUDE.md — pi_rs485_servo_ctrl

Project-specific rules. Inherits the general AI-agent engineering rules
(environment tiering, TDD rhythm, verification discipline, automation
guardrails, etc.) — this file only covers what's specific to *this* repo.
Deep memory: `memory/lessons.md` (see §7).

## 1. Project Overview
Controls an AC servo motor (Shihlin SDE-series driver) over RS485 from a
Raspberry Pi. Flask web app + serial/Modbus (RTU and ASCII) protocol
handling + GPIO. `main.py` boots `web/app.py`.

## 2. Canonical Module — CONFIRMED 2026-07-22, UPDATED 2026-07-22
Repo has several parallel module folders. None are duplicates or dead
code — each serves a distinct, confirmed purpose:

| Folder | Purpose | Status |
|---|---|---|
| `servo_comm_shihlin/` | Type 2 motor (Shihlin SDE-series driver). See README "AC Servo Motor Type 2 Info." | **Actively developed** (404 commits). Self-contained Flask UI in its own `app.py` (full action-route set). Also run via `python osc_2.py` (OSC server), per README. |
| `servo_comm_shihlin_50W/` | Hardware-variant fork of `servo_comm_shihlin/`, for the SDE-010A2 driver + SME-L00530 (50W) motor specifically. | Active. Only motor-specific files differ (`servo_config.json`, `servo_control.py`, `servo_p_register.py`, `serial_port_manager.py`, `osc_2.py`) — **but `app.py` is now identical between the two folders too** (both have the full Shihlin action-route set), so `app.py` joins the "kept in sync manually" list below. |
| `servo_communication/` | Type 1 motor (different brand). See README "AC Servo Motor Type 1 Info." | **Active, and now the real entrypoint** — `app.py` is self-contained with its own full action-route set and local imports; it can run standalone without `web/`. |
| `web/` | Formerly a separate Flask UI. `main.py` still boots it (`from web.app import app`). | **Superseded by `servo_communication/app.py` (confirmed 2026-07-22).** `web/app.py` was rewritten to import directly from the `servo_communication` package rather than duplicating its helpers, and its routes now just mirror `servo_communication/app.py`. Treat `web/` + `main.py` as the legacy path — don't build new features here; extend `servo_communication/app.py` instead. `main.py` has not been updated to reflect this yet (see §5). |
| `examples/` | Scratch/dev sandbox. | **Not deployed, not canonical.** Reference-only. |

Practical routing: Shihlin/Type 2 work → `servo_comm_shihlin*` (mind the
manual-sync list above). Type 1 work → `servo_communication/app.py`
directly. Exploratory/reference only → `examples/`. If a task still
doesn't map cleanly onto this table, ask before editing rather than
guessing.

Because `servo_comm_shihlin/` and `servo_comm_shihlin_50W/` share logic
by copy, not by import, a bug fix or behavior change in one that isn't
in the motor-specific diff list above (including `app.py` now) should
generally be mirrored in the other — confirm with the user which scope
a change should cover.

## 3. Hardware Safety (CRITICAL)
This code sends real commands over RS485 to a physical AC servo motor.
Any code path that can enable, move, or write live state to the motor
(e.g. SVON, jog mode, homing, `write_*` parameter calls) requires
**explicit user confirmation before considering the change complete** —
treat this the same as a production deploy gate. Read-only/status-query
paths don't need this gate.

## 4. Testing
* No pytest config. Tests use `unittest` with **relative imports**
  (`from test_x import Y`), so they only work when run from *inside*
  their own directory (e.g. `cd servo_comm_shihlin/ && python -m
  unittest test_servo_control.py`), not via top-level pytest discovery.
  This is a known repo quirk — don't "fix" the imports unless asked.
* Some test files are smoke/print scripts (print output, no assertions),
  especially ones exercising serial/GPIO hardware — they can't be
  asserted without the physical Pi + servo driver attached. Don't treat
  "no assertions" as a bug to silently patch.
* `tests/test_crc.py` is misnamed — it's actually a `CRC16CCITT`
  implementation, not a test file.

## 5. Known Issues (documented, not auto-fixed)
* `main.py` still boots `web/app.py`, but `web/` is now superseded by
  `servo_communication/app.py` (see §2). **This is a paused/idled
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

## 6. Deployment Context
No dev/staging/prod DB tiers here. Deployment is PM2 or a systemd
service running directly on the Raspberry Pi (see README). Treat "the
Pi" as production — changes there affect a live, physically-attached
motor immediately, not just data.

## 7. Continuous Learning
Same two-tier pattern as the general rules doc: verified, dated lessons
go into `memory/lessons.md`; if one turns out recurring/load-bearing,
distill and promote it into this file.
