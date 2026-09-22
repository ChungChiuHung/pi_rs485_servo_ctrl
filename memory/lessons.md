# Lessons Learned (Deep Memory) — pi_rs485_servo_ctrl

Dated, verified, incident-specific lessons from real development on this
repo. CLAUDE.md stays lean; this file can grow indefinitely. Only add
entries once verified in practice — not speculative.

Format:

```
## [YYYY-MM-DD] Short title
**Relates to:** CLAUDE.md §N (or "new pattern")
**What happened:** ...
**Lesson:** ...
**Status:** active | promoted to CLAUDE.md | superseded
```

---

## [2026-07-22] Canonical module structure resolved
**Relates to:** CLAUDE.md §2
**What happened:** Investigated the module-folder question via git
archaeology (commit counts, last-touched dates per folder) plus direct
diff between `servo_comm_shihlin/` and `servo_comm_shihlin_50W/`, then
confirmed the interpretation with the user in three rounds of Q&A.
Findings: `servo_comm_shihlin*` = Type 2 (Shihlin) motor, actively
developed, run via `osc_2.py`; `servo_communication/` = Type 1 (other
brand) motor; `web/` = Flask UI for `servo_communication/`, still live
despite stale git history (main.py boots it); `examples/` = scratch
sandbox, confirmed not deployed. Initial git-history-only guess (that
web/servo_communication/examples were all uniformly "abandoned") was
**wrong** — user corrected that `servo_communication/` and `web/` are
both still actively used, just for a different hardware variant than
the one with the most recent commits.
**Lesson:** Commit recency/count alone is not sufficient to judge
whether a module is "live" vs "dead" in a multi-hardware-variant repo —
a folder can look stale simply because that variant's code is stable,
not because it's abandoned. Always confirm with the user before
labeling anything legacy, even with strong-looking git evidence.
**Status:** promoted to CLAUDE.md §2

## [2026-07-22] Web UI integrated into each parallel project's app.py
**Relates to:** CLAUDE.md §2, §5
**What happened:** User reported having integrated "the Web" into each
parallel project's own `app.py`. Verified by reading all four `app.py`
files: `servo_comm_shihlin/app.py` and `servo_comm_shihlin_50W/app.py`
are now identical (177 lines, full Shihlin action-route set) —
confirming `app.py` is now part of their manual-sync surface, not just
the previously-documented motor-config files. `servo_communication/app.py`
gained a full, self-contained action-route set and local imports
(no longer a stub). `web/app.py` was rewritten to import directly from
the `servo_communication` package instead of duplicating its helper
modules, and its routes now just mirror `servo_communication/app.py`.
User confirmed `web/` (and by extension `main.py`, which still boots
it) is now the superseded path — `servo_communication/app.py` is the
real entrypoint going forward.
**Lesson:** Structural claims from the user ("I integrated X") are
worth verifying against the actual files even when plausible — in this
case the claim was fully accurate, but verifying surfaced a knock-on
fact the user hadn't mentioned: `main.py` was NOT updated to point at
the new entrypoint, so the documented boot path and the real one have
now diverged. Surface knock-on inconsistencies like this rather than
only checking the specific claim made.
**Status:** promoted to CLAUDE.md §2 and §5

## [2026-07-22] main.py's simplicity is a paused plan, not neglect
**Relates to:** CLAUDE.md §5
**What happened:** User clarified that `main.py`'s stale, minimal state
(just `from web.app import app`) isn't an oversight — the original
intent was for `main.py` plus a JSON config file to select which
motor/scenario module to run at boot. Checked git history to confirm:
the very first commit touching `main.py` (7a73528, 2024-03-21) imported
both `web` and `servo_communication` behind a `main()` stub with the
comment "Main application logic here" — consistent with a planned
dispatcher. Ten days later (6618a27, 2024-03-31) it was simplified to
directly boot `web.app`, and has been untouched since. The
config-driven selector was never implemented; per the user, that task
is idle, not scrapped.
**Lesson:** A file that looks stale/neglected in git history can
actually be a paused feature with real, still-intended future scope.
Don't reduce "known issue" documentation to just "this is wrong, fix
it eventually" — capture the original intent so a future agent doesn't
"fix" it by prematurely collapsing it to one hardcoded path (e.g. just
repointing to `servo_communication.app`), which would foreclose the
actual planned functionality (multi-motor config-driven selection).
**Status:** promoted to CLAUDE.md §5

## [2026-07-22] Absolute encoder overflow risk in servo_comm_shihlin*
**Relates to:** CLAUDE.md §3 (hardware safety), servo merge design doc §2.4
**What happened:** User flagged that encoder overflow wasn't considered
in the merge design. Verified against
`docs/SDE_English_manual_UL_v107.pdf` §8 "Servo absolute system": the
SME motors use an absolute-type encoder (4,194,304 = 2^22 pulses/rev,
±32767 revolutions full range, with driver-side `AL.29` alarm and a
`PA31` status register exposing lost-position/overflow/battery-low
flags, plus `PA32`/`PA33` as a wide-range rev+pulse-within-rev
register pair). Both `servo_comm_shihlin/` and `servo_comm_shihlin_50W/`
only read the raw `0x0000`/`0x0024` registers (confirmed 2-word/32-bit
via manual, and confirmed unsigned via
`ModbusResponse.get_value()`'s `int.from_bytes(..., byteorder='big')`
with no `signed=True`). Since 2^32 / 2^22 = 1024, that raw register
silently wraps every 1024 motor-shaft revolutions — about 34 output
revs for the 400W profile (gear ratio 30) or 102 for the 50W profile
(gear ratio 10) — with no relation to the driver's own `AL.29`/`PA31`
overflow detection, which only guards the ±32767-revolution absolute
range, not this narrower 32-bit rolling counter. Manual p.187 also
lists "speed control mode" and "single way rotation" as operation
conditions "not suitable" for the absolute system, which directly
describes the continuous-rotation features (`motionStart_CW/CCW`,
`speed_ctrl_action`) already confirmed for the merge (§2.2 items 1–2).
**Lesson:** When a code path reads a fixed-width hardware register
(here, 32-bit Modbus registers) into an unbounded-precision language
value (Python int), the *language* won't overflow but the *source
register* still can — silently, without the driver's own alarm
mechanism necessarily catching it, if the software-visible register is
narrower than the range the alarm actually monitors. Don't assume "no
error was raised" means "no overflow occurred" — check whether the
error/alarm path actually covers the specific register your code
reads, not just the device's nominal absolute range.
**Status:** decided — user chose to switch to reading `PA32`(APR)+
`PA33`(APP) instead of the raw `0x0000`/`0x0024` registers, covering the
full ±32767-revolution range (`Total = R × 4,194,304 + p`). This also
requires fixing `ModbusResponse.get_value()` to parse `APR` as signed
(currently unsigned-only, a related latent bug). Remaining blocker
before implementation: confirm on real hardware whether `PA28` (ABS
mode select) is already set to 1 — PA32/PA33 are only valid when it is,
and nothing in the current codebase sets or checks it, suggesting it's
a one-time hardware/DIP setting rather than software-controlled. §2.2
items 4/7 are now unblocked in principle (a full-range position source
exists to redesign them on) but still await final confirmation pending
the PA28 check. Not yet implemented — still design phase.

## [2026-07-22] PA28 fail-safe check built (TDD, read-only)
**Relates to:** CLAUDE.md §3 (hardware safety), servo merge design doc §2.4,
`servo_comm_shihlin_unified/`
**What happened:** Built the PA28 (absolute-mode) fail-safe check requested
by the user: copied the communication-layer files (`modbus_ascii_client.py`,
`modbus_response.py`, `modbus_command_code.py`, `modbus_utils.py`,
`servo_control_registers.py`, `serial_port_manager.py`, `servo_p_register.py`)
from `servo_comm_shihlin_50W/` (the decided §2.2 baseline) into the new
`servo_comm_shihlin_unified/` folder, added a `PA.ABS` (PA28) register
definition, then wrote `absolute_mode_check.py` (the check function),
`test_absolute_mode_check.py` (7 unit tests, mocked `ModbusResponse`, all
passing — verified by actually running `python3 -m unittest`), and
`check_pa28.py` (a standalone script to run against real hardware). Did NOT
copy `servo_control.py` yet, since §2.2 items 4/7 and the config-driven
gear_ratio refactor aren't finalized — copying it now would misrepresent
progress. This is a read-only status query, so it didn't need the
hardware-safety confirmation gate (CLAUDE.md §3 exempts read-only paths) —
proceeded straight to implementation instead of asking first.
**Lesson:** When a task is explicitly scoped ("just add X check") inside a
larger paused merge, resist the urge to either (a) do the minimum humanly
possible (a throwaway script disconnected from the real infrastructure) or
(b) over-deliver by pulling in the whole pending merge early. The right
scope was: copy only the infrastructure the one new feature actually
depends on, skip the files still gated on undecided design questions, and
make sure what's copied now is reusable by the real merge later (not
throwaway).
**Status:** active — waiting on user to run `check_pa28.py` on the Pi and
report back whether PA28 == 1.

<!-- New verified entries go below this line -->

## [2026-09-22] ENABLE SPEED CONTROL MODE could auto-resume rotation -- 0x0904 is sticky
**Relates to:** `servo_comm_shihlin_unified/servo_control.py` (`enable_speed_ctrl()`)
**What happened:** User reported that a single click of "ENABLE SPEED CONTROL
MODE" in the web UI could still start continuous rotation, even with the
MOTION START CW/CCW buttons already removed (keyboard-only control). Root
cause: `enable_speed_ctrl()`'s enable=True path never wrote 0x0904
(JOG_OPERATION, the actual run/stop register) -- it only entered JOG mode
and configured accel/speed. Confirmed live: 0x0904 is a STICKY register on
this drive, not reset by entering/leaving JOG mode. If a prior session left
it at 1/2 (CW/CCW) -- e.g. a MOTION PAUSE keyup that never landed -- merely
re-arming JOG mode later resumed rotation with no CW/CCW pressed that time.
**Lesson:** Also confirmed live: leaving JOG mode (0x0901=0) DOES stop the
physical rotation itself (encoder settled to ~0 delta within 0.5s), so the
danger was specifically the NEXT arm, not a rotation that never stopped.
Don't assume a register the manual only documents as part of one
mode-entry sequence gets reset by that same sequence -- verify explicitly.
**Fix:** `enable_speed_ctrl()`'s enable=True path now ends with an explicit
`speed_ctrl_action(0)` (0x0904=0) before starting the keep-alive polling,
guaranteeing every arm ends in a definite stopped state regardless of
leftover register state. The CW/CCW/stop API itself is unchanged.
**Status:** verified live 2026-09-22 via `verify_jog_enable_no_autospin.py`
(kept in the repo): manually left 0x0904 dirty at CW, confirmed rotation
had genuinely stopped after leaving JOG mode, then confirmed the fixed
`enable_speed_ctrl(enable=True)` produced only 115 pulses of drift over 2s
(noise) instead of resuming rotation, and that an explicit CW afterward
still worked normally.

## [2026-09-22] "Released the arrow key but the motor kept turning" -- 5s decel ramp, not a stuck stop
**Relates to:** `app.py`'s `enableSpeedCtrlMode` action
**What happened:** User reported releasing a JOG arrow key didn't stop the
motor. `enableSpeedCtrlMode` called `enable_speed_ctrl(speed_rpm)` with no
`acc_time`, silently defaulting to `enable_speed_ctrl()`'s own 5000ms
smooth-ramp default. MOTION PAUSE (0x0904=0) genuinely was sent immediately
on keyup, but the drive then took up to 5s to decelerate -- easily mistaken
for "didn't stop" in a press-and-hold interaction where release is supposed
to feel immediate.
**Lesson:** A method's own sensible default for one calling context (a
smooth industrial ramp) can be actively wrong for a different calling
context (snappy keyboard press/release) that never overrides it. Don't
assume a shared default is fine everywhere it's used without checking what
each caller actually needs.
**Fix:** Added `JOG_ACC_DEC_MS = 200` (matching the precedent
`DEFAULT_POS_TEST_STEP_ACC_DEC_MS` already used for POS TEST's own
keyboard nudge) and pass it explicitly from `enableSpeedCtrlMode`. OSC's
`/set_continous_motion` and Art-Net's Channel 1 both already take/configure
their own acc_time explicitly, so neither needed a change.
**Status:** verified live 2026-09-22 via `verify_jog_release_stops_fast.py`
(kept in the repo): stop-to-settled time dropped from up to 5s to 0.344s.

## [2026-09-22] Live JOG speed change confirmed on real hardware
**Relates to:** `servo_comm_shihlin_unified/servo_control.py`
(`change_jog_speed_by()`), `app.py` (`jogSpeedAdjust` action), `osc_server.py`
(`/jog_speed_adjust`), `artnet_server.py` (Channel 1's docstring)
**What happened:** User asked for a merged ENABLE SPEED CONTROL MODE /
MOTION CANCEL toggle button, arrow-key JOG control (Left/Right = MOTION
START CW/CCW + release = MOTION PAUSE, Up/Down = speed +/-1rpm), and asked
to investigate whether the JOG speed (0x0903) can actually be changed while
the motor is already rotating -- Art-Net's `artnet_server.py` was designed
assuming yes (Channel 1 writes speed live), but nothing in this repo's
history had confirmed it against a real drive. Verified via
`verify_jog_speed_adjust_live.py`: commanded 10rpm CW JOG rotation, measured
real speed from the raw encoder (9.94rpm), nudged to 15rpm mid-rotation via
`change_jog_speed_by(+5)` (a bare 0x0903 write, no re-trigger of 0x0904),
measured again (15.08rpm) -- the change took effect live, no stop/restart
needed.
**Lesson:** For this drive, 0x0903 (JOG speed command) is a true live
setpoint while 0x0904 (JOG_OPERATION) is running, not a value only read at
the moment 0x0904 is triggered. Safe to keep relying on this for both
Art-Net's continuous fader (already did) and the new discrete +/-1 rpm
keyboard/OSC nudge.
**Status:** done. All three surfaces (web keyboard, OSC `/jog_speed_adjust`,
Art-Net Channel 1) share the same underlying confirmed behavior; 243+
new/updated unit tests pass, full suite 514 tests green.

## [2026-09-22] PA28 switched to 1 (absolute mode) on real hardware — supersedes prior "PA28=0" entries
**Relates to:** CLAUDE.md §3 (`servo_comm_shihlin_unified` row), the two
entries above ("Absolute encoder overflow risk", "PA28 fail-safe check
built"), `docs/servo_comm_shihlin_merge_design.md` §2.4/§7.C
**What happened:** User confirmed the SDH-BAT-SET backup battery is now
installed and the motor (SME-L04030MCB) is a genuine absolute-encoder type.
Ran the design doc's pre-written §7.C "C1" switch sequence against the real
drive via COM4/RTU/115200, using `ServoController.write_PA28_Encoder_Mode()`
(already implemented, previously only unit-tested):
wrote PA28=1 (read-back confirmed) -> physical power-cycle -> AL.2A (expected)
-> physical power-cycle again -> AL.2C (expected) -> `write_PA29_Initial_Abs_Pos()`
(PA29=1) cleared AL.2C, PA31(APST) read back 0 (no fault bits) ->
power-cycling reset PD16/PD25 to 0, so AL.12 (EMG) reappeared (known existing
behavior, not new) -> cleared via the established
`write_PD_16_Enable_DI_Control()` + `clear_alarm_12()` sequence -> final state:
alarm=0xFF (none), PA28=1, PA31=0, `read_absolute_position_pulses()` reading
-1 then -95 (noise-level, consistent with the origin PA29=1 just set). No
AL.24 appeared at any point.
**Lesson:** CLAUDE.md §3's `servo_comm_shihlin_unified` row currently states
"PA28 confirmed 0/incremental-mode, so the merge uses software-side
wraparound tracking, not PA32/PA33" — **this is now stale** and needs
updating; the drive is live in absolute mode as of this date. Don't trust a
dated "confirmed on real hardware" note as permanent — PA28 is exactly the
kind of parameter this project itself documented as switchable, and it was
switched.
**Status:** C1 (the switch itself) verified live; CLAUDE.md §3 updated same
session. C5's core claim also now verified live the same session: SET HOME
+ a real 90deg/20rpm move (landed +0.0008deg) -> user power-cycled the
drive -> a fresh ServoController, WITHOUT calling set_home_position()
again, read back an angle only 0.0039deg off -- position genuinely survives
a power cycle. C2-C4, C6-C10 in `docs/servo_comm_shihlin_merge_design.md`
§7.C (PA30 handshake timing, APR/APP register layout, battery-removed
degraded behavior, moving the shaft while powered off, JOG coexistence,
switching back to PA28=0) are still unverified on real hardware — only
unit-tested.
