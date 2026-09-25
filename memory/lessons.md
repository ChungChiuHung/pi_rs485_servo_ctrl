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

## [2026-09-22] Ported unified's JOG/Position-Mode keyboard-control feature set to servo_comm_shihlin (ASCII)
**Relates to:** CLAUDE.md §3 (`servo_comm_shihlin/` row), `servo_comm_shihlin_unified/`'s
whole 2026-09-22 session (mutual exclusion, sticky-0x0904 fix, toggle race
fix, JOG_ACC_DEC_MS fix, hardware_lock STOP_ACTIONS fix, arrow-key UI)
**What happened:** User asked to port everything developed/fixed in
`servo_comm_shihlin_unified` this session, except Art-Net, into
`servo_comm_shihlin`. Before starting, found the current branch's
`servo_comm_shihlin` was missing an entire separate, already-complete,
never-merged local branch (`fix/shihlin-positioning-test-reliability`: the
Lock->RLock fix, Servo-ON JOG-mode precondition, ASCII transaction-lock
reliability, `pos_test_step()`) -- merged that in first (clean, no
conflicts, only touches `servo_comm_shihlin/` files) as the correct
baseline, per user confirmation.
Also found (and fixed as part of the port, since the ported mutual-exclusion
feature depends on it): `read_test_mode_0x0901()` never actually returned a
value (logged and implicitly returned `None` always) -- copied unified's
working implementation. `/status`'s own docstring claimed "no serial lock
for that" as a reason to avoid fresh reads, but the merged reliability
branch had already added the needed `_transaction_lock` to
`modbus_ascii_client.py`, making that claim stale; safe to add the
`ctrl_mode_sel` read now.
**Explicitly out of scope (user-confirmed):** OSC's `/jog_speed_adjust` --
this folder's OSC server has no continuous-motion foundation at all
(`/set_continous_motion`, `/ctrl_continuous_motion` don't exist), so
porting just the nudge endpoint would have nothing to nudge; porting the
whole foundation was judged separate, bigger scope. Art-Net -- this folder
never had it.
**Noteworthy, NOT changed:** `speed_ctrl_action()`'s CW/CCW numeric mapping
here is `1=CW, 2=CCW` in `app.py`'s `motionStart_CW/CCW` calls -- the
OPPOSITE of unified's `2=CW, 1=CCW`. This folder's own existing comment
already flags the mapping as "unconfirmed" (manual says 1=forward/CCW,
2=reverse/CW; this code assumed the opposite for logging purposes only).
Left as-is -- correcting it would flip real motor direction for these
buttons and is a separate decision, not part of this port.
**Status:** done. 26 new tests (`test_jog_speed_control.py`,
`test_mode_mutual_exclusion.py`, `test_hardware_lock.py`) plus updates to
2 pre-existing tests whose assertions matched the old (now intentionally
changed) behavior. Full suite: 181 tests, only 2 pre-existing unrelated
failures (`test_modbus_rtu_client.py` references a
`ServoControlRegistry.POS_PULSES_CMD_1` constant that doesn't exist --
predates this port, not touched). Unit-tested with mocks only, per this
folder's own convention -- never run against a drive.

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

## [2026-09-22] Follow-up: "click twice+ still spins" was a client-side race, not the server
**Relates to:** the entry above, `templates/index.html` (ENABLE POS MODE /
ENABLE SPEED CONTROL MODE toggle buttons)
**What happened:** User reported that clicking ENABLE SPEED CONTROL MODE
two-or-more times still produced continuous rotation, even with the fix
above in place. Code review found the real mechanism: the toggle buttons'
"enable vs disable" decision reads `jogModeActive`/`posModeActive`, which
were ONLY ever updated by the periodic `/status` poll -- never immediately
from the toggle's own click response. Neither button has a `data-action`
attribute, so `setButtonsEnabled()` (which disables `.button[data-action]`
while a request is in flight) never covers them either. A user clicking
twice quickly (trying to turn it back off) would have BOTH clicks see the
same stale "not active" flag and take the SAME "enable" branch -- so every
click kept re-arming instead of the second one ever actually disarming,
and the button never visibly changed to "DISABLE" either.
**Investigated on real hardware whether the re-arm itself was the danger**:
called `enable_speed_ctrl(enable=True)` twice in a row 0.3s apart (a
realistic fast double-click, simulating the race directly) via
`verify_double_enable_no_spin.py` (kept in the repo) -- result: 34 pulses
over 3s (noise), motor did NOT spin. So the server/drive side was already
safe after the earlier fix; the "still spinning" symptom was entirely the
client never routing a second click to the correct (disable) action.
**Fix:** both toggle buttons now update their own tracked
state (`jogModeActive`/`posModeActive`) and the buttons/lock/indicator UI
immediately from their own AJAX success callback, plus a
`modeToggleInFlight` guard so a second click before the first's response
lands is ignored outright rather than racing.
**Status:** client-side fix in place; server-side re-arm safety confirmed
live. Not yet re-tested end-to-end through an actual browser double-click
(no browser automation available in this environment) -- the underlying
ServoController calls are verified and the JS race is gone by code
inspection, but the user should click through it once in a real browser
to be sure.

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

## [2026-09-22] "Fast tap on the arrow key still spins forever" -- the STOP request itself was being dropped
**Relates to:** `hardware_lock.py` (`hardware_serialized`, `STOP_ACTIONS`)
**What happened:** User reported that pressing and quickly releasing a JOG
arrow key still left the motor spinning continuously. Root cause: keydown's
motionStart_CW/CCW and keyup's motionPause are two independent /action
requests; `hardware_serialized`'s busy-lock REJECTS (429) any request that
arrives while another is in flight rather than queuing it (deliberate
design, to stop a stale double-click firing later out of order -- see the
module's own docstring). A fast enough tap fires motionPause while
motionStart_CW/CCW is still being processed, so the stop gets flatly
rejected -- and since the key has already been released, nothing ever
retries it. The motor keeps running with no other path to stop it.
**Lesson:** A "reject if busy" concurrency policy that's correct for
preventing stale duplicate actions is actively dangerous for a STOP-type
action specifically: unlike a duplicate start, a stop is never "stale" --
it's always correct to run as soon as the bus is free, and dropping it has
no other safety net. Don't apply one blanket concurrency policy to every
action without asking whether some of them have fundamentally different
correctness requirements.
**Fix:** `STOP_ACTIONS = {"motionPause", "motionCancel"}` now WAIT
(blocking, bounded by `STOP_ACTION_WAIT_TIMEOUT_S = 2.0`) for the lock
instead of being rejected immediately; every other action keeps the
original immediate-reject behavior unchanged.
**Status:** verified two ways 2026-09-22: (1) `test_hardware_lock.py` (9
tests, kept) exercises the race directly via real threading against a
throwaway Flask app -- confirms the wait/timeout/fallback logic in
isolation; (2) `verify_fast_tap_stops_motor.py` (kept) imports the REAL
app.py (real hardware) and fires motionStart_CW/motionPause from two
threads via Flask's test client (which runs the WSGI app synchronously per
calling thread, so two threads genuinely race the same lock -- no live
server or extra HTTP client library needed). Both requests returned 200;
the motor moved only ~7173 pulses (~0.02deg output-shaft, two-plus orders
of magnitude below what 3s of sustained 10rpm rotation would cover) before
settling, then read exactly 0 pulses drift on a follow-up check. (The
script's own first-pass pulse threshold of a flat 5000 wrongly flagged
this tiny, expected blip as "still spinning" -- fixed to scale against the
theoretical sustained-rotation pulse count instead of a flat number.)

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

## [2026-09-24] OSC/Art-Net status-feedback verified live; found a real bug in continuous-JOG-start angle reporting (absolute mode)
**Relates to:** `servo_comm_shihlin_unified/artnet_server.py`/`osc_server.py`
(the 2026-09-24 status-feedback feature), `servo_control.py`
(`start_continuous_reading()`/`_read_continuously()`/`_encoder_and_angle_for()`),
design doc §7.E1 (the related, already-fixed bug)
**What happened:** User approved real-hardware Phase 3 verification (on-site,
watching) of the newly-added OSC/Art-Net status-feedback feature, specifically
Art-Net channels 10-12's immediate-`_send_feedback()` fix, plus Modbus-traffic
timing during a feedback-enabled continuous move. Ran three new `verify_*.py`
scripts (kept in the repo) against the real drive (`/dev/ttyUSB0`, RTU,
absolute mode, PA28=1):
- **OSC feedback**: servo on/off, a real ~5deg move and back, set_home, and
  clear-alarm all produced the documented feedback messages
  (`/servo_on`, `/set_point`, `/moving`, `/motion_complete`,
  `/set_home_position`, `/servo_off`, `/clear`) at a local UDP listener.
  `/pr_step_path` and `/reset_initial_abs_position` were deliberately NOT
  triggered for real (PR mode unvalidated per design doc §8; PA29 rewrites
  the drive's absolute calibration reference) -- their feedback-echo logic
  was already covered by mocked unit tests, so only delivery needed proving,
  which the other handlers already did. All checks passed.
- **Art-Net channels 10-12**: back home (real ~2.7deg move), set home
  (persisted), and reset-initial-abs-position (handler call verified,
  `write_PA29_Initial_Abs_Pos()` itself mocked -- same reasoning as above)
  all produced immediate feedback packets, confirming the 2026-09-24 fix
  (adding `_send_feedback()` calls after these three handlers) works live.
  All checks passed.
- **Modbus traffic/timing**: a 10rpm continuous JOG with feedback enabled
  produced feedback packets at 127-143ms intervals (vs. the loop's nominal
  100ms), consistent with the documented "roughly doubles Modbus traffic"
  effect from channel 4's fresh `read_current_alarm_code()` call on every
  poll -- well under the drive's 1s test-mode keep-alive ceiling, no
  CRC/timeout failures during that window. One unrelated transient
  "No response received" / retry happened ~600ms after the back-home
  positioning command was issued while the continuous-reading loop's own
  poll was in flight -- self-recovered in ~120ms via the loop's existing
  retry logic; looks like an ordinary read/write collision at a command
  handoff, not something caused by the feedback feature.

**Bug found (pre-existing, not part of the 2026-09-24 feedback change):**
starting continuous JOG (`start_continuous_reading()`, e.g. via
`enable_speed_ctrl()`/Art-Net channel 1/OSC `/set_continous_motion`) as the
*first* action in a process, in absolute mode, before any discrete move or
explicit `_refresh_current_angle_from_hardware()` call, reports a WRONG
`current_angle` for the duration of that JOG run. Root cause:
`_encoder_and_angle_for()` (used by `_read_continuously()`'s poll loop) only
uses the correct absolute home reference (`abs_home_pos_absolute`) when
`self._absolute_offset` is already set -- and that field is only populated
by `_refresh_current_angle_from_hardware()` or `pos_step_motion_by()`
(part of the §7.E1 fix), neither of which `start_continuous_reading()`
calls. Until something else sets it, `_encoder_and_angle_for()` silently
falls back to the *incremental*-scale `self.abs_home_pos` (a stale/unused
value from before the PA28=1 switch -- currently `4247289155` in
`servo_config_shihlin_400W.json`, left over and never cleaned up), producing
a plausible-looking but wrong angle. Confirmed live: after servo-on with no
prior move, the real position was ~-2.7deg but the continuous loop's first
several polls reported ~19.5deg, decaying toward the true value only because
the motor was physically rotating during the JOG -- not because the offset
ever got corrected. `app.py` doesn't call `_refresh_current_angle_from_hardware()`
at connect time either (only `refresh_encoder_mode()`, which reads PA28 but
not position), so this is a real field-facing gap, not just a test-script
artifact: any user whose first action after an app restart is to start JOG
(rather than a discrete move) would see/broadcast a wrong angle. The new
Art-Net/OSC feedback feature makes this externally visible (channels 1-2)
where before it was only an internal display quirk.
**Lesson:** A fix scoped to "the discrete-move path" (§7.E1's
`post_step_motion_by()`/`pos_step_motion_by()` refresh) doesn't
automatically cover every other path that reads position in the same mode --
the continuous-reading loop reads `current_angle` through a completely
different function (`_encoder_and_angle_for()`) with its own fallback
behavior. When a bug's root cause is "a piece of state defaults to None/stale
until some specific call sets it," audit *every* caller that depends on that
state, not just the one the original bug report came through.
**Status:** fixed and verified live same day (2026-09-24), after explicit
user confirmation to proceed while the rig was still connected. Fix: gated
inside `start_continuous_reading()` (right after the "already active"
early-return, before the thread is spawned) --
`if self.absolute_mode and self._absolute_offset is None:
self._refresh_current_angle_from_hardware()` (logs a warning and still
proceeds to start on a failed refresh; JOG is speed-based, not
position-based, so a stale angle for one extra session isn't unsafe).
Placed inside `start_continuous_reading()` itself rather than only in
`enable_speed_ctrl()`, because a second, separate caller
(`app.py`'s raw `"enablePosMode"` action) also starts continuous reading
directly without a prior refresh -- putting the fix at the one shared
choke point covers both instead of requiring the same patch twice.
**First attempt regressed the test suite** (`start_servo_control.py`
discover hung indefinitely, confirmed via bisection down to
`TestSoftwareMotionCompleteDetection`): the first version called
`_refresh_current_angle_from_hardware()` **unconditionally** on every fresh
start, which silently consumes one value from
`read_motor_feedback_pulses()` -- tests that feed it a fixed
`side_effect` sequence (`[0, settled_value, settled_value, ...]`, simulating
"moved once then held still") starting right before
`start_continuous_reading()` had that leading value eaten before the
background thread's own loop ever saw it, breaking the encoder-moved
detection those tests depend on and leaving `auto_stop_on_stillness=False`
sessions (and, transitively, `auto_stop_on_stillness=True` sessions whose
completion never got detected either) running forever with no natural
throttling (`delay_ms` mocked to a no-op in those tests) -- one leaked
background thread from one failed/never-completing test then starved the
rest of the entire suite via CPU/lock contention. Scoping the guard to
`absolute_mode and _absolute_offset is None` fixed it: it's a no-op for
every incremental-mode test (the overwhelming majority, since absolute mode
is newer) and a no-op whenever the offset is already known, so it only ever
fires in the one scenario that actually needs it. **Lesson (compounding the
one above):** a fix's *placement* matters as much as its logic -- putting a
new synchronous side-effecting call at a shared entry point silently widens
its blast radius to every caller, including test harnesses that assumed
that entry point was side-effect-free beyond what they explicitly mocked.
Guard defensively (only run when actually needed) rather than
unconditionally "to be safe," especially at a choke point with many
callers. 3 new regression tests added
(`TestAbsoluteModePositioning.test_start_continuous_reading_refreshes_absolute_offset_first`
/ `..._skips_refresh_when_offset_already_known` /
`..._does_not_refresh_in_incremental_mode`) covering: the fix fires and
correctly sets `_absolute_offset` in absolute mode; it's skipped when the
offset is already known; it's skipped entirely in incremental mode. Full
suite 558/558 passing (555 + 3 new).
**Live re-verification** (`verify_phase3_jog_start_angle_fix.py`, kept in
the repo): reproduced the exact failure scenario -- a fresh
`ServoController`, `refresh_encoder_mode()` only (mirrors `app.py`'s
connect sequence), then continuous JOG as the literal first action, via
the real Art-Net feedback path. `_absolute_offset` was confirmed `None`
right after connect (the bug's precondition still exists on its own); the
**first** feedback packet reported `0.01deg` against an independently-read
true baseline of `0.0061deg` (0.0039deg difference, noise-level) --
compare to the original ~22deg error (19.5deg reported vs. ~-2.7deg real).
Rig left in a safe resting state afterward: alarm clear (0xFF), PA31=0,
servo off, current_angle -1.3836deg (a small residual from the brief
verification JOG, not re-homed since it's well within normal range).
Gotcha #9 (documenting the bug as a workaround-required limitation) was
added to `OSC_ARTNET_GUIDE.md` earlier the same day and has been removed
now that it's fixed; `CLAUDE.md` §3/§6 updated to match.

## [2026-09-25] Live test of every OSC address: two bugs, and /cancel_loop drops Servo ON
**Relates to:** `servo_comm_shihlin_unified/osc_server.py` (`/servo` dedupe,
`/cancel_loop`), `servo_control.py` (`initial_abs_home()`),
`OSC_ARTNET_GUIDE.md` Gotcha 9
**What happened:** User asked for every OSC address to be exercised against the
running server (real UDP to :5005, effects read back through `/status` and
`/log`; user on-site). All 10 addresses worked (set_point landed within
0.001deg, back_home within 0.001deg, JOG start/stop/reversal-refusal/live
speed nudge all correct, bad args logged without crashing). Two defects:
1. **`/servo` dedupe went stale.** `_check_duplicated()` only compared the new
   value with the last OSC argument. `/cancel_loop` (leaving JOG) makes the
   *drive* drop Servo ON, so `/servo 1.0` afterwards was ignored as a
   duplicate and the servo stayed off. Fix: a repeat is re-checked against
   `read_servo_state()` (throttled to once per `SERVO_STATE_RECHECK_S` = 1 s so
   a frame-rate resend never becomes a serial read per frame; an unreadable
   state stays suppressed), and `/cancel_loop` clears the cached value. 7
   regression tests (`TestServoDuplicateFilterFollowsHardware`).
2. **`/back_home` logged a negative "Estimate Timeout"** for moves in the
   negative direction (`angle_rotated` is signed). Fix: `abs()`. Regression
   test in `TestAbsoluteModePositioning`. **Not fixed, deliberately:** the
   estimate ignores the gear ratio (~30x short at 30:1), so "Operation timed
   out" still logs for every real move. Making it accurate would make the
   caller block for the whole move (web UI busy lock, Art-Net receive thread,
   OSC handler thread) -- a behavior change that needs its own decision.
**Also learned:** `/cancel_loop` turns Servo OFF as a hardware side effect;
documented for frontend authors (Gotcha 9).
**Test-harness lesson:** my first `settle()` helper returned as soon as
`reading_active` was False, which is also true *before* the move thread
starts, so early "landed" readings were mid-move and the next command
interrupted them. Wait for the start edge, then for the idle edge.
**Side effect of the test:** `/set_home` was run for real and moved the saved
home by 3,992 pulses (0.011deg). The config file was restored byte-for-byte
from a pre-test backup (`abs_home_pos_absolute` = -42289741); the running app
kept the shifted value in memory until it is restarted.
**Status:** fixed; suite green (see commit). Feedback packets to TouchDesigner
(192.168.0.101:5008) were not observable from the Pi side -- the user is
verifying those.

## [2026-09-25] Known behavior: `/back_home` always logs a premature "Operation timed out"
**Relates to:** `servo_control.py` `initial_abs_home()`, the entry above
**What:** the wait timeout is `1.2 * (abs(angle)/360) * (60/12)` -- it treats the
*output* angle as motor-shaft degrees, so it is roughly `gear_ratio` (30x for
`shihlin_400W`) too short. Every real `/back_home` (also web HOME and Art-Net
channel 10) therefore logs "Timeout reached while waiting for stop process" /
"Operation timed out" almost immediately and returns False, even though the
move runs to completion on the drive and the background reader reports
`/motion_complete` normally (seen live: 80deg move, ~22 s, landed within
0.001deg). Also clears `on_initial_home` early, so a second `/back_home`
during the move is not rejected.
**Decision (user, 2026-09-25):** leave it. A correct timeout would make the
caller block for the whole move (web UI busy lock, Art-Net receive thread, OSC
handler thread), which is worse than a spurious warning. Treat the warning as
noise; rely on `/motion_complete` (or `reading_active` in `/status`) to know
when the move finished.
**Status:** accepted limitation, not a bug to chase.
