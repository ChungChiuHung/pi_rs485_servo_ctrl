# OSC / Art-Net User Guide

`servo_comm_shihlin_unified` accepts continuous-motion and positioning
commands from three input sources that all drive the same
`ServoController` instance: the web UI, OSC, and Art-Net. **Only one of
OSC or Art-Net can run at a time** (starting one while the other is
active is rejected) — there's no reason for two remote protocols to be
able to issue conflicting motion commands to the same motor
simultaneously. The web UI's own buttons work regardless of whether an
input server is running.

This guide documents every OSC address and every Art-Net DMX channel as
actually implemented in `osc_server.py` / `artnet_server.py`, including
the non-obvious behavior found during real-hardware testing (2026-09-18
— 2026-09-19). If you're integrating a lighting console, TouchDesigner,
or similar, read the **Gotchas** section before wiring anything up.

## Starting/stopping the input servers

Via the web UI: the "Continuous Motion Input" section has a protocol
dropdown (OSC / Art-Net) and Start/Stop buttons.

Via the HTTP API directly:

```bash
# Start OSC (listens on UDP, default port 5005)
curl -X POST http://<HOST>:5000/server/start \
     -H "Content-Type: application/json" \
     -d '{"type": "osc", "listen_port": 5005}'

# Start Art-Net (listens on UDP 6454, the Art-Net standard port)
curl -X POST http://<HOST>:5000/server/start \
     -H "Content-Type: application/json" \
     -d '{"type": "artnet", "listen_ip": "192.168.1.50", "universe": 1, "allowed_sources": "192.168.1.10", "signal_timeout_s": 2, "max_speed_rpm": 100, "acc_time": 5000}'

# Stop whichever is running
curl -X POST http://<HOST>:5000/server/stop

# Check status
curl http://<HOST>:5000/server/status
```

OSC's optional feedback (see below) isn't exposed in the web UI — pass
`feedback_ip`/`feedback_port` in the `/server/start` body directly if you
need it.

---

## OSC

Default listen port: **5005** (UDP). Every address below is handled by
`OSCInputServer` in `osc_server.py`.

### Configuring feedback (status output)

Feedback is off by default and uses a **separate UDP socket from the
command-listen socket above** — `listen_port` (default 5005) is where this
server *receives* commands; `feedback_port` is where it *sends* status
messages, and the two are unrelated. **There is no default feedback port**:
both `feedback_ip` and `feedback_port` must be given together in the
`/server/start` body, or feedback stays disabled (command-only, silent).
`feedback_port` should be set to whatever port your receiver (e.g.
TouchDesigner's OSC In DAT/CHOP) is actually listening on — it does not need
to match this server's own `listen_port`.

```bash
curl -X POST http://<HOST>:5000/server/start \
     -H "Content-Type: application/json" \
     -d '{"type": "osc", "listen_port": 5005, "feedback_ip": "10.12.1.164", "feedback_port": 5008}'
```

Since this is plain UDP, sending starts as soon as feedback is enabled
regardless of whether anything is actually listening at `feedback_ip:feedback_port`
— an unreachable or wrong destination fails silently (no error surfaced back
to the command sender), so double-check the address/port against your
receiving application before relying on it.

| Address | Arguments | Action | Feedback sent |
|---|---|---|---|
| `/servo` | `data` (float): `1.0` = on, `0.0` = off | `servo_on()` / `servo_off()` | `/servo_on "on"` / `/servo_off "off"` |
| `/clear` | — | `clear_alarm_12()` | `/clear "cleared"` |
| `/set_point` | `angle` (deg, absolute), `acc_time` (ms), `rpm` | `post_step_motion_by()` — a real move | `/set_point angle acc_time rpm` |
| `/back_home` | — | `initial_abs_home()` — a real move back to the saved home position | `/back_home "back_home"` |
| `/set_home` | — | `set_home_position()` — **persists** the current position as the new home reference | `/set_home_position "set_home_position"` |
| `/reset_initial_abs_position` | — | `write_PA29_Initial_Abs_Pos()` | `/reset_initial_abs_position "reset"` |
| `/set_continous_motion` | `speed_rpm`, `acc_time` (ms), `enable` (bool) | Arms/disarms continuous JOG mode (does **not** move by itself) | `/continuous_mode_start speed_rpm acc_time` |
| `/ctrl_continuous_motion` | `action` (`"start"`/`"stop"`), `CW_CCW` (`"CW"`/`"CCW"`) | Starts/stops continuous rotation (requires `/set_continous_motion` with `enable=true` first) | `/continuous_mode_start CW_CCW` / `/continuous_mode_stop "stop"` |
| `/jog_speed_adjust` | `delta_rpm` (int, e.g. `1`/`-1`) | `change_jog_speed_by()` — **nudges** the running JOG speed by this amount (requires JOG mode already armed via `/set_continous_motion`); raises/logs an error instead of moving if it isn't | `/jog_speed_adjust <new speed_rpm>` |
| `/cancel_loop` | — | Stops continuous reading **and** explicitly exits whatever test mode is active | `/cancel_loop <current_angle>` |

Two more feedback-only messages fire automatically while a move is in
progress, regardless of which address triggered it:
- `/moving <angle>` — sent on every encoder poll while continuous
  reading is active.
- `/motion_complete "complete"` — sent when the software auto-detects a
  discrete move has finished (see "Auto-stop" under Gotchas).

`(spelling: /set_continous_motion is missing an "u" — that's the actual
address name in the code, not a typo in this doc.)`

### Typical OSC sequences

**Move to an absolute angle:**
```
/set_point 90.0 3000 20
```

**Continuous rotation, then stop:**
```
/set_continous_motion 50 5000 true   # arm JOG mode at 50rpm
/ctrl_continuous_motion "start" "CW" # start spinning
/jog_speed_adjust 1                  # nudge to 51rpm while it's still running
/jog_speed_adjust -1                 # nudge back to 50rpm
...
/ctrl_continuous_motion "stop" "CW"  # pause (CW_CCW arg is ignored when stopping)
/cancel_loop                          # fully exit JOG mode when done
```

The web UI's Up/Down arrow keys send `/jog_speed_adjust` under the hood
(via the `/action` `jogSpeedAdjust` HTTP action) while the Speed Control
(JOG) section is armed; Left/Right arrow keys map to
`/ctrl_continuous_motion "start" "CW"/"CCW"`, and releasing either sends
`"stop"`. Art-Net's Channel 1 already supports changing the speed while
running (a continuous fader value, not a nudge) — see artnet_server.py's
Channel 1 docstring; no separate Art-Net channel was added for
`/jog_speed_adjust`'s equivalent.

**⚠️ Reversing direction requires a pause in between** — sending
`"start" "CCW"` directly after `"start" "CW"` (no `"stop"` in between)
is refused by `ServoController` (a fail-safe against shocking the
mechanism with an abrupt reversal). You'll see it logged server-side;
OSC gets no feedback message about the refusal currently.

---

## Art-Net

Default listen port: **6454** (the Art-Net standard, UDP). Channels are
1-indexed DMX slots within the configured universe (default **`1`**).
Handled by `ArtNetInputServer` in `artnet_server.py`. **This channel
layout is a project-specific convention, not an Art-Net/DMX standard** —
adjust the constructor args (`max_speed_rpm`, `acc_time`) to fit your
actual console, or repurpose the
channel numbers if they conflict with something else in your universe.

### Status feedback (optional, off by default)

`ArtNetInputServer` can send its own outbound ArtDMX packet reporting
angle/servo/motion state, separate from the command-input universe above.
Enable it with `feedback_ip` (+ optionally `feedback_universe`) in the
`/server/start` body:

```bash
curl -X POST http://<HOST>:5000/server/start \
     -H "Content-Type: application/json" \
     -d '{"type": "artnet", "universe": 1, "feedback_ip": "10.12.1.164", "feedback_universe": 2}'
```

- **Off by default.** `feedback_universe` defaults to `universe + 1` if
  `feedback_ip` is given without it. It must differ from `universe` — the
  server refuses to start otherwise: a node both reading and writing the
  same universe risks reacting to its own broadcast as if it were a new
  command, and confuses any console reading that universe back.
- Sent to UDP port 6454 by default (the Art-Net standard, same as the
  command-input side, just a different universe) — override with
  `feedback_port` in the `/server/start` body if your receiver listens on a
  non-standard port. From its own dedicated outbound socket, separate from
  the command-input listening socket.
- Channel layout re-uses the existing input encoding so nothing new has to
  be learned:
  - Channels 1-2: current angle — identical 16-bit, 0.01°/step, `32768` = 0°
    encoding as input channels 5-6/13-14.
  - Channel 3: servo on/off (`0`/`255`), tracked from channel 8 — no extra
    serial read.
  - Channel 4: alarm active (`0`/`255`) — `is_alarm_active()` on a fresh
    `read_current_alarm_code()` call. Unlike channels 3/5, this **is** a new
    serial round trip, and it happens on every send — including every
    on-`/moving` encoder poll while continuous reading is active, so it
    roughly doubles Modbus traffic on the shared UART for the duration of a
    move. Watch for timing regressions on the Pi's mini-UART (CLAUDE.md §2)
    if that matters for your setup; a read failure is reported as alarm
    active (`255`), never silently as "no alarm".
  - Channel 5: moving/idle (`0` = idle, `255` = continuous reading active) —
    the same signal as OSC's `/moving` vs `/motion_complete`.
- Sent once per encoder poll while continuous reading is active (matches
  OSC's `/moving` cadence), plus immediately on a servo on/off change
  (channel 8), a clear-alarm trigger (channel 9), or a channel 10-12 rising
  edge (back home / set home / reset initial absolute position) that was
  actually acted on (i.e. not while those channels are ignored per "Enable
  channels 10-12" below). This matters most for channel 11 (set home):
  `set_home_position()` resets the current angle to 0 synchronously with no
  motion, so without this immediate send a feedback consumer would keep
  showing the pre-reset angle until the next real move.
- Same fire-and-forget UDP semantics as OSC feedback: an unreachable
  destination fails silently, no error surfaced to the command sender. The
  whole send (including the alarm read above) is one try/except — a failure
  anywhere in it is logged and swallowed, never raised into the command path
  or the background reading thread.
- This makes the device an Art-Net **sender** for the feedback universe
  specifically; "pure receiver" (below) continues to describe only the
  command-input universe, not this one.

| Channel | Meaning | Values |
|---|---|---|
| 1 | Continuous motion enable/speed | `0` = disable; `1-255` linearly scales to `1..max_speed_rpm`. Applied at start **and** when it changes while running |
| 2 | Direction | `0` = stop; `1-127` = CCW; `128-255` = CW. May be set before or after channel 1 |
| 3 | Cancel | `0` = normal; a `0→nonzero` edge triggers the same cancel as OSC's `/cancel_loop` |
| 4 | Absolute-move trigger | `0` = idle; a `0→nonzero` edge triggers one absolute-angle move using channels 5-7's *current* values. Never looks at channels 13-17 |
| 5-6 | Target angle (high byte, low byte) | 16-bit, one step = 0.01°, **`32768` = 0°**: angle = (value − 32768) / 100, so `32768+n` = `+n/100°`, `32768−n` = `−n/100°` (about ±327°) |
| 7 | Position move speed | `1-255` linearly scales to `1..max_speed_rpm` |
| 8 | Servo on/off | `0` = off; `1-255` = on (level-based, not edge-triggered) |
| 9 | Clear alarm 12 | `0` = normal; a `0→nonzero` edge clears Alarm 12 |
| 10 | Back home | `0` = idle; a `0→nonzero` edge triggers a real move back to the saved home position |
| 11 | Set home | `0` = idle; a `0→nonzero` edge **persists** the current position as the new home reference |
| 12 | Reset initial absolute position | `0` = idle; a `0→nonzero` edge writes PA29 |
| 13-14 | Move **by** angle (high byte, low byte) | Same encoding as channels 5-6: 16-bit, one step = 0.01°, `32768` = no move |
| 15-16 | Move time (high byte, low byte) | 16-bit, one step = 0.01 s (0.01–655.35 s). `0` = no time given: the move is refused |
| 17 | Move-by trigger | `0` = idle; a `0→nonzero` edge moves by channels 13-14 in the time of channels 15-16. Never looks at channels 4-7 |

**Two independent moves.** Channel 4 moves to an *absolute* angle (channels
5-6) at an rpm you give (channel 7). Channel 17 moves *by* an angle
(channels 13-14) *in* a time (channels 15-16). Each has its own trigger and
reads only its own channels, so filling in one set never changes the other.
If both triggers rise in the very same frame, only the absolute move runs.

**Move by an angle in a time (channels 13-17).** The channel-17 trigger
moves to *(current angle + channels 13-14)*. The rpm is calculated for you from the
angle and the time, including the motor profile's gear ratio (90° in 3 s
on the 30:1 motor = 150 rpm). It is rounded to a whole rpm, at least 1 and
at most the server's `max_speed_rpm` (default 100): if the time you ask for
would need more than that, the move is done at `max_speed_rpm` and takes
longer. The acceleration ramp (`acc_time`) is in addition to the time you
ask for, so short times end up longer than requested. A zero move
(channels 13-14 = 32768) does nothing. Channels 13-17 are only read from
frames that are at least 17 channels long, so a sender that stops at
channel 12 (or 7) is unaffected, and a sender that transmits a full universe
can leave channels 13-16 at any value: nothing happens until channel 17 rises.

**Continuous motion (channels 1-3), what to expect.**
- Channel 1 sets the speed when motion starts and follows changes while it
  runs (a fader works). Changes are written at most every 100 ms and the
  latest value wins. *Not yet checked on a real drive: whether the drive
  applies a new JOG speed while it is turning.*
- The direction is always sent when motion starts, so it does not matter
  whether channel 2 or channel 1 is set first, or whether channel 2 changed
  since the last run.
- The drive refuses a direct CW ↔ CCW switch (safety). Set channel 2 to `0`,
  then the new direction. The refusal is shown in the Channel Monitor and the
  log instead of being ignored silently.
- Channel 3 (cancel) leaves JOG mode. If channel 1 was up, channels 1-2 are
  then ignored until channel 1 is set to `0`; that starts a fresh run.

Channels 10-12 (back home, set home, reset absolute position) are
**ignored unless "Enable channels 10-12" is ticked when starting the
server** (`"enable_dangerous_channels": true` in the HTTP API): they make
a real move or overwrite the saved home, and a lighting console sending a
full universe can hit them by accident. The Channel Monitor marks them
`IGNORED` while disabled.

Channels 4-12 are optional and independent: a sender filling only
channels 1-3 (continuous motion only) still works, channels 4-12 are
simply never triggered. All edge-triggered channels compare against the
*previously received frame*, not a running total — this matters because
real Art-Net sources typically resend the full frame 30-44 times/second
even when nothing changed; without edge-detection, every field would
re-fire on every single frame.

### Network setup and safety

The Art-Net server is a **pure receiver**: it never sends anything (no
ArtPollReply, no echo), so it cannot disturb other devices by itself. What
can go wrong is other traffic reaching it, or it losing the sender:

* **One Art-Net Out per destination, in Unicast.** In TouchDesigner use a
  separate Art-Net Out for the motor (unicast to this Pi's IP) and one for
  any other DMX gear (unicast to that box). Avoid broadcast: every device
  and Wi-Fi client then receives every universe.
* **Different universes.** This server defaults to universe **1**; other
  DMX equipment on the network usually listens on 0. If both used the same
  universe, that equipment would drive its outputs from the motor channels
  (and this server would react to its data). Packets for other universes
  are dropped and counted.
* **Bind to the Pi's IP** ("Bind to IP", `listen_ip`) so only unicast to that
  address is received. Linux does not deliver broadcast to a socket bound to
  a specific unicast address, so the sender must be set to unicast. `0.0.0.0`
  accepts broadcast too.
* **Only accept from** (`allowed_sources`): a comma-separated list of sender
  IPs; anything else is dropped and counted ("unlisted-sender").
* **Loss-of-signal stop** (`signal_timeout_s`, default 2 s, `0` = off). If no
  frame for this universe arrives for that long while continuous rotation is
  active, rotation is stopped. It does **not** restart by itself when the
  signal returns: set channel 1 to `0` (this re-arms), then start again. Pick
  a value above your Wi-Fi's worst latency spikes. Note that the check assumes
  the sender keeps sending frames while values are static; if your sender only
  transmits on change, use `0` or the watchdog will stop a steady rotation.
  The Channel Monitor shows the real frame rate and the age of the last frame
  so you can check.
* **Frame rate.** Edge detection works at any rate. Lowering the sender to
  ~30 fps (or less) reduces load on a Wi-Fi link; keep the timeout well above
  a few frame intervals.
* Out-of-order packets (Art-Net sequence byte, `0` = unused) that arrive
  behind a newer one are dropped, so a stale frame cannot overwrite a newer
  one.
* On a Raspberry Pi over Wi-Fi, disable Wi-Fi power saving
  (`sudo iw dev wlan0 set power_save off`) to avoid latency spikes.

The web UI itself can move the motor and has no login: set
`SERVO_WEB_PASSWORD` (see README, "Web UI access") before exposing it to a
network.

### Constructing a test packet (Python)

```python
import socket, struct

ARTNET_ID = b"Art-Net\x00"

def build_artdmx_packet(universe, dmx_data):
    sub_uni, net = universe & 0xFF, (universe >> 8) & 0xFF
    header = (ARTNET_ID + struct.pack('<H', 0x5000) + bytes([0, 14]) +
              bytes([0, 0, sub_uni, net]) + struct.pack('>H', len(dmx_data)))
    return header + dmx_data

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Channel 1=100 (speed), channel 2=200 (CW) -> start continuous CW rotation
sock.sendto(build_artdmx_packet(0, bytes([100, 200, 0])), ("<HOST>", 6454))
```

### Encoding a position-mode target (channels 5-6)

```python
target_deg = 90.0            # -327.68 .. +327.67
angle_raw = 32768 + round(target_deg * 100)      # 0.01 deg per step, 32768 = 0 deg
high, low = (angle_raw >> 8) & 0xFF, angle_raw & 0xFF
speed_rpm = 50
speed_channel = round(speed_rpm / 100 * 255)  # 100 = this server's max_speed_rpm
# frame: [enable, direction, cancel, trigger, high, low, speed]
sock.sendto(build_artdmx_packet(0, bytes([0, 0, 0, 255, high, low, speed_channel])), ("<HOST>", 6454))
```

### Encoding "move by an angle in a time" (channels 13-17)

```python
move_deg, seconds = -45.0, 2.5           # negative = the other direction
angle_raw = 32768 + round(move_deg * 100)        # 32768 = no move, 0.01 deg per step
time_raw = round(seconds * 100)                  # 0.01 s per step, must be > 0
frame = bytes([0, 0, 0, 0, 0, 0, 0,             # channels 1-7 (channel 4 = 0: no absolute move)
               0, 0, 0, 0, 0,                    # channels 8-12
               angle_raw >> 8, angle_raw & 0xFF,  # channels 13-14
               time_raw >> 8, time_raw & 0xFF,    # channels 15-16
               255])                              # channel 17: the move-by trigger
sock.sendto(build_artdmx_packet(0, frame), ("<HOST>", 6454))
```

---

## Gotchas (read before wiring up a real console)

**1. `0x0904`'s direction values are reversed from what you'd guess.**
Per the SDE manual, `1` = forward rotation (labeled CCW), `2` = reverse
rotation (labeled CW). Both OSC's `"CW"`/`"CCW"` strings and Art-Net's
channel 2 range are already mapped to match the manual — you don't need
to compensate for this yourself, but if you're ever reading raw values
off the wire (e.g. in a diagnostic script) and they look backwards, this
is why.

**2. `/servo 0.0` (and Art-Net channel 8 → 0) re-triggers Alarm 12 as a
documented side effect.** `servo_off()` clears a DI-simulation bit that
also happens to suppress Alarm 12; turning it off deliberately
re-triggers the alarm. This is existing, intentional driver behavior
(matches the web UI's SERVO OFF button) — not a bug — but a remote
controller gets **no feedback message** telling it this happened. Expect
to see `alarm_active: true` in `/status` right after a servo-off command,
and send `/clear` (or Art-Net channel 9) if you need it clear again.

**3. Position-mode targets are absolute, but `current_angle` is
cumulative/unbounded — it does NOT wrap to 0-360.** After enough moves in
one direction, `current_angle` can drift to well outside `[0, 360]` (it's
whatever accumulated since the last `/set_home`), and a target such as
`0` then means "go all the way back to the saved home", however many degrees
that is. Art-Net's channels 5-6 can only *name* targets in about
`[-327.68, +327.67]`; if `current_angle` is outside that, use OSC's
`/set_point` (any float) or channels 13-17 (move by an angle) instead.

**4. There is no limit on how far one move may go.** An earlier version
refused any move of 180° or more; that came from a previous application's
requirement and was removed on 2026-09-21. Check what a target means before
sending it: a wrong number or unit (say `3000` instead of `30`) is now carried
out as sent, up to the one thing the drive itself cannot represent — a move
of more than 2³¹−1 command pulses (about 6144° of output shaft at 30:1),
which is refused with an error and nothing is sent. So are angles that are
not numbers (NaN, infinity). Keep the speed low the first time you use a new
sender.

**5. Reversing continuous-motion direction requires an explicit stop in
between.** Sending CW immediately followed by CCW (no stop/pause) is
refused by `ServoController.speed_ctrl_action()` — a fail-safe against
shocking the mechanism with an abrupt reversal. Applies identically to
both OSC and Art-Net (they share the same underlying method). The web
UI surfaces this as an error message; OSC/Art-Net currently do not.

**6. `/set_home` and Art-Net channel 11 change what "home" means, and it
persists to disk** (`servo_config_<profile>.json`). This isn't a
transient in-memory setting — it survives a restart. Don't trigger it
casually while testing; it overwrites the reference every other
position-mode command is measured against.

**7. Auto-stop after a move can take a moment, and doesn't always fully
exit test mode by itself.** Once a discrete move (position mode)
settles, the software auto-detects completion via encoder stillness and
stops the background polling — but it doesn't explicitly write the
drive out of test mode. The drive's own ~1s communication-timeout
usually does that shortly after. If you need an immediate, deterministic
exit (e.g. before switching motor profiles), send `/cancel_loop` (OSC)
or a channel-3 rising edge (Art-Net) rather than waiting.

**8. `enable` arguments should be sent as native OSC booleans, not
strings.** `ServoController.enable_speed_ctrl()` coerces common string
forms (`"true"`/`"false"`/`"1"`/`"0"`/`"on"`/`"off"`/`"yes"`/`"no"`,
case-insensitive) as a safety net, but native OSC `True`/`False` (or
plain ints `1`/`0`) is the reliable choice if your OSC library gives you
a choice.

---

## Safety

Every address/channel that can move the motor (`/set_point`,
`/back_home`, `/ctrl_continuous_motion "start"`, and the Art-Net
equivalents) sends a **real command to physical hardware** the moment
it's triggered. There is no confirmation step at the protocol level —
that's the nature of OSC/Art-Net as fire-and-forget UDP. Whoever
operates the console/controller sending these messages is responsible
for the same physical safety checks as anyone using the web UI directly
(motor and load clear of people/obstacles before sending anything that
can move it).
