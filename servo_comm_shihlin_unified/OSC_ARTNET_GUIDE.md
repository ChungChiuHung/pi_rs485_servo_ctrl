# OSC / Art-Net User Guide

OSC and Art-Net both drive the same `ServoController` the web UI uses.
**Only one runs at a time** — starting one while the other is active is
rejected. Read **Gotchas** before wiring up a real console.

## Starting/stopping

Web UI: "Continuous Motion Input" section (protocol dropdown + Start/Stop).

```bash
curl -X POST http://<HOST>:5000/server/start -H "Content-Type: application/json" \
     -d '{"type": "osc", "listen_port": 5005}'

curl -X POST http://<HOST>:5000/server/start -H "Content-Type: application/json" \
     -d '{"type": "artnet", "listen_port": 6454, "universe": 0, "max_speed_rpm": 100, "acc_time": 5000, "position_mode_max_angle": 360}'

curl -X POST http://<HOST>:5000/server/stop
curl http://<HOST>:5000/server/status
```

OSC feedback (`feedback_ip`/`feedback_port`) is API-only, not exposed in the web UI.

---

## OSC (default port 5005, UDP)

| Address | Arguments | Action |
|---|---|---|
| `/servo` | `data`: `1.0`=on, `0.0`=off | Servo on/off |
| `/clear` | — | Clear Alarm 12 |
| `/set_point` | `angle` (deg, absolute), `acc_time` (ms), `rpm` | Move — **real motion** |
| `/back_home` | — | Move to saved home — **real motion** |
| `/set_home` | — | **Persist** current position as new home |
| `/reset_initial_abs_position` | — | Write PA29 |
| `/set_continous_motion` *(sic)* | `speed_rpm`, `acc_time`, `enable` | Arm/disarm JOG mode (no motion by itself) |
| `/ctrl_continuous_motion` | `action` (`start`/`stop`), `CW_CCW` | Start/stop continuous rotation — **real motion** |
| `/cancel_loop` | — | Stop + fully exit test mode |

Feedback mirrors each address (`/servo_on`, `/clear`, etc.) if
`feedback_ip`/`feedback_port` are set. `/moving <angle>` and
`/motion_complete` fire automatically during/after a move.

```
/set_point 90.0 3000 20                # move to 90°
/set_continous_motion 50 5000 true     # arm JOG at 50rpm
/ctrl_continuous_motion "start" "CW"   # spin
/ctrl_continuous_motion "stop" "CW"    # pause
/cancel_loop                           # exit JOG mode
```

---

## Art-Net (default port 6454, UDP)

Provisional, project-specific channel layout — not a DMX standard.

| Ch | Meaning | Values |
|---|---|---|
| 1 | Enable/speed | `0`=off; `1-255` → `1..max_speed_rpm` |
| 2 | Direction | `0`=stop; `1-127`=CCW; `128-255`=CW |
| 3 | Cancel | `0→nonzero` edge = cancel |
| 4 | Position-mode trigger | `0→nonzero` edge = move to angle in ch 5-6 at speed in ch 7 |
| 5-6 | Target angle (hi/lo byte) | `0-65535` → `0..position_mode_max_angle`° |
| 7 | Position move speed | `1-255` → `1..max_speed_rpm` |
| 8 | Servo on/off | `0`=off, `1-255`=on (level, not edge) |
| 9 | Clear Alarm 12 | `0→nonzero` edge |
| 10 | Back home | `0→nonzero` edge — **real motion** |
| 11 | Set home | `0→nonzero` edge — **persists** |
| 12 | Reset initial abs position | `0→nonzero` edge |

Channels 4-12 are optional — a sender using only 1-3 still works. All
edge-triggers compare to the *previous frame* (a real console resends
unchanged frames 30-44×/sec).

**Encoding a position-mode target:**
```python
angle_raw = round(target_deg / position_mode_max_angle * 65535)
high, low = (angle_raw >> 8) & 0xFF, angle_raw & 0xFF
speed_channel = round(speed_rpm / max_speed_rpm * 255)
# frame = [enable, direction, cancel, 255, high, low, speed_channel]
```

---

## Gotchas

- **Direction values look backwards on the wire**: per the manual, `1`=CCW, `2`=CW. Already handled — OSC's `"CW"/"CCW"` and Art-Net's channel-2 ranges map correctly.
- **`/servo 0.0` (Art-Net ch 8→0) re-triggers Alarm 12** — a real, documented driver side effect, not a bug. `/clear` (or ch 9) to clear it again. No feedback message warns you this happened.
- **Position targets are absolute; `current_angle` is cumulative and unbounded** (doesn't wrap to 0-360). If it's drifted far from `[0, 360]`, no Art-Net channel-5-6 value may be reachable within the 180° guard — check `/status`'s `current_angle` if moves stop having any effect.
- **Moves ≥180° are silently refused** — no error, no feedback, `/status` just doesn't change.
- **Reversing direction needs an explicit stop first** — CW→CCW with no stop in between is refused (shock-prevention fail-safe). Same for OSC and Art-Net.
- **`/set_home` / Art-Net ch 11 persist to disk** and redefine what every future move is measured against — don't trigger casually.
- **Auto-stop after a move doesn't always exit test mode immediately** — it stops polling; the drive's own ~1s timeout finishes the exit. Use `/cancel_loop` / channel 3 for an immediate, deterministic exit.
- **Send `enable` as a real boolean**, not a string, when you have the choice (strings are coerced but it's a safety net, not the primary path).

---

## Safety

Every address/channel marked **real motion** above sends a live command
the instant it's triggered — there's no confirmation step in OSC/Art-Net
(fire-and-forget UDP). Whoever operates the sending console is
responsible for the same physical safety checks as anyone using the web
UI directly.
