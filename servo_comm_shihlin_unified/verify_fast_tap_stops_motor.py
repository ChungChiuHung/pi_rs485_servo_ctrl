"""
Full end-to-end real-hardware verification, 2026-09-22: does a fast arrow-
key tap (motionStart_CW fired, then motionPause fired almost immediately
after, before the first request has finished) actually stop the motor now,
or does the STOP request still get dropped?

Imports the REAL app.py (connects to the real drive at module import time,
same as `python app.py` would) and uses Flask's test client from two
separate Python threads to fire /action requests concurrently -- this goes
through the real hardware_serialized decorator, the real
_hardware_busy_lock, and the real ServoController calls against real
hardware, genuinely racing on the same shared lock exactly as two
overlapping HTTP requests from a live server would (Flask's test client
runs the WSGI app synchronously in the calling thread, so two threads
calling it at once are genuinely concurrent -- no live server/socket or
extra HTTP client library needed).

HARDWARE-AFFECTING: arms JOG mode and fires a real fast CW tap at 10rpm.
Per CLAUDE.md §4, only run this with the user present and after explicit
confirmation (given 2026-09-22).

    cd servo_comm_shihlin_unified/
    python verify_fast_tap_stops_motor.py
"""
import sys
import threading
import time

import app as app_module

SPEED_RPM = 10
MEASURE_S = 3.0


def main():
    client = app_module.app.test_client()
    ctrl = app_module.servo_ctrller
    if ctrl is None:
        print(f"app.py could not connect: {app_module._connection_error}")
        return 2

    code = ctrl.read_current_alarm_code()
    print(f"Alarm before: {hex(code) if code is not None else None}")
    if code not in (0xFF, 0):
        print("Clearing alarm 12 first...")
        ctrl.write_PD_16_Enable_DI_Control()
        ctrl.clear_alarm_12()

    print(f"Arming via /action enableSpeedCtrlMode ({SPEED_RPM} rpm)...")
    r = client.post("/action", json={"action": "enableSpeedCtrlMode", "speed_rpm": SPEED_RPM})
    print(f"  -> {r.status_code} {r.get_json()}")

    results = {}

    def fire(action, key):
        results[key] = client.post("/action", json={"action": action})

    print("\nFiring motionStart_CW and motionPause from two threads, "
          "as close together as possible (simulates a fast key tap)...")
    t1 = threading.Thread(target=fire, args=("motionStart_CW", "start"))
    t2 = threading.Thread(target=fire, args=("motionPause", "stop"))
    t_fire = time.monotonic()
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    print(f"Both requests completed in {time.monotonic() - t_fire:.3f}s")

    print(f"  start -> {results['start'].status_code} {results['start'].get_json()}")
    print(f"  stop  -> {results['stop'].status_code} {results['stop'].get_json()}")

    print(f"\nMeasuring encoder over {MEASURE_S}s...")
    p1 = ctrl.read_motor_feedback_pulses()
    time.sleep(MEASURE_S)
    p2 = ctrl.read_motor_feedback_pulses()
    delta = p2 - p1
    print(f"Encoder delta: {delta} pulses over {MEASURE_S}s")
    # A fast tap SHOULD move the motor a tiny amount before stopping (the
    # brief window between the two requests landing) -- that is correct,
    # not a bug. The threshold distinguishes that from genuine SUSTAINED
    # rotation: at SPEED_RPM for the full MEASURE_S window, the motor would
    # cover several hundred thousand pulses at minimum (10rpm * 3s already
    # implies ~2,000,000+ pulses); a few thousand from the tap's own blip is
    # two-plus orders of magnitude below that.
    sustained_rotation_pulses = (SPEED_RPM / 60) * MEASURE_S * ctrl.profile["encoder_pulses_per_rev"]
    spinning = abs(delta) > sustained_rotation_pulses * 0.1
    print(f"{'FAIL -- motor IS still spinning' if spinning else 'PASS -- motor stopped (a small blip from the tap itself is expected and fine)'}")

    print("\nCleaning up (motionCancel, disable)...")
    client.post("/action", json={"action": "motionCancel"})
    result = 1 if spinning else 0
    return result


if __name__ == "__main__":
    sys.exit(main())
