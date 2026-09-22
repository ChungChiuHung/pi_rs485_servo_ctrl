"""
Real-hardware verification, 2026-09-22: user reported "released the arrow
key but the motor kept turning". Root cause found by code review: the web
UI's ENABLE SPEED CONTROL MODE action called enable_speed_ctrl(speed_rpm)
with no acc_time, defaulting to 5000ms (a smooth ramp) -- MOTION PAUSE
(0x0904=0) is sent immediately on keyup, but the drive then takes up to 5s
to actually decelerate to zero, which looks like "didn't stop". Fixed by
passing JOG_ACC_DEC_MS=200 explicitly for this action.

This script arms with acc_time=200 (matching the fix), starts CW, then
stops, and measures how long the encoder actually takes to settle --
should be a small fraction of a second, not several seconds.

HARDWARE-AFFECTING: runs real JOG rotation briefly at 10rpm. Per CLAUDE.md
§4, only run this with the user present and after explicit confirmation
(given 2026-09-22).

    cd servo_comm_shihlin_unified/
    python verify_jog_release_stops_fast.py --port COM4
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile

SPEED_RPM = 10
ACC_MS = 200  # matches app.py's JOG_ACC_DEC_MS
SPIN_S = 1.5
POLL_INTERVAL_S = 0.05
STILL_THRESHOLD = 300      # pulses between polls -- below this counts as "stopped"
MAX_WAIT_S = 3.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial port (e.g. COM4). Default: auto-detect.")
    parser.add_argument("--profile", default=None, help="Profile name in motor_profiles.json. Default: active_profile.")
    args = parser.parse_args()

    profiles_data = load_profiles()
    profile_name = args.profile or profiles_data["active_profile"]
    profile = resolve_profile(profiles_data, profile_name)
    print(f"Profile: {profile_name} (baud={profile['baud_rate']}, device_number={profile['modbus_device_number']})")

    serial_manager = SerialPortManager(port=args.port, baud_rate=profile["baud_rate"])
    serial_manager.connect()
    if not serial_manager.get_serial_instance():
        print("Could not open any serial port. Check wiring/port name and retry.")
        return 2
    print(f"Connected port: {serial_manager.get_connected_port()}")

    ctrl = ServoController(serial_manager, profile)
    try:
        ctrl.refresh_encoder_mode()
        code = ctrl.read_current_alarm_code()
        print(f"Alarm before: {hex(code) if code is not None else None} ({alarm_name(code)})")
        if code not in (0xFF, 0):
            print("Clearing alarm 12 first...")
            ctrl.write_PD_16_Enable_DI_Control()
            ctrl.clear_alarm_12()

        print(f"Arming with acc_time={ACC_MS}ms (the new default for this action), {SPEED_RPM} rpm...")
        ctrl.enable_speed_ctrl(speed_rpm=SPEED_RPM, acc_time=ACC_MS, enable=True)

        print("Commanding CW (simulates ArrowLeft keydown)...")
        ctrl.speed_ctrl_action(2)
        time.sleep(SPIN_S)

        print("Commanding stop (simulates ArrowLeft keyup -> MOTION PAUSE) -- "
              "timing how long until the encoder actually settles...")
        t_stop_sent = time.monotonic()
        ctrl.speed_ctrl_action(0)

        last = ctrl.read_motor_feedback_pulses()
        settled_at = None
        deadline = time.monotonic() + MAX_WAIT_S
        while time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL_S)
            now_val = ctrl.read_motor_feedback_pulses()
            delta = abs(now_val - last)
            last = now_val
            if delta < STILL_THRESHOLD:
                settled_at = time.monotonic()
                break

        if settled_at is None:
            print(f"FAIL -- did not settle within {MAX_WAIT_S}s of the stop command.")
            result = 1
        else:
            elapsed = settled_at - t_stop_sent
            print(f"Time from stop command to settled: {elapsed:.3f}s")
            result = 0 if elapsed < 1.0 else 1
            print(f"{'PASS -- stops fast' if result == 0 else 'FAIL -- still too slow'}")

        ctrl.enable_speed_ctrl(enable=False)
        print("Disabled JOG mode.")
        return result
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
