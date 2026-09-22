"""
Design doc §7.C "C4": does the absolute position (PA32/PA33-derived) advance
in lockstep with the raw incremental counter (0x0000, via EncoderPulseTracker)?
_read_continuously()'s background loop only re-reads the cheap raw counter
every poll and reconstructs the absolute-scale position via a fixed offset
(_absolute_offset = absolute_pulses - tracker_value) captured once at the
last full read -- this assumes both counters tick at the same rate. If they
drift apart, that offset (and therefore every angle the UI shows between
full reads) becomes unreliable.

Diagnostic: capture _absolute_offset at rest, command a >=1 revolution move
(90deg/20rpm, same parameters used for C3/C5), capture _absolute_offset
again after landing, and report the drift. Design doc's pass bar: drift of
only a few pulses.

HARDWARE-AFFECTING: moves the motor. Per CLAUDE.md §4, only run this with
the user present and after explicit confirmation (given 2026-09-22 for
90deg/20rpm, same as the C3/C5 runs).

    cd servo_comm_shihlin_unified/
    python c4_check_offset_drift.py --port COM4
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from servo_control import ServoController
from motor_profile import load_profiles, resolve_profile

MOVE_DEG = 90.0
SPEED_RPM = 20
ACC_DEC_MS = 5000
POLL_INTERVAL_S = 0.5
TIMEOUT_S = 90


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
        absolute_mode = ctrl.refresh_encoder_mode()
        print(f"refresh_encoder_mode(): absolute_mode={absolute_mode}")
        if absolute_mode is not True:
            print("PA28 did not read back as 1 -- aborting.")
            return 1

        code = ctrl.read_current_alarm_code()
        print(f"Current alarm: {hex(code) if code is not None else None} -- "
              f"if this is not 0xff, clear it first (see clear_al12_and_check_abs.py) "
              f"or the move below will silently no-op.")

        ok = ctrl._refresh_current_angle_from_hardware()
        offset_before = ctrl._absolute_offset
        print(f"AT REST: refresh ok={ok}, current_angle={ctrl.current_angle}, "
              f"_absolute_offset={offset_before}")
        if offset_before is None:
            print("No offset captured at rest -- aborting.")
            return 1

        print(f"Commanding relative move of +{MOVE_DEG} deg at {SPEED_RPM} rpm (acc/dec {ACC_DEC_MS} ms)...")
        angle_before_move = ctrl.current_angle
        ctrl.post_step_motion_by(MOVE_DEG, acc_dec_time=ACC_DEC_MS, speed_rpm=SPEED_RPM, relative=True)

        start = time.monotonic()
        while ctrl.reading_active and (time.monotonic() - start) < TIMEOUT_S:
            time.sleep(POLL_INTERVAL_S)
        if ctrl.reading_active:
            print("TIMEOUT waiting for motion to complete -- stopping continuous reading anyway.")
            ctrl.stop_continuous_reading()

        moved_deg = ctrl.current_angle - angle_before_move
        print(f"Landed at current_angle={ctrl.current_angle} deg (moved {moved_deg:+.4f} deg -- "
              f"if this is ~0, the move silently failed, likely an uncleared alarm; check above.")

        ok2 = ctrl._refresh_current_angle_from_hardware()
        offset_after = ctrl._absolute_offset
        print(f"AFTER MOVE: refresh ok={ok2}, current_angle={ctrl.current_angle}, "
              f"_absolute_offset={offset_after}")
        if offset_after is None:
            print("No offset captured after move -- aborting.")
            return 1

        drift = offset_after - offset_before
        print(f"\nOffset drift over this move: {drift} pulses "
              f"({'PASS -- lockstep confirmed' if abs(drift) <= 50 else 'INVESTIGATE -- larger than a few pulses'})")
        return 0
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
