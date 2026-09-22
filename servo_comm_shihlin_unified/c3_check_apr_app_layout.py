"""
Design doc §7.C "C3": PA32(APR) vs PA33(APP) layout disagreement between the
Chinese V1.07 manual (PA32=pulses-within-revolution 0~4194303, PA33=signed
revolution count) and the English manual (opposite). Current code assumes
the Chinese layout (`abs_rev_register` defaults to "APP" = revolutions).

Diagnostic: read both raw registers, command a known move (default: +90deg
relative, 20rpm -- same parameters as the C5 test), read both raw registers
again, and compare each register's raw delta against:
  - the expected WHOLE-REVOLUTION count (total_pulses // pulses_per_rev)
  - the expected WITHIN-REVOLUTION remainder (total_pulses % pulses_per_rev)
Whichever register's delta matches the whole-revolution count is the
"revolutions" register; the other is "pulses-within-revolution".

HARDWARE-AFFECTING: moves the motor. Per CLAUDE.md §4, only run this with
the user present and after explicit confirmation (given 2026-09-22 for
90deg/20rpm).

    cd servo_comm_shihlin_unified/
    python c3_check_apr_app_layout.py --port COM4
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


def read_raw_pair(ctrl):
    ok = ctrl._update_absolute_registers()
    apr = ctrl.read_PA32_Abs_Revolutions()
    app = ctrl.read_PA33_Encoder_ABS_Pos()
    return ok, apr, app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial port (e.g. COM4). Default: auto-detect.")
    parser.add_argument("--profile", default=None, help="Profile name in motor_profiles.json. Default: active_profile.")
    args = parser.parse_args()

    profiles_data = load_profiles()
    profile_name = args.profile or profiles_data["active_profile"]
    profile = resolve_profile(profiles_data, profile_name)
    print(f"Profile: {profile_name} (baud={profile['baud_rate']}, device_number={profile['modbus_device_number']})")
    pulses_per_rev = profile["encoder_pulses_per_rev"]

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

        ok_before, apr_before, app_before = read_raw_pair(ctrl)
        print(f"BEFORE: PA30 handshake ok={ok_before}, PA32(APR)={apr_before}, PA33(APP)={app_before}")

        print(f"Commanding relative move of +{MOVE_DEG} deg at {SPEED_RPM} rpm (acc/dec {ACC_DEC_MS} ms)...")
        ctrl.post_step_motion_by(MOVE_DEG, acc_dec_time=ACC_DEC_MS, speed_rpm=SPEED_RPM, relative=True)

        start = time.monotonic()
        while ctrl.reading_active and (time.monotonic() - start) < TIMEOUT_S:
            time.sleep(POLL_INTERVAL_S)
        if ctrl.reading_active:
            print("TIMEOUT waiting for motion to complete -- stopping continuous reading anyway.")
            ctrl.stop_continuous_reading()

        print(f"Landed at current_angle={ctrl.current_angle} deg")

        ok_after, apr_after, app_after = read_raw_pair(ctrl)
        print(f"AFTER:  PA30 handshake ok={ok_after}, PA32(APR)={apr_after}, PA33(APP)={app_after}")

        if None in (apr_before, apr_after, app_before, app_after):
            print("Could not read one of the raw registers cleanly -- cannot compute deltas.")
            return 1

        apr_delta = apr_after - apr_before
        app_delta = app_after - app_before
        total_pulses = ctrl.base_pulse_per_degree * MOVE_DEG
        expected_revs = int(total_pulses // pulses_per_rev)
        expected_remainder = total_pulses % pulses_per_rev

        print(f"\nPA32(APR) delta: {apr_delta}")
        print(f"PA33(APP) delta: {app_delta}")
        print(f"Expected whole-revolution count for this move: {expected_revs}")
        print(f"Expected within-revolution remainder (pulses): {expected_remainder:.0f}")

        def classify(name, delta):
            if abs(delta - expected_revs) <= 1:
                return f"{name} looks like REVOLUTIONS (delta {delta} ~= expected {expected_revs})"
            if abs(delta) <= pulses_per_rev and abs(abs(delta) - expected_remainder) < pulses_per_rev * 0.05:
                return f"{name} looks like PULSES-WITHIN-REVOLUTION (delta {delta} ~= expected remainder {expected_remainder:.0f}, mod wraparound)"
            return f"{name} delta {delta} does not clearly match either pattern -- inspect manually"

        print("\n" + classify("PA32(APR)", apr_delta))
        print(classify("PA33(APP)", app_delta))
        print(f"\nCurrent code assumption: abs_rev_register={ctrl.abs_rev_register!r} "
              f"(i.e. {'APP' if ctrl.abs_rev_register == 'APP' else 'APR'} = revolutions)")
        return 0
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
