"""
Design doc §7.C "C5" phase 1 (real-hardware, absolute mode now live -- see
memory/lessons.md 2026-09-22 entry): SET HOME in absolute mode (no motion),
then command a real 90deg move at 20rpm, and record the landing angle +
absolute pulses. The drive should then be physically power-cycled (motor
shaft untouched) and c5_phase2_check_after_power_cycle.py run WITHOUT
calling set_home_position() again -- if absolute mode is doing its job, the
angle read there should match what this script records, because
abs_home_pos_absolute is loaded from servo_config_shihlin_400W.json and the
drive's own absolute encoder position survives the power cycle (that's the
entire point of the backup battery).

HARDWARE-AFFECTING: writes a parameter (SET HOME) and moves the motor
(90deg/20rpm). Per CLAUDE.md §4, only run this with the user present and
after explicit confirmation -- both were given for this specific run
(2026-09-22).

    cd servo_comm_shihlin_unified/
    python c5_phase1_set_home_and_move.py --port COM4
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from servo_control import ServoController
from motor_profile import load_profiles, resolve_profile

TARGET_ANGLE_DEG = 90.0
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
            print("PA28 did not read back as 1 -- aborting, this test requires absolute mode live.")
            return 1

        print("Calling set_home_position() (absolute-mode SET HOME, no motion)...")
        ctrl.set_home_position()
        print(f"After SET HOME: current_angle={ctrl.current_angle}, "
              f"abs_home_pos_absolute={ctrl.abs_home_pos_absolute}")

        print(f"Commanding move to {TARGET_ANGLE_DEG} deg at {SPEED_RPM} rpm "
              f"(acc/dec {ACC_DEC_MS} ms)...")
        ctrl.post_step_motion_by(TARGET_ANGLE_DEG, acc_dec_time=ACC_DEC_MS, speed_rpm=SPEED_RPM)

        start = time.monotonic()
        while ctrl.reading_active and (time.monotonic() - start) < TIMEOUT_S:
            time.sleep(POLL_INTERVAL_S)
            print(f"  ... current_angle={ctrl.current_angle}, current_encoder={ctrl.current_encoder}")

        if ctrl.reading_active:
            print("TIMEOUT waiting for motion to complete -- stopping continuous reading and reporting "
                  "whatever was last read.")
            ctrl.stop_continuous_reading()

        final_angle = ctrl.current_angle
        final_encoder = ctrl.current_encoder
        error_deg = final_angle - TARGET_ANGLE_DEG
        print(f"\nFINAL: current_angle={final_angle} deg (target {TARGET_ANGLE_DEG}, "
              f"error {error_deg:+.4f} deg)")
        print(f"FINAL: current_encoder (absolute pulses) = {final_encoder}")
        print(f"abs_home_pos_absolute (saved) = {ctrl.abs_home_pos_absolute}")
        print("\nRecord these numbers, then physically power-cycle the drive (do NOT "
              "touch the motor shaft), and run c5_phase2_check_after_power_cycle.py.")
        return 0
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
