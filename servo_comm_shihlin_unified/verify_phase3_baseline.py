"""
Phase 3 real-hardware verification, step 0: read-only baseline check.
Connects to the real drive, reads (never writes) alarm code, encoder mode
(PA28), absolute position, and current angle, and computes how far
back_home (channel 10 / /back_home) would actually move the shaft. No
motion, no writes -- safe to run without further confirmation per
CLAUDE.md §4 (read-only paths are exempt from the hardware-safety gate).

    cd servo_comm_shihlin_unified/
    python verify_phase3_baseline.py --port /dev/ttyUSB0
"""
import argparse
import sys

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyUSB0). Default: auto-detect.")
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
        print(f"Absolute mode (PA28): {absolute_mode}")

        code = ctrl.read_current_alarm_code()
        print(f"Current alarm: {hex(code) if code is not None else None} ({alarm_name(code)})")

        pa31 = ctrl.read_PA31_Abs_Position_Status()
        print(f"PA31 (absolute status): {pa31}")

        ok = ctrl._refresh_current_angle_from_hardware()
        print(f"Fresh position read ok: {ok}")
        print(f"current_encoder (absolute scale): {ctrl.current_encoder}")
        print(f"current_angle: {ctrl.current_angle} deg")
        print(f"abs_home_pos_absolute (saved home, absolute scale): {ctrl.abs_home_pos_absolute}")
        print(f"base_pulse_per_degree: {ctrl.base_pulse_per_degree}")

        if ok:
            home = ctrl._active_home_pos()
            diff_pulses = home - ctrl.current_encoder
            diff_deg = diff_pulses / ctrl.base_pulse_per_degree
            print(f"\nBack-home would move: {diff_deg:+.4f} deg ({diff_pulses:+d} pulses) at 12rpm "
                  f"(initial_abs_home()'s fixed speed).")
        else:
            print("\nCould not compute back-home distance (position read failed).")

        return 0
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
