"""
Design doc §7.C "C5" phase 2: run AFTER c5_phase1_set_home_and_move.py and
AFTER physically power-cycling the drive (motor shaft untouched). Builds a
brand-new ServoController (simulating a real app restart) and deliberately
does NOT call set_home_position() again -- if absolute mode is doing its
job, _refresh_current_angle_from_hardware() should recompute the same
angle phase 1 ended at, purely from the saved abs_home_pos_absolute
(servo_config_shihlin_400W.json) and a fresh PA32/PA33 read, because the
drive's own absolute position is supposed to survive a power-off (that's
the entire point of the backup battery + PA28=1).

Read-only from this script's point of view (no writes, no motion) --
compares against the number you recorded from phase 1's FINAL current_angle.

    cd servo_comm_shihlin_unified/
    python c5_phase2_check_after_power_cycle.py --port COM4
"""
import argparse
import sys

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from servo_p_register import PA
from motor_profile import load_profiles, resolve_profile


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
        code = ctrl.read_current_alarm_code()
        print(f"Current alarm: {hex(code) if code is not None else None} ({alarm_name(code)})")

        absolute_mode = ctrl.refresh_encoder_mode()
        print(f"refresh_encoder_mode(): absolute_mode={absolute_mode} (should still be True -- PA28 is a "
              f"(*) parameter that IS supposed to persist across a power cycle)")

        print(f"Loaded from config file: abs_home_pos_absolute={ctrl.abs_home_pos_absolute}")

        status = ctrl.read_PA31_Abs_Position_Status()
        print(f"PA31 (APST) raw: {status}")
        if status is not None:
            faults = PA.decode_APST(status)
            print(f"PA31 faults: {faults if faults else 'none'}")

        print("Deliberately NOT calling set_home_position(). Refreshing current_angle from hardware only...")
        ok = ctrl._refresh_current_angle_from_hardware()
        if not ok:
            print("_refresh_current_angle_from_hardware() returned False -- could not compute an angle. "
                  "See warnings above (unreadable position, or no abs_home_pos_absolute saved).")
            return 1

        print(f"\nRESULT: current_angle={ctrl.current_angle} deg, current_encoder (absolute pulses)={ctrl.current_encoder}")
        print("Compare this current_angle to phase 1's FINAL current_angle -- they should match to "
              "within about 0.001-0.01 deg (a few hundred pulses) if position survived the power cycle "
              "without needing SET HOME again.")
        return 0
    finally:
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
