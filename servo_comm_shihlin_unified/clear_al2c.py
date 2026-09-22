"""
Standalone tool: clear AL.2C (absolute-system coordinate not initialised) by
writing PA29=1 (write_PA29_Initial_Abs_Pos()) -- a pure parameter write, the
motor does not move. Part of the design doc §7.C "C1" absolute-mode switch
sequence, run after PA28=1 has been written and the drive has been
power-cycled twice (AL.2A then AL.2C observed).

HARDWARE-AFFECTING (parameter write). Per CLAUDE.md §4, only run this with
the user present and after explicit confirmation.

    cd servo_comm_shihlin_unified/
    python clear_al2c.py --port COM4
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
        code_before = ctrl.read_current_alarm_code()
        print(f"Current alarm before: {hex(code_before) if code_before is not None else None} "
              f"({alarm_name(code_before)})")

        print("Writing PA29 = 1 (Initial Absolute Position)...")
        ctrl.write_PA29_Initial_Abs_Pos()

        code_after = ctrl.read_current_alarm_code()
        print(f"Current alarm after: {hex(code_after) if code_after is not None else None} "
              f"({alarm_name(code_after)})")

        status = ctrl.read_PA31_Abs_Position_Status()
        print(f"PA31 (APST) raw: {status}")
        if status is not None:
            faults = PA.decode_APST(status)
            print(f"PA31 faults: {faults if faults else 'none'}")

        pos = ctrl.read_absolute_position_pulses()
        print(f"read_absolute_position_pulses(): {pos}")

        return 0
    finally:
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
