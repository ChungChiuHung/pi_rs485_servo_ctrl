"""
Standalone tool: clear AL.12 (EMG, expected after a power-cycle resets
PD16/PD25 comm-DI-control back to 0) via the same
write_PD_16_Enable_DI_Control() + clear_alarm_12() sequence already
confirmed on this rig earlier in the project, then re-check the absolute
position (PA28=1 switch verification, design doc §7.C "C1"/"C5").

HARDWARE-AFFECTING (parameter writes; does not move the motor). Per
CLAUDE.md §4, only run this with the user present and after explicit
confirmation.

    cd servo_comm_shihlin_unified/
    python clear_al12_and_check_abs.py --port COM4
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
        print(f"Current alarm before: {hex(code_before) if code_before is not None else None} ({alarm_name(code_before)})")

        ctrl.write_PD_16_Enable_DI_Control()
        ctrl.clear_alarm_12()

        code_after = ctrl.read_current_alarm_code()
        print(f"Current alarm after: {hex(code_after) if code_after is not None else None} ({alarm_name(code_after)})")

        pa28 = ctrl.read_PA28_Encoder_Mode()
        print(f"PA28: {pa28}")

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
