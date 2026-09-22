"""
Standalone tool: write PA28 (absolute/incremental encoder mode select) on the
real servo driver, using ServoController.write_PA28_Encoder_Mode() -- the
same method the web app would use, so this exercises the real EEPROM-
protection lift/restore dance (design doc §7.B "B5") and the read-back
verification, not a bare register write.

HARDWARE-AFFECTING: writes a drive parameter. Per CLAUDE.md §4 this requires
the user to be present and to have explicitly confirmed before running it.
See docs/servo_comm_shihlin_merge_design.md §7.C for the full switch
sequence and the alarms expected at each step.

Run directly against the drive (e.g. via COM4 on this Windows dev box):

    cd servo_comm_shihlin_unified/
    python write_pa28.py --to 1 --port COM4

Exit codes:
    0 -- write confirmed by read-back (or already at the requested value).
    1 -- write failed, or read-back didn't match.
    2 -- could not connect / could not read PA28 or PA23 at all.
"""
import argparse
import sys

from serial_port_manager import SerialPortManager
from servo_control import ServoController
from motor_profile import load_profiles, resolve_profile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--to", type=int, choices=[0, 1], required=True,
                         help="Target PA28 value: 1 = absolute mode, 0 = incremental mode.")
    parser.add_argument("--port", default=None,
                         help="Serial port (e.g. COM4). Default: auto-detect.")
    parser.add_argument("--profile", default=None,
                         help="Profile name in motor_profiles.json. Default: active_profile.")
    args = parser.parse_args()

    profiles_data = load_profiles()
    profile_name = args.profile or profiles_data["active_profile"]
    profile = resolve_profile(profiles_data, profile_name)

    print(f"Profile: {profile_name} (baud={profile['baud_rate']}, "
          f"device_number={profile['modbus_device_number']})")

    serial_manager = SerialPortManager(port=args.port, baud_rate=profile["baud_rate"])
    serial_manager.connect()
    if not serial_manager.get_serial_instance():
        print("Could not open any serial port. Check wiring/port name and retry.")
        return 2

    print(f"Connected port: {serial_manager.get_connected_port()}")

    ctrl = ServoController(serial_manager, profile)
    try:
        pa23 = ctrl.read_PA23_Memory_Write_Inhibit()
        pa28_before = ctrl.read_PA28_Encoder_Mode()
        print(f"PA23 (EEPROM write-inhibit) currently: {pa23}")
        print(f"PA28 (encoder mode) currently: {pa28_before} "
              f"({'absolute' if pa28_before == 1 else 'incremental' if pa28_before == 0 else 'unknown'})")

        if pa28_before is None:
            print("Could not read PA28 at all. Aborting -- not safe to write blind.")
            return 2

        if pa28_before == args.to:
            print(f"PA28 is already {args.to}. Nothing to write.")
            return 0

        print(f"Writing PA28 = {args.to} ...")
        ok = ctrl.write_PA28_Encoder_Mode(bool(args.to))
        if not ok:
            print("Write FAILED or read-back did not confirm the requested value. "
                  "See log output above for details.")
            return 1

        print(f"PA28 write CONFIRMED: now {args.to} "
              f"({'absolute' if args.to == 1 else 'incremental'}).")

        if args.to == 1:
            print(
                "\nNEXT STEPS (manual, physical -- design doc §7.C 'C1'):\n"
                "  1. Power-cycle the drive now. Expect alarm AL.2A after it comes back up.\n"
                "  2. Power-cycle the drive again. Expect alarm AL.2C.\n"
                "  3. Clear AL.2C with either:\n"
                "       - ctrl.write_PA29_Initial_Abs_Pos()  (PA29 = 1), or\n"
                "       - a home-return operation.\n"
                "  4. Then confirm PA31 (APST) has no fault bits and "
                "read_absolute_position_pulses() returns a plausible value.\n"
                "  If AL.24 appears instead, determine whether it means "
                "'wrong encoder type' or 'position lost / battery low' before continuing."
            )
        else:
            print(
                "\nNEXT STEPS (manual, physical -- design doc §7.C 'C9'):\n"
                "  1. Power-cycle the drive now for PA28=0 to take effect.\n"
                "  2. Confirm behavior returns to incremental-mode as before."
            )
        return 0
    finally:
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
