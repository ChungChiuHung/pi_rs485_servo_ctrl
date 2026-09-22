"""
Real-hardware investigation, 2026-09-22: user reported that clicking ENABLE
SPEED CONTROL MODE two or more times causes continuous rotation, even after
the earlier fix (enable_speed_ctrl() now ends with an explicit
speed_ctrl_action(0)). Code review found a client-side race: the web UI's
toggle button decides "enable" vs "disable" from a jogModeActive flag that
was only ever updated by the periodic /status poll, never immediately after
the toggle's own click -- so two rapid clicks (both firing before the next
poll) would BOTH take the "enable" branch and call
enable_speed_ctrl(enable=True) twice in a row, re-arming JOG mode while it
was already active, rather than the second click correctly disarming it.
That race is now fixed client-side (the toggle updates its own tracked
state immediately from its own AJAX response).

This script tests the SERVER/DRIVE side directly: is calling
enable_speed_ctrl(enable=True) twice in a row (simulating the exact race,
short delay) safe, or does re-arming an already-active JOG mode behave
differently from arming a fresh one and leave the motor spinning?

HARDWARE-AFFECTING: arms JOG mode twice in a row at 10rpm and measures
whether the motor spins. Per CLAUDE.md §4, only run this with the user
present and after explicit confirmation (given 2026-09-22).

    cd servo_comm_shihlin_unified/
    python verify_double_enable_no_spin.py --port COM4
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile

SPEED_RPM = 10
ACC_MS = 2000
DOUBLE_CLICK_GAP_S = 0.3   # realistic fast-double-click spacing
MEASURE_S = 3.0


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

        print(f"\nCalling enable_speed_ctrl(enable=True) -- 1st 'click' ...")
        ctrl.enable_speed_ctrl(speed_rpm=SPEED_RPM, acc_time=ACC_MS, enable=True)

        print(f"Waiting {DOUBLE_CLICK_GAP_S}s (simulating a fast double-click, "
              "well before the client would normally have polled /status)...")
        time.sleep(DOUBLE_CLICK_GAP_S)

        print("Calling enable_speed_ctrl(enable=True) AGAIN -- 2nd 'click' "
              "(re-arming while already active) ...")
        ctrl.enable_speed_ctrl(speed_rpm=SPEED_RPM, acc_time=ACC_MS, enable=True)

        print(f"Measuring encoder over {MEASURE_S}s after the 2nd enable...")
        p1 = ctrl.read_motor_feedback_pulses()
        time.sleep(MEASURE_S)
        p2 = ctrl.read_motor_feedback_pulses()
        delta = p2 - p1
        print(f"Encoder delta: {delta} pulses over {MEASURE_S}s")
        spinning = abs(delta) > 5000
        print(f"{'FAIL -- motor IS spinning after a double enable' if spinning else 'PASS -- motor did NOT spin'}")

        if spinning:
            print("Stopping now.")
            ctrl.speed_ctrl_action(0)
            time.sleep(0.3)

        ctrl.enable_speed_ctrl(enable=False)
        print("Disabled JOG mode.")
        return 1 if spinning else 0
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
