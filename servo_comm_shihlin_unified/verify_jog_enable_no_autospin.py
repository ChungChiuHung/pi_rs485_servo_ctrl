"""
Real-hardware verification of the 2026-09-22 fix: "ENABLE SPEED CONTROL
MODE" (enable_speed_ctrl(enable=True)) must never resume rotation by
itself, even if 0x0904 (JOG_OPERATION) was left non-zero by a previous
session that didn't cleanly stop (e.g. a MOTION PAUSE that never landed).

Method:
  1. Deliberately reproduce a "stale 0x0904" drive state: manually enter JOG
     mode, command CW rotation, let it run briefly, then leave JOG mode
     WITHOUT sending the stop (0x0904 stays at 2 on the drive).
  2. Call the FIXED enable_speed_ctrl(enable=True) -- exactly what a fresh
     "ENABLE SPEED CONTROL MODE" click does -- and measure whether the
     motor is moving right after. It must not be.
  3. Confirm the underlying API still works: speed_ctrl_action(2) (CW)
     explicitly commanded afterward must still spin the motor.
  4. Clean up: stop, disable JOG mode.

HARDWARE-AFFECTING: runs real JOG rotation twice, briefly, at 10rpm. Per
CLAUDE.md §4 and this session's own <=20rpm test cap, only run this with
the user present and after explicit confirmation (given 2026-09-22).

    cd servo_comm_shihlin_unified/
    python verify_jog_enable_no_autospin.py --port COM4
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile

SPEED_RPM = 10
ACC_MS = 2000
SPIN_S = 1.0
MEASURE_S = 2.0


def read_pulses(ctrl):
    return ctrl.read_motor_feedback_pulses()


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

        # --- Step 1: manually reproduce a stale 0x0904 --------------------
        print("\n--- Step 1: manually entering JOG mode and commanding CW, "
              "then leaving WITHOUT stopping (reproduces the reported bug's precondition) ---")
        ctrl.clear_alarm_12()
        ctrl.delay_ms(100)
        ctrl.Enable_JOG_Mode(True)
        ctrl.delay_ms(100)
        ctrl.config_acc_dec_0x0902(ACC_MS)
        ctrl.delay_ms(100)
        ctrl.set_jog_speed(SPEED_RPM)
        ctrl.delay_ms(100)
        ctrl.speed_ctrl_action(2)  # CW -- 0x0904 = 2
        print(f"Commanded CW at {SPEED_RPM} rpm; letting it spin {SPIN_S}s...")
        p_before = read_pulses(ctrl)
        time.sleep(SPIN_S)
        p_after = read_pulses(ctrl)
        print(f"  Encoder moved {p_after - p_before} pulses while CW was running (confirms it was really spinning).")

        print("Leaving JOG mode WITHOUT sending stop (0x0904 stays 2 on the drive)...")
        ctrl.Enable_JOG_Mode(False)  # 0x0901 = 0 -- deliberately skip speed_ctrl_action(0)
        ctrl.delay_ms(200)

        p_left = read_pulses(ctrl)
        time.sleep(0.5)
        p_left2 = read_pulses(ctrl)
        print(f"  After leaving JOG mode: encoder delta over 0.5s = {p_left2 - p_left} "
              f"(should be ~0 -- confirms physical rotation actually stopped when the MODE was left, "
              "even though 0x0904 itself was never reset)")

        # --- Step 2: the fixed enable_speed_ctrl(enable=True) -------------
        print("\n--- Step 2: calling the FIXED enable_speed_ctrl(enable=True) "
              "(exactly what a fresh ENABLE SPEED CONTROL MODE click does) ---")
        p_arm_before = read_pulses(ctrl)
        ctrl.enable_speed_ctrl(speed_rpm=SPEED_RPM, acc_time=ACC_MS, enable=True)
        time.sleep(MEASURE_S)
        p_arm_after = read_pulses(ctrl)
        arm_delta = p_arm_after - p_arm_before
        print(f"  Encoder delta over {MEASURE_S}s right after arming: {arm_delta}")
        fix_confirmed = abs(arm_delta) < 5000  # a couple thousand pulses is noise; real rotation is orders of magnitude more
        print(f"  {'PASS -- did NOT auto-spin' if fix_confirmed else 'FAIL -- STILL AUTO-SPINNING, investigate immediately'}")

        # --- Step 3: confirm the API itself still works --------------------
        print("\n--- Step 3: confirming speed_ctrl_action(2) (CW) still works after arming ---")
        p_cw_before = read_pulses(ctrl)
        ctrl.speed_ctrl_action(2)
        time.sleep(SPIN_S)
        p_cw_after = read_pulses(ctrl)
        cw_delta = p_cw_after - p_cw_before
        print(f"  Encoder delta over {SPIN_S}s with CW explicitly commanded: {cw_delta}")
        print(f"  {'PASS -- CW still works' if abs(cw_delta) > 5000 else 'FAIL -- CW did not move the motor'}")

        # --- Cleanup --------------------------------------------------------
        ctrl.speed_ctrl_action(0)
        time.sleep(0.3)
        ctrl.enable_speed_ctrl(enable=False)
        print("\nStopped and disabled JOG mode.")
        return 0
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
