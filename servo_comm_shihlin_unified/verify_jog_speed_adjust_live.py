"""
Real-hardware verification: does writing 0x0903 (JOG speed) WHILE the motor
is already running in JOG mode actually change the live rotation speed?
The manual documents 0x0903 only as part of the JOG-test entry sequence
(docs/en_manual.txt:10346-10385) and never explicitly says whether it can be
changed after 0x0904 has already started the motor -- Art-Net's Channel 1
was designed assuming yes (artnet_server.py's docstring), but nothing in
memory/lessons.md or the design doc confirms this against a real drive.

This exercises exactly the same ServoController calls the web UI's merged
"ENABLE SPEED CONTROL MODE" toggle button, "MOTION START CW" button, and
new Up/Down arrow-key change_jog_speed_by() nudge would make -- just without
a browser in between.

Method: command 10rpm CW, measure real rpm from the raw encoder over a few
seconds, nudge to 15rpm via change_jog_speed_by(+5), measure again. If the
live speed change works, the second measurement should read close to 15rpm,
not 10rpm.

HARDWARE-AFFECTING: starts real continuous rotation (JOG mode). Per
CLAUDE.md §4 and this session's own <=20rpm test cap, only run this with the
user present and after explicit confirmation (given 2026-09-22).

    cd servo_comm_shihlin_unified/
    python verify_jog_speed_adjust_live.py --port COM4
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile

INITIAL_RPM = 10
BUMPED_RPM = 15
MEASURE_S = 3.0
SETTLE_S = 1.5


def measure_rpm(ctrl, duration_s):
    p1 = ctrl.read_motor_feedback_pulses()
    t1 = time.monotonic()
    time.sleep(duration_s)
    p2 = ctrl.read_motor_feedback_pulses()
    t2 = time.monotonic()
    if p1 is None or p2 is None:
        return None, None, None
    delta_pulses = p2 - p1
    delta_t = t2 - t1
    pulses_per_rev = ctrl.profile["encoder_pulses_per_rev"]
    rpm = abs(delta_pulses) / pulses_per_rev / delta_t * 60
    return rpm, delta_pulses, delta_t


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

        print(f"enable_speed_ctrl(speed_rpm={INITIAL_RPM}, acc_time=2000, enable=True) ...")
        ctrl.enable_speed_ctrl(speed_rpm=INITIAL_RPM, acc_time=2000, enable=True)

        print("speed_ctrl_action(2) -- CW ...")
        applied = ctrl.speed_ctrl_action(2)
        print(f"  applied={applied}")
        time.sleep(SETTLE_S)  # let it ramp up past acc_time

        rpm1, d1, t1 = measure_rpm(ctrl, MEASURE_S)
        print(f"Measured speed at commanded {INITIAL_RPM} rpm: {rpm1:.2f} rpm "
              f"(delta_pulses={d1}, dt={t1:.2f}s)")

        new_speed = ctrl.change_jog_speed_by(BUMPED_RPM - INITIAL_RPM)
        print(f"change_jog_speed_by({BUMPED_RPM - INITIAL_RPM}) -> {new_speed} rpm")
        time.sleep(SETTLE_S)

        rpm2, d2, t2 = measure_rpm(ctrl, MEASURE_S)
        print(f"Measured speed at commanded {new_speed} rpm: {rpm2:.2f} rpm "
              f"(delta_pulses={d2}, dt={t2:.2f}s)")

        print("speed_ctrl_action(0) -- stop ...")
        ctrl.speed_ctrl_action(0)
        time.sleep(0.3)
        print("enable_speed_ctrl(enable=False) -- exit JOG mode ...")
        ctrl.enable_speed_ctrl(enable=False)

        print(f"\nRESULT: {rpm1:.2f} rpm -> {rpm2:.2f} rpm after a live +{BUMPED_RPM - INITIAL_RPM} rpm nudge "
              f"({'PASS -- live speed change confirmed' if rpm2 > rpm1 * 1.2 else 'DID NOT CLEARLY CHANGE -- investigate'})")
        return 0
    finally:
        ctrl.stop_continuous_reading()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
