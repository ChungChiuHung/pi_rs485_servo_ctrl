"""
Phase 3 real-hardware re-verification: the start_continuous_reading()
absolute-offset fix (2026-09-24, memory/lessons.md). Reproduces the exact
failure scenario found earlier today -- a fresh connect, then continuous
JOG as the FIRST action, with no discrete move or explicit refresh in
between -- and confirms the very first feedback angle now matches reality,
via the real Art-Net feedback path.

HARDWARE-AFFECTING: turns the servo on and runs a short (~1.5s, 5rpm)
continuous rotation. Per CLAUDE.md §4, only run this with the user present
and after explicit confirmation (given 2026-09-24, reaffirmed for this fix).

    cd servo_comm_shihlin_unified/
    python verify_phase3_jog_start_angle_fix.py --port /dev/ttyUSB0
"""
import argparse
import socket
import sys
import threading
import time

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile
from artnet_server import ArtNetInputServer

COMMAND_UNIVERSE = 0
FEEDBACK_UNIVERSE = 1
JOG_SPEED_RPM = 5
MAX_SPEED_RPM = 20
JOG_DURATION_S = 1.5
# Generous tolerance: this is checking "not wildly wrong" (the bug produced
# a ~22deg error), not sub-degree precision -- the shaft is moving during
# the window between the true baseline read and the first feedback packet.
TOLERANCE_DEG = 3.0


def frame3(speed=0, direction=0):
    return bytes([speed, direction, 0])


class ArtDMXCapture:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.packets = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                data, _addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            parsed = ArtNetInputServer.parse_artdmx(data)
            if parsed is None:
                continue
            universe, dmx = parsed
            if universe != FEEDBACK_UNIVERSE:
                continue
            self.packets.append((list(dmx), time.monotonic()))

    def first(self):
        return self.packets[0] if self.packets else None

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument("--profile", default=None)
    args = parser.parse_args()

    profiles_data = load_profiles()
    profile_name = args.profile or profiles_data["active_profile"]
    profile = resolve_profile(profiles_data, profile_name)
    print(f"Profile: {profile_name} (baud={profile['baud_rate']})")

    # Step 1: true baseline, via a SEPARATE connection (read-only, mirrors
    # verify_phase3_baseline.py) -- not the connection under test below.
    baseline_sm = SerialPortManager(port=args.port, baud_rate=profile["baud_rate"])
    baseline_sm.connect()
    if not baseline_sm.get_serial_instance():
        print("Could not open any serial port.")
        return 2
    baseline_ctrl = ServoController(baseline_sm, profile)
    baseline_ctrl.refresh_encoder_mode()
    code = baseline_ctrl.read_current_alarm_code()
    print(f"Alarm before: {hex(code) if code is not None else None} ({alarm_name(code)})")
    if code not in (0xFF, 0):
        print("Clearing alarm 12 first...")
        baseline_ctrl.write_PD_16_Enable_DI_Control()
        baseline_ctrl.clear_alarm_12()
        time.sleep(0.3)
    ok = baseline_ctrl._refresh_current_angle_from_hardware()
    if not ok:
        print("Could not read true baseline angle; aborting.")
        return 3
    true_baseline_angle = baseline_ctrl.current_angle
    print(f"True baseline angle (separate connection, fresh read): {true_baseline_angle} deg")
    baseline_ctrl.stop_continuous_reading()
    baseline_sm.disconnect()

    # Step 2: the actual repro -- a FRESH ServoController/connection,
    # refresh_encoder_mode() only (mirrors app.py's connect sequence), then
    # continuous JOG as the very first action. No discrete move, no
    # explicit _refresh_current_angle_from_hardware() call in between.
    sm = SerialPortManager(port=args.port, baud_rate=profile["baud_rate"])
    sm.connect()
    ctrl = ServoController(sm, profile)
    capture = ArtDMXCapture()
    print(f"\nLocal feedback listener on 127.0.0.1:{capture.port} (universe {FEEDBACK_UNIVERSE})")

    server = ArtNetInputServer(
        ctrl, listen_ip="127.0.0.1", listen_port=0, universe=COMMAND_UNIVERSE,
        max_speed_rpm=MAX_SPEED_RPM, acc_time=1500, enable_dangerous_channels=False,
        feedback_ip="127.0.0.1", feedback_universe=FEEDBACK_UNIVERSE,
    )

    def send_feedback_to_capture(angle_deg):
        if not server.feedback_enabled or server._feedback_sock is None:
            return
        try:
            from artnet_server import encode_angle_deg, is_alarm_active
            high, low = encode_angle_deg(angle_deg)
            servo_on = server._last_servo_channel not in (None, 0)
            alarm_active = is_alarm_active(server.servo_ctrller.read_current_alarm_code())
            moving = bool(server.servo_ctrller.reading_active)
            data = [high, low, 255 if servo_on else 0, 255 if alarm_active else 0, 255 if moving else 0]
            server._feedback_sock.sendto(server._build_artdmx(data), ("127.0.0.1", capture.port))
        except Exception as e:
            print(f"  [feedback send error] {e}")

    from unittest.mock import patch
    try:
        ctrl.refresh_encoder_mode()  # exactly what app.py does at connect -- no position refresh
        print(f"ctrl._absolute_offset right after connect (should be None): {ctrl._absolute_offset}")

        with patch.object(server, "_send_feedback", side_effect=send_feedback_to_capture):
            server.start()
            print("ArtNetInputServer started. Servo on, then continuous JOG as the FIRST action...")

            ctrl.servo_on()
            time.sleep(0.3)

            speed_byte = round(JOG_SPEED_RPM / MAX_SPEED_RPM * 255)
            server._handle_dmx(COMMAND_UNIVERSE, frame3(speed=speed_byte, direction=200))  # CW
            time.sleep(JOG_DURATION_S)
            server._handle_dmx(COMMAND_UNIVERSE, frame3(speed=0, direction=200))
            time.sleep(0.3)

        first = capture.first()
        if first is None:
            print("\nNo feedback packet captured at all -- cannot verify.")
            return 4
        data, _t = first
        angle_raw = (data[0] << 8) | data[1]
        first_reported_angle = (angle_raw - 32768) / 100
        print(f"\nFirst feedback packet reported angle: {first_reported_angle} deg")
        print(f"True baseline angle was: {true_baseline_angle} deg")
        diff = abs(first_reported_angle - true_baseline_angle)
        print(f"Difference: {diff:.4f} deg (tolerance: {TOLERANCE_DEG} deg)")
        print(f"\nctrl._absolute_offset after first poll: {ctrl._absolute_offset}")

        if diff <= TOLERANCE_DEG:
            print("\nRESULT: PASS -- first reported angle matches reality (fix confirmed live).")
            result = 0
        else:
            print("\nRESULT: FAIL -- first reported angle is still wrong.")
            result = 1

        ctrl._refresh_current_angle_from_hardware()
        print(f"\nFinal actual angle: {ctrl.current_angle} deg")
        return result
    finally:
        server.stop()
        ctrl.servo_off()
        time.sleep(0.2)
        ctrl.write_PD_16_Enable_DI_Control()
        ctrl.clear_alarm_12()
        ctrl.stop_continuous_reading()
        capture.close()
        sm.disconnect()


if __name__ == "__main__":
    sys.exit(main())
