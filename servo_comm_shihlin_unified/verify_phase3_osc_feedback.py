"""
Phase 3 real-hardware verification: OSC status-feedback (2026-09-24 feature,
never run against a drive before). Confirms /servo_on, /servo_off, /clear,
/set_point (echo + /moving + /motion_complete), /set_home_position actually
arrive at a real UDP receiver when triggered against the real drive.

HARDWARE-AFFECTING: turns the servo on, makes a small (~5deg) real move and
back, and persists a new home position. Per CLAUDE.md §4, only run this with
the user present and after explicit confirmation (given 2026-09-24). Speed
capped at 10rpm.

Does NOT exercise /pr_step_path or /reset_initial_abs_position live -- PR
mode has never been validated against this drive (see
docs/servo_comm_shihlin_merge_design.md §8: PATH tables were never written,
so triggering PF82 could run an unconfigured/unknown path) and PA29 rewrites
the drive's own absolute-position calibration reference (heavier than our
software abs_home_pos_absolute). Both handlers' feedback-echo logic is
already covered by mocked unit tests (test_osc_server.py); this script only
needs to confirm UDP delivery works, which the other handlers already prove.

    cd servo_comm_shihlin_unified/
    python verify_phase3_osc_feedback.py --port /dev/ttyUSB0
"""
import argparse
import socket
import sys
import threading
import time

from pythonosc.osc_packet import OscPacket

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile
from osc_server import OSCInputServer

MOVE_DEG = 5.0
SPEED_RPM = 10
ACC_TIME = 2000


class UDPCapture:
    """Background listener for the feedback messages OSCInputServer sends.
    Bound to an ephemeral localhost port; decodes each datagram as an OSC
    message and stores (address, args, t_monotonic)."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.messages = []
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
            try:
                packet = OscPacket(data)
                for timed_msg in packet.messages:
                    msg = timed_msg.message
                    self.messages.append((msg.address, list(msg.params), time.monotonic()))
            except Exception as e:
                print(f"  [capture] failed to decode packet: {e}")

    def wait_for(self, address, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for addr, args, _t in self.messages:
                if addr == address:
                    return args
            time.sleep(0.02)
        return None

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        self.sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial port (e.g. /dev/ttyUSB0). Default: auto-detect.")
    parser.add_argument("--profile", default=None, help="Profile name in motor_profiles.json. Default: active_profile.")
    args = parser.parse_args()

    profiles_data = load_profiles()
    profile_name = args.profile or profiles_data["active_profile"]
    profile = resolve_profile(profiles_data, profile_name)
    print(f"Profile: {profile_name} (baud={profile['baud_rate']})")

    serial_manager = SerialPortManager(port=args.port, baud_rate=profile["baud_rate"])
    serial_manager.connect()
    if not serial_manager.get_serial_instance():
        print("Could not open any serial port.")
        return 2
    print(f"Connected port: {serial_manager.get_connected_port()}")

    ctrl = ServoController(serial_manager, profile)
    capture = UDPCapture()
    print(f"Local feedback listener on 127.0.0.1:{capture.port}")

    results = {}

    def check(label, address, timeout=3.0):
        args_ = capture.wait_for(address, timeout=timeout)
        ok = args_ is not None
        results[label] = ok
        print(f"  [{'OK' if ok else 'MISSING'}] {address} -> {args_}")
        return args_

    osc = OSCInputServer(ctrl, feedback_ip="127.0.0.1", feedback_port=capture.port)

    try:
        ctrl.refresh_encoder_mode()
        code = ctrl.read_current_alarm_code()
        print(f"Alarm before: {hex(code) if code is not None else None} ({alarm_name(code)})")
        if code not in (0xFF, 0):
            print("Clearing alarm 12 first...")
            ctrl.write_PD_16_Enable_DI_Control()
            ctrl.clear_alarm_12()
            time.sleep(0.3)

        ok = ctrl._refresh_current_angle_from_hardware()
        if not ok:
            print("Could not read starting position; aborting.")
            return 3
        start_angle = ctrl.current_angle
        print(f"Starting angle: {start_angle} deg")

        osc.start()
        print("OSCInputServer started (feedback wired).\n")

        print("1) servo_on()")
        osc._servo_handler(None, [], 1.0)
        check("servo_on", "/servo_on")

        target = round(start_angle + MOVE_DEG, 4)
        print(f"\n2) set_point -> {target} deg @ {SPEED_RPM}rpm (real move)")
        osc._set_point_handler(None, [], target, ACC_TIME, SPEED_RPM)
        check("set_point_echo", "/set_point")
        check("moving", "/moving", timeout=2.0)
        check("motion_complete", "/motion_complete", timeout=8.0)
        time.sleep(0.5)
        ctrl._refresh_current_angle_from_hardware()
        print(f"  Landed at: {ctrl.current_angle} deg (target {target})")

        print(f"\n3) set_point -> {start_angle} deg @ {SPEED_RPM}rpm (return to start)")
        osc._set_point_handler(None, [], start_angle, ACC_TIME, SPEED_RPM)
        time.sleep(3.0)
        ctrl._refresh_current_angle_from_hardware()
        print(f"  Landed at: {ctrl.current_angle} deg (target {start_angle})")

        print("\n4) set_home()  -- persists current position as new home")
        osc._set_home_position_handler(None, [], 1.0)
        check("set_home", "/set_home_position")

        print("\n5) servo_off()  -- expected to re-trigger AL.12 (documented behavior)")
        osc._servo_handler(None, [], 0.0)
        check("servo_off", "/servo_off")

        print("\n6) clear()")
        osc._clear_handler(None, [], 1.0)
        check("clear", "/clear")
        time.sleep(0.3)
        code_after = ctrl.read_current_alarm_code()
        print(f"  Alarm after clear: {hex(code_after) if code_after is not None else None}")

        print("\n=== SUMMARY ===")
        for label, ok in results.items():
            print(f"  {label}: {'PASS' if ok else 'FAIL (no packet received)'}")
        all_ok = all(results.values())
        print(f"\nOverall: {'PASS' if all_ok else 'SOME CHECKS FAILED'}")
        return 0 if all_ok else 1
    finally:
        osc.stop()
        ctrl.stop_continuous_reading()
        capture.close()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
