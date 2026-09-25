"""
Phase 3 real-hardware verification: Art-Net status-feedback, focused on
channels 10-12 (back home / set home / reset initial absolute position) --
the immediate-_send_feedback() fix added 2026-09-24 -- plus a short
continuous-motion run to check Modbus traffic/serial timing with feedback
enabled (channel 4's alarm read is a fresh serial round trip on every poll).

HARDWARE-AFFECTING: turns the servo on, runs a short (~3s, 10rpm) continuous
rotation, triggers a real (tiny, since home was just set to ~0deg by
verify_phase3_osc_feedback.py) back-home move, and persists a new home
position. Per CLAUDE.md §4, only run this with the user present and after
explicit confirmation (given 2026-09-24). All motion capped at 10rpm.

Channel 12 (reset initial absolute position / PA29) is deliberately NOT
triggered for real here -- PA29 rewrites the drive's own absolute-position
calibration reference (heavier than our software abs_home_pos_absolute,
and not part of what was explicitly confirmed for this session). Its
write_PA29_Initial_Abs_Pos() call is patched to a no-op so only the
Art-Net handler's trigger-then-feedback logic is exercised against the
real drive; the call itself is already proven safe from the 2026-09-22
PA28 switch session (memory/lessons.md).

    cd servo_comm_shihlin_unified/
    python verify_phase3_artnet_feedback.py --port /dev/ttyUSB0
"""
import argparse
import socket
import sys
import threading
import time
from unittest.mock import patch

from serial_port_manager import SerialPortManager
from servo_control import ServoController, alarm_name
from motor_profile import load_profiles, resolve_profile
from artnet_server import ArtNetInputServer

COMMAND_UNIVERSE = 0
FEEDBACK_UNIVERSE = 1
JOG_SPEED_RPM = 10
MAX_SPEED_RPM = 20
JOG_DURATION_S = 3.0


def frame17(speed=0, direction=0, cancel=0, abs_trigger=0, angle_hi=0, angle_lo=0, pos_speed=0,
            servo=0, clear=0, back_home=0, set_home=0, reset_abs=0,
            rel_hi=0, rel_lo=0, time_hi=0, time_lo=0, rel_trigger=0):
    return bytes([speed, direction, cancel, abs_trigger, angle_hi, angle_lo, pos_speed,
                  servo, clear, back_home, set_home, reset_abs,
                  rel_hi, rel_lo, time_hi, time_lo, rel_trigger])


class ArtDMXCapture:
    """Background listener for the feedback packets ArtNetInputServer sends
    on FEEDBACK_UNIVERSE. Decodes with the module's own parse_artdmx() so
    this test uses the exact same parser the class relies on."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.2)
        self.port = self.sock.getsockname()[1]
        self.packets = []  # (channels: list[int], t_monotonic)
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

    def latest(self):
        return self.packets[-1][0] if self.packets else None

    def count_since(self, t0):
        return sum(1 for _p, t in self.packets if t >= t0)

    def intervals_since(self, t0):
        times = [t for _p, t in self.packets if t >= t0]
        return [b - a for a, b in zip(times, times[1:])]

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
    capture = ArtDMXCapture()
    print(f"Local feedback listener on 127.0.0.1:{capture.port} (universe {FEEDBACK_UNIVERSE})")

    server = ArtNetInputServer(
        ctrl, listen_ip="127.0.0.1", listen_port=0, universe=COMMAND_UNIVERSE,
        max_speed_rpm=MAX_SPEED_RPM, acc_time=1500, enable_dangerous_channels=True,
        feedback_ip="127.0.0.1", feedback_universe=FEEDBACK_UNIVERSE,
    )
    # Redirect the feedback socket to our capture port instead of the real
    # Art-Net port 6454 -- everything else about the send path is real.
    real_send_feedback = server._send_feedback

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

    results = {}

    def check(label, predicate, description):
        ok = predicate()
        results[label] = ok
        print(f"  [{'OK' if ok else 'MISSING'}] {description}")

    try:
        ctrl.refresh_encoder_mode()
        code = ctrl.read_current_alarm_code()
        print(f"Alarm before: {hex(code) if code is not None else None} ({alarm_name(code)})")
        if code not in (0xFF, 0):
            print("Clearing alarm 12 first...")
            ctrl.write_PD_16_Enable_DI_Control()
            ctrl.clear_alarm_12()
            time.sleep(0.3)

        with patch.object(server, "_send_feedback", side_effect=send_feedback_to_capture):
            server.start()
            print("ArtNetInputServer started (feedback redirected to local capture).\n")

            print("1) channel 8: servo on")
            n0 = len(capture.packets)
            server._handle_dmx(COMMAND_UNIVERSE, frame17(servo=200))
            time.sleep(0.3)
            check("servo_on_feedback", lambda: len(capture.packets) > n0, "feedback packet sent on servo-on")

            print(f"\n2) channels 1-2: continuous JOG, {JOG_SPEED_RPM}rpm CW, for {JOG_DURATION_S}s "
                  "(watching Modbus/serial timing with feedback enabled)")
            speed_byte = round(JOG_SPEED_RPM / MAX_SPEED_RPM * 255)
            server._handle_dmx(COMMAND_UNIVERSE, frame17(speed=speed_byte, direction=200))
            t_jog_start = time.monotonic()
            time.sleep(JOG_DURATION_S)
            server._handle_dmx(COMMAND_UNIVERSE, frame17(speed=0, direction=200))
            time.sleep(0.5)
            intervals = capture.intervals_since(t_jog_start)
            n_packets = capture.count_since(t_jog_start)
            print(f"  Feedback packets during JOG window: {n_packets}")
            if intervals:
                print(f"  Inter-packet interval: min={min(intervals)*1000:.1f}ms "
                      f"max={max(intervals)*1000:.1f}ms avg={sum(intervals)/len(intervals)*1000:.1f}ms")
                check("jog_timing_under_1s", lambda: max(intervals) < 1.0,
                      "all inter-packet gaps stayed under the drive's 1s test-mode keep-alive ceiling")
            else:
                results["jog_timing_under_1s"] = False
                print("  [MISSING] no feedback packets captured during JOG")
            check("jog_feedback_sent", lambda: n_packets > 0, "feedback packets sent during continuous motion")

            print("\n3) channel 9: clear alarm")
            n0 = len(capture.packets)
            server._handle_dmx(COMMAND_UNIVERSE, frame17(servo=200, clear=255))
            time.sleep(0.3)
            check("clear_feedback", lambda: len(capture.packets) > n0, "feedback packet sent on clear-alarm")

            ctrl._refresh_current_angle_from_hardware()
            print(f"\nCurrent angle before back-home: {ctrl.current_angle} deg "
                  f"(should be small -- home was just set near here)")

            print("\n4) channel 10: back home (real move)")
            n0 = len(capture.packets)
            server._handle_dmx(COMMAND_UNIVERSE, frame17(servo=200, clear=255, back_home=255))
            time.sleep(2.0)
            check("back_home_feedback", lambda: len(capture.packets) > n0, "feedback packet sent on back-home trigger")
            latest = capture.latest()
            if latest:
                angle_raw = (latest[0] << 8) | latest[1]
                print(f"  Feedback-reported angle after back-home: {(angle_raw - 32768) / 100:.2f} deg")

            print("\n5) channel 11: set home (persists)")
            n0 = len(capture.packets)
            server._handle_dmx(COMMAND_UNIVERSE, frame17(servo=200, clear=255, set_home=255))
            time.sleep(0.3)
            check("set_home_feedback", lambda: len(capture.packets) > n0, "feedback packet sent on set-home trigger")

            print("\n6) channel 12: reset initial absolute position (PA29 write MOCKED -- see module docstring)")
            n0 = len(capture.packets)
            with patch.object(ctrl, "write_PA29_Initial_Abs_Pos") as mock_pa29:
                server._handle_dmx(COMMAND_UNIVERSE, frame17(servo=200, clear=255, reset_abs=255))
                time.sleep(0.3)
                check("reset_abs_pos_handler_called", lambda: mock_pa29.called,
                      "handler called write_PA29_Initial_Abs_Pos() (mocked, not sent to drive)")
            check("reset_abs_pos_feedback", lambda: len(capture.packets) > n0, "feedback packet sent on reset-abs-pos trigger")

            print("\n7) servo off")
            server._handle_dmx(COMMAND_UNIVERSE, frame17(servo=0))
            time.sleep(0.3)
            ctrl.write_PD_16_Enable_DI_Control()
            ctrl.clear_alarm_12()

        print("\n=== SUMMARY ===")
        for label, ok in results.items():
            print(f"  {label}: {'PASS' if ok else 'FAIL'}")
        all_ok = all(results.values())
        print(f"\nOverall: {'PASS' if all_ok else 'SOME CHECKS FAILED'}")
        return 0 if all_ok else 1
    finally:
        server.stop()
        ctrl.stop_continuous_reading()
        capture.close()
        serial_manager.disconnect()


if __name__ == "__main__":
    sys.exit(main())
