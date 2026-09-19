"""
Tests for artnet_server.py. servo_ctrller is always a MagicMock -- no real
hardware I/O. The end-to-end tests start a real ArtNetInputServer on
localhost and send it real UDP ArtDMX packets to prove the socket/parsing/
dispatch wiring works; the mock stands in for what would otherwise reach
the driver.
"""
import socket
import struct
import time
import unittest
from unittest.mock import MagicMock

from artnet_server import ArtNetInputServer, ARTNET_ID, OP_OUTPUT_DMX


def build_artdmx_packet(universe: int, dmx_data: bytes, sequence=0, physical=0) -> bytes:
    sub_uni = universe & 0xFF
    net = (universe >> 8) & 0xFF
    header = (
        ARTNET_ID +
        struct.pack('<H', OP_OUTPUT_DMX) +
        bytes([0, 14]) +           # ProtVerHi, ProtVerLo (14 is the current spec version)
        bytes([sequence, physical, sub_uni, net]) +
        struct.pack('>H', len(dmx_data))
    )
    return header + dmx_data


def make_server(**kwargs):
    servo_ctrller = MagicMock()
    server = ArtNetInputServer(servo_ctrller, listen_ip="127.0.0.1", listen_port=0, **kwargs)
    return server, servo_ctrller


class TestParseArtDMX(unittest.TestCase):

    def test_valid_packet_parses_universe_and_data(self):
        packet = build_artdmx_packet(universe=3, dmx_data=bytes([10, 20, 30]))
        result = ArtNetInputServer.parse_artdmx(packet)
        self.assertIsNotNone(result)
        universe, data = result
        self.assertEqual(universe, 3)
        self.assertEqual(data, bytes([10, 20, 30]))

    def test_universe_spans_net_and_subuni_bytes(self):
        packet = build_artdmx_packet(universe=0x0105, dmx_data=bytes([1]))
        universe, _data = ArtNetInputServer.parse_artdmx(packet)
        self.assertEqual(universe, 0x0105)

    def test_wrong_id_returns_none(self):
        packet = b"NotArtNet" + b'\x00' * 20
        self.assertIsNone(ArtNetInputServer.parse_artdmx(packet))

    def test_wrong_opcode_returns_none(self):
        packet = bytearray(build_artdmx_packet(universe=0, dmx_data=bytes([1])))
        packet[8:10] = struct.pack('<H', 0x2000)  # OpPoll, not OpOutput
        self.assertIsNone(ArtNetInputServer.parse_artdmx(bytes(packet)))

    def test_too_short_packet_returns_none(self):
        self.assertIsNone(ArtNetInputServer.parse_artdmx(b'\x01\x02'))


class TestHandleDMXWrongUniverseIgnored(unittest.TestCase):

    def test_frame_for_different_universe_is_ignored(self):
        server, ctrl = make_server(universe=0)
        server._handle_dmx(universe=1, data=bytes([255, 128, 0]))
        ctrl.enable_speed_ctrl.assert_not_called()

    def test_frame_shorter_than_3_channels_is_ignored(self):
        server, ctrl = make_server(universe=0)
        server._handle_dmx(universe=0, data=bytes([255, 128]))
        ctrl.enable_speed_ctrl.assert_not_called()


class TestEnableChannel(unittest.TestCase):

    def test_rising_edge_enables_continuous_motion_scaled_to_max_speed(self):
        server, ctrl = make_server(max_speed_rpm=200)
        server._handle_dmx(universe=0, data=bytes([255, 0, 0]))  # channel1=255 -> full speed
        ctrl.enable_speed_ctrl.assert_called_once_with(200, 5000, True)

    def test_mid_range_value_scales_linearly(self):
        server, ctrl = make_server(max_speed_rpm=100)
        server._handle_dmx(universe=0, data=bytes([128, 0, 0]))
        expected_rpm = round(128 / 255 * 100)
        ctrl.enable_speed_ctrl.assert_called_once_with(expected_rpm, 5000, True)

    def test_does_not_re_trigger_on_repeated_frames_with_same_value(self):
        """Real Art-Net sources resend the full frame 30-44x/second even
        with no change -- enable_speed_ctrl() must fire once per rising
        edge, not once per frame."""
        server, ctrl = make_server()
        for _ in range(5):
            server._handle_dmx(universe=0, data=bytes([200, 0, 0]))
        ctrl.enable_speed_ctrl.assert_called_once()

    def test_falling_edge_stops_motion(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 0, 0]))  # enable
        server._handle_dmx(universe=0, data=bytes([0, 0, 0]))    # disable
        ctrl.speed_ctrl_action.assert_called_with(0)

    def test_zero_at_start_does_not_call_stop_spuriously(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0]))
        ctrl.speed_ctrl_action.assert_not_called()


class TestDirectionChannel(unittest.TestCase):
    """Per docs/en_manual.txt:10380-10382 (JOG_OPERATION, 0x0904): 1 =
    forward rotation (CCW), 2 = reverse rotation (CW)."""

    def test_low_range_is_ccw(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 50, 0]))
        ctrl.speed_ctrl_action.assert_called_with(1)

    def test_high_range_is_cw(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))
        ctrl.speed_ctrl_action.assert_called_with(2)

    def test_direction_change_only_fires_on_change(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))  # CW
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))  # same CW again
        ctrl.speed_ctrl_action.assert_called_once_with(2)

    def test_direction_change_from_cw_to_ccw_fires_again(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))  # CW
        server._handle_dmx(universe=0, data=bytes([200, 50, 0]))   # CCW
        self.assertEqual(ctrl.speed_ctrl_action.call_count, 2)
        ctrl.speed_ctrl_action.assert_called_with(1)


class TestCancelChannel(unittest.TestCase):

    def test_rising_edge_triggers_cancel(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 255]))
        ctrl.cancel_continuous_reading.assert_called_once()

    def test_sustained_high_does_not_retrigger(self):
        server, ctrl = make_server()
        for _ in range(5):
            server._handle_dmx(universe=0, data=bytes([0, 0, 255]))
        ctrl.cancel_continuous_reading.assert_called_once()

    def test_cancel_can_fire_again_after_returning_to_zero(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 255]))
        server._handle_dmx(universe=0, data=bytes([0, 0, 0]))
        server._handle_dmx(universe=0, data=bytes([0, 0, 255]))
        self.assertEqual(ctrl.cancel_continuous_reading.call_count, 2)


class TestPositionModeChannels(unittest.TestCase):
    """Channels 4-7 -- absolute-angle position mode, mirroring OSC's
    /set_point (ServoController.post_step_motion_by()). Requires a 7-byte
    (or longer) DMX frame; a sender using only channels 1-3 (continuous
    mode) never triggers this, so the two modes coexist in one universe."""

    def test_short_frame_without_position_channels_never_triggers(self):
        """Backward compatible with a sender that only fills channels 1-3."""
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0]))
        ctrl.post_step_motion_by.assert_not_called()

    def test_rising_edge_triggers_move_with_default_max_angle(self):
        server, ctrl = make_server(max_speed_rpm=100, acc_time=5000)
        # channel4=255 (trigger), channels5-6=0xFFFF (max angle), channel7=255 (max speed)
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 255, 255, 255]))
        ctrl.post_step_motion_by.assert_called_once_with(360.0, 5000, 100)

    def test_angle_zero_when_channels_5_and_6_are_zero(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 0, 0, 0]))
        ctrl.post_step_motion_by.assert_called_once_with(0.0, 5000, 1)

    def test_angle_scales_with_custom_position_mode_max_angle(self):
        server, ctrl = make_server(position_mode_max_angle=180)
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 255, 255, 0]))
        angle_arg = ctrl.post_step_motion_by.call_args[0][0]
        self.assertAlmostEqual(angle_arg, 180.0, places=2)

    def test_speed_channel_scales_with_max_speed_rpm(self):
        server, ctrl = make_server(max_speed_rpm=200)
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 0, 0, 128]))
        speed_arg = ctrl.post_step_motion_by.call_args[0][2]
        self.assertEqual(speed_arg, round(128 / 255 * 200))

    def test_zero_speed_channel_is_clamped_to_minimum_1_rpm(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 0, 0, 0]))
        speed_arg = ctrl.post_step_motion_by.call_args[0][2]
        self.assertEqual(speed_arg, 1)

    def test_sustained_high_trigger_does_not_retrigger(self):
        server, ctrl = make_server()
        for _ in range(5):
            server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 100, 0, 100]))
        ctrl.post_step_motion_by.assert_called_once()

    def test_trigger_can_fire_again_after_returning_to_zero(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 100, 0, 100]))
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 0, 100, 0, 100]))
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 100, 0, 100]))
        self.assertEqual(ctrl.post_step_motion_by.call_count, 2)

    def test_position_mode_does_not_interfere_with_continuous_mode_channels(self):
        """Both modes read the same DMX frame -- a position-mode trigger on
        channel 4 must not also fire continuous-motion methods, and vice
        versa; they're independent, edge-triggered channels."""
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 100, 0, 100]))
        ctrl.enable_speed_ctrl.assert_not_called()
        ctrl.speed_ctrl_action.assert_not_called()


def frame12(servo=0, clear=0, back_home=0, set_home=0, reset_abs=0):
    """Builds a 12-channel frame with channels 1-7 at their idle/inert
    values and channels 8-12 set to the given values."""
    return bytes([0, 0, 0, 0, 0, 0, 0, servo, clear, back_home, set_home, reset_abs])


class TestExtendedChannels(unittest.TestCase):
    """Channels 8-12 -- servo on/off, clear alarm, back home, set home,
    reset initial absolute position. Mirrors OSC's /servo, /clear,
    /back_home, /set_home, /reset_initial_abs_position respectively, giving
    Art-Net feature parity with OSC's non-continuous-motion functions."""

    def test_short_frame_without_extended_channels_never_triggers(self):
        """Backward compatible with a sender that only fills channels 1-7."""
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 0, 0, 0, 0]))
        ctrl.servo_on.assert_not_called()
        ctrl.clear_alarm_12.assert_not_called()
        ctrl.initial_abs_home.assert_not_called()
        ctrl.set_home_position.assert_not_called()
        ctrl.write_PA29_Initial_Abs_Pos.assert_not_called()

    def test_channel8_nonzero_turns_servo_on(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(servo=200))
        ctrl.servo_on.assert_called_once()
        ctrl.servo_off.assert_not_called()

    def test_channel8_zero_after_nonzero_turns_servo_off(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(servo=200))
        server._handle_dmx(universe=0, data=frame12(servo=0))
        ctrl.servo_off.assert_called_once()

    def test_channel8_zero_at_start_does_not_call_off_spuriously(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(servo=0))
        ctrl.servo_off.assert_not_called()

    def test_channel8_repeated_same_value_does_not_retrigger(self):
        server, ctrl = make_server()
        for _ in range(5):
            server._handle_dmx(universe=0, data=frame12(servo=200))
        ctrl.servo_on.assert_called_once()

    def test_channel9_rising_edge_clears_alarm(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(clear=255))
        ctrl.clear_alarm_12.assert_called_once()

    def test_channel9_sustained_high_does_not_retrigger(self):
        server, ctrl = make_server()
        for _ in range(5):
            server._handle_dmx(universe=0, data=frame12(clear=255))
        ctrl.clear_alarm_12.assert_called_once()

    def test_channel10_rising_edge_triggers_back_home(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(back_home=255))
        ctrl.initial_abs_home.assert_called_once()

    def test_channel10_sustained_high_does_not_retrigger(self):
        server, ctrl = make_server()
        for _ in range(5):
            server._handle_dmx(universe=0, data=frame12(back_home=255))
        ctrl.initial_abs_home.assert_called_once()

    def test_channel11_rising_edge_triggers_set_home(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(set_home=255))
        ctrl.set_home_position.assert_called_once()

    def test_channel12_rising_edge_triggers_reset_initial_abs_position(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(reset_abs=255))
        ctrl.write_PA29_Initial_Abs_Pos.assert_called_once()

    def test_channels_can_fire_again_after_returning_to_zero(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(clear=255))
        server._handle_dmx(universe=0, data=frame12(clear=0))
        server._handle_dmx(universe=0, data=frame12(clear=255))
        self.assertEqual(ctrl.clear_alarm_12.call_count, 2)

    def test_extended_channels_do_not_interfere_with_other_channels(self):
        """All four groups (continuous motion, position mode, cancel,
        extended) read the same DMX frame -- triggering one must not fire
        methods belonging to the others."""
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=frame12(back_home=255))
        ctrl.enable_speed_ctrl.assert_not_called()
        ctrl.speed_ctrl_action.assert_not_called()
        ctrl.post_step_motion_by.assert_not_called()
        ctrl.cancel_continuous_reading.assert_not_called()
        ctrl.servo_on.assert_not_called()
        ctrl.clear_alarm_12.assert_not_called()
        ctrl.set_home_position.assert_not_called()
        ctrl.write_PA29_Initial_Abs_Pos.assert_not_called()


class TestGetChannelSnapshot(unittest.TestCase):
    """get_channel_snapshot() -- the web UI's Art-Net Channel Monitor, so
    a user can see what a console/controller actually sent without an
    external DMX tool."""

    def test_no_frame_received_yet_returns_none(self):
        server, ctrl = make_server()
        self.assertIsNone(server.get_channel_snapshot())

    def test_short_frame_only_reports_channels_1_to_3(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([100, 200, 0]))

        snapshot = server.get_channel_snapshot()

        self.assertIsNotNone(snapshot)
        self.assertEqual([c["channel"] for c in snapshot["channels"]], [1, 2, 3])

    def test_channel_1_interpreted_as_rpm(self):
        server, ctrl = make_server(max_speed_rpm=100)
        server._handle_dmx(universe=0, data=bytes([255, 0, 0]))

        ch1 = server.get_channel_snapshot()["channels"][0]

        self.assertEqual(ch1["value"], 255)
        self.assertEqual(ch1["interpreted"], "100 rpm")

    def test_channel_1_zero_is_disabled(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0]))

        ch1 = server.get_channel_snapshot()["channels"][0]

        self.assertEqual(ch1["interpreted"], "disabled")

    def test_direction_channel_interpreted(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 200, 0]))

        ch2 = server.get_channel_snapshot()["channels"][1]

        self.assertEqual(ch2["interpreted"], "CW")

    def test_seven_byte_frame_reports_position_mode_channels(self):
        server, ctrl = make_server(max_speed_rpm=100, position_mode_max_angle=360)
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 0x80, 0x00, 128]))

        snapshot = server.get_channel_snapshot()

        self.assertEqual([c["channel"] for c in snapshot["channels"]], [1, 2, 3, 4, 5, 6, 7])
        angle_channel = snapshot["channels"][4]
        self.assertIn("deg", angle_channel["interpreted"])

    def test_twelve_byte_frame_reports_extended_channels(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 0, 0, 0, 0, 200, 255, 0, 0, 0]))

        snapshot = server.get_channel_snapshot()

        self.assertEqual([c["channel"] for c in snapshot["channels"]], list(range(1, 13)))
        self.assertEqual(snapshot["channels"][7]["interpreted"], "on")  # channel 8, servo
        self.assertEqual(snapshot["channels"][8]["interpreted"], "triggered")  # channel 9, clear alarm

    def test_snapshot_reflects_most_recent_frame_not_the_first(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([100, 0, 0]))
        server._handle_dmx(universe=0, data=bytes([200, 0, 0]))

        ch1 = server.get_channel_snapshot()["channels"][0]

        self.assertEqual(ch1["value"], 200)

    def test_frame_for_a_different_universe_does_not_update_snapshot(self):
        server, ctrl = make_server(universe=0)
        server._handle_dmx(universe=1, data=bytes([100, 0, 0]))

        self.assertIsNone(server.get_channel_snapshot())

    def test_received_at_timestamp_is_present(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0]))

        self.assertIsNotNone(server.get_channel_snapshot()["received_at"])


class TestStartStopLifecycle(unittest.TestCase):

    def test_start_sets_is_running_and_stop_clears_it(self):
        server, ctrl = make_server()
        server.start()
        try:
            self.assertTrue(server.is_running)
        finally:
            server.stop()
        self.assertFalse(server.is_running)

    def test_starting_twice_raises(self):
        server, ctrl = make_server()
        server.start()
        try:
            with self.assertRaises(RuntimeError):
                server.start()
        finally:
            server.stop()

    def test_stop_when_not_running_is_a_noop(self):
        server, ctrl = make_server()
        server.stop()  # must not raise

    def test_real_udp_packet_reaches_handler(self):
        server, ctrl = make_server(universe=0)
        server.start()
        try:
            actual_port = server._sock.getsockname()[1]
            packet = build_artdmx_packet(universe=0, dmx_data=bytes([255, 200, 0]))
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sender.sendto(packet, ("127.0.0.1", actual_port))
            finally:
                sender.close()

            deadline = time.time() + 2
            while not ctrl.enable_speed_ctrl.called and time.time() < deadline:
                time.sleep(0.01)

            ctrl.enable_speed_ctrl.assert_called_once()
        finally:
            server.stop()


if __name__ == "__main__":
    unittest.main()
