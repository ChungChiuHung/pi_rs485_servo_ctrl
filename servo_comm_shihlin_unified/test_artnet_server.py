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
from unittest.mock import MagicMock, call

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
    # The legacy tests below address universe 0 and exercise channels 10-12;
    # the server's own defaults (universe 1, channels 10-12 off) are covered
    # explicitly in TestSafetyDefaults.
    kwargs.setdefault("universe", 0)
    kwargs.setdefault("enable_dangerous_channels", True)
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


    def test_refused_move_is_not_retried_and_does_not_block_other_channels(self):
        """A position-mode move that is refused (drive position unreadable,
        PositionUnavailableError) must consume the trigger edge -- otherwise
        it is retried on every 30-44 fps frame -- and must not abort the
        rest of the frame (here: servo-on on channel 8)."""
        server, ctrl = make_server()
        ctrl.post_step_motion_by.side_effect = ValueError("position unavailable")
        frame = bytes([0, 0, 0, 255, 0x80, 0x00, 128, 200, 0, 0, 0, 0])

        server._handle_dmx(universe=0, data=frame)
        server._handle_dmx(universe=0, data=frame)

        self.assertEqual(ctrl.post_step_motion_by.call_count, 1)
        ctrl.servo_on.assert_called_once()


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


class TestSafetyDefaults(unittest.TestCase):
    """The defaults are the safe ones: universe 1 (not the 0 other DMX gear
    usually listens on), channels 10-12 off, loss-of-signal stop on."""

    def test_default_universe_is_1(self):
        server = ArtNetInputServer(MagicMock())
        self.assertEqual(server.universe, 1)

    def test_default_signal_timeout_is_on(self):
        self.assertGreater(ArtNetInputServer(MagicMock()).signal_timeout_s, 0)

    def test_dangerous_channels_are_ignored_by_default(self):
        ctrl = MagicMock()
        server = ArtNetInputServer(ctrl, universe=0)
        server._handle_dmx(0, bytes([0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255]))
        ctrl.initial_abs_home.assert_not_called()
        ctrl.set_home_position.assert_not_called()
        ctrl.write_PA29_Initial_Abs_Pos.assert_not_called()

    def test_non_dangerous_channels_still_work_by_default(self):
        ctrl = MagicMock()
        server = ArtNetInputServer(ctrl, universe=0)
        server._handle_dmx(0, bytes([0, 0, 0, 0, 0, 0, 0, 200, 255, 0, 0, 0]))
        ctrl.servo_on.assert_called_once()
        ctrl.clear_alarm_12.assert_called_once()

    def test_dangerous_channels_work_once_enabled(self):
        ctrl = MagicMock()
        server = ArtNetInputServer(ctrl, universe=0, enable_dangerous_channels=True)
        server._handle_dmx(0, bytes([0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255]))
        ctrl.initial_abs_home.assert_called_once()
        ctrl.set_home_position.assert_called_once()
        ctrl.write_PA29_Initial_Abs_Pos.assert_called_once()

    def test_monitor_marks_ignored_dangerous_channels(self):
        server = ArtNetInputServer(MagicMock(), universe=0)
        server._handle_dmx(0, bytes([0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0, 0]))
        ch10 = server.get_channel_snapshot()["channels"][9]
        self.assertIn("IGNORED", ch10["interpreted"])

    def test_the_module_never_sends_anything(self):
        """Pure receiver: no ArtPollReply, no echo, nothing that could
        disturb other devices on the network."""
        import inspect
        import artnet_server
        source = inspect.getsource(artnet_server)
        for forbidden in ("sendto", ".send(", "SO_BROADCAST"):
            self.assertNotIn(forbidden, source)

    def test_socket_does_not_set_reuseaddr(self):
        import inspect
        import artnet_server
        self.assertNotIn("setsockopt", inspect.getsource(artnet_server.ArtNetInputServer.start))


class TestSignalWatchdog(unittest.TestCase):
    """Continuous rotation must stop when frames stop arriving, and must not
    restart by itself when they come back."""

    def _running_server(self, timeout=2.0):
        server, ctrl = make_server(signal_timeout_s=timeout)
        server._handle_dmx(0, bytes([255, 200, 0]))  # rotating CW
        ctrl.speed_ctrl_action.reset_mock()
        ctrl.enable_speed_ctrl.reset_mock()
        return server, ctrl

    def test_stops_rotation_after_the_timeout(self):
        server, ctrl = self._running_server(timeout=2.0)
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 2.5)
        ctrl.speed_ctrl_action.assert_called_once_with(0)
        self.assertTrue(server.get_stats()["watchdog_tripped"])
        self.assertEqual(server.get_stats()["watchdog_trips"], 1)

    def test_does_nothing_before_the_timeout(self):
        server, ctrl = self._running_server(timeout=2.0)
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 1.5)
        ctrl.speed_ctrl_action.assert_not_called()

    def test_fires_only_once(self):
        server, ctrl = self._running_server()
        late = server._last_valid_frame_monotonic + 10
        server._check_signal_watchdog(now=late)
        server._check_signal_watchdog(now=late + 1)
        ctrl.speed_ctrl_action.assert_called_once_with(0)

    def test_idle_server_is_left_alone(self):
        # Nothing rotating (channel 1 = 0): silence is not an emergency.
        server, ctrl = make_server(signal_timeout_s=2.0)
        server._handle_dmx(0, bytes([0, 0, 0]))
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 60)
        ctrl.speed_ctrl_action.assert_not_called()

    def test_never_armed_before_the_first_frame(self):
        server, ctrl = make_server(signal_timeout_s=2.0)
        server._check_signal_watchdog(now=time.monotonic() + 60)
        ctrl.speed_ctrl_action.assert_not_called()

    def test_timeout_zero_disables_it(self):
        server, ctrl = self._running_server(timeout=0)
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 600)
        ctrl.speed_ctrl_action.assert_not_called()

    def test_does_not_restart_by_itself_when_the_signal_returns(self):
        server, ctrl = self._running_server()
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 10)
        ctrl.speed_ctrl_action.reset_mock()

        server._handle_dmx(0, bytes([255, 200, 0]))  # sender back, still "enabled"

        ctrl.enable_speed_ctrl.assert_not_called()
        ctrl.speed_ctrl_action.assert_not_called()

    def test_channel_1_zero_rearms_and_motion_can_start_again(self):
        server, ctrl = self._running_server()
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 10)
        server._handle_dmx(0, bytes([0, 0, 0]))          # explicit disable
        self.assertFalse(server.get_stats()["watchdog_tripped"])
        ctrl.enable_speed_ctrl.reset_mock()

        server._handle_dmx(0, bytes([255, 200, 0]))      # deliberate restart

        ctrl.enable_speed_ctrl.assert_called_once()

    def test_other_channels_keep_working_while_tripped(self):
        server, ctrl = self._running_server()
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 10)
        server._handle_dmx(0, bytes([255, 200, 255]))    # cancel channel goes high
        ctrl.cancel_continuous_reading.assert_called_once()

    def test_a_failing_stop_command_is_logged_not_raised(self):
        server, ctrl = self._running_server()
        ctrl.speed_ctrl_action.side_effect = RuntimeError("serial down")
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 10)  # must not raise
        self.assertTrue(server.get_stats()["watchdog_tripped"])

    def test_foreign_universe_traffic_does_not_keep_the_watchdog_quiet(self):
        server, ctrl = self._running_server(timeout=2.0)
        for _ in range(50):
            server._handle_dmx(7, bytes([1, 2, 3]))      # someone else's universe
        server._check_signal_watchdog(now=server._last_valid_frame_monotonic + 3)
        ctrl.speed_ctrl_action.assert_called_once_with(0)


class TestSourceAllowList(unittest.TestCase):

    def _packet(self):
        return build_artdmx_packet(0, bytes([0, 0, 0]))

    def test_any_source_accepted_when_no_list(self):
        server, ctrl = make_server()
        server._process_packet(self._packet(), "10.0.0.99")
        self.assertEqual(server.get_stats()["frames_ok"], 1)

    def test_listed_source_accepted(self):
        server, ctrl = make_server(allowed_sources=["10.0.0.5"])
        server._process_packet(self._packet(), "10.0.0.5")
        self.assertEqual(server.get_stats()["frames_ok"], 1)

    def test_unlisted_source_is_dropped_and_counted(self):
        server, ctrl = make_server(allowed_sources=["10.0.0.5"])
        server._process_packet(self._packet(), "10.0.0.6")
        self.assertEqual(server.get_stats()["frames_ok"], 0)
        self.assertEqual(server.get_stats()["dropped_source"], 1)
        self.assertIsNone(server.get_channel_snapshot())

    def test_dropped_source_cannot_trigger_anything(self):
        server, ctrl = make_server(allowed_sources=["10.0.0.5"])
        packet = build_artdmx_packet(0, bytes([255, 200, 0, 0, 0, 0, 0, 200, 255, 255, 255, 255]))
        server._process_packet(packet, "10.0.0.66")
        self.assertEqual(ctrl.method_calls, [])


class TestUniverseFilterAndStats(unittest.TestCase):

    def test_other_universe_is_dropped_before_any_handling_and_counted(self):
        server, ctrl = make_server(universe=1)
        server._process_packet(build_artdmx_packet(0, bytes([255, 200, 0])), "10.0.0.5")
        self.assertEqual(server.get_stats()["dropped_universe"], 1)
        self.assertEqual(ctrl.method_calls, [])
        self.assertIsNone(server.get_channel_snapshot())

    def test_full_15_bit_port_address_is_compared(self):
        # Universe 1 with a non-zero Net byte is a different port address.
        server, ctrl = make_server(universe=1)
        server._process_packet(build_artdmx_packet(0x0101, bytes([255, 200, 0])), "10.0.0.5")
        self.assertEqual(server.get_stats()["frames_ok"], 0)

    def test_frame_rate_is_estimated_from_recent_frames(self):
        server, _ = make_server()
        base = time.monotonic()
        server._frame_times.extend(base - 1.0 + i * 0.1 for i in range(11))  # 10 fps, ending now
        server._last_valid_frame_monotonic = server._frame_times[-1]
        fps = server.get_stats()["frames_per_s"]
        self.assertAlmostEqual(fps, 10.0, places=3)

    def test_stats_report_configuration(self):
        server, _ = make_server(signal_timeout_s=3, allowed_sources=["10.0.0.5"])
        stats = server.get_stats()
        self.assertEqual(stats["signal_timeout_s"], 3)
        self.assertEqual(stats["allowed_sources"], ["10.0.0.5"])
        self.assertTrue(stats["dangerous_channels_enabled"])


class TestSequenceNumbers(unittest.TestCase):

    def _send(self, server, sequence, first_channel=0):
        packet = build_artdmx_packet(0, bytes([first_channel, 0, 0]), sequence=sequence)
        server._process_packet(packet, "10.0.0.5")

    def test_in_order_frames_accepted(self):
        server, _ = make_server()
        for seq in (1, 2, 3):
            self._send(server, seq)
        self.assertEqual(server.get_stats()["frames_ok"], 3)

    def test_a_reordered_older_frame_is_dropped_and_does_not_overwrite(self):
        server, _ = make_server()
        self._send(server, 10, first_channel=0)
        self._send(server, 9, first_channel=0)    # arrives late
        self.assertEqual(server.get_stats()["dropped_sequence"], 1)
        self.assertEqual(server.get_stats()["frames_ok"], 1)

    def test_sequence_zero_means_disabled_and_is_always_accepted(self):
        server, _ = make_server()
        self._send(server, 50)
        self._send(server, 0)
        self.assertEqual(server.get_stats()["frames_ok"], 2)

    def test_counter_wraparound_is_handled(self):
        server, _ = make_server()
        self._send(server, 254)
        self._send(server, 255)
        self._send(server, 1)                      # wrapped past 255 (0 is skipped)
        self.assertEqual(server.get_stats()["frames_ok"], 3)

    def test_a_restarted_sender_is_accepted_after_a_quiet_second(self):
        server, _ = make_server()
        self._send(server, 100)
        server._last_sequence_time -= 2.0          # >1s of silence
        self._send(server, 1)                      # counter restarted
        self.assertEqual(server.get_stats()["frames_ok"], 2)


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

    def _send_udp(self, port, packet):
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(packet, ("127.0.0.1", port))
        finally:
            sender.close()

    def _wait_for(self, condition, seconds=3.0):
        deadline = time.time() + seconds
        while not condition() and time.time() < deadline:
            time.sleep(0.02)
        return condition()

    def test_real_udp_signal_loss_stops_rotation_end_to_end(self):
        """Real socket + background thread: a sender that starts a rotation
        and then goes silent must get the rotation stopped by the watchdog."""
        server, ctrl = make_server(universe=0, signal_timeout_s=0.5)
        server.start()
        try:
            port = server._sock.getsockname()[1]
            self._send_udp(port, build_artdmx_packet(0, bytes([255, 200, 0])))
            self.assertTrue(self._wait_for(lambda: ctrl.enable_speed_ctrl.called))
            # (the initial CW command is also a speed_ctrl_action call, so
            # wait for the specific stop)
            self.assertTrue(self._wait_for(lambda: call(0) in ctrl.speed_ctrl_action.call_args_list))
        finally:
            server.stop()

    def test_real_udp_stream_of_foreign_universe_does_not_defeat_the_watchdog(self):
        server, ctrl = make_server(universe=0, signal_timeout_s=0.5)
        server.start()
        try:
            port = server._sock.getsockname()[1]
            self._send_udp(port, build_artdmx_packet(0, bytes([255, 200, 0])))
            self.assertTrue(self._wait_for(lambda: ctrl.enable_speed_ctrl.called))
            deadline = time.time() + 3
            while call(0) not in ctrl.speed_ctrl_action.call_args_list and time.time() < deadline:
                self._send_udp(port, build_artdmx_packet(5, bytes([1, 2, 3])))  # never our universe
                time.sleep(0.05)
            self.assertIn(call(0), ctrl.speed_ctrl_action.call_args_list)
        finally:
            server.stop()

    def test_binds_to_the_configured_address_only(self):
        server, ctrl = make_server(universe=0)
        server.listen_ip = "127.0.0.1"
        server.start()
        try:
            self.assertEqual(server._sock.getsockname()[0], "127.0.0.1")
        finally:
            server.stop()

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


def relative_frame(delta_centideg, duration_centisec, trigger=255):
    """16-channel frame: trigger on channel 4, relative angle on 13-14 (32768 =
    no move), duration on 15-16; channels 5-7 hold values that must be ignored."""
    angle_raw = delta_centideg + 32768
    return bytes([0, 0, 0, trigger, 200, 200, 200, 0, 0, 0, 0, 0,
                  angle_raw >> 8, angle_raw & 0xFF,
                  duration_centisec >> 8, duration_centisec & 0xFF])


def make_geared_server(gear_ratio=30, **kwargs):
    """Server whose mock controller has a real profile's pulse constants."""
    server, ctrl = make_server(**kwargs)
    ctrl.base_pulse_per_degree = 4194304 * gear_ratio / 360
    ctrl.profile = {"encoder_pulses_per_rev": 4194304}
    return server, ctrl


class TestRelativeMoveByAngleInTime(unittest.TestCase):
    """Channels 13-16: "move BY this angle in this many seconds" -- rpm is
    calculated from pulses, then handed to the same post_step_motion_by()."""

    def test_rpm_is_derived_from_angle_and_time(self):
        server, ctrl = make_geared_server(max_speed_rpm=1000, acc_time=5000)
        # 90 deg of output shaft at 30:1 = 7.5 motor revolutions in 3 s = 150 rpm
        server._handle_dmx(universe=0, data=relative_frame(9000, 300))
        ctrl.post_step_motion_by.assert_called_once_with(90.0, 5000, 150, relative=True)

    def test_gear_ratio_is_taken_into_account(self):
        server, ctrl = make_geared_server(gear_ratio=10, max_speed_rpm=1000)
        # 90 deg at 10:1 = 2.5 revolutions in 3 s = 50 rpm
        server._handle_dmx(universe=0, data=relative_frame(9000, 300))
        self.assertEqual(ctrl.post_step_motion_by.call_args[0][2], 50)

    def test_negative_angle_moves_the_other_way_at_the_same_rpm(self):
        server, ctrl = make_geared_server(max_speed_rpm=1000)
        server._handle_dmx(universe=0, data=relative_frame(-9000, 300))
        ctrl.post_step_motion_by.assert_called_once_with(-90.0, 5000, 150, relative=True)

    def test_channels_5_to_7_are_ignored_when_a_time_is_given(self):
        server, ctrl = make_geared_server(max_speed_rpm=1000)
        server._handle_dmx(universe=0, data=relative_frame(3600, 200))
        (angle, _acc, rpm), kwargs = ctrl.post_step_motion_by.call_args
        self.assertEqual((angle, rpm, kwargs), (36.0, 90, {"relative": True}))

    def test_rpm_is_capped_at_max_speed_rpm(self):
        server, ctrl = make_geared_server(max_speed_rpm=100)
        server._handle_dmx(universe=0, data=relative_frame(9000, 100))  # would need 450 rpm
        self.assertEqual(ctrl.post_step_motion_by.call_args[0][2], 100)

    def test_tiny_move_over_a_long_time_is_at_least_1_rpm(self):
        server, ctrl = make_geared_server()
        server._handle_dmx(universe=0, data=relative_frame(1, 65535))
        self.assertEqual(ctrl.post_step_motion_by.call_args[0][2], 1)

    def test_zero_time_falls_back_to_the_absolute_channels_5_to_7(self):
        server, ctrl = make_geared_server(max_speed_rpm=100)
        server._handle_dmx(universe=0, data=relative_frame(9000, 0))
        # channels 5-6 = 0xC8C8, channel 7 = 200 -- the legacy path, no relative kwarg
        args, kwargs = ctrl.post_step_motion_by.call_args
        self.assertEqual(kwargs, {})
        self.assertEqual(args[2], round(200 / 255 * 100))

    def test_short_frame_without_channels_13_to_16_uses_the_legacy_path(self):
        server, ctrl = make_geared_server()
        server._handle_dmx(universe=0, data=bytes([0, 0, 0, 255, 0, 0, 0]))
        self.assertEqual(ctrl.post_step_motion_by.call_args[1], {})

    def test_zero_move_sends_nothing(self):
        server, ctrl = make_geared_server()
        server._handle_dmx(universe=0, data=relative_frame(0, 300))
        ctrl.post_step_motion_by.assert_not_called()

    def test_zero_time_never_divides_by_zero(self):
        server, _ = make_geared_server()
        with self.assertRaises(ValueError):
            server.calculate_move_rpm(9000, 0)

    def test_fires_once_per_rising_edge(self):
        server, ctrl = make_geared_server()
        for _ in range(5):
            server._handle_dmx(universe=0, data=relative_frame(3600, 200))
        ctrl.post_step_motion_by.assert_called_once()
        server._handle_dmx(universe=0, data=relative_frame(3600, 200, trigger=0))
        server._handle_dmx(universe=0, data=relative_frame(3600, 200))
        self.assertEqual(ctrl.post_step_motion_by.call_count, 2)

    def test_a_refused_move_is_not_retried_every_frame(self):
        server, ctrl = make_geared_server()
        ctrl.post_step_motion_by.side_effect = ValueError("position unreadable")
        for _ in range(5):
            server._handle_dmx(universe=0, data=relative_frame(3600, 200))
        ctrl.post_step_motion_by.assert_called_once()

    def test_snapshot_shows_channels_13_to_16_and_the_calculated_rpm(self):
        server, _ = make_geared_server(max_speed_rpm=1000)
        server._handle_dmx(universe=0, data=relative_frame(9000, 300))
        channels = server.get_channel_snapshot()["channels"]
        self.assertEqual([c["channel"] for c in channels], list(range(1, 17)))
        self.assertIn("+90.00 deg", channels[12]["interpreted"])
        self.assertIn("3 s", channels[14]["interpreted"])
        self.assertIn("150 rpm", channels[15]["interpreted"])


if __name__ == "__main__":
    unittest.main()
