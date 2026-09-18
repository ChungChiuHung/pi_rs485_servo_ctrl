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

    def test_low_range_is_ccw(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 50, 0]))
        ctrl.speed_ctrl_action.assert_called_with(2)

    def test_high_range_is_cw(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))
        ctrl.speed_ctrl_action.assert_called_with(1)

    def test_direction_change_only_fires_on_change(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))  # CW
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))  # same CW again
        ctrl.speed_ctrl_action.assert_called_once_with(1)

    def test_direction_change_from_cw_to_ccw_fires_again(self):
        server, ctrl = make_server()
        server._handle_dmx(universe=0, data=bytes([200, 200, 0]))  # CW
        server._handle_dmx(universe=0, data=bytes([200, 50, 0]))   # CCW
        self.assertEqual(ctrl.speed_ctrl_action.call_count, 2)
        ctrl.speed_ctrl_action.assert_called_with(2)


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
