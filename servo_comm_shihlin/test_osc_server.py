"""
Tests for osc_server.py (servo_comm_shihlin). servo_ctrller is always a
MagicMock -- nothing here touches hardware, even the end-to-end test that starts
a real OSCInputServer on localhost and sends it real OSC packets (only the
network/dispatch layer is real). Run from inside this directory:
python -m unittest test_osc_server
"""
import time
import unittest
from unittest.mock import MagicMock

from pythonosc import udp_client

from osc_server import OSCInputServer


def make_server():
    ctrl = MagicMock()
    return OSCInputServer(ctrl, listen_ip="127.0.0.1", listen_port=0), ctrl


class ServoHandlerTests(unittest.TestCase):

    def test_1_turns_on_and_0_turns_off(self):
        server, ctrl = make_server()
        server._servo_handler(None, [], 1.0)
        ctrl.servo_on.assert_called_once()
        server._servo_handler(None, [], 0.0)
        ctrl.servo_off.assert_called_once()

    def test_a_repeated_value_is_ignored(self):
        server, ctrl = make_server()
        server._servo_handler(None, [], 1.0)
        server._servo_handler(None, [], 1.0)
        ctrl.servo_on.assert_called_once()

    def test_each_address_has_its_own_duplicate_slot(self):
        """osc.py used ONE slot for every address, so /servo 1.0 followed by
        /clear 1.0 silently dropped the /clear."""
        server, ctrl = make_server()
        server._servo_handler(None, [], 1.0)
        server._clear_handler(None, [], 1.0)
        ctrl.clear_alarm_12.assert_called_once()


class ClearAndHomeHandlerTests(unittest.TestCase):

    def test_clear_only_acts_on_1(self):
        server, ctrl = make_server()
        server._clear_handler(None, [], 0.5)
        ctrl.clear_alarm_12.assert_not_called()
        server._clear_handler(None, [], 1.0)
        ctrl.clear_alarm_12.assert_called_once()

    def test_back_home_and_set_home_only_act_on_1(self):
        server, ctrl = make_server()
        server._back_home_handler(None, [], 0.0)
        server._set_home_position_handler(None, [], 0.0)
        ctrl.initial_abs_home.assert_not_called()
        ctrl.set_home_position.assert_not_called()
        server._back_home_handler(None, [], 1.0)
        server._set_home_position_handler(None, [], 1.0)
        ctrl.initial_abs_home.assert_called_once()
        ctrl.set_home_position.assert_called_once()

    def test_exceptions_are_caught(self):
        server, ctrl = make_server()
        ctrl.initial_abs_home.side_effect = RuntimeError("comm failure")
        ctrl.set_home_position.side_effect = RuntimeError("comm failure")
        server._back_home_handler(None, [], 1.0)
        server._set_home_position_handler(None, [], 1.0)


class MoveHandlerTests(unittest.TestCase):

    def test_set_point_moves_to_the_angle(self):
        server, ctrl = make_server()
        server._set_point_handler(None, [], 45.0, 3000, 20)
        ctrl.post_step_motion_by.assert_called_once_with(45.0, 3000, 20)

    def test_set_point_2_is_the_same_move(self):
        server, ctrl = make_server()
        server._set_point_handler_2(None, [], 90.0, 3000, 20)
        ctrl.post_step_motion_by.assert_called_once_with(90.0, 3000, 20)

    def test_the_same_angle_twice_moves_once(self):
        server, ctrl = make_server()
        server._set_point_handler(None, [], 45.0, 3000, 20)
        server._set_point_handler(None, [], 45.0, 3000, 20)
        ctrl.post_step_motion_by.assert_called_once()

    def test_a_refused_move_is_logged_not_raised(self):
        """post_step_motion_by raises PositionUnavailableError when the real
        position cannot be read; the OSC thread must survive it."""
        server, ctrl = make_server()
        ctrl.post_step_motion_by.side_effect = ValueError("could not read the position")
        server._set_point_handler(None, [], 45.0, 3000, 20)


class PrModeHandlerTests(unittest.TestCase):

    def test_writes_the_requested_path(self):
        server, ctrl = make_server()
        server._pr_mode_ctrl_handler(None, [], 5.0)
        ctrl.write_PF82.assert_called_once_with(5.0)

    def test_an_invalid_path_is_caught(self):
        server, ctrl = make_server()
        ctrl.write_PF82.side_effect = ValueError("64~999 is prohibited")
        server._pr_mode_ctrl_handler(None, [], 100.0)


class LifecycleTests(unittest.TestCase):

    def test_start_twice_raises_and_stop_when_idle_is_a_noop(self):
        server, _ = make_server()
        server.stop()
        server.start()
        try:
            self.assertTrue(server.is_running)
            with self.assertRaises(RuntimeError):
                server.start()
        finally:
            server.stop()
        self.assertFalse(server.is_running)

    def test_every_address_reaches_its_handler_over_real_udp(self):
        server, ctrl = make_server()
        server.start()
        try:
            host, port = server._server.server_address
            client = udp_client.SimpleUDPClient(host, port)
            client.send_message("/servo", 1.0)
            client.send_message("/set_point", [30.0, 3000, 20])
            client.send_message("/set_point_2", [60.0, 3000, 20])
            client.send_message("/back_home", 1.0)
            client.send_message("/pr_step_path", 2)
            client.send_message("/set_home", 1.0)
            deadline = time.time() + 3
            while time.time() < deadline and not (
                    ctrl.servo_on.called and ctrl.post_step_motion_by.call_count >= 2
                    and ctrl.initial_abs_home.called and ctrl.write_PF82.called
                    and ctrl.set_home_position.called):
                time.sleep(0.02)
        finally:
            server.stop()
        ctrl.servo_on.assert_called_once()
        self.assertEqual(ctrl.post_step_motion_by.call_count, 2)
        ctrl.initial_abs_home.assert_called_once()
        ctrl.write_PF82.assert_called_once_with(2)
        ctrl.set_home_position.assert_called_once()


if __name__ == "__main__":
    unittest.main()
