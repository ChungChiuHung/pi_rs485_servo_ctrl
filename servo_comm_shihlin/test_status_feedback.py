"""
Tests for the optional TouchDesigner-style feedback mechanism: ServoController's
on_alarm event (servo_control.py) and OSCInputServer's feedback wiring
(osc_server.py). Everything here is mocked -- nothing touches hardware or a
real socket. Run from inside this directory: python -m unittest test_status_feedback
"""
import unittest
from unittest.mock import MagicMock, patch

from servo_control import ServoController
from osc_server import OSCInputServer


def make_controller():
    fake_serial = MagicMock()
    fake_serial.keep_running = True
    ctrl = ServoController(fake_serial)
    ctrl.modbus_client = MagicMock()
    return ctrl


def alarm_read_reply(code: int) -> bytes:
    """A Modbus ASCII read reply for the 1-word (2-byte) alarm register."""
    return f":010302{code:04X}00\r\n".encode()


class AlarmEventTests(unittest.TestCase):

    def test_a_successful_read_notifies_on_alarm_with_the_code(self):
        ctrl = make_controller()
        ctrl.modbus_client.send_and_receive.return_value = alarm_read_reply(0x12)
        seen = []
        ctrl.register_event_listener("on_alarm", lambda code: seen.append(code))
        result = ctrl.read_current_alarm_code()
        self.assertEqual(result, 0x12)
        self.assertEqual(seen, [0x12])

    def test_a_failed_read_does_not_notify(self):
        ctrl = make_controller()
        ctrl.modbus_client.send_and_receive.return_value = None
        seen = []
        ctrl.register_event_listener("on_alarm", lambda code: seen.append(code))
        result = ctrl.read_current_alarm_code()
        self.assertIsNone(result)
        self.assertEqual(seen, [])

    def test_a_raising_listener_is_logged_not_crashed(self):
        """Regression test: _notify_event_listeners used to reference an
        undefined 'event' name in its except branch, so a raising callback
        crashed with NameError instead of being logged."""
        ctrl = make_controller()
        ctrl.modbus_client.send_and_receive.return_value = alarm_read_reply(0x12)
        ctrl.register_event_listener("on_alarm", lambda code: (_ for _ in ()).throw(RuntimeError("boom")))
        with self.assertLogs("servo_control", level="ERROR") as log:
            result = ctrl.read_current_alarm_code()
        self.assertEqual(result, 0x12)
        self.assertIn("on_alarm", log.output[0])


def make_server(feedback_ip=None, feedback_port=None):
    ctrl = MagicMock()
    server = OSCInputServer(ctrl, listen_ip="127.0.0.1", listen_port=0,
                             feedback_ip=feedback_ip, feedback_port=feedback_port)
    return server, ctrl


class FeedbackDisabledTests(unittest.TestCase):

    def test_feedback_is_off_by_default(self):
        server, ctrl = make_server()
        self.assertFalse(server.feedback_enabled)
        server._servo_handler(None, [], 1.0)
        ctrl.register_event_listener.assert_not_called()

    def test_send_feedback_is_a_noop_without_a_client(self):
        server, _ = make_server()
        server._send_feedback("/servo_on", "on")  # must not raise


class FeedbackEnabledTests(unittest.TestCase):

    def setUp(self):
        patcher = patch("osc_server.udp_client.SimpleUDPClient")
        self.mock_client_cls = patcher.start()
        self.addCleanup(patcher.stop)
        self.mock_client = self.mock_client_cls.return_value

    def test_start_wires_listeners_and_stop_unwires_them(self):
        server, ctrl = make_server(feedback_ip="10.0.0.5", feedback_port=5008)
        server.start()
        try:
            self.mock_client_cls.assert_called_once_with("10.0.0.5", 5008)
            registered = {call.args[0] for call in ctrl.register_event_listener.call_args_list}
            self.assertEqual(registered, {"on_motion_completed", "on_moving", "on_alarm"})
        finally:
            server.stop()
        unregistered = {call.args[0] for call in ctrl.unregister_event_listener.call_args_list}
        self.assertEqual(unregistered, {"on_motion_completed", "on_moving", "on_alarm"})

    def test_servo_on_sends_feedback(self):
        server, ctrl = make_server(feedback_ip="10.0.0.5", feedback_port=5008)
        server.start()
        try:
            server._servo_handler(None, [], 1.0)
        finally:
            server.stop()
        self.mock_client.send_message.assert_any_call("/servo_on", ["on"])

    def test_a_handler_exception_sends_error_feedback(self):
        server, ctrl = make_server(feedback_ip="10.0.0.5", feedback_port=5008)
        ctrl.servo_on.side_effect = RuntimeError("comm failure")
        server.start()
        try:
            server._servo_handler(None, [], 1.0)
        finally:
            server.stop()
        sent_addresses = [call.args[0] for call in self.mock_client.send_message.call_args_list]
        self.assertIn("/error", sent_addresses)

    def test_a_dropped_feedback_packet_does_not_raise(self):
        server, _ = make_server(feedback_ip="10.0.0.5", feedback_port=5008)
        self.mock_client.send_message.side_effect = OSError("network down")
        server.start()
        try:
            server._on_moving_feedback(12.3)  # must not raise
        finally:
            server.stop()

    def test_clear_back_home_set_home_and_pr_step_path_echo_on_success(self):
        server, ctrl = make_server(feedback_ip="10.0.0.5", feedback_port=5008)
        server.start()
        try:
            server._clear_handler(None, [], 1.0)
            server._back_home_handler(None, [], 1.0)
            server._set_home_position_handler(None, [], 1.0)
            server._pr_mode_ctrl_handler(None, [], 5.0)
        finally:
            server.stop()
        self.mock_client.send_message.assert_any_call("/clear", ["cleared"])
        self.mock_client.send_message.assert_any_call("/back_home", ["back_home"])
        self.mock_client.send_message.assert_any_call("/set_home_position", ["set_home_position"])
        self.mock_client.send_message.assert_any_call("/pr_step_path", [5.0])


if __name__ == "__main__":
    unittest.main()
