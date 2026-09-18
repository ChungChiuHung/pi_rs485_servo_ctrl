"""
Tests for osc_server.py. servo_ctrller is always a MagicMock -- none of
this touches real hardware, even the end-to-end tests that start a real
OSCInputServer on localhost and send it real OSC packets (only the
network/dispatch layer is real; what it calls into is a mock).
"""
import time
import unittest
from unittest.mock import MagicMock, call

from pythonosc import udp_client

from osc_server import OSCInputServer


def make_server(feedback_ip=None, feedback_port=None):
    servo_ctrller = MagicMock()
    servo_ctrller.current_angle = 12.5
    server = OSCInputServer(
        servo_ctrller, listen_ip="127.0.0.1", listen_port=0,
        feedback_ip=feedback_ip, feedback_port=feedback_port
    )
    return server, servo_ctrller


class TestServoHandler(unittest.TestCase):

    def test_data_1_turns_servo_on(self):
        server, ctrl = make_server()
        server._servo_handler(None, [], 1.0)
        ctrl.servo_on.assert_called_once()
        ctrl.servo_off.assert_not_called()

    def test_data_0_turns_servo_off(self):
        server, ctrl = make_server()
        server._servo_handler(None, [], 0.0)
        ctrl.servo_off.assert_called_once()
        ctrl.servo_on.assert_not_called()

    def test_duplicate_message_is_ignored(self):
        server, ctrl = make_server()
        server._servo_handler(None, [], 1.0)
        server._servo_handler(None, [], 1.0)  # same value again
        ctrl.servo_on.assert_called_once()

    def test_different_value_after_duplicate_check_is_not_ignored(self):
        server, ctrl = make_server()
        server._servo_handler(None, [], 1.0)
        server._servo_handler(None, [], 0.0)
        ctrl.servo_on.assert_called_once()
        ctrl.servo_off.assert_called_once()


class TestClearHandler(unittest.TestCase):

    def test_calls_clear_alarm_12(self):
        server, ctrl = make_server()
        server._clear_handler(None)
        ctrl.clear_alarm_12.assert_called_once()


class TestContinuousMotionHandlers(unittest.TestCase):

    def test_set_continuous_motion_passes_int_speed_and_acc_time(self):
        server, ctrl = make_server()
        server._set_continuous_motion_handler(None, [], "100", "5000", True)
        ctrl.enable_speed_ctrl.assert_called_once_with(100, 5000, True)

    def test_ctrl_continuous_motion_start_cw(self):
        """Per docs/en_manual.txt:10380-10382 (JOG_OPERATION, 0x0904): 1 =
        forward rotation (CCW), 2 = reverse rotation (CW)."""
        server, ctrl = make_server()
        server._ctrl_continuous_motion_handler(None, [], "start", "CW")
        ctrl.speed_ctrl_action.assert_called_once_with(2)

    def test_ctrl_continuous_motion_start_ccw(self):
        """Regression test for a bug found in the original
        servo_comm_shihlin_50W/osc_2.py: after the if/elif that correctly
        picked the direction value, it unconditionally called
        speed_ctrl_action("CW") again -- so a CCW request would send CCW
        immediately followed by a bogus "CW" string call. Not ported
        forward; speed_ctrl_action must be called exactly once, with 1."""
        server, ctrl = make_server()
        server._ctrl_continuous_motion_handler(None, [], "start", "CCW")
        ctrl.speed_ctrl_action.assert_called_once_with(1)

    def test_ctrl_continuous_motion_stop(self):
        server, ctrl = make_server()
        server._ctrl_continuous_motion_handler(None, [], "stop", "CW")
        ctrl.speed_ctrl_action.assert_called_once_with(0)


class TestSetPointHandler(unittest.TestCase):

    def test_converts_types_and_forwards(self):
        server, ctrl = make_server()
        server._set_point_handler(None, [], "90.5", "3000", "20")
        ctrl.post_step_motion_by.assert_called_once_with(90.5, 3000, 20)


class TestCancelLoopHandler(unittest.TestCase):

    def test_calls_cancel_continuous_reading(self):
        server, ctrl = make_server()
        server._cancel_loop_handler(None)
        ctrl.cancel_continuous_reading.assert_called_once()


class TestFeedback(unittest.TestCase):

    def test_no_feedback_client_means_no_error_and_no_send(self):
        server, ctrl = make_server()  # feedback_ip=None
        server._send_feedback("/whatever", 1, 2)  # must not raise

    def test_feedback_sent_when_configured(self):
        server, ctrl = make_server(feedback_ip="127.0.0.1", feedback_port=0)
        server._osc_client = MagicMock()
        server._send_feedback("/servo_on", "on")
        server._osc_client.send_message.assert_called_once_with("/servo_on", ("on",))

    def test_feedback_send_exception_is_caught(self):
        server, ctrl = make_server(feedback_ip="127.0.0.1", feedback_port=0)
        server._osc_client = MagicMock()
        server._osc_client.send_message.side_effect = Exception("network down")
        server._send_feedback("/servo_on", "on")  # must not raise


class TestStartStopLifecycle(unittest.TestCase):
    """End-to-end over real UDP on localhost -- servo_ctrller is still a
    mock, so no hardware is touched, but the dispatcher/threading wiring is
    genuinely exercised."""

    def test_start_registers_listeners_and_stop_unregisters(self):
        server, ctrl = make_server()
        server.start()
        try:
            self.assertTrue(server.is_running)
            ctrl.register_event_listener.assert_any_call(
                "on_motion_completed", server._on_motion_completed
            )
            ctrl.register_event_listener.assert_any_call("on_moving", server._on_moving)
        finally:
            server.stop()

        self.assertFalse(server.is_running)
        ctrl.unregister_event_listener.assert_any_call(
            "on_motion_completed", server._on_motion_completed
        )
        ctrl.unregister_event_listener.assert_any_call("on_moving", server._on_moving)

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

    def test_real_osc_message_reaches_handler(self):
        server, ctrl = make_server()
        server.start()
        try:
            actual_port = server._server.server_address[1]
            client = udp_client.SimpleUDPClient("127.0.0.1", actual_port)
            try:
                client.send_message("/clear", [])

                deadline = time.time() + 2
                while not ctrl.clear_alarm_12.called and time.time() < deadline:
                    time.sleep(0.01)

                ctrl.clear_alarm_12.assert_called_once()
            finally:
                client._sock.close()
        finally:
            server.stop()


if __name__ == "__main__":
    unittest.main()
