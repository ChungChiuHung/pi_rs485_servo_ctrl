"""
Tests for the POS TEST START CW/CCW web-UI action -- the fixed-size nudge
(via post_step_motion_by()'s relative=True path) that replaced the old bare
0x0907 trigger (pos_step_motion_test()) after real-hardware testing
2026-09-22 found that trigger only reliably moved the motor on the first
press after ENABLE POS MODE. A second bare press left reading_active stuck
True with the encoder barely moving -- consistent with the drive's own
documented behavior (see pos_step_motion_test()'s comment: any
communication gap over 1s auto-exits positioning-test mode) combined with
the software's own keep-alive polling stopping once the first move's
completion was detected, so nothing re-entered the mode before the second
bare trigger arrived.

Mocked serial port / app; nothing touches hardware. Run from inside this
directory: python -m unittest test_pos_test_start_action
"""
import unittest
from unittest.mock import MagicMock

from servo_control import PositionUnavailableError, MoveOutOfRangeError
from test_app_no_serial import load_app


class PosTestStartActionTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.ctrl = self.app_module.servo_ctrller
        self.ctrl.post_step_motion_by = MagicMock()

    def action(self, name, **body):
        return self.client.post("/action", json={"action": name, **body})

    def test_cw_moves_forward_by_the_default_half_degree(self):
        response = self.action("posTestStart_CW")
        self.assertEqual(response.status_code, 200)
        self.ctrl.post_step_motion_by.assert_called_once_with(
            0.5, acc_dec_time=200, speed_rpm=10, relative=True)

    def test_ccw_moves_backward_by_the_default_half_degree(self):
        response = self.action("posTestStart_CCW")
        self.assertEqual(response.status_code, 200)
        self.ctrl.post_step_motion_by.assert_called_once_with(
            -0.5, acc_dec_time=200, speed_rpm=10, relative=True)

    def test_custom_degrees_and_speed_are_forwarded(self):
        response = self.action("posTestStart_CW", degrees=2.5, speed_rpm=15)
        self.assertEqual(response.status_code, 200)
        self.ctrl.post_step_motion_by.assert_called_once_with(
            2.5, acc_dec_time=200, speed_rpm=15, relative=True)

    def test_invalid_degrees_is_refused(self):
        response = self.action("posTestStart_CW", degrees=0)
        self.assertEqual(response.status_code, 400)
        self.ctrl.post_step_motion_by.assert_not_called()

    def test_invalid_speed_is_refused(self):
        response = self.action("posTestStart_CW", speed_rpm=5000)
        self.assertEqual(response.status_code, 400)
        self.ctrl.post_step_motion_by.assert_not_called()

    def test_unreadable_position_is_refused_with_502(self):
        self.ctrl.post_step_motion_by.side_effect = PositionUnavailableError("no position")
        response = self.action("posTestStart_CW")
        self.assertEqual(response.status_code, 502)

    def test_move_out_of_range_is_refused_with_400(self):
        self.ctrl.post_step_motion_by.side_effect = MoveOutOfRangeError("too far")
        response = self.action("posTestStart_CW")
        self.assertEqual(response.status_code, 400)


if __name__ == "__main__":
    unittest.main()
