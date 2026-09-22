"""
Tests for ServoController.pos_test_step() and the POS TEST START CW/CCW
web-UI action -- the fixed-size nudge that replaced the old bare 0x0907
trigger (pos_step_motion_test()) after real-hardware testing 2026-09-22
found that trigger only worked once per ENABLE POS MODE click.

Mocked serial port / app; nothing touches hardware. Run from inside this
directory: python -m unittest test_pos_test_step
"""
import unittest
from unittest.mock import MagicMock, patch

from servo_control import ServoController, PositionUnavailableError
from test_app_web import load_app

BASE_PULSE_PER_DEGREE = 349525.3333333333


def make_controller():
    fake_serial = MagicMock()
    fake_serial.keep_running = True
    ctrl = ServoController(fake_serial)
    ctrl.modbus_client = MagicMock()
    return ctrl


class PosTestStepTests(unittest.TestCase):

    def test_cw_moves_forward_by_the_requested_degrees(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 0
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)

        ctrl.pos_test_step(cw=True, degrees=0.5)

        ctrl._execute_positioning.assert_called_once()
        target_encoder = ctrl._execute_positioning.call_args[0][0]
        self.assertEqual(target_encoder, int(BASE_PULSE_PER_DEGREE * 0.5))

    def test_ccw_moves_backward_by_the_requested_degrees(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 0
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)

        ctrl.pos_test_step(cw=False, degrees=0.5)

        ctrl._execute_positioning.assert_called_once()
        target_encoder = ctrl._execute_positioning.call_args[0][0]
        self.assertEqual(target_encoder, -int(BASE_PULSE_PER_DEGREE * 0.5))

    def test_steps_from_wherever_the_drive_currently_is_not_from_zero(self):
        """A second press must move another 0.5deg from the NEW position,
        not repeat the same absolute target -- the bug this whole feature
        replaced (a bare trigger with stale configuration) effectively
        froze the target the second time around."""
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 0
        ctrl.read_encoder_before_gear_ratio = MagicMock(
            return_value=round(10 * BASE_PULSE_PER_DEGREE))

        ctrl.pos_test_step(cw=True, degrees=0.5)

        target_encoder = ctrl._execute_positioning.call_args[0][0]
        expected = round(10 * BASE_PULSE_PER_DEGREE) + int(BASE_PULSE_PER_DEGREE * 0.5)
        self.assertEqual(target_encoder, expected)

    def test_unreadable_position_refuses_to_move(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)

        with self.assertRaises(PositionUnavailableError):
            ctrl.pos_test_step(cw=True)
        ctrl._execute_positioning.assert_not_called()

    def test_default_step_is_half_a_degree(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 0
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)

        ctrl.pos_test_step(cw=True)

        target_encoder = ctrl._execute_positioning.call_args[0][0]
        self.assertEqual(target_encoder, int(BASE_PULSE_PER_DEGREE * 0.5))


class PosTestStartActionTests(unittest.TestCase):
    """Through the actual /action endpoint, using test_app_web.py's own
    load_app() helper (a real ServoController, only its RS-485-touching
    methods mocked out)."""

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.ctrl = self.app_module.servo_ctrller
        self.ctrl.pos_test_step = MagicMock()

    def action(self, name, **body):
        with patch.object(self.ctrl, "write_PD_16_Enable_DI_Control"):
            return self.client.post("/action", json={"action": name, **body})

    def test_cw_calls_pos_test_step_with_defaults(self):
        response = self.action("posTestStart_CW")
        self.assertEqual(response.status_code, 200)
        self.ctrl.pos_test_step.assert_called_once_with(cw=True, degrees=0.5, speed_rpm=10)

    def test_ccw_calls_pos_test_step_with_cw_false(self):
        response = self.action("posTestStart_CCW")
        self.assertEqual(response.status_code, 200)
        self.ctrl.pos_test_step.assert_called_once_with(cw=False, degrees=0.5, speed_rpm=10)

    def test_custom_degrees_and_speed_are_forwarded(self):
        response = self.action("posTestStart_CW", degrees=2.5, speed_rpm=15)
        self.assertEqual(response.status_code, 200)
        self.ctrl.pos_test_step.assert_called_once_with(cw=True, degrees=2.5, speed_rpm=15)

    def test_invalid_degrees_is_refused(self):
        response = self.action("posTestStart_CW", degrees=0)
        self.assertEqual(response.status_code, 400)
        self.ctrl.pos_test_step.assert_not_called()

    def test_unreadable_position_is_refused_with_502(self):
        self.ctrl.pos_test_step.side_effect = PositionUnavailableError("no position")
        response = self.action("posTestStart_CW")
        self.assertEqual(response.status_code, 502)


if __name__ == "__main__":
    unittest.main()
