"""
Tests for the "jogSpeedAdjust" web-UI action -- the Up/Down arrow-key +/-1
rpm nudge of a running JOG speed (ServoController.change_jog_speed_by()),
added 2026-09-22 alongside the merged ENABLE SPEED CONTROL MODE / MOTION
CANCEL toggle button and Left/Right arrow-key MOTION START CW/CCW / MOTION
PAUSE keyboard shortcuts.

Mocked serial port / app; nothing touches hardware. Run from inside this
directory: python -m unittest test_jog_speed_adjust_action
"""
import unittest
from unittest.mock import MagicMock

from test_app_no_serial import load_app


class JogSpeedAdjustActionTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.ctrl = self.app_module.servo_ctrller
        self.ctrl.change_jog_speed_by = MagicMock(return_value=101)

    def action(self, name, **body):
        return self.client.post("/action", json={"action": name, **body})

    def test_default_delta_is_plus_one(self):
        response = self.action("jogSpeedAdjust")
        self.assertEqual(response.status_code, 200)
        self.ctrl.change_jog_speed_by.assert_called_once_with(1)
        self.assertEqual(response.get_json()["speed_rpm"], 101)

    def test_negative_delta_is_forwarded(self):
        response = self.action("jogSpeedAdjust", delta_rpm=-1)
        self.assertEqual(response.status_code, 200)
        self.ctrl.change_jog_speed_by.assert_called_once_with(-1)

    def test_custom_delta_is_forwarded(self):
        response = self.action("jogSpeedAdjust", delta_rpm=25)
        self.assertEqual(response.status_code, 200)
        self.ctrl.change_jog_speed_by.assert_called_once_with(25)

    def test_delta_out_of_range_is_refused(self):
        response = self.action("jogSpeedAdjust", delta_rpm=99999)
        self.assertEqual(response.status_code, 400)
        self.ctrl.change_jog_speed_by.assert_not_called()

    def test_jog_mode_not_active_is_refused_with_409(self):
        self.ctrl.change_jog_speed_by.side_effect = RuntimeError("JOG mode is not active")
        response = self.action("jogSpeedAdjust")
        self.assertEqual(response.status_code, 409)


if __name__ == "__main__":
    unittest.main()
