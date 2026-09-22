"""
Tests for ENABLE POS MODE / ENABLE SPEED CONTROL MODE mutual exclusion,
added 2026-09-22: the drive can only be latched into one CTRL_MODE_SEL test
mode at a time (0x0901: 0=idle, 3=JOG, 4=Positioning), so arming one while
the other is already active would silently conflict. app.py's
_reject_if_other_mode_active() rejects with 409 based on a fresh
read_test_mode_0x0901() read, rather than trusting client-side state alone.

Mocked serial port / app; nothing touches hardware. Run from inside this
directory: python -m unittest test_mode_mutual_exclusion
"""
import unittest
from unittest.mock import MagicMock

from test_app_no_serial import load_app


class ModeMutualExclusionTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.ctrl = self.app_module.servo_ctrller
        self.ctrl.read_test_mode_0x0901 = MagicMock()
        self.ctrl.Enable_Position_Mode = MagicMock()
        self.ctrl.enable_speed_ctrl = MagicMock()
        self.ctrl.clear_alarm_12 = MagicMock()
        self.ctrl.config_acc_dec_0x0902 = MagicMock()
        self.ctrl.config_speed_0x0903 = MagicMock()
        self.ctrl.config_pulses_0x0905_low_byte = MagicMock()
        self.ctrl.config_pulses_0x0906_high_byte = MagicMock()
        self.ctrl.start_continuous_reading = MagicMock()

    def action(self, name, **body):
        return self.client.post("/action", json={"action": name, **body})

    def test_enable_pos_mode_refused_while_jog_is_active(self):
        self.ctrl.read_test_mode_0x0901.return_value = 3  # JOG
        response = self.action("enablePosMode")
        self.assertEqual(response.status_code, 409)
        self.assertIn("Speed Control (JOG)", response.get_json()["message"])
        self.ctrl.Enable_Position_Mode.assert_not_called()
        self.ctrl.clear_alarm_12.assert_not_called()

    def test_enable_pos_mode_allowed_when_idle(self):
        self.ctrl.read_test_mode_0x0901.return_value = 0  # idle
        response = self.action("enablePosMode")
        self.assertEqual(response.status_code, 200)
        self.ctrl.Enable_Position_Mode.assert_called_once_with(True)

    def test_enable_pos_mode_allowed_when_mode_read_fails(self):
        """Fails open: an unreadable CTRL_MODE_SEL must not permanently
        lock both toggles out over one flaky read."""
        self.ctrl.read_test_mode_0x0901.return_value = None
        response = self.action("enablePosMode")
        self.assertEqual(response.status_code, 200)
        self.ctrl.Enable_Position_Mode.assert_called_once_with(True)

    def test_enable_speed_ctrl_mode_refused_while_positioning_is_active(self):
        self.ctrl.read_test_mode_0x0901.return_value = 4  # Positioning
        response = self.action("enableSpeedCtrlMode")
        self.assertEqual(response.status_code, 409)
        self.assertIn("Position Mode", response.get_json()["message"])
        self.ctrl.enable_speed_ctrl.assert_not_called()

    def test_enable_speed_ctrl_mode_allowed_when_idle(self):
        self.ctrl.read_test_mode_0x0901.return_value = 0  # idle
        response = self.action("enableSpeedCtrlMode")
        self.assertEqual(response.status_code, 200)
        self.ctrl.enable_speed_ctrl.assert_called_once()

    def test_enable_speed_ctrl_mode_uses_a_short_acc_time_not_the_5s_default(self):
        """Regression coverage for the 2026-09-22 report "released the
        arrow key but the motor kept turning": enable_speed_ctrl()'s own
        default acc_time is 5000ms (a smooth ramp), but the web UI's
        press-and-hold arrow keys need release to feel immediate -- this
        action must pass a short acc_time explicitly, not rely on that
        default."""
        self.ctrl.read_test_mode_0x0901.return_value = 0  # idle
        self.action("enableSpeedCtrlMode", speed_rpm=50)
        self.ctrl.enable_speed_ctrl.assert_called_once_with(50, acc_time=self.app_module.JOG_ACC_DEC_MS)
        self.assertLess(self.app_module.JOG_ACC_DEC_MS, 1000)

    def test_enable_speed_ctrl_mode_allowed_when_mode_read_fails(self):
        self.ctrl.read_test_mode_0x0901.return_value = None
        response = self.action("enableSpeedCtrlMode")
        self.assertEqual(response.status_code, 200)
        self.ctrl.enable_speed_ctrl.assert_called_once()

    def test_motion_cancel_is_not_blocked_by_the_guard(self):
        """The guard only covers arming a mode, not turning one off --
        MOTION CANCEL (the merged toggle's "off" path) must always work
        regardless of what CTRL_MODE_SEL currently reads."""
        self.ctrl.read_test_mode_0x0901.return_value = 3
        self.ctrl.stop_continuous_reading = MagicMock()
        response = self.action("motionCancel")
        self.assertEqual(response.status_code, 200)

    def test_disable_pos_mode_is_not_blocked_by_the_guard(self):
        self.ctrl.read_test_mode_0x0901.return_value = 3
        self.ctrl.stop_continuous_reading = MagicMock()
        response = self.action("disablePosMode")
        self.assertEqual(response.status_code, 200)


if __name__ == "__main__":
    unittest.main()
