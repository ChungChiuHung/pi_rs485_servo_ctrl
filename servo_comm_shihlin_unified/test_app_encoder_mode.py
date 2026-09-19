"""
Endpoint tests for /encoder_mode (PA28) -- the confirm gating, the safety
refusals, and the "no adoption until the user says they power-cycled" rule.

app.py opens a real serial port at import time, so SerialPortManager is
replaced with a stub and ServoController's connect-time PA28 read is
neutralised BEFORE importing it; the module-level servo_ctrller is then
swapped for a MagicMock per test. Nothing here touches hardware.
"""
import unittest
from unittest.mock import MagicMock, patch

import serial_port_manager

_fake_manager_cls = MagicMock()
_fake_manager_cls.return_value.get_serial_instance.return_value = MagicMock()

with patch.object(serial_port_manager, "SerialPortManager", _fake_manager_cls), \
        patch("servo_control.ServoController.refresh_encoder_mode", return_value=None):
    import app as app_module


class EncoderModeEndpointTests(unittest.TestCase):

    def setUp(self):
        self.ctrl = MagicMock()
        self.ctrl.reading_active = False
        self.ctrl.absolute_mode = False
        self.ctrl.abs_home_pos_absolute = None
        self.ctrl.read_PA28_Encoder_Mode.return_value = 0
        self.ctrl.write_PA28_Encoder_Mode.return_value = True
        patches = [
            patch.object(app_module, "servo_ctrller", self.ctrl),
            patch.object(app_module, "active_input_server", None),
            patch.object(app_module, "_encoder_mode_pending_power_cycle", None),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.client = app_module.app.test_client()

    def test_get_reports_pa28_and_active_mode_without_writing(self):
        body = self.client.get("/encoder_mode").get_json()
        self.assertEqual(body["pa28"], 0)
        self.assertFalse(body["active_absolute_mode"])
        self.ctrl.write_PA28_Encoder_Mode.assert_not_called()

    def test_get_includes_absolute_health_flags_only_when_configured_absolute(self):
        self.ctrl.read_PA28_Encoder_Mode.return_value = 1
        self.ctrl.read_PA31_Abs_Position_Status.return_value = 0b10
        body = self.client.get("/encoder_mode").get_json()
        self.assertEqual(body["absolute_status"], ["battery low voltage"])

    def test_post_without_confirm_is_rejected_and_writes_nothing(self):
        response = self.client.post("/encoder_mode", json={"absolute": True})
        self.assertEqual(response.status_code, 400)
        self.ctrl.write_PA28_Encoder_Mode.assert_not_called()

    def test_post_with_non_boolean_absolute_is_rejected(self):
        response = self.client.post("/encoder_mode", json={"absolute": "yes", "confirm": True})
        self.assertEqual(response.status_code, 400)
        self.ctrl.write_PA28_Encoder_Mode.assert_not_called()

    def test_post_refused_while_motion_is_running(self):
        self.ctrl.reading_active = True
        response = self.client.post("/encoder_mode", json={"absolute": True, "confirm": True})
        self.assertEqual(response.status_code, 409)
        self.ctrl.write_PA28_Encoder_Mode.assert_not_called()

    def test_post_refused_while_an_input_server_is_running(self):
        with patch.object(app_module, "active_input_server", "artnet"):
            response = self.client.post("/encoder_mode", json={"absolute": True, "confirm": True})
        self.assertEqual(response.status_code, 409)
        self.ctrl.write_PA28_Encoder_Mode.assert_not_called()

    def test_successful_post_writes_and_sets_pending_but_does_not_adopt(self):
        response = self.client.post("/encoder_mode", json={"absolute": True, "confirm": True})
        self.assertEqual(response.status_code, 200)
        self.ctrl.write_PA28_Encoder_Mode.assert_called_once_with(True)
        self.assertIn("Power-cycle", response.get_json()["message"])
        self.assertIs(app_module._encoder_mode_pending_power_cycle, True)
        self.ctrl.refresh_encoder_mode.assert_not_called()

    def test_unconfirmed_write_reports_error_and_no_pending_state(self):
        self.ctrl.write_PA28_Encoder_Mode.return_value = False
        response = self.client.post("/encoder_mode", json={"absolute": True, "confirm": True})
        self.assertEqual(response.status_code, 502)
        self.assertIsNone(app_module._encoder_mode_pending_power_cycle)

    def test_adopt_rereads_mode_and_clears_pending(self):
        self.ctrl.refresh_encoder_mode.return_value = True
        with patch.object(app_module, "_encoder_mode_pending_power_cycle", True):
            response = self.client.post("/encoder_mode/adopt")
            pending_after = app_module._encoder_mode_pending_power_cycle
        self.assertEqual(response.status_code, 200)
        self.assertTrue(response.get_json()["active_absolute_mode"])
        self.assertIsNone(pending_after)

    def test_adopt_unreadable_pa28_keeps_pending_and_errors(self):
        self.ctrl.refresh_encoder_mode.return_value = None
        with patch.object(app_module, "_encoder_mode_pending_power_cycle", True):
            response = self.client.post("/encoder_mode/adopt")
            pending_after = app_module._encoder_mode_pending_power_cycle
        self.assertEqual(response.status_code, 503)
        self.assertIs(pending_after, True)


if __name__ == "__main__":
    unittest.main()
