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


class EepromProtectionWiringTests(unittest.TestCase):
    """PD16/PD25 are EEPROM-backed and written by nearly every action; PA23
    protection must be verified BEFORE the first such write of each action."""

    def setUp(self):
        self.ctrl = MagicMock()
        self.ctrl.eeprom_protection = 2
        self.ctrl.absolute_mode = False
        self.ctrl.abs_home_pos_absolute = None
        self.ctrl.electronic_gear = None
        self.ctrl.electronic_gear_unity = None
        self.ctrl.home_set_since_start = False
        self.ctrl.modbus_client.last_sent = None
        self.ctrl.modbus_client.last_received = None
        self.ctrl.modbus_client.format_hex.return_value = ""
        patcher = patch.object(app_module, "servo_ctrller", self.ctrl)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.client = app_module.app.test_client()

    def test_action_verifies_protection_before_the_pd16_write(self):
        self.client.post("/action", json={"action": "servoOff"})
        called = [c[0] for c in self.ctrl.method_calls]
        self.assertIn("ensure_eeprom_write_protection", called)
        self.assertLess(called.index("ensure_eeprom_write_protection"),
                        called.index("write_PD_16_Enable_DI_Control"))

    def test_action_check_is_throttled(self):
        self.client.post("/action", json={"action": "servoOff"})
        kwargs = self.ctrl.ensure_eeprom_write_protection.call_args.kwargs
        self.assertEqual(kwargs.get("max_age_s"), app_module.EEPROM_GUARD_INTERVAL_S)

    def test_status_reports_cached_protection_without_extra_writes(self):
        self.ctrl.read_current_alarm_code.return_value = 255
        self.ctrl.read_servo_state.return_value = True
        self.ctrl.read_test_mode_0x0901.return_value = 0
        self.ctrl.reading_active = False
        self.ctrl.current_angle = 0.0
        self.ctrl.current_encoder = 0
        self.ctrl.set_point_1 = self.ctrl.set_point_2 = None
        self.ctrl.jog_speed_rpm = None
        manager = MagicMock()
        manager.get_connected_port.return_value = "COM_TEST"
        manager.get_baud_rate.return_value = 115200
        with patch.object(app_module, "serial_manager", manager):
            body = self.client.get("/status").get_json()
        self.assertEqual(body["eeprom_protection"], 2)
        self.ctrl.ensure_eeprom_write_protection.assert_not_called()


class HomeReminderAndSetHomeTests(unittest.TestCase):
    """The UI reminds the operator to SET HOME after every start (the
    incremental counter restarts at drive power-on)."""

    def setUp(self):
        self.ctrl = MagicMock()
        self.ctrl.reading_active = False
        self.ctrl.absolute_mode = False
        self.ctrl.abs_home_pos_absolute = None
        self.ctrl.eeprom_protection = 2
        self.ctrl.electronic_gear = (1, 1)
        self.ctrl.electronic_gear_unity = True
        self.ctrl.home_set_since_start = False
        self.ctrl.current_angle = 0.0
        self.ctrl.current_encoder = 0
        self.ctrl.set_point_1 = self.ctrl.set_point_2 = None
        self.ctrl.jog_speed_rpm = None
        self.ctrl.read_current_alarm_code.return_value = 255
        self.ctrl.read_servo_state.return_value = True
        self.ctrl.read_test_mode_0x0901.return_value = 0
        self.ctrl.modbus_client.format_hex.return_value = ""
        manager = MagicMock()
        manager.get_connected_port.return_value = "COM_TEST"
        manager.get_baud_rate.return_value = 115200
        for p in (patch.object(app_module, "servo_ctrller", self.ctrl),
                  patch.object(app_module, "serial_manager", manager)):
            p.start()
            self.addCleanup(p.stop)
        self.client = app_module.app.test_client()

    def status(self):
        return self.client.get("/status").get_json()

    def test_reminder_needed_until_home_is_set_this_session(self):
        body = self.status()
        self.assertFalse(body["home_set_since_start"])
        self.assertTrue(body["home_reminder_needed"])
        self.ctrl.home_set_since_start = True
        body = self.status()
        self.assertTrue(body["home_set_since_start"])
        self.assertFalse(body["home_reminder_needed"])

    def test_absolute_mode_with_a_saved_absolute_home_needs_no_reminder(self):
        self.ctrl.absolute_mode = True
        self.ctrl.abs_home_pos_absolute = 123
        self.assertFalse(self.status()["home_reminder_needed"])

    def test_absolute_mode_without_a_saved_home_still_reminds(self):
        self.ctrl.absolute_mode = True
        self.assertTrue(self.status()["home_reminder_needed"])

    def test_status_reports_electronic_gear(self):
        body = self.status()
        self.assertEqual(body["electronic_gear"], [1, 1])
        self.assertTrue(body["electronic_gear_ok"])
        self.ctrl.electronic_gear = (2, 1)
        self.ctrl.electronic_gear_unity = False
        body = self.status()
        self.assertEqual(body["electronic_gear"], [2, 1])
        self.assertFalse(body["electronic_gear_ok"])

    def test_set_home_action_sets_home(self):
        def fake_set_home():
            self.ctrl.home_set_since_start = True
        self.ctrl.set_home_position.side_effect = fake_set_home
        response = self.client.post("/action", json={"action": "setHome"})
        self.assertEqual(response.status_code, 200)
        self.ctrl.set_home_position.assert_called_once()

    def test_page_contains_the_set_home_dialog_and_button(self):
        html = self.client.get("/index").get_data(as_text=True)
        for marker in ("homeModal", "homeModalConfirmBtn", "setHomeBtn", "status_home", "status_gear"):
            self.assertIn(marker, html)
        # The dialog must start hidden; the page opens it only when /status
        # says a reminder is needed.
        self.assertIn('id="homeModal" style="display:none;"', html)

    def test_home_reports_502_when_the_position_cannot_be_read(self):
        self.ctrl.post_step_motion_by.side_effect = app_module.PositionUnavailableError("no position")
        response = self.client.post("/action", json={"action": "Home"})
        self.assertEqual(response.status_code, 502)
        self.assertIn("no position", response.get_json()["message"])

    def test_home_reports_400_when_the_move_is_out_of_the_drives_range(self):
        self.ctrl.post_step_motion_by.side_effect = app_module.MoveOutOfRangeError("too far for the drive")
        response = self.client.post("/action", json={"action": "Home"})
        self.assertEqual(response.status_code, 400)
        self.assertIn("too far", response.get_json()["message"])

    def test_home_reports_502_when_the_drive_does_not_acknowledge_a_setup_write(self):
        self.ctrl.post_step_motion_by.side_effect = app_module.DriveCommunicationError(
            "Writing positioning speed (0x903) was not acknowledged by the drive (no reply). Nothing was started.")
        response = self.client.post("/action", json={"action": "Home"})
        self.assertEqual(response.status_code, 502)
        self.assertIn("Nothing was started", response.get_json()["message"])

    def test_home_moves_when_the_position_is_readable(self):
        response = self.client.post("/action", json={"action": "Home"})
        self.assertEqual(response.status_code, 200)
        self.ctrl.post_step_motion_by.assert_called_once_with(0)

    def test_set_home_refused_while_motion_is_running(self):
        self.ctrl.reading_active = True
        response = self.client.post("/action", json={"action": "setHome"})
        self.assertEqual(response.status_code, 409)
        self.ctrl.set_home_position.assert_not_called()

    def test_set_home_reports_failure_when_position_unreadable(self):
        response = self.client.post("/action", json={"action": "setHome"})  # flag stays False
        self.assertEqual(response.status_code, 502)
        self.assertIn("NOT set", response.get_json()["message"])


if __name__ == "__main__":
    unittest.main()
