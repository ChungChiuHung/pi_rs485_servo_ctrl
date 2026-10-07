"""
GET /io/do -- the web UI's read-only view of DO1~DO6 (state + assigned function).
app.py opens a real serial port at import, so SerialPortManager is stubbed first
(same approach as test_app_encoder_mode.py); servo_ctrller is a MagicMock, so
nothing here touches hardware.
"""
import unittest
from unittest.mock import MagicMock, patch

import serial_port_manager

_fake_manager_cls = MagicMock()
_fake_manager_cls.return_value.get_serial_instance.return_value = MagicMock()

with patch.object(serial_port_manager, "SerialPortManager", _fake_manager_cls), \
        patch("servo_control.ServoController.refresh_encoder_mode", return_value=None):
    import app as app_module

import hardware_lock

# What the real drive returned on 2026-09-21 (0x0205=0x26, 0x020C=0x103, 0x020D=0x825).
SAMPLE = {
    "DO1": {"pin": "CN1-41", "on": False, "function_code": 0x03, "function": "INP_SA"},
    "DO2": {"pin": "CN1-42", "on": True, "function_code": 0x08, "function": "ZSP"},
    "DO3": {"pin": "CN1-43", "on": True, "function_code": 0x00, "function": "unassigned"},
    "DO4": {"pin": "CN1-44", "on": False, "function_code": 0x05, "function": "TLC_VLC"},
    "DO5": {"pin": "CN1-45", "on": False, "function_code": 0x01, "function": "RD"},
    "DO6": {"pin": "CN1-46", "on": True, "function_code": 0x02, "function": "ALM"},
}


class DoStatusEndpointTests(unittest.TestCase):

    def setUp(self):
        self.ctrl = MagicMock()
        self.ctrl.read_do_status.return_value = SAMPLE
        patcher = patch.object(app_module, "servo_ctrller", self.ctrl)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.client = app_module.app.test_client()

    def test_returns_the_six_outputs(self):
        response = self.client.get("/io/do")
        self.assertEqual(response.status_code, 200)
        body = response.get_json()
        self.assertEqual(body["status"], "success")
        self.assertEqual(list(body["do"]), [f"DO{n}" for n in range(1, 7)])
        self.assertEqual(body["do"]["DO6"], SAMPLE["DO6"])
        self.assertEqual(body["do"]["DO1"]["pin"], "CN1-41")

    def test_says_the_polarity_is_not_applied(self):
        note = self.client.get("/io/do").get_json()["note"]
        self.assertIn("PD27", note)

    def test_a_failed_read_is_503_and_never_looks_like_all_off(self):
        self.ctrl.read_do_status.return_value = None
        response = self.client.get("/io/do")
        self.assertEqual(response.status_code, 503)
        body = response.get_json()
        self.assertEqual(body["status"], "error")
        self.assertNotIn("do", body)
        self.assertIn("Nothing was written", body["message"])

    def test_only_reads_and_only_get(self):
        self.client.get("/io/do")
        self.ctrl.read_do_status.assert_called_once_with()
        self.ctrl.modbus_client.build_write_message.assert_not_called()
        for method in ("post", "put", "delete"):
            self.assertEqual(getattr(self.client, method)("/io/do").status_code, 405)

    def test_a_busy_drive_answers_429_without_reading(self):
        self.assertTrue(hardware_lock._hardware_busy_lock.acquire(blocking=False))
        try:
            response = self.client.get("/io/do")
        finally:
            hardware_lock._hardware_busy_lock.release()
        self.assertEqual(response.status_code, 429)
        self.ctrl.read_do_status.assert_not_called()

    def test_without_a_serial_connection_it_is_503_and_sends_nothing(self):
        with patch.object(app_module, "servo_ctrller", None):
            response = self.client.get("/io/do")
        self.assertEqual(response.status_code, 503)
        self.assertIn("No RS-485 serial port connection", response.get_json()["message"])
        self.ctrl.read_do_status.assert_not_called()

    def test_needs_the_password_when_one_is_configured(self):
        with patch.object(app_module, "WEB_PASSWORD", "s3cret"):
            self.assertEqual(self.client.get("/io/do").status_code, 401)
        self.ctrl.read_do_status.assert_not_called()


class DoPanelTests(unittest.TestCase):

    def setUp(self):
        self.ctrl = MagicMock()
        patcher = patch.object(app_module, "servo_ctrller", self.ctrl)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.html = app_module.app.test_client().get("/index").get_data(as_text=True)

    def test_the_page_has_the_panel_and_a_read_button(self):
        self.assertIn("Digital Outputs", self.html)
        self.assertIn('id="readDoBtn"', self.html)
        self.assertIn('id="doStatusTableBody"', self.html)

    def test_the_page_does_not_read_them_by_itself(self):
        """Auto-refresh exists but is off until the operator ticks it (three
        serial reads per refresh; the Pi 3 B has little headroom)."""
        self.assertIn('id="doAutoRefresh"', self.html)
        checkbox = self.html[self.html.index('id="doAutoRefresh"'):].split(">", 1)[0]
        self.assertNotIn("checked", checkbox)
        self.ctrl.read_do_status.assert_not_called()


if __name__ == "__main__":
    unittest.main()
