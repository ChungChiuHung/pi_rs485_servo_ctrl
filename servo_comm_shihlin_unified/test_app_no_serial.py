"""
The web UI must still start when no RS-485 serial port can be opened, say so,
refuse everything that would talk to the drive, and let the operator retry.

app.py connects at import time, so each test imports a fresh copy of it under
its own module name with SerialPortManager stubbed (a manager that cannot open
a port, then one that can). Nothing here touches hardware.
"""
import importlib.util
import logging
import os
import unittest
from unittest.mock import MagicMock, patch

import serial_port_manager

APP_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "app.py")


def manager_class(port_opens: bool):
    cls = MagicMock()
    instance = cls.return_value
    instance.get_serial_instance.return_value = MagicMock() if port_opens else None
    instance.get_connected_port.return_value = "COM_TEST" if port_opens else None
    instance.get_baud_rate.return_value = 115200
    return cls


def load_app(port_opens: bool):
    spec = importlib.util.spec_from_file_location("app_under_test_no_serial", APP_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.object(serial_port_manager, "SerialPortManager", manager_class(port_opens)), \
            patch("servo_control.ServoController.refresh_encoder_mode", return_value=None):
        spec.loader.exec_module(module)
    return module


class NoSerialPortTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=False)
        # app.py attaches its activity-log handler to the root logger.
        self.addCleanup(logging.getLogger().removeHandler, self.app_module._activity_log_handler)
        self.client = self.app_module.app.test_client()

    def test_app_starts_disconnected_and_remembers_why(self):
        self.assertIsNone(self.app_module.servo_ctrller)
        self.assertIn("Could not open a serial port", self.app_module._connection_error)

    def test_the_page_and_status_still_work(self):
        self.assertEqual(self.client.get("/index").status_code, 200)
        self.assertEqual(self.client.get("/profile").status_code, 200)
        self.assertEqual(self.client.get("/log").status_code, 200)
        self.assertEqual(self.client.get("/server/status").status_code, 200)
        status = self.client.get("/status")
        self.assertEqual(status.status_code, 200)
        body = status.get_json()
        self.assertIs(body["connected"], False)
        self.assertIn("Could not open a serial port", body["connection_error"])
        self.assertIsNone(body["connected_port"])

    def test_every_route_that_uses_the_drive_answers_503_and_sends_nothing(self):
        for method, path, payload in (
                ("post", "/action", {"action": "servoOn"}),
                ("post", "/action", {"action": "motionStart_CW"}),
                ("post", "/alarm/clear", {"confirm": True}),
                ("get", "/encoder_mode", None),
                ("post", "/encoder_mode", {"absolute": True, "confirm": True}),
                ("post", "/encoder_mode/adopt", None),
                ("post", "/server/start", {"type": "osc"}),
                ("post", "/server/start", {"type": "artnet"})):
            with self.subTest(path=path, payload=payload):
                response = getattr(self.client, method)(path, json=payload)
                self.assertEqual(response.status_code, 503)
                body = response.get_json()
                self.assertIs(body["connected"], False)
                self.assertIn("No RS-485 serial port connection", body["message"])

    def test_unknown_paths_are_still_404(self):
        self.assertEqual(self.client.get("/nope").status_code, 404)

    def test_reconnect_fails_cleanly_while_the_port_is_still_missing(self):
        response = self.client.post("/reconnect")
        self.assertEqual(response.status_code, 503)
        self.assertIs(response.get_json()["connected"], False)
        self.assertIsNone(self.app_module.servo_ctrller)

    def test_reconnect_connects_once_the_port_appears(self):
        self.app_module.SerialPortManager = manager_class(port_opens=True)
        with patch("servo_control.ServoController.refresh_encoder_mode", return_value=None):
            response = self.client.post("/reconnect")
        self.assertEqual(response.status_code, 200)
        self.assertIs(response.get_json()["connected"], True)
        self.assertIsNotNone(self.app_module.servo_ctrller)
        self.assertIsNone(self.app_module._connection_error)

        # ...and the routes that were refused are no longer blocked by the guard.
        response = self.client.post("/action", json={"action": "not-a-real-action"})
        self.assertNotEqual(response.status_code, 503)

    def test_reconnect_when_already_connected_does_nothing(self):
        self.app_module.SerialPortManager = manager_class(port_opens=True)
        with patch("servo_control.ServoController.refresh_encoder_mode", return_value=None):
            self.client.post("/reconnect")
            controller = self.app_module.servo_ctrller
            response = self.client.post("/reconnect")
        self.assertEqual(response.status_code, 200)
        self.assertIs(self.app_module.servo_ctrller, controller)

    def test_selecting_the_current_profile_retries_the_connection(self):
        self.app_module.SerialPortManager = manager_class(port_opens=True)
        with patch("servo_control.ServoController.refresh_encoder_mode", return_value=None):
            response = self.client.post(
                "/profile", json={"profile": self.app_module.current_profile_name})
        self.assertEqual(response.status_code, 200)
        self.assertIsNotNone(self.app_module.servo_ctrller)

    def test_a_failed_connect_closes_the_half_opened_port(self):
        failing = manager_class(port_opens=False)
        self.app_module.SerialPortManager = failing
        self.client.post("/reconnect")
        failing.return_value.disconnect.assert_called()

    def test_eeprom_guard_skips_while_disconnected(self):
        with patch.object(self.app_module.time, "sleep", side_effect=[None, StopIteration]), \
                patch.object(self.app_module, "run_when_idle") as run_when_idle:
            with self.assertRaises(StopIteration):
                self.app_module._eeprom_guard_loop()
        run_when_idle.assert_not_called()


class ConnectedTests(unittest.TestCase):

    def test_status_reports_connected_true_when_there_is_a_port(self):
        app_module = load_app(port_opens=True)
        self.addCleanup(logging.getLogger().removeHandler, app_module._activity_log_handler)
        self.assertIsNotNone(app_module.servo_ctrller)
        self.assertIsNone(app_module._connection_error)


if __name__ == "__main__":
    unittest.main()
