"""
Web UI tests for servo_comm_shihlin/app.py: it must start without a serial
port and say so, refuse everything that would talk to the drive, reconnect,
report status, ask for a password when one is configured, and refuse a move
whose starting position cannot be read. app.py connects at import time, so each
test imports a fresh copy under its own module name with SerialPortManager
stubbed. Nothing here touches hardware. Run from inside this directory:
python -m unittest test_app_web
"""
import base64
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
    serial_instance = MagicMock()
    serial_instance.in_waiting = 0
    serial_instance.read.return_value = b""
    instance.get_serial_instance.return_value = serial_instance if port_opens else None
    instance.get_connected_port.return_value = "COM_TEST" if port_opens else None
    instance.get_baud_rate.return_value = 115200
    return cls


def load_app(port_opens: bool):
    spec = importlib.util.spec_from_file_location("legacy_app_under_test", APP_PATH)
    module = importlib.util.module_from_spec(spec)
    with patch.object(serial_port_manager, "SerialPortManager", manager_class(port_opens)):
        spec.loader.exec_module(module)
    return module


def basic(user, password):
    token = base64.b64encode(f"{user}:{password}".encode("utf-8")).decode("ascii")
    return {"Authorization": f"Basic {token}"}


class NoSerialPortTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=False)
        self.client = self.app_module.app.test_client()

    def test_app_starts_disconnected_and_remembers_why(self):
        self.assertIsNone(self.app_module.servo_ctrller)
        self.assertIn("Could not configure any serial port", self.app_module._connection_error)

    def test_the_page_and_status_still_work(self):
        self.assertEqual(self.client.get("/index").status_code, 200)
        self.assertEqual(self.client.get("/").status_code, 302)  # redirects to /index
        self.assertEqual(self.client.get("/log").status_code, 200)
        self.assertEqual(self.client.get("/server/status").status_code, 200)
        status = self.client.get("/status")
        self.assertEqual(status.status_code, 200)
        body = status.get_json()
        self.assertIs(body["connected"], False)
        self.assertIn("Could not configure any serial port", body["connection_error"])

    def test_the_page_contains_the_banner_and_reconnect_button(self):
        html = self.client.get("/index").get_data(as_text=True)
        self.assertIn("No RS-485 serial port connection", html)
        self.assertIn("reconnectBtn", html)

    def test_routes_that_use_the_drive_answer_503_and_send_nothing(self):
        for path, payload in (("/action", {"action": "servoOn"}),
                              ("/action", {"action": "motionStart_CW"}),
                              ("/action", {"action": "setPoint_1"}),
                              ("/server/start", {"type": "osc"}),
                              ("/alarm/clear", {"confirm": True})):
            with self.subTest(path=path, payload=payload):
                response = self.client.post(path, json=payload)
                self.assertEqual(response.status_code, 503)
                self.assertIs(response.get_json()["connected"], False)
                self.assertIn("No RS-485 serial port connection", response.get_json()["message"])

    def test_unknown_paths_are_still_404(self):
        self.assertEqual(self.client.get("/nope").status_code, 404)

    def test_reconnect_fails_cleanly_while_the_port_is_still_missing(self):
        response = self.client.post("/reconnect")
        self.assertEqual(response.status_code, 503)
        self.assertIsNone(self.app_module.servo_ctrller)

    def test_reconnect_connects_once_the_port_appears(self):
        self.app_module.SerialPortManager = manager_class(port_opens=True)
        response = self.client.post("/reconnect")
        self.assertEqual(response.status_code, 200)
        self.assertIsNotNone(self.app_module.servo_ctrller)
        self.assertIsNone(self.app_module._connection_error)
        status = self.client.get("/status").get_json()
        self.assertIs(status["connected"], True)
        self.assertEqual(status["connected_port"], "COM_TEST")
        # ...and the guard no longer blocks the routes that were refused.
        with patch.object(self.app_module.servo_ctrller, "write_PD_16_Enable_DI_Control"):
            response = self.client.post("/action", json={"action": "not-a-real-action"})
        self.assertEqual(response.status_code, 400)  # reached the handler, which refuses it

    def test_the_new_controller_talks_through_the_new_serial_manager(self):
        """The ASCII client is a singleton; a reconnect must re-bind it."""
        new_manager_cls = manager_class(port_opens=True)
        self.app_module.SerialPortManager = new_manager_cls
        self.client.post("/reconnect")
        self.assertIs(self.app_module.servo_ctrller.modbus_client.serial_port_manager,
                      new_manager_cls.return_value)

    def test_a_failed_connect_closes_the_half_opened_port(self):
        failing = manager_class(port_opens=False)
        self.app_module.SerialPortManager = failing
        self.client.post("/reconnect")
        failing.return_value.disconnect.assert_called()


class ConnectedTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.ctrl = self.app_module.servo_ctrller

    def test_status_reports_home_reminder_until_home_is_set(self):
        status = self.client.get("/status").get_json()
        self.assertIs(status["connected"], True)
        self.assertIs(status["home_reminder_needed"], True)
        self.ctrl.home_set_since_start = True
        self.assertIs(self.client.get("/status").get_json()["home_reminder_needed"], False)

    def test_status_reports_a_gear_ratio_that_is_not_one_to_one(self):
        self.ctrl.electronic_gear = (4, 1)
        self.ctrl.electronic_gear_unity = False
        status = self.client.get("/status").get_json()
        self.assertEqual(status["electronic_gear"], [4, 1])
        self.assertIs(status["electronic_gear_ok"], False)

    def test_status_sends_nothing_to_the_drive(self):
        self.ctrl.modbus_client = MagicMock()
        self.client.get("/status")
        self.ctrl.modbus_client.send_and_receive.assert_not_called()
        self.ctrl.modbus_client.send.assert_not_called()

    def action(self, name):
        with patch.object(self.ctrl, "write_PD_16_Enable_DI_Control"):
            return self.client.post("/action", json={"action": name})

    def test_home_is_refused_with_502_when_the_position_cannot_be_read(self):
        self.ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)
        self.ctrl._execute_positioning = MagicMock()
        response = self.action("Home")
        self.assertEqual(response.status_code, 502)
        self.assertIn("refusing to move", response.get_json()["message"])
        self.ctrl._execute_positioning.assert_not_called()

    def test_set_home_is_refused_while_a_move_is_running(self):
        self.ctrl.reading_active = True
        self.ctrl.set_home_position = MagicMock()
        response = self.action("setHome")
        self.assertEqual(response.status_code, 409)
        self.ctrl.set_home_position.assert_not_called()

    def test_set_home_reports_502_when_the_position_cannot_be_read(self):
        self.ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)
        self.ctrl.save_abs_home_pos = MagicMock()
        response = self.action("setHome")
        self.assertEqual(response.status_code, 502)
        self.ctrl.save_abs_home_pos.assert_not_called()

    def test_set_home_succeeds_and_clears_the_reminder(self):
        self.ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=5000)
        self.ctrl.save_abs_home_pos = MagicMock()
        self.ctrl.delay_ms = MagicMock()
        response = self.action("setHome")
        self.assertEqual(response.status_code, 200)
        self.assertIs(self.client.get("/status").get_json()["home_reminder_needed"], False)

    def test_enable_pos_mode_no_longer_raises_a_type_error(self):
        for name in ("Enable_Position_Mode", "config_acc_dec_0x0902", "config_speed_0x0903",
                     "config_pulses_0x0905_low_byte", "config_pulses_0x0906_high_byte"):
            setattr(self.ctrl, name, MagicMock())
        self.ctrl.start_continuous_reading = MagicMock()
        response = self.action("enablePosMode")
        self.assertEqual(response.status_code, 200)
        self.ctrl.start_continuous_reading.assert_called_once_with(0.1)

    def test_clearing_alarm_12_treats_0xff_as_cleared(self):
        self.ctrl.read_current_alarm_code = MagicMock(side_effect=[0x12, 0xFF])
        self.ctrl.write_PD_16_Enable_DI_Control = MagicMock()
        self.ctrl.clear_alarm_12 = MagicMock()
        self.ctrl.clear_alarm_via_register = MagicMock()
        response = self.client.post("/alarm/clear", json={"confirm": True})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json()["mechanism_used"], "clear_alarm_12")
        self.ctrl.clear_alarm_via_register.assert_not_called()


class SetPointTests(unittest.TestCase):
    """SET POINT 1/2 now RECORD the current angle (they used to move to a
    hard-coded 90 / 180 deg); MOVE TO SET POINT n goes there."""

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.ctrl = self.app_module.servo_ctrller
        self.ctrl._save_config_value = MagicMock()
        self.ctrl._execute_positioning = MagicMock()
        self.ctrl.abs_home_pos = 0
        self.ctrl.set_point_1 = None
        self.ctrl.set_point_2 = None

    def action(self, name):
        with patch.object(self.ctrl, "write_PD_16_Enable_DI_Control"):
            return self.client.post("/action", json={"action": name})

    def test_set_point_records_the_angle_read_from_the_drive(self):
        self.ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=round(30 * 349525.3333333333))
        response = self.action("setPoint_1")
        self.assertEqual(response.status_code, 200)
        self.assertAlmostEqual(self.ctrl.set_point_1, 30.0, places=3)
        self.ctrl._save_config_value.assert_called_once()
        self.ctrl._execute_positioning.assert_not_called()  # recording never moves
        self.assertAlmostEqual(self.client.get("/status").get_json()["set_point_1"], 30.0, places=3)

    def test_set_point_is_refused_with_502_when_the_position_cannot_be_read(self):
        self.ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)
        response = self.action("setPoint_2")
        self.assertEqual(response.status_code, 502)
        self.assertIsNone(self.ctrl.set_point_2)
        self.ctrl._save_config_value.assert_not_called()

    def test_goto_an_unrecorded_set_point_is_a_400_and_does_not_move(self):
        response = self.action("gotoSetPoint_1")
        self.assertEqual(response.status_code, 400)
        self.assertIn("not been recorded", response.get_json()["message"])
        self.ctrl._execute_positioning.assert_not_called()

    def test_goto_moves_from_the_position_read_now(self):
        self.ctrl.set_point_1 = 90.0
        self.ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
        response = self.action("gotoSetPoint_1")
        self.assertEqual(response.status_code, 200)
        self.assertAlmostEqual(self.ctrl._execute_positioning.call_args[0][0], 90.0, places=3)

    def test_goto_is_refused_with_502_when_the_position_cannot_be_read(self):
        self.ctrl.set_point_1 = 90.0
        self.ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)
        response = self.action("gotoSetPoint_1")
        self.assertEqual(response.status_code, 502)
        self.ctrl._execute_positioning.assert_not_called()


class ActionValidationTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.ctrl = self.app_module.servo_ctrller
        for name in ("Enable_Position_Mode", "config_acc_dec_0x0902", "config_speed_0x0903",
                     "config_pulses_0x0905_low_byte", "config_pulses_0x0906_high_byte",
                     "start_continuous_reading", "enable_speed_ctrl", "write_PD_16_Enable_DI_Control"):
            setattr(self.ctrl, name, MagicMock())

    def post(self, **payload):
        return self.client.post("/action", json=payload)

    def test_unknown_action_is_a_400_not_a_fake_success(self):
        response = self.post(action="nope")
        self.assertEqual(response.status_code, 400)
        self.assertEqual(response.get_json()["status"], "error")

    def test_pos_mode_defaults_keep_the_old_hard_coded_values(self):
        self.assertEqual(self.post(action="enablePosMode").status_code, 200)
        self.ctrl.config_speed_0x0903.assert_called_once_with(10)
        self.ctrl.config_pulses_0x0905_low_byte.assert_called_once_with(0x0000)
        self.ctrl.config_pulses_0x0906_high_byte.assert_called_once_with(0x0780)

    def test_pos_mode_uses_the_pulses_and_speed_sent(self):
        self.post(action="enablePosMode", pulses=0x00012345, speed_rpm=50)
        self.ctrl.config_speed_0x0903.assert_called_once_with(50)
        self.ctrl.config_pulses_0x0905_low_byte.assert_called_once_with(0x2345)
        self.ctrl.config_pulses_0x0906_high_byte.assert_called_once_with(0x0001)

    def test_out_of_range_values_are_rejected_before_anything_is_sent(self):
        for payload in ({"pulses": -1}, {"pulses": 2**31}, {"speed_rpm": 3001}, {"speed_rpm": "fast"}):
            with self.subTest(payload=payload):
                self.ctrl.Enable_Position_Mode.reset_mock()
                self.assertEqual(self.post(action="enablePosMode", **payload).status_code, 400)
                self.ctrl.Enable_Position_Mode.assert_not_called()
        self.assertEqual(self.post(action="enableSpeedCtrlMode", speed_rpm=5000).status_code, 400)
        self.ctrl.enable_speed_ctrl.assert_not_called()

    def test_jog_speed_defaults_to_100_and_can_be_set(self):
        self.post(action="enableSpeedCtrlMode")
        self.ctrl.enable_speed_ctrl.assert_called_with(100)
        self.post(action="enableSpeedCtrlMode", speed_rpm=250)
        self.ctrl.enable_speed_ctrl.assert_called_with(250)

    def test_disable_pos_mode_stops_reading_and_leaves_test_mode(self):
        self.ctrl.stop_continuous_reading = MagicMock()
        self.assertEqual(self.post(action="disablePosMode").status_code, 200)
        self.ctrl.stop_continuous_reading.assert_called_once()
        self.ctrl.Enable_Position_Mode.assert_called_with(False)


class HardwareLockTests(unittest.TestCase):

    def test_a_second_command_while_one_is_running_is_refused_with_429(self):
        import hardware_lock
        app_module = load_app(port_opens=True)
        client = app_module.app.test_client()
        with patch.object(app_module.servo_ctrller, "write_PD_16_Enable_DI_Control") as write_pd16:
            self.assertTrue(hardware_lock._hardware_busy_lock.acquire(blocking=False))
            try:
                response = client.post("/action", json={"action": "servoOn"})
            finally:
                hardware_lock._hardware_busy_lock.release()
        self.assertEqual(response.status_code, 429)
        write_pd16.assert_not_called()


class OscServerRouteTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        import socket
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("127.0.0.1", 0))
            self.port = probe.getsockname()[1]

    def tearDown(self):
        if self.app_module.active_input_server:
            self.client.post("/server/stop")

    def test_start_status_stop(self):
        start = self.client.post("/server/start", json={"type": "osc", "listen_ip": "127.0.0.1",
                                                        "listen_port": self.port})
        self.assertEqual(start.status_code, 200)
        status = self.client.get("/server/status").get_json()
        self.assertEqual(status["active_input_server"], "osc")
        self.assertIs(status["is_running"], True)
        stop = self.client.post("/server/stop")
        self.assertEqual(stop.status_code, 200)
        self.assertIsNone(self.client.get("/server/status").get_json()["active_input_server"])

    def test_a_second_start_is_refused_and_stop_with_nothing_running_is_400(self):
        self.assertEqual(self.client.post("/server/stop").status_code, 400)
        self.client.post("/server/start", json={"listen_ip": "127.0.0.1", "listen_port": self.port})
        self.assertEqual(self.client.post("/server/start", json={"listen_ip": "127.0.0.1",
                                                                 "listen_port": self.port}).status_code, 409)

    def test_bad_port_and_unknown_type_are_400(self):
        self.assertEqual(self.client.post("/server/start", json={"listen_port": 0}).status_code, 400)
        self.assertEqual(self.client.post("/server/start", json={"listen_port": "abc"}).status_code, 400)
        self.assertEqual(self.client.post("/server/start", json={"type": "artnet"}).status_code, 400)


class ActivityLogAndTrafficTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()
        self.addCleanup(logging.getLogger().removeHandler, self.app_module._activity_log_handler)

    def test_log_returns_what_the_app_logged_and_supports_since(self):
        logging.getLogger("something").warning("hello from the test")
        entries = self.client.get("/log").get_json()["entries"]
        self.assertIn("hello from the test", [e["message"] for e in entries])
        last = max(e["id"] for e in entries)
        self.assertEqual(self.client.get(f"/log?since={last}").get_json()["entries"], [])

    def test_last_rs485_traffic_shows_the_real_ascii_frames(self):
        client = self.app_module.servo_ctrller.modbus_client
        client.last_sent = b":010301000001FA\r\n"
        client.last_received = bytearray(b":01030200FF00\r\n")
        self.assertEqual(self.app_module._current_rs485_traffic(),
                         (":010301000001FA", ":01030200FF00"))
        self.assertIn(":010301000001FA", self.client.get("/index").get_data(as_text=True))

    def test_no_traffic_yet_is_empty_not_a_placeholder(self):
        client = self.app_module.servo_ctrller.modbus_client
        client.last_sent = client.last_received = None
        self.assertEqual(self.app_module._current_rs485_traffic(), ("", ""))


class WebPasswordTests(unittest.TestCase):

    def setUp(self):
        self.app_module = load_app(port_opens=True)
        self.client = self.app_module.app.test_client()

    def test_open_when_no_password_is_configured(self):
        with patch.object(self.app_module, "WEB_PASSWORD", ""):
            self.assertEqual(self.client.get("/status").status_code, 200)

    def test_requires_credentials_when_a_password_is_configured(self):
        with patch.object(self.app_module, "WEB_PASSWORD", "s3cret"):
            response = self.client.get("/status")
        self.assertEqual(response.status_code, 401)
        self.assertIn("Basic", response.headers["WWW-Authenticate"])

    def test_correct_and_wrong_credentials(self):
        with patch.object(self.app_module, "WEB_PASSWORD", "s3cret"), \
                patch.object(self.app_module, "WEB_USER", "servo"):
            self.assertEqual(self.client.get("/status", headers=basic("servo", "s3cret")).status_code, 200)
            self.assertEqual(self.client.get("/status", headers=basic("servo", "nope")).status_code, 401)
            self.assertEqual(self.client.get("/status", headers=basic("servo", "密碼")).status_code, 401)

    def test_every_route_including_the_ones_that_move_the_motor_is_protected(self):
        with patch.object(self.app_module, "WEB_PASSWORD", "s3cret"):
            for method, path in (("get", "/index"), ("get", "/"), ("post", "/action"),
                                 ("post", "/alarm/clear"), ("post", "/reconnect")):
                with self.subTest(path=path):
                    self.assertEqual(getattr(self.client, method)(path).status_code, 401)


if __name__ == "__main__":
    unittest.main()
