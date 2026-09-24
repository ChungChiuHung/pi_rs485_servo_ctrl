"""
Endpoint tests for the web UI's optional password (HTTP Basic auth) and the
validated Art-Net start options. app.py opens a real serial port at import, so
SerialPortManager is stubbed first (same approach as test_app_encoder_mode.py);
nothing here touches hardware or opens a UDP socket.
"""
import base64
import json
import os
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import serial_port_manager

_fake_manager_cls = MagicMock()
_fake_manager_cls.return_value.get_serial_instance.return_value = MagicMock()

with patch.object(serial_port_manager, "SerialPortManager", _fake_manager_cls), \
        patch("servo_control.ServoController.refresh_encoder_mode", return_value=None):
    import app as app_module


def basic(user, password):
    token = base64.b64encode(f"{user}:{password}".encode("utf-8")).decode("ascii")
    return {"Authorization": f"Basic {token}"}


class WebPasswordTests(unittest.TestCase):

    def setUp(self):
        self.client = app_module.app.test_client()

    def test_open_when_no_password_is_configured(self):
        with patch.object(app_module, "WEB_PASSWORD", ""):
            self.assertEqual(self.client.get("/profile").status_code, 200)

    def test_requires_credentials_when_a_password_is_configured(self):
        with patch.object(app_module, "WEB_PASSWORD", "s3cret"):
            response = self.client.get("/profile")
        self.assertEqual(response.status_code, 401)
        self.assertIn("Basic", response.headers["WWW-Authenticate"])

    def test_correct_credentials_are_accepted(self):
        with patch.object(app_module, "WEB_PASSWORD", "s3cret"), patch.object(app_module, "WEB_USER", "servo"):
            response = self.client.get("/profile", headers=basic("servo", "s3cret"))
        self.assertEqual(response.status_code, 200)

    def test_wrong_password_or_user_is_rejected(self):
        with patch.object(app_module, "WEB_PASSWORD", "s3cret"), patch.object(app_module, "WEB_USER", "servo"):
            self.assertEqual(self.client.get("/profile", headers=basic("servo", "nope")).status_code, 401)
            self.assertEqual(self.client.get("/profile", headers=basic("root", "s3cret")).status_code, 401)

    def test_non_ascii_credentials_do_not_crash(self):
        with patch.object(app_module, "WEB_PASSWORD", "s3cret"):
            response = self.client.get("/profile", headers=basic("servo", "密碼"))
        self.assertEqual(response.status_code, 401)

    def test_every_route_is_protected_including_the_ones_that_move_the_motor(self):
        with patch.object(app_module, "WEB_PASSWORD", "s3cret"):
            for method, path in (("get", "/index"), ("get", "/status"), ("post", "/action"),
                                 ("post", "/alarm/clear"), ("post", "/server/start"),
                                 ("post", "/encoder_mode"), ("get", "/log")):
                with self.subTest(path=path):
                    response = getattr(self.client, method)(path)
                    self.assertEqual(response.status_code, 401)


class ArtNetStartOptionsTests(unittest.TestCase):

    def setUp(self):
        self.client = app_module.app.test_client()
        self.server_cls = MagicMock()
        self.server_cls.return_value.is_running = True
        patches = [
            patch.object(app_module, "ArtNetInputServer", self.server_cls),
            patch.object(app_module, "active_input_server", None),
            patch.object(app_module, "_input_server_instance", None),
            patch.object(app_module, "servo_ctrller", MagicMock()),
            patch.object(app_module, "WEB_PASSWORD", ""),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def start(self, **payload):
        return self.client.post("/server/start", json={"type": "artnet", **payload})

    def test_safe_defaults_when_only_the_type_is_given(self):
        response = self.start()
        self.assertEqual(response.status_code, 200)
        kwargs = self.server_cls.call_args.kwargs
        self.assertEqual(kwargs["universe"], 1)
        self.assertEqual(kwargs["signal_timeout_s"], 2.0)
        self.assertFalse(kwargs["enable_dangerous_channels"])
        self.assertEqual(kwargs["allowed_sources"], [])
        self.assertEqual(kwargs["listen_ip"], "0.0.0.0")
        body = response.get_json()
        self.assertFalse(body["enable_dangerous_channels"])
        self.assertEqual(body["universe"], 1)

    def test_options_are_passed_through(self):
        self.start(listen_ip="192.168.1.50", universe=3, signal_timeout_s=5,
                   allowed_sources="192.168.1.10, 192.168.1.11", enable_dangerous_channels=True)
        kwargs = self.server_cls.call_args.kwargs
        self.assertEqual(kwargs["listen_ip"], "192.168.1.50")
        self.assertEqual(kwargs["universe"], 3)
        self.assertEqual(kwargs["signal_timeout_s"], 5.0)
        self.assertEqual(kwargs["allowed_sources"], ["192.168.1.10", "192.168.1.11"])
        self.assertTrue(kwargs["enable_dangerous_channels"])

    def test_allowed_sources_may_be_a_json_list(self):
        self.start(allowed_sources=["10.0.0.5"])
        self.assertEqual(self.server_cls.call_args.kwargs["allowed_sources"], ["10.0.0.5"])

    def test_zero_timeout_is_allowed_to_disable_the_watchdog(self):
        self.assertEqual(self.start(signal_timeout_s=0).status_code, 200)
        self.assertEqual(self.server_cls.call_args.kwargs["signal_timeout_s"], 0.0)

    def test_invalid_values_are_rejected_with_400_and_start_nothing(self):
        bad = [
            {"listen_ip": "not-an-ip"}, {"listen_ip": "10.0.0.999"}, {"listen_ip": "::1"},
            {"listen_port": 0}, {"listen_port": 70000}, {"listen_port": "abc"},
            {"universe": -1}, {"universe": 40000},
            {"signal_timeout_s": -1}, {"signal_timeout_s": 3600},
            {"allowed_sources": "10.0.0.5, banana"}, {"allowed_sources": 5},
            {"enable_dangerous_channels": "yes"}, {"enable_dangerous_channels": 1},
            {"max_speed_rpm": 0},
        ]
        for payload in bad:
            with self.subTest(payload=payload):
                response = self.start(**payload)
                self.assertEqual(response.status_code, 400, response.get_data(as_text=True))
        self.server_cls.assert_not_called()

    def test_feedback_is_off_by_default(self):
        body = self.start().get_json()
        kwargs = self.server_cls.call_args.kwargs
        self.assertIsNone(kwargs["feedback_ip"])
        self.assertNotIn("feedback_universe", kwargs)
        self.assertIsNone(body["feedback_ip"])
        self.assertIsNone(body["feedback_universe"])

    def test_feedback_ip_is_passed_through_with_default_universe(self):
        response = self.start(feedback_ip="10.12.1.164")
        self.assertEqual(response.status_code, 200)
        kwargs = self.server_cls.call_args.kwargs
        self.assertEqual(kwargs["feedback_ip"], "10.12.1.164")
        self.assertEqual(kwargs["feedback_universe"], 2)  # universe (default 1) + 1
        body = response.get_json()
        self.assertEqual(body["feedback_ip"], "10.12.1.164")
        self.assertEqual(body["feedback_universe"], 2)

    def test_feedback_universe_can_be_given_explicitly(self):
        self.start(feedback_ip="10.12.1.164", universe=1, feedback_universe=5)
        kwargs = self.server_cls.call_args.kwargs
        self.assertEqual(kwargs["feedback_universe"], 5)

    def test_feedback_universe_equal_to_universe_is_rejected(self):
        response = self.start(feedback_ip="10.12.1.164", universe=3, feedback_universe=3)
        self.assertEqual(response.status_code, 400)
        self.server_cls.assert_not_called()

    def test_invalid_feedback_ip_is_rejected(self):
        for bad_ip in ("not-an-ip", "10.0.0.999", "::1"):
            with self.subTest(feedback_ip=bad_ip):
                response = self.start(feedback_ip=bad_ip)
                self.assertEqual(response.status_code, 400, response.get_data(as_text=True))
        self.server_cls.assert_not_called()

    def test_feedback_port_defaults_to_6454(self):
        response = self.start(feedback_ip="10.12.1.164")
        kwargs = self.server_cls.call_args.kwargs
        self.assertEqual(kwargs["feedback_port"], 6454)
        self.assertEqual(response.get_json()["feedback_port"], 6454)

    def test_feedback_port_can_be_overridden(self):
        self.start(feedback_ip="10.12.1.164", feedback_port=7777)
        kwargs = self.server_cls.call_args.kwargs
        self.assertEqual(kwargs["feedback_port"], 7777)

    def test_feedback_port_out_of_range_is_rejected(self):
        for bad_port in (0, 70000):
            with self.subTest(feedback_port=bad_port):
                response = self.start(feedback_ip="10.12.1.164", feedback_port=bad_port)
                self.assertEqual(response.status_code, 400)
        self.server_cls.assert_not_called()

    def test_feedback_port_is_absent_when_feedback_is_off(self):
        self.start()
        kwargs = self.server_cls.call_args.kwargs
        self.assertNotIn("feedback_port", kwargs)

    def test_channel_monitor_endpoint_includes_receive_stats(self):
        instance = MagicMock()
        instance.get_channel_snapshot.return_value = None
        instance.get_stats.return_value = {"frames_ok": 3, "dropped_source": 1}
        with patch.object(app_module, "active_input_server", "artnet"), \
                patch.object(app_module, "_input_server_instance", instance):
            body = self.client.get("/server/artnet_channels").get_json()
        self.assertTrue(body["active"])
        self.assertEqual(body["stats"]["dropped_source"], 1)

    def test_channel_monitor_endpoint_reports_no_stats_when_inactive(self):
        body = self.client.get("/server/artnet_channels").get_json()
        self.assertFalse(body["active"])
        self.assertIsNone(body["stats"])

    def test_page_offers_the_new_art_net_safety_controls(self):
        html = app_module.app.test_client().get("/index").get_data(as_text=True)
        for marker in ("artnet_listen_ip", "artnet_allowed_sources", "artnet_signal_timeout",
                       "artnet_dangerous_channels", "artnetStats"):
            self.assertIn(marker, html)
        self.assertIn('id="artnet_universe" value="1"', html)
        # channels 10-12 must not be pre-ticked
        self.assertNotIn('id="artnet_dangerous_channels" checked', html)


class AutostartTests(unittest.TestCase):
    """POST /server/start's `autostart` flag, GET/DELETE /server/autostart,
    and _autostart_input_server() (called once at process start -- see
    __main__). Each test gets its own throwaway file path so nothing here
    touches the real input_server_autostart.json."""

    def setUp(self):
        self.client = app_module.app.test_client()
        self.tmp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp_dir.cleanup)
        self.autostart_file = os.path.join(self.tmp_dir.name, "input_server_autostart.json")
        self.server_cls = MagicMock()
        self.server_cls.return_value.is_running = True
        patches = [
            patch.object(app_module, "ArtNetInputServer", self.server_cls),
            patch.object(app_module, "OSCInputServer", self.server_cls),
            patch.object(app_module, "INPUT_SERVER_AUTOSTART_FILE", self.autostart_file),
            patch.object(app_module, "active_input_server", None),
            patch.object(app_module, "_input_server_instance", None),
            patch.object(app_module, "servo_ctrller", MagicMock()),
            patch.object(app_module, "WEB_PASSWORD", ""),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def test_autostart_false_by_default_saves_nothing(self):
        response = self.client.post("/server/start", json={"type": "artnet"})
        self.assertEqual(response.status_code, 200)
        self.assertFalse(os.path.exists(self.autostart_file))

    def test_autostart_true_on_success_saves_the_exact_payload(self):
        payload = {"type": "artnet", "universe": 3, "autostart": True}
        response = self.client.post("/server/start", json=payload)
        self.assertEqual(response.status_code, 200)
        with open(self.autostart_file) as f:
            saved = json.load(f)
        self.assertEqual(saved, payload)

    def test_autostart_true_on_failure_saves_nothing(self):
        response = self.client.post("/server/start", json={"type": "artnet", "universe": -1, "autostart": True})
        self.assertEqual(response.status_code, 400)
        self.assertFalse(os.path.exists(self.autostart_file))

    def test_get_autostart_config_reports_null_when_none_saved(self):
        body = self.client.get("/server/autostart").get_json()
        self.assertIsNone(body["config"])

    def test_get_autostart_config_reports_the_saved_payload(self):
        self.client.post("/server/start", json={"type": "artnet", "autostart": True})
        body = self.client.get("/server/autostart").get_json()
        self.assertEqual(body["config"]["type"], "artnet")

    def test_delete_autostart_config_removes_it(self):
        self.client.post("/server/start", json={"type": "artnet", "autostart": True})
        self.assertTrue(os.path.exists(self.autostart_file))
        response = self.client.delete("/server/autostart")
        self.assertEqual(response.get_json()["removed"], True)
        self.assertFalse(os.path.exists(self.autostart_file))

    def test_delete_autostart_config_when_nothing_saved_reports_false(self):
        response = self.client.delete("/server/autostart")
        self.assertEqual(response.get_json()["removed"], False)

    def test_autostart_config_endpoints_work_without_a_serial_connection(self):
        with patch.object(app_module, "servo_ctrller", None):
            self.assertEqual(self.client.get("/server/autostart").status_code, 200)
            self.assertEqual(self.client.delete("/server/autostart").status_code, 200)

    def test_autostart_input_server_starts_the_saved_config(self):
        self.client.post("/server/start", json={"type": "artnet", "universe": 5, "autostart": True})
        self.client.post("/server/stop")  # undo the start above -- test the bootstrap path itself next
        self.server_cls.reset_mock()

        app_module._autostart_input_server()

        self.assertEqual(self.server_cls.call_args.kwargs["universe"], 5)
        self.assertEqual(app_module.active_input_server, "artnet")

    def test_autostart_input_server_does_nothing_without_a_saved_config(self):
        app_module._autostart_input_server()
        self.server_cls.assert_not_called()
        self.assertIsNone(app_module.active_input_server)

    def test_autostart_input_server_does_nothing_without_a_connection(self):
        with open(self.autostart_file, "w") as f:
            json.dump({"type": "artnet"}, f)
        with patch.object(app_module, "servo_ctrller", None):
            app_module._autostart_input_server()
        self.server_cls.assert_not_called()

    def test_autostart_input_server_logs_and_survives_a_bad_saved_config(self):
        with open(self.autostart_file, "w") as f:
            f.write("not valid json")
        app_module._autostart_input_server()  # must not raise
        self.server_cls.assert_not_called()


if __name__ == "__main__":
    unittest.main()
