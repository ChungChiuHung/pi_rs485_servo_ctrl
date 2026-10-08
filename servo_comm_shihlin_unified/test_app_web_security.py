"""
Endpoint tests for the web UI's optional password (HTTP Basic auth) and the
validated Art-Net start options. app.py opens a real serial port at import, so
SerialPortManager is stubbed first (same approach as test_app_encoder_mode.py);
nothing here touches hardware or opens a UDP socket.
"""
import base64
import json
import os
import re
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


class ActionFeedbackTargetTests(unittest.TestCase):
    """Every action result the page reports must land in an element that
    exists. The JOG arrow keys' motionStart_CW/CCW and motionPause results
    used to target #<action>_status ids that were never in the page, so a
    refused direction change or a STOP that never ran was invisible. There is
    no JS test runner here, so this checks the template's structure."""

    JOG_ACTIONS = ("motionStart_CW", "motionStart_CCW", "motionPause")

    def setUp(self):
        with patch.object(app_module, "WEB_PASSWORD", ""):
            self.html = app_module.app.test_client().get("/index").get_data(as_text=True)
        self.ids = set(re.findall(r'\bid="([^"]+)"', self.html))
        block = re.search(r"const STATUS_KEY_FOR_ACTION = \{(.*?)\};", self.html, re.S)
        self.assertIsNotNone(block, "STATUS_KEY_FOR_ACTION mapping not found in index.html")
        self.mapping = dict(re.findall(r"(\w+):\s*'(\w+)'", block.group(1)))

    def status_id(self, action):
        return self.mapping.get(action, action) + "_status"

    def test_feedback_resolves_through_the_mapping(self):
        self.assertIn("$('#' + statusKey(action) + '_status')", self.html)

    def test_jog_actions_share_one_visible_status_line_in_the_jog_section(self):
        for action in self.JOG_ACTIONS:
            with self.subTest(action=action):
                self.assertEqual(self.status_id(action), "jogMotion_status")
        self.assertIn('<span class="section-note" id="jogMotion_status"></span>', self.html)
        jog_section = self.html[self.html.index('id="arrowKeyPad"'):]
        jog_section = jog_section[:jog_section.index("</nav>")]
        self.assertIn('id="jogMotion_status"', jog_section)

    def test_every_reported_action_has_an_existing_status_element(self):
        actions = set(re.findall(r'data-action="(\w+)"', self.html))
        actions |= set(re.findall(r"sendCommand\('(\w+)'", self.html))
        actions |= set(re.findall(r"setFeedback\('(\w+)'", self.html))
        self.assertTrue(set(self.JOG_ACTIONS) <= actions)
        for action in sorted(actions):
            with self.subTest(action=action):
                self.assertIn(self.status_id(action), self.ids)

    def test_unrelated_actions_keep_their_own_status_element(self):
        for action in ("servoOn", "servoOff", "Home", "setHome", "posTest", "jogSpeedAdjust",
                       "enableSpeedCtrlMode", "clearAlarm12"):
            with self.subTest(action=action):
                self.assertNotIn(action, self.mapping)
                self.assertIn(action + "_status", self.ids)


def js_block(html, marker):
    """Source of the {...} body that follows `marker` in the page's script --
    braces inside '...', "..." and `...` strings and // comments are skipped,
    so '{type}' placeholders don't throw the matching off."""
    start = html.index(marker)
    i = html.index("{", start)
    depth = 0
    while True:
        c = html[i]
        if html.startswith("//", i):
            i = html.index("\n", i)
            continue
        if c in "'\"`":
            j = i + 1
            while html[j] != c:
                j += 2 if html[j] == "\\" else 1
            i = j + 1
            continue
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return html[start:i + 1]
        i += 1


class UiRobustnessTests(unittest.TestCase):
    """Phase A2: pending guards, stale server status, autostart errors,
    profile sync and the profile/encoder-mode lock while an input server runs.
    No JS test runner here, so these check the template's structure; the
    backend rules the UI mirrors are checked over HTTP."""

    def setUp(self):
        with patch.object(app_module, "WEB_PASSWORD", ""):
            self.html = app_module.app.test_client().get("/index").get_data(as_text=True)

    def block(self, marker):
        return js_block(self.html, marker)

    def assert_in_order(self, text, *parts):
        positions = [text.index(part) for part in parts]
        self.assertEqual(positions, sorted(positions), parts)

    def assert_guarded(self, handler_marker, flag):
        body = self.block(handler_marker)
        first_statement = body[body.index("{") + 1:].lstrip()
        self.assertTrue(first_statement.startswith(f"if ({flag}) {{ return; }}"), body[:300])
        self.assert_in_order(body, f"{flag} = true;", "$.ajax(", ".always(function() {",
                             f"{flag} = false;")

    # A2.1
    def test_error_text_prefers_message_then_details_then_error(self):
        self.assertIn("(r && (r.message || r.details || r.error)) || fallback",
                      self.block("function errorText("))
        self.assertNotIn("responseJSON.message", self.html)
        self.assertIn("const msg = errorText(xhr, 'Error performing action');",
                      self.block("function sendCommand("))

    # A2.2
    def test_server_start_and_stop_have_an_in_flight_guard(self):
        self.assert_guarded("$('#startInputServerBtn').click(", "serverRequestInFlight")
        self.assert_guarded("$('#stopInputServerBtn').click(", "serverRequestInFlight")
        buttons = self.block("function updateInputServerButtons(")
        self.assertIn("'disabled', serverRequestInFlight || inputServerActive", buttons)
        self.assertIn("'disabled', serverRequestInFlight || !inputServerActive", buttons)

    # A2.3
    def test_encoder_switch_and_adopt_have_an_in_flight_guard(self):
        self.assert_guarded("$('#encoderModeSwitchBtn').click(", "encoderRequestInFlight")
        self.assert_guarded("$('#encoderModeAdoptBtn').click(", "encoderRequestInFlight")
        # The click-twice confirmation is still there, after the guard.
        self.assert_in_order(self.block("$('#encoderModeSwitchBtn').click("),
                             "if (encoderRequestInFlight)", "if (!encoderSwitchArmed)",
                             "confirm: true")

    # A2.4
    def test_server_status_failure_is_shown_and_success_restores_it(self):
        poll = self.block("function pollInputServerStatus(")
        success, failure = poll.split(".fail(", 1)
        self.assertIn("t('serverStatusUnavailable')", failure)
        self.assertIn("addClass('action-feedback status-error')", failure)
        self.assertNotIn("inputServerActive =", failure)  # locks keep the last known state
        self.assertIn("removeClass('action-feedback status-error')", success)
        self.assertIn("inputServerActive = !!response.active_input_server;", success)
        self.assertEqual(self.html.count("serverStatusUnavailable:"), 2)  # en + zh-tw

    # A2.5
    def test_autostart_read_and_clear_failures_are_shown(self):
        read = self.block("function refreshAutostartStatus(")
        self.assertIn(".fail(function(xhr)", read)
        self.assertIn("showAutostartError('autostartReadFailed', xhr)", read)
        clear = self.block("$('#autostartStatus').on('click', 'a.clear-autostart'")
        self.assertIn("showAutostartError('autostartClearFailed', xhr)", clear)
        self.assertIn("errorText(xhr,", self.block("function showAutostartError("))
        for key in ("autostartReadFailed:", "autostartClearFailed:"):
            self.assertEqual(self.html.count(key), 2)

    # A2.6
    def test_status_poll_syncs_the_profile_without_posting(self):
        self.assert_in_order(self.block("function pollStatus("),
                             "syncProfileSelect(response.profile);", "response.connected === false")
        sync = self.block("function syncProfileSelect(")
        self.assertIn("profileChangeInFlight || document.activeElement === select[0]", sync)
        self.assertIn("select.val(profile);", sync)
        for forbidden in (".change(", ".trigger(", "$.ajax", "$.post"):
            self.assertNotIn(forbidden, sync)

    def test_status_reports_the_profile_even_without_a_serial_connection(self):
        with patch.object(app_module, "WEB_PASSWORD", ""), \
                patch.object(app_module, "servo_ctrller", None):
            body = app_module.app.test_client().get("/status").get_json()
        self.assertEqual(body["profile"], app_module.current_profile_name)

    # A2.7
    def test_profile_and_encoder_controls_follow_the_input_server_state(self):
        self.assertIn("'disabled', inputServerActive || profileChangeInFlight",
                      self.block("function updateProfileSelectLock("))
        encoder = self.block("function updateEncoderSwitchButton(")
        self.assertIn("const locked = encoderRequestInFlight || inputServerActive;", encoder)
        self.assertIn("$('#encoderModeAdoptBtn').prop('disabled', locked)", encoder)
        self.assertIn("btn.prop('disabled', locked);", encoder)
        success = self.block("function pollInputServerStatus(").split(".fail(", 1)[0]
        self.assert_in_order(success, "inputServerActive = !!response.active_input_server;",
                             "updateProfileSelectLock();", "updateEncoderSwitchButton();")

    def test_server_status_reports_osc_and_artnet_so_the_ui_locks_for_both(self):
        server = MagicMock(is_running=True)
        for server_type in ("osc", "artnet", None):
            with self.subTest(server_type=server_type), \
                    patch.object(app_module, "WEB_PASSWORD", ""), \
                    patch.object(app_module, "active_input_server", server_type), \
                    patch.object(app_module, "_input_server_instance", server if server_type else None):
                body = app_module.app.test_client().get("/server/status").get_json()
            self.assertEqual(body["active_input_server"], server_type)

    def test_backend_still_refuses_profile_and_encoder_changes_while_a_server_runs(self):
        ctrl = MagicMock(reading_active=False)
        other_profile = next(name for name in app_module._profiles_data["profiles"]
                             if name != app_module.current_profile_name)
        for server_type in ("osc", "artnet"):
            with self.subTest(server_type=server_type), \
                    patch.object(app_module, "WEB_PASSWORD", ""), \
                    patch.object(app_module, "servo_ctrller", ctrl), \
                    patch.object(app_module, "active_input_server", server_type):
                client = app_module.app.test_client()
                profile = client.post("/profile", json={"profile": other_profile})
                encoder = client.post("/encoder_mode", json={"absolute": True, "confirm": True})
            self.assertEqual(profile.status_code, 409)
            self.assertEqual(encoder.status_code, 409)
        ctrl.write_PA28_Encoder_Mode.assert_not_called()

    # Phase A1 still intact
    def test_jog_feedback_mapping_is_unchanged(self):
        self.assertIn("$('#' + statusKey(action) + '_status')", self.block("function setFeedback("))
        mapping = self.block("const STATUS_KEY_FOR_ACTION =")
        for action in ("motionStart_CW", "motionStart_CCW", "motionPause"):
            self.assertIn(f"{action}: 'jogMotion'", mapping)


class ExternalControlOwnershipTests(unittest.TestCase):
    """Phase B2: while OSC or Art-Net runs (active_input_server set), the web
    UI may only use an allowlist of /action values; everything else, plus
    /alarm/clear and /encoder_mode/adopt, answers 409 having touched nothing
    drive-facing. servo_ctrller is a bare MagicMock, so "touched nothing" is
    checked as "no call recorded on it at all"."""

    ALLOWED = ("getMsg", "setPoint_1", "setPoint_2",
               "motionPause", "motionCancel", "disablePosMode", "servoOff")
    BLOCKED = ("servoOn", "enablePosMode", "posTestStart_CW", "posTestStart_CCW",
               "gotoSetPoint_1", "gotoSetPoint_2", "setHome", "Home",
               "enableSpeedCtrlMode", "motionStart_CW", "motionStart_CCW", "jogSpeedAdjust")
    SERVERS = (("osc", "OSC"), ("artnet", "Art-Net"))

    def setUp(self):
        self.ctrl = MagicMock()
        self.ctrl.modbus_client.format_hex.return_value = ""
        self.ctrl.read_test_mode_0x0901.return_value = 0
        self.ctrl.speed_ctrl_action.return_value = True
        self.ctrl.Read_Pos_Related_Paremters.return_value = []
        self.ctrl.refresh_encoder_mode.return_value = False
        self.ctrl.read_current_alarm_code.return_value = 0xFF
        patches = [
            patch.object(app_module, "servo_ctrller", self.ctrl),
            patch.object(app_module, "WEB_PASSWORD", ""),
            patch.object(app_module, "active_input_server", None),
            patch.object(app_module, "_input_server_instance", None),
            patch.object(app_module, "_encoder_mode_pending_power_cycle", None),
            patch.object(app_module.time, "sleep"),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.client = app_module.app.test_client()

    def action(self, name):
        return self.client.post("/action", json={"action": name})

    def external(self, server_type):
        return patch.object(app_module, "active_input_server", server_type)

    def test_every_dispatched_action_is_classified(self):
        source = open(app_module.__file__, encoding="utf-8").read()
        dispatcher = source[source.index("def handle_action("):]
        dispatched = set(re.findall(r'action == "(\w+)"', dispatcher))
        for group in re.findall(r"action in \(([^)]*)\)", dispatcher):
            dispatched |= set(re.findall(r'"(\w+)"', group))
        self.assertEqual(dispatched, set(self.ALLOWED) | set(self.BLOCKED))
        self.assertEqual(app_module.WEB_ACTIONS_ALLOWED_DURING_EXTERNAL_CONTROL, set(self.ALLOWED))

    def test_blocked_actions_answer_409_and_touch_nothing(self):
        for server_type, name in self.SERVERS:
            for action in self.BLOCKED + ("noSuchAction",):
                with self.subTest(server=server_type, action=action), self.external(server_type):
                    self.ctrl.reset_mock()
                    response = self.action(action)
                    self.assertEqual(response.status_code, 409)
                    body = response.get_json()
                    self.assertEqual(body["message"],
                                     f"{name} is controlling the motor. Stop the input server first.")
                    self.assertEqual(body["action"], action)
                    self.assertEqual(self.ctrl.mock_calls, [])  # no prelude, no command

    def test_allowed_actions_still_run_while_a_server_is_active(self):
        for server_type, _ in self.SERVERS:
            for action in self.ALLOWED:
                with self.subTest(server=server_type, action=action), self.external(server_type):
                    self.ctrl.reset_mock()
                    response = self.action(action)
                    self.assertEqual(response.status_code, 200, response.get_data(as_text=True))
                    self.ctrl.ensure_eeprom_write_protection.assert_called_once()

    def test_allowed_actions_reach_their_controller_calls(self):
        expected = {"servoOff": "servo_off", "motionPause": "speed_ctrl_action",
                    "motionCancel": "stop_continuous_reading", "disablePosMode": "Enable_Position_Mode",
                    "getMsg": "Read_Pos_Related_Paremters", "setPoint_1": "record_set_point"}
        for action, method in expected.items():
            with self.subTest(action=action), self.external("osc"):
                self.ctrl.reset_mock()
                self.action(action)
                getattr(self.ctrl, method).assert_called()

    def test_unknown_action_is_still_400_without_a_server(self):
        response = self.action("noSuchAction")
        self.assertEqual(response.status_code, 400)
        self.assertIn("not recognized", response.get_json()["message"])

    def test_web_control_returns_after_the_server_stops(self):
        server = MagicMock()
        with patch.object(app_module, "active_input_server", "osc"), \
                patch.object(app_module, "_input_server_instance", server):
            self.assertEqual(self.action("motionStart_CW").status_code, 409)
            self.assertEqual(self.client.post("/server/stop").status_code, 200)
            server.stop.assert_called_once()
            self.assertIsNone(app_module.active_input_server)
            self.ctrl.reset_mock()
            self.assertEqual(self.action("motionStart_CW").status_code, 200)
        self.ctrl.speed_ctrl_action.assert_called_once_with(2)

    def test_alarm_clear_is_refused_while_a_server_is_active(self):
        for server_type, name in self.SERVERS:
            with self.subTest(server=server_type), self.external(server_type):
                self.ctrl.reset_mock()
                response = self.client.post("/alarm/clear", json={"confirm": True})
                self.assertEqual(response.status_code, 409)
                self.assertIn(f"{name} is controlling the motor", response.get_json()["message"])
                self.assertEqual(self.ctrl.mock_calls, [])

    def test_alarm_clear_is_unchanged_without_a_server(self):
        response = self.client.post("/alarm/clear", json={"confirm": True})
        self.assertEqual(response.status_code, 200)
        self.ctrl.write_PD_16_Enable_DI_Control.assert_called_once()
        self.ctrl.clear_alarm_12.assert_called_once()

    def test_encoder_adopt_is_refused_while_a_server_is_active(self):
        for server_type, name in self.SERVERS:
            with self.subTest(server=server_type), self.external(server_type):
                self.ctrl.reset_mock()
                response = self.client.post("/encoder_mode/adopt")
                self.assertEqual(response.status_code, 409)
                self.assertIn(f"{name} is controlling the motor", response.get_json()["message"])
                self.assertEqual(self.ctrl.mock_calls, [])

    def test_encoder_adopt_is_unchanged_without_a_server(self):
        with patch.object(app_module, "_encoder_mode_pending_power_cycle", True):
            response = self.client.post("/encoder_mode/adopt")
            self.assertEqual(response.status_code, 200)
            self.assertIsNone(app_module._encoder_mode_pending_power_cycle)
        self.ctrl.refresh_encoder_mode.assert_called_once()


class OscStartOptionsTests(unittest.TestCase):
    """POST /server/start with type "osc". OSCInputServer is a mock -- no
    UDP socket is opened and nothing reaches the drive."""

    def setUp(self):
        self.client = app_module.app.test_client()
        self.server_cls = MagicMock()
        patches = [
            patch.object(app_module, "OSCInputServer", self.server_cls),
            patch.object(app_module, "active_input_server", None),
            patch.object(app_module, "_input_server_instance", None),
            patch.object(app_module, "servo_ctrller", MagicMock()),
            patch.object(app_module, "WEB_PASSWORD", ""),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def start(self, **payload):
        return self.client.post("/server/start", json={"type": "osc", **payload})

    def test_defaults_are_unchanged(self):
        response = self.start()
        self.assertEqual(response.status_code, 200)
        self.server_cls.assert_called_once_with(
            app_module.servo_ctrller, listen_ip="0.0.0.0", listen_port=5005,
            feedback_ip=None, feedback_port=None)
        self.server_cls.return_value.start.assert_called_once()
        self.assertEqual(response.get_json(), {
            "status": "success", "active_input_server": "osc",
            "listen_ip": "0.0.0.0", "listen_port": 5005})

    def test_options_are_passed_through(self):
        response = self.start(listen_ip="192.168.1.50", listen_port="6000",
                              feedback_ip="192.168.1.10", feedback_port=5008)
        self.assertEqual(response.status_code, 200)
        self.server_cls.assert_called_once_with(
            app_module.servo_ctrller, listen_ip="192.168.1.50", listen_port=6000,
            feedback_ip="192.168.1.10", feedback_port=5008)

    def test_invalid_ports_are_rejected_with_400_and_start_nothing(self):
        bad = [
            {"listen_port": "abc"}, {"listen_port": ""}, {"listen_port": None},
            {"listen_port": 0}, {"listen_port": 70000},
            {"feedback_ip": "192.168.1.10", "feedback_port": "abc"},
            {"feedback_ip": "192.168.1.10", "feedback_port": 70000},
        ]
        for payload in bad:
            with self.subTest(payload=payload):
                response = self.start(**payload)
                self.assertEqual(response.status_code, 400, response.get_data(as_text=True))
                self.assertIn("must be", response.get_json()["message"])
        self.server_cls.assert_not_called()

    def test_feedback_port_without_feedback_ip_is_rejected(self):
        for payload in ({"feedback_port": 5008}, {"feedback_ip": "", "feedback_port": 5008}):
            with self.subTest(payload=payload):
                response = self.start(**payload)
                self.assertEqual(response.status_code, 400)
                self.assertIn("feedback_ip", response.get_json()["message"])
        self.server_cls.assert_not_called()

    def test_controller_not_connected_is_a_readable_503(self):
        with patch.object(app_module, "servo_ctrller", None):
            body, status = app_module._do_start_input_server({"type": "osc"})
        self.assertEqual(status, 503)
        self.assertEqual(body["message"], "Servo controller is not connected.")
        self.server_cls.assert_not_called()


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
