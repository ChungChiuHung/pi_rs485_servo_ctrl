"""
Tests for the JOG speed-control additions ported from
servo_comm_shihlin_unified 2026-09-22: enable_speed_ctrl()'s enable=False
path, acc_time, and explicit stop-on-arm; speed_ctrl_action()'s direction-
reversal guard; set_jog_speed()/clear_jog_speed()/change_jog_speed_by()
(the web UI's Up/Down arrow-key +/-1 rpm nudge).

Mocked serial port; nothing touches hardware. Run from inside this
directory: python -m unittest test_jog_speed_control
"""
import unittest
from unittest.mock import MagicMock

from servo_control import ServoController


def make_controller():
    fake_serial = MagicMock()
    fake_serial.keep_running = True
    ctrl = ServoController(fake_serial)
    ctrl.modbus_client = MagicMock()
    # A minimally valid ASCII WRITE_DATA response (ADR=01, CMD=06/WRITE_DATA,
    # start_address=0904, data=0000; LRC is never checked by ModbusResponse)
    # -- speed_ctrl_action() parses whatever send_and_receive() returns, so a
    # bare MagicMock (the default) fails ModbusResponse's format check.
    ctrl.modbus_client.send_and_receive.return_value = ":010609040000CB\r\n"
    return ctrl


class SpeedCtrlActionReversalGuardTests(unittest.TestCase):

    def test_repeating_the_same_direction_is_allowed(self):
        ctrl = make_controller()
        self.assertTrue(ctrl.speed_ctrl_action(1))
        self.assertTrue(ctrl.speed_ctrl_action(1))

    def test_direct_reversal_is_refused(self):
        ctrl = make_controller()
        self.assertTrue(ctrl.speed_ctrl_action(1))
        self.assertFalse(ctrl.speed_ctrl_action(2))

    def test_reversal_is_allowed_after_a_stop(self):
        ctrl = make_controller()
        self.assertTrue(ctrl.speed_ctrl_action(1))
        self.assertTrue(ctrl.speed_ctrl_action(0))
        self.assertTrue(ctrl.speed_ctrl_action(2))

    def test_stop_is_always_allowed(self):
        ctrl = make_controller()
        self.assertTrue(ctrl.speed_ctrl_action(1))
        self.assertTrue(ctrl.speed_ctrl_action(0))


class EnableSpeedCtrlTests(unittest.TestCase):
    """Regression coverage for the 2026-09-22 real-hardware finding on
    servo_comm_shihlin_unified: 0x0904 (JOG_OPERATION) is a sticky register
    on this drive family -- entering JOG mode does NOT reset it, so a
    stale direction left over from an earlier session resumed rotation the
    instant JOG mode was re-armed, with no direction ever pressed this
    time. Every arm must now explicitly stop first."""

    def test_enable_true_enters_jog_mode_configures_and_stops(self):
        ctrl = make_controller()
        call_order = []
        ctrl.clear_alarm_12 = MagicMock(side_effect=lambda: call_order.append("clear_alarm_12"))
        ctrl.Enable_JOG_Mode = MagicMock(side_effect=lambda v: call_order.append("jog_mode"))
        ctrl.config_acc_dec_0x0902 = MagicMock(side_effect=lambda v: call_order.append("accel"))
        ctrl.config_speed_0x0903 = MagicMock(side_effect=lambda v: call_order.append("speed"))
        ctrl.speed_ctrl_action = MagicMock(side_effect=lambda v: call_order.append("stop"))
        ctrl.start_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=100, acc_time=2000, enable=True)

        ctrl.clear_alarm_12.assert_called_once()
        ctrl.Enable_JOG_Mode.assert_called_once_with(True)
        ctrl.config_acc_dec_0x0902.assert_called_once_with(2000)
        ctrl.config_speed_0x0903.assert_called_once_with(100)
        self.assertEqual(call_order, ["clear_alarm_12", "jog_mode", "accel", "speed", "stop"])
        ctrl.speed_ctrl_action.assert_called_once_with(0)
        ctrl.start_continuous_reading.assert_called_once_with(0.1, auto_stop_on_stillness=False)

    def test_enable_true_records_the_jog_speed(self):
        ctrl = make_controller()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.speed_ctrl_action = MagicMock()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=150, enable=True)

        self.assertEqual(ctrl.jog_speed_rpm, 150)

    def test_enable_false_exits_jog_mode_and_clears_jog_speed(self):
        ctrl = make_controller()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.stop_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()
        ctrl.jog_speed_rpm = 100

        ctrl.enable_speed_ctrl(enable=False)

        ctrl.Enable_JOG_Mode.assert_called_once_with(False)
        ctrl.stop_continuous_reading.assert_called_once()
        self.assertIsNone(ctrl.jog_speed_rpm)

    def test_enable_as_string_true_is_coerced(self):
        ctrl = make_controller()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.speed_ctrl_action = MagicMock()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=100, enable="True")

        ctrl.Enable_JOG_Mode.assert_called_once_with(True)

    def test_enable_as_string_false_is_coerced(self):
        ctrl = make_controller()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.stop_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(enable="False")

        ctrl.Enable_JOG_Mode.assert_called_once_with(False)


class ChangeJogSpeedByTests(unittest.TestCase):
    """change_jog_speed_by() -- the web UI's Up/Down arrow-key +/-1 rpm
    nudge, distinct from enable_speed_ctrl() which sets an absolute
    starting speed."""

    def test_raises_if_jog_mode_was_never_enabled(self):
        ctrl = make_controller()
        self.assertIsNone(ctrl.jog_speed_rpm)
        with self.assertRaises(RuntimeError):
            ctrl.change_jog_speed_by(1)

    def test_nudges_up_from_the_speed_enable_speed_ctrl_set(self):
        ctrl = make_controller()
        ctrl.jog_speed_rpm = 100
        ctrl.config_speed_0x0903 = MagicMock()

        result = ctrl.change_jog_speed_by(1)

        self.assertEqual(result, 101)
        self.assertEqual(ctrl.jog_speed_rpm, 101)
        ctrl.config_speed_0x0903.assert_called_once_with(101)

    def test_nudges_down(self):
        ctrl = make_controller()
        ctrl.jog_speed_rpm = 100
        ctrl.config_speed_0x0903 = MagicMock()

        result = ctrl.change_jog_speed_by(-1)

        self.assertEqual(result, 99)
        ctrl.config_speed_0x0903.assert_called_once_with(99)

    def test_clamped_to_zero_not_negative(self):
        ctrl = make_controller()
        ctrl.jog_speed_rpm = 0
        ctrl.config_speed_0x0903 = MagicMock()

        result = ctrl.change_jog_speed_by(-1)

        self.assertEqual(result, 0)

    def test_clamped_to_the_manual_documented_maximum(self):
        ctrl = make_controller()
        ctrl.jog_speed_rpm = 3000
        ctrl.config_speed_0x0903 = MagicMock()

        result = ctrl.change_jog_speed_by(1)

        self.assertEqual(result, 3000)

    def test_after_motion_cancel_clears_jog_speed_it_raises_again(self):
        """Regression coverage for the merged toggle button's off-path
        (app.py's "motionCancel" action): it must clear jog_speed_rpm
        itself (it does not call enable_speed_ctrl(enable=False)), or a
        stale value would let this method appear to succeed after JOG
        mode was actually torn down."""
        ctrl = make_controller()
        ctrl.jog_speed_rpm = 100
        ctrl.clear_jog_speed()
        with self.assertRaises(RuntimeError):
            ctrl.change_jog_speed_by(1)


if __name__ == "__main__":
    unittest.main()
