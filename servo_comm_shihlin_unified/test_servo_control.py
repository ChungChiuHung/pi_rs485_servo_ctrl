"""
Unit tests for servo_control.py.

Two groups:
1. New logic introduced by the servo_comm_shihlin_unified merge: closed-loop
   diff_angle basis, EncoderPulseTracker integration, the abs()-based
   180-degree guard, directional float_error accumulation.
2. Register/address-correctness coverage for every read_*/write_*/config_*
   method ported from servo_comm_shihlin -- these were mechanically switched
   from ModbusASCIIClient to ModbusRTUClient (see design doc §0), and a
   mechanical port across ~50 methods is exactly the kind of change where a
   copy-paste address/word-length mistake is easy to introduce and easy to
   miss by eye. These tests mock the modbus client (or patch
   ModbusRTUResponse, mirroring test_absolute_mode_check.py's convention)
   rather than crafting real wire bytes -- wire-level framing/CRC is already
   covered elsewhere (test_encoder_pulse_tracker.py, the ModbusRTUClient
   tests). None of this touches real hardware.
"""
import os
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

from servo_control import (
    ServoController, is_alarm_active, NO_ALARM_CODES,
    STILL_THRESHOLD_PULSES, STILL_COUNT_TO_COMPLETE,
)
from servo_utility import ServoUtility
from servo_control_registers import ServoControlRegistry
from servo_p_register import PA, PD, PF

PROFILE_400W = {
    "name": "test_400W",
    "baud_rate": 115200,
    "gear_ratio": 30,
    "abs_home_pos_default": 62369153,
    "encoder_pulses_per_rev": 4194304,
    "base_pulse_per_degree": 4194304 * 30 / 360,
    "modbus_device_number": 1,
}


def make_controller(profile=None, tmp_dir=None):
    fake_serial = MagicMock()
    fake_serial.keep_running = True
    ctrl = ServoController(fake_serial, dict(profile or PROFILE_400W))
    if tmp_dir is not None:
        ctrl.config_file = os.path.join(tmp_dir, ctrl.config_file)
    return ctrl


class TestBasePulsePerDegreeFromProfile(unittest.TestCase):

    def test_uses_profile_value_not_a_hardcoded_constant(self):
        ctrl = make_controller()
        self.assertAlmostEqual(ctrl.base_pulse_per_degree, 349525.3333333333)

    def test_different_profile_gives_different_constant(self):
        profile_50w = dict(PROFILE_400W)
        profile_50w["gear_ratio"] = 10
        profile_50w["base_pulse_per_degree"] = 4194304 * 10 / 360
        ctrl = make_controller(profile_50w)
        self.assertAlmostEqual(ctrl.base_pulse_per_degree, 116508.44444444444)


class TestPosStepMotionByUsesTracker(unittest.TestCase):
    """pos_step_motion_by() must route the live raw reading through
    EncoderPulseTracker, not use it directly -- otherwise comparing it
    against a tracker-derived target_pos (e.g. abs_home_pos) silently
    breaks once a wraparound has occurred (design doc §2.4 Plan C)."""

    def test_diff_computed_against_unwrapped_cumulative_value(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()

        # Seed the tracker as if continuous reading had already run for a
        # while and the raw register had wrapped once.
        ctrl._encoder_tracker.reset(4_294_000_000)
        ctrl._encoder_tracker.update(50)  # simulates a wrap: cumulative ~ 4294000050+

        # A fresh raw reading right at/near the same post-wrap value.
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=60)

        target_pos = ctrl._encoder_tracker.cumulative + 1000
        angle_rotated = ctrl.pos_step_motion_by(target_pos=target_pos, speed_rpm=5)

        # If this used the raw 60 directly instead of the tracker's
        # unwrapped cumulative value, diff_pulses would be enormous
        # (billions) instead of the small, correct 1000ish value.
        ctrl._execute_positioning.assert_called_once()
        diff_pulses_arg = ctrl._execute_positioning.call_args[0][0]
        self.assertLess(abs(diff_pulses_arg), 2000)
        self.assertGreater(angle_rotated, 0)

    def test_returns_zero_and_no_response_on_empty_encoder_read(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)

        result = ctrl.pos_step_motion_by(target_pos=1000)

        self.assertEqual(result, 0.0)
        ctrl._execute_positioning.assert_not_called()

    def test_180_degree_guard_blocks_large_positive_move(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)

        huge_target = int(ctrl.base_pulse_per_degree * 200)  # > 180 degrees worth
        result = ctrl.pos_step_motion_by(target_pos=huge_target)

        self.assertEqual(result, 0.0)
        ctrl._execute_positioning.assert_not_called()

    def test_180_degree_guard_blocks_large_negative_move(self):
        """The pre-merge servo_comm_shihlin_50W guard only checked
        diff_pulses >= threshold (no abs()), so it never caught large
        negative moves. This is the regression test for that fix."""
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)

        huge_negative_target = -int(ctrl.base_pulse_per_degree * 200)
        result = ctrl.pos_step_motion_by(target_pos=huge_negative_target)

        self.assertEqual(result, 0.0)
        ctrl._execute_positioning.assert_not_called()

    def test_move_just_under_180_degrees_is_allowed(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)

        just_under = int(ctrl.base_pulse_per_degree * 170)
        result = ctrl.pos_step_motion_by(target_pos=just_under)

        ctrl._execute_positioning.assert_called_once()
        self.assertNotEqual(result, 0.0)


class TestPostStepMotionByClosedLoop(unittest.TestCase):
    """post_step_motion_by() must compute diff_angle from
    target_angle - current_angle, where current_angle is only ever set by
    the continuous-reading feedback loop -- never overwritten here (design
    doc §2.2 #4, the closed-loop decision)."""

    def test_diff_uses_target_minus_current_angle(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 30.0  # as if fed by the continuous-reading loop

        ctrl.post_step_motion_by(angle=50.0, speed_rpm=5)

        self.assertEqual(ctrl.target_angle, 50.0)
        self.assertEqual(ctrl.previous_angle, 30.0)
        # current_angle must NOT be overwritten by the command path itself.
        self.assertEqual(ctrl.current_angle, 30.0)
        ctrl._execute_positioning.assert_called_once()
        diff_angle_arg = ctrl._execute_positioning.call_args[0][0]
        self.assertAlmostEqual(diff_angle_arg, 20.0)

    def test_180_degree_guard_blocks_large_positive_change(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0

        ctrl.post_step_motion_by(angle=200.0)

        ctrl._execute_positioning.assert_not_called()

    def test_180_degree_guard_blocks_large_negative_change(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0

        ctrl.post_step_motion_by(angle=-200.0)

        ctrl._execute_positioning.assert_not_called()

    def test_directional_float_error_accumulation(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0
        ctrl.float_error = 0.0

        ctrl.post_step_motion_by(angle=0.001)  # small positive move
        error_after_positive = ctrl.float_error

        ctrl.current_angle = 0.001
        ctrl.float_error = 0.0
        ctrl.post_step_motion_by(angle=0.0)  # move back: diff_angle is negative
        error_after_negative = ctrl.float_error

        # Accumulation direction should mirror the sign of diff_angle.
        self.assertGreaterEqual(error_after_positive, 0.0)
        self.assertLessEqual(error_after_negative, 0.0)

    def test_accumulate_pulse_increases_with_motion(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0
        ctrl.accumulate_pulse = 0

        ctrl.post_step_motion_by(angle=10.0)

        self.assertGreater(ctrl.accumulate_pulse, 0)


class TestCancelContinuousReading(unittest.TestCase):

    def test_delegates_to_stop_continuous_reading_not_duplicated(self):
        ctrl = make_controller()
        ctrl.reading_active = True
        ctrl.stop_continuous_reading = MagicMock(wraps=ctrl.stop_continuous_reading)
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1000)

        ctrl.cancel_continuous_reading()

        ctrl.stop_continuous_reading.assert_called_once()

    def test_explicitly_exits_test_mode(self):
        """Regression coverage for a gap found via live OSC testing
        (2026-09-19): OSC/Art-Net's cancel path only stopped the keep-alive
        poll thread, relying on the drive's own ~1s communication-timeout
        to fall out of test mode on its own instead of exiting immediately
        -- unlike the web UI's MOTION CANCEL, which calls
        Enable_Position_Mode(False) explicitly. Enable_Position_Mode(False)
        is the documented generic "quit test mode" write regardless of
        whether JOG or Positioning test mode was active."""
        ctrl = make_controller()
        ctrl.reading_active = True
        ctrl.Enable_Position_Mode = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1000)

        ctrl.cancel_continuous_reading()

        ctrl.Enable_Position_Mode.assert_called_once_with(False)

    def test_updates_tracker_and_fires_on_cancel(self):
        ctrl = make_controller()
        ctrl.reading_active = True
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1000)
        ctrl.abs_home_pos = 0

        received = []
        ctrl.register_event_listener("on_cancel", lambda angle: received.append(angle))

        ctrl.cancel_continuous_reading()

        self.assertEqual(ctrl.current_encoder, ctrl._encoder_tracker.cumulative)
        self.assertEqual(len(received), 1)
        self.assertAlmostEqual(received[0], 1000 / ctrl.base_pulse_per_degree, places=4)

    def test_no_crash_and_no_notify_on_empty_encoder_read(self):
        ctrl = make_controller()
        ctrl.reading_active = True
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)

        received = []
        ctrl.register_event_listener("on_cancel", lambda angle: received.append(angle))

        ctrl.cancel_continuous_reading()  # should not raise

        self.assertEqual(received, [])


class TestSetHomePosition(unittest.TestCase):

    def test_resets_tracker_consistently_with_saved_abs_home_pos(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=777)

            ctrl.set_home_position()

            self.assertEqual(ctrl.current_encoder, 777)
            self.assertEqual(ctrl._encoder_tracker.cumulative, 777)
            self.assertEqual(ctrl.abs_home_pos, 62369153)  # unchanged until reload
            with open(ctrl.config_file) as f:
                import json
                saved = json.load(f)
            self.assertEqual(saved["abs_home_pos"], 777)

    def test_resets_float_error_and_accumulate_pulse(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1)
            ctrl.float_error = 0.5
            ctrl.accumulate_pulse = 12345

            ctrl.set_home_position()

            self.assertEqual(ctrl.float_error, 0.0)
            self.assertEqual(ctrl.accumulate_pulse, 0)


class TestPerProfileConfigFile(unittest.TestCase):

    def test_config_file_name_includes_profile_name(self):
        ctrl = make_controller()
        self.assertEqual(ctrl.config_file, "servo_config_test_400W.json")

    def test_different_profiles_do_not_share_a_config_file(self):
        profile_a = dict(PROFILE_400W, name="profile_a")
        profile_b = dict(PROFILE_400W, name="profile_b")
        ctrl_a = make_controller(profile_a)
        ctrl_b = make_controller(profile_b)
        self.assertNotEqual(ctrl_a.config_file, ctrl_b.config_file)


class TestLockIsReentrant(unittest.TestCase):

    def test_lock_is_rlock_not_plain_lock(self):
        # start_continuous_reading() can call stop_continuous_reading() from
        # within its own `with self.lock:` block when reading is already
        # active -- a plain threading.Lock would deadlock there.
        ctrl = make_controller()
        with ctrl.lock:
            with ctrl.lock:
                pass  # must not deadlock/raise


# --- Register/address correctness for every read_*/write_*/config_* method ---
#
# (method_name, expected_address, expected_word_length, extra_kwargs)
# One row per simple "build_read_message(address, word_length)" method.
READ_METHOD_CASES = [
    ("read_PA01_Ctrl_Mode", PA.STY.address, 2),
    ("read_PA33_Encoder_ABS_Pos", 0x0340, 2),
    ("read_PD_16", PD.SDI.address, 1),
    ("read_PD_25", PD.ITST.address, 1),
    ("read_PD_01", PD.DIA1.address, 2),
    ("read_PD_02", PD.DI1.address, 2),
    ("read_PD_08", PD.DI7.address, 2),
    ("read_servo_state", 0x0200, 1),
    ("read_control_mode", 0x0201, 1),
    ("read_alarm_msg", 0x0100, 11),
    ("read_test_mode_0x0901", 0x0901, 1),
    ("read_0x0905_low_byte", 0x0905, 1),
    ("read_0x0906_high_byte", 0x0906, 1),
    ("read_PF82", PF.PRCM.address, 1),
    ("read_encoder_before_gear_ratio", 0x0000, 2),
    ("read_encoder_after_gear_ratio", 0x0024, 2),
]

# (method_name, expected_address, expected_value)
WRITE_METHOD_CASES = [
    ("write_PA01_Ctrl_Mode", PA.STY.address, ServoUtility.config_hex_with(0, 0, 1, 0)),
    ("write_PA29_Initial_Abs_Pos", 0x0338, 1),
    ("write_PD_16_Enable_DI_Control", PD.SDI.address, ServoUtility.config_hex_with(0, 0xF, 0xF, 0xF)),
    ("write_PD_25", PD.ITST.address, ServoUtility.config_hex_with(0, 0, 4, 1)),
    ("clear_alarm", PD.ITST.address, ServoUtility.config_hex_with(0, 3, 4, 0)),
    ("servo_on", PD.ITST.address, ServoUtility.config_hex_with(0, 3, 4, 1)),
    ("clear_alarm_12", PD.ITST.address, ServoUtility.config_hex_with(0, 0, 4, 0)),
    ("servo_off", PD.ITST.address, ServoUtility.config_hex_with(0, 0, 0, 0)),
    ("clear_alarm_via_register", 0x0130, 0x1EA5),
]


class TestReadMethodAddresses(unittest.TestCase):
    """Every simple read_* method must build_read_message() with the
    documented address/word_length -- catches address typos from the
    ASCII -> RTU mechanical port."""

    @patch("servo_control.ModbusRTUResponse")
    def test_all_read_methods_use_correct_address_and_word_length(self, mock_response_cls):
        mock_response_cls.return_value.get_value.return_value = 0
        mock_response_cls.return_value.data_bytes = b'\x00' * 12

        for method_name, expected_address, expected_word_length in READ_METHOD_CASES:
            with self.subTest(method=method_name):
                ctrl = make_controller()
                ctrl.modbus_client = MagicMock()
                ctrl.modbus_client.send_and_receive.return_value = b'not-empty'

                getattr(ctrl, method_name)()

                ctrl.modbus_client.build_read_message.assert_called_once_with(
                    expected_address, expected_word_length
                )

    def test_read_encoder_before_gear_ratio_returns_none_on_empty_response(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = None
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = None
            result = ctrl.read_encoder_before_gear_ratio()
        self.assertIsNone(result)

    def test_read_encoder_before_gear_ratio_returns_int(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = 12345
            result = ctrl.read_encoder_before_gear_ratio()
        self.assertEqual(result, 12345)
        self.assertIsInstance(result, int)


class TestWriteMethodAddressesAndValues(unittest.TestCase):
    """Every simple write_* method must build_write_message() with the
    documented address/value."""

    @patch("servo_control.ModbusRTUResponse")
    def test_all_write_methods_use_correct_address_and_value(self, mock_response_cls):
        mock_response_cls.return_value.get_value.return_value = 0

        for method_name, expected_address, expected_value in WRITE_METHOD_CASES:
            with self.subTest(method=method_name):
                ctrl = make_controller()
                ctrl.modbus_client = MagicMock()
                ctrl.modbus_client.send_and_receive.return_value = b'not-empty'

                getattr(ctrl, method_name)()

                ctrl.modbus_client.build_write_message.assert_called_once_with(
                    expected_address, expected_value
                )

    @patch("servo_control.ModbusRTUResponse")
    def test_write_PD_02_writes_value_1(self, mock_response_cls):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        ctrl.write_PD_02()
        ctrl.modbus_client.build_write_message.assert_called_once_with(PD.DI1.address, 1)

    @patch("servo_control.ModbusRTUResponse")
    def test_write_PD_08_writes_0x02F(self, mock_response_cls):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        ctrl.write_PD_08()
        ctrl.modbus_client.build_write_message.assert_called_once_with(PD.DI7.address, 0x02F)

    @patch("servo_control.ModbusRTUResponse")
    def test_write_PD_01_writes_all_zero_config(self, mock_response_cls):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        ctrl.write_PD_01()
        ctrl.modbus_client.build_write_message.assert_called_once_with(
            PD.DIA1.address, ServoUtility.config_hex_with(0, 0, 0, 0)
        )


class TestConfigAndModeMethods(unittest.TestCase):
    """Position/JOG mode toggles and motion-parameter config writes --
    these use modbus_client.send()/send_and_receive() directly rather than
    a response_object, so there's nothing to parse, just the right
    address/value pair."""

    def test_enable_position_mode_true_writes_0x0004(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.Enable_Position_Mode(True)
        ctrl.modbus_client.build_write_message.assert_called_once_with(
            ServoControlRegistry.CTRL_MODE_SEL.value, 0x0004
        )
        # send_and_receive (not the old fire-and-forget send()) -- the
        # driver's write echo must be drained here, or it sits unread and
        # concatenates onto a later, unrelated transaction's response. See
        # modbus_rtu_client.py's _infer_expected_length() docstring.
        ctrl.modbus_client.send_and_receive.assert_called_once()

    def test_enable_position_mode_false_writes_0x0000(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.Enable_Position_Mode(False)
        ctrl.modbus_client.build_write_message.assert_called_once_with(
            ServoControlRegistry.CTRL_MODE_SEL.value, 0x0000
        )

    def test_enable_jog_mode_true_writes_0x0003(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.Enable_JOG_Mode(True)
        ctrl.modbus_client.build_write_message.assert_called_once_with(
            ServoControlRegistry.CTRL_MODE_SEL.value, 0x0003
        )

    def test_config_acc_dec_passes_value_through(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.config_acc_dec_0x0902(5000)
        ctrl.modbus_client.build_write_message.assert_called_once_with(0x0902, 5000)

    def test_config_speed_passes_value_through(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.config_speed_0x0903(42)
        ctrl.modbus_client.build_write_message.assert_called_once_with(0x0903, 42)

    def test_config_pulses_low_byte_uses_registry_address(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.config_pulses_0x0905_low_byte(0x1234)
        ctrl.modbus_client.build_write_message.assert_called_once_with(
            ServoControlRegistry.POS_PULSES_CMD_L.value, 0x1234
        )

    def test_config_pulses_high_byte_uses_registry_address(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.config_pulses_0x0906_high_byte(0x5678)
        ctrl.modbus_client.build_write_message.assert_called_once_with(
            ServoControlRegistry.POS_PULSES_CMD_H.value, 0x5678
        )

    def test_pos_motion_start_writes_value_to_0x0907(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.pos_motion_start_0x0907(2)
        ctrl.modbus_client.build_write_message.assert_called_once_with(0x0907, 2)


class TestSpeedCtrlAction(unittest.TestCase):

    def test_writes_action_value_to_0x0904(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse"):
            ctrl.speed_ctrl_action(1)
        ctrl.modbus_client.build_write_message.assert_called_once_with(0x0904, 1)

    def test_returns_true_on_a_normal_write(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse"):
            self.assertTrue(ctrl.speed_ctrl_action(1))


class TestSpeedCtrlActionDirectionReversalGuard(unittest.TestCase):
    """Regression coverage for the user-requested fail-safe (2026-09-18):
    reversing direction while the motor is still running the other way,
    without an intervening MOTION PAUSE, could shock the mechanism."""

    def _controller_with_mocked_wire(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        return ctrl

    def test_fresh_controller_allows_either_direction_first(self):
        ctrl = self._controller_with_mocked_wire()
        with patch("servo_control.ModbusRTUResponse"):
            self.assertTrue(ctrl.speed_ctrl_action(2))  # CW, no prior direction

    def test_repeating_the_same_direction_is_allowed(self):
        ctrl = self._controller_with_mocked_wire()
        with patch("servo_control.ModbusRTUResponse"):
            ctrl.speed_ctrl_action(2)
            self.assertTrue(ctrl.speed_ctrl_action(2))
        self.assertEqual(ctrl.modbus_client.build_write_message.call_count, 2)

    def test_direct_reversal_without_pause_is_refused(self):
        ctrl = self._controller_with_mocked_wire()
        with patch("servo_control.ModbusRTUResponse"):
            ctrl.speed_ctrl_action(2)  # CW
            ctrl.modbus_client.build_write_message.reset_mock()
            result = ctrl.speed_ctrl_action(1)  # CCW, no pause in between
        self.assertFalse(result)
        ctrl.modbus_client.build_write_message.assert_not_called()

    def test_reversal_is_allowed_after_an_explicit_pause(self):
        ctrl = self._controller_with_mocked_wire()
        with patch("servo_control.ModbusRTUResponse"):
            ctrl.speed_ctrl_action(2)  # CW
            ctrl.speed_ctrl_action(0)  # MOTION PAUSE
            result = ctrl.speed_ctrl_action(1)  # CCW now allowed
        self.assertTrue(result)

    def test_stop_continuous_reading_clears_the_guard(self):
        """A full stop (e.g. MOTION CANCEL) means no direction is active
        any more -- the next action shouldn't be refused as a "reversal"
        just because it doesn't match whatever was running before."""
        ctrl = self._controller_with_mocked_wire()
        with patch("servo_control.ModbusRTUResponse"):
            ctrl.speed_ctrl_action(2)  # CW
        ctrl.read_thread = None
        ctrl.reading_active = True
        ctrl.stop_continuous_reading()
        with patch("servo_control.ModbusRTUResponse"):
            result = ctrl.speed_ctrl_action(1)  # CCW, no pause, but state was reset
        self.assertTrue(result)


class TestReadMotionCompletedSignal(unittest.TestCase):

    def test_returns_true_when_value_nonzero(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = 1
            self.assertTrue(ctrl.Read_Motion_Completed_Signal())

    def test_returns_false_when_value_zero(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = 0
            self.assertFalse(ctrl.Read_Motion_Completed_Signal())

    def test_returns_false_on_communication_error_not_raise(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.side_effect = Exception("boom")
        self.assertFalse(ctrl.Read_Motion_Completed_Signal())


class TestReadPD16ToPD11DecodeFix(unittest.TestCase):
    """read_0x0206_To_0x020B() was ported from a version that iterated
    ModbusResponse's ASCII-only `.data` (hex-string chunks). ModbusRTUResponse
    has no such attribute -- only data_bytes (raw bytes) -- so the ported
    version must decode from data_bytes instead, or it silently never
    matches any DI_Function_Code (caught by the surrounding try/except, so
    it wouldn't crash, but would never work)."""

    def test_decodes_di_function_codes_from_raw_data_bytes(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            # SON = 0x01 (see status_bit_map.DI_Function_Code) as the first word.
            mock_cls.return_value.data_bytes = (0x0001).to_bytes(2, "big") + b'\x00\x00' * 5
            # Must not raise (this used to hit AttributeError: no `.data`).
            ctrl.read_0x0206_To_0x020B()

        ctrl.modbus_client.build_read_message.assert_called_once_with(0x0206, 6)


class TestWritePF82Validation(unittest.TestCase):

    def test_rejects_negative_value(self):
        ctrl = make_controller()
        with self.assertRaises(ValueError):
            ctrl.write_PF82(-1)

    def test_rejects_value_over_9999(self):
        ctrl = make_controller()
        with self.assertRaises(ValueError):
            ctrl.write_PF82(10000)

    def test_accepts_in_range_value(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse"):
            ctrl.write_PF82(5)  # must not raise
        ctrl.modbus_client.build_write_message.assert_called_once_with(PF.PRCM.address, 1)


class TestReadPosRelatedParameters(unittest.TestCase):

    def test_reads_each_expected_register_once(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        ctrl.delay_ms = MagicMock()  # skip the real 100ms sleep per register

        ctrl.Read_Pos_Related_Paremters()

        self.assertEqual(ctrl.modbus_client.build_read_message.call_count, 9)


class TestPosStepMotionTestAndExecutePositioning(unittest.TestCase):

    def test_pos_step_motion_test_cw_sends_1(self):
        ctrl = make_controller()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.pos_motion_start_0x0907 = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.pos_step_motion_test(CW=True)

        ctrl.pos_motion_start_0x0907.assert_called_once_with(1)

    def test_pos_step_motion_test_ccw_sends_2(self):
        ctrl = make_controller()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.pos_motion_start_0x0907 = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.pos_step_motion_test(CW=False)

        ctrl.pos_motion_start_0x0907.assert_called_once_with(2)

    def test_execute_positioning_clears_alarm_12_before_entering_position_mode(self):
        """Regression coverage for the 2026-09-18 finding: Positioning-test
        mode has the identical Step 1 precondition as JOG mode (no alarm +
        Servo OFF, docs/en_manual.txt:10390). Without clearing Alarm 12
        first, SET POINT 1/2, HOME, etc. would silently fail to enter
        position-test mode whenever Servo was already ON."""
        ctrl = make_controller()
        call_order = []
        ctrl.clear_alarm_12 = MagicMock(side_effect=lambda: call_order.append("clear_alarm_12"))
        ctrl.Enable_Position_Mode = MagicMock(side_effect=lambda v: call_order.append("position_mode"))
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.config_pulses_0x0905_low_byte = MagicMock()
        ctrl.config_pulses_0x0906_high_byte = MagicMock()
        ctrl.pos_step_motion_test = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl._execute_positioning(angle=10, low_byte=1, high_byte=0, acc_dec_time=100, speed_rpm=5)

        ctrl.clear_alarm_12.assert_called_once()
        self.assertEqual(call_order, ["clear_alarm_12", "position_mode"])

    def test_execute_positioning_positive_angle_runs_cw(self):
        ctrl = make_controller()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl.Enable_Position_Mode = MagicMock()
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.config_pulses_0x0905_low_byte = MagicMock()
        ctrl.config_pulses_0x0906_high_byte = MagicMock()
        ctrl.pos_step_motion_test = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl._execute_positioning(angle=10, low_byte=1, high_byte=0, acc_dec_time=100, speed_rpm=5)

        ctrl.pos_step_motion_test.assert_called_once_with(True)

    def test_execute_positioning_negative_angle_runs_ccw(self):
        ctrl = make_controller()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl.Enable_Position_Mode = MagicMock()
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.config_pulses_0x0905_low_byte = MagicMock()
        ctrl.config_pulses_0x0906_high_byte = MagicMock()
        ctrl.pos_step_motion_test = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl._execute_positioning(angle=-10, low_byte=1, high_byte=0, acc_dec_time=100, speed_rpm=5)

        ctrl.pos_step_motion_test.assert_called_once_with(False)


class TestEnableSpeedCtrl(unittest.TestCase):
    """Regression coverage for the 2026-09-18 real-hardware finding: per the
    SDE manual's JOG-test procedure, entering JOG mode requires writing
    0x0003 to CTRL_MODE_SEL (0x0901) via Enable_JOG_Mode(), not
    Enable_Position_Mode() (0x0004). The old enable=True branch called
    Enable_Position_Mode(False) and never configured speed/accel or entered
    JOG mode at all -- meaning the "ENABLE SPEED CONTROL MODE" button (which
    calls enable_speed_ctrl(100), leaving enable at its True default) never
    actually switched the drive into JOG mode before speed_ctrl_action()
    (MOTION START CW/CCW) tried to trigger movement via 0x0904."""

    def test_enable_true_clears_alarm_12_then_configures_speed_and_accel_then_enters_jog_mode(self):
        """clear_alarm_12() (not servo_off()) is what satisfies the manual's
        Step 1 precondition (no alarm + Servo OFF) without re-triggering
        Alarm 12 itself -- see the comment on this branch in
        servo_control.py."""
        ctrl = make_controller()
        call_order = []
        ctrl.clear_alarm_12 = MagicMock(side_effect=lambda: call_order.append("clear_alarm_12"))
        ctrl.config_speed_0x0903 = MagicMock(side_effect=lambda v: call_order.append("speed"))
        ctrl.config_acc_dec_0x0902 = MagicMock(side_effect=lambda v: call_order.append("accel"))
        ctrl.Enable_JOG_Mode = MagicMock(side_effect=lambda v: call_order.append("jog_mode"))
        ctrl.start_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=100, acc_time=5000, enable=True)

        ctrl.clear_alarm_12.assert_called_once()
        ctrl.config_speed_0x0903.assert_called_once_with(100)
        ctrl.config_acc_dec_0x0902.assert_called_once_with(5000)
        ctrl.Enable_JOG_Mode.assert_called_once_with(True)
        # Manual step order (docs/en_manual.txt:10346-10373): enter JOG mode
        # (Step 2) BEFORE setting accel (Step 3) and speed (Step 4) -- not
        # after. Getting this backwards was why the typed-in speed never
        # actually took effect on the drive.
        self.assertEqual(call_order, ["clear_alarm_12", "jog_mode", "accel", "speed"])
        # auto_stop_on_stillness=False: JOG mode runs continuously until
        # explicitly stopped -- a deliberate MOTION PAUSE must not be
        # mistaken for "the move finished" (see
        # TestSoftwareMotionCompleteDetection's matching test).
        ctrl.start_continuous_reading.assert_called_once_with(0.1, auto_stop_on_stillness=False)

    def test_enable_false_exits_jog_mode_without_touching_speed_or_accel(self):
        ctrl = make_controller()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.stop_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=200, acc_time=3000, enable=False)

        ctrl.clear_alarm_12.assert_not_called()
        ctrl.config_speed_0x0903.assert_not_called()
        ctrl.config_acc_dec_0x0902.assert_not_called()
        ctrl.Enable_JOG_Mode.assert_called_once_with(False)
        # Regression coverage for a bug found via live OSC testing
        # (2026-09-19): this used to fall through to the same
        # start_continuous_reading() call as enable=True, leaving
        # reading_active=True (background poll thread running) forever
        # after an explicit "disable" request.
        ctrl.start_continuous_reading.assert_not_called()
        ctrl.stop_continuous_reading.assert_called_once()

    def test_enable_as_string_true_is_coerced(self):
        """Regression coverage for a bug found via live OSC testing
        (2026-09-19): /set_continous_motion sent enable as the string
        "True" (not a native OSC boolean) -- `"True" == True` is False in
        Python, so this silently took the disable branch instead."""
        ctrl = make_controller()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=100, acc_time=5000, enable="True")

        ctrl.Enable_JOG_Mode.assert_called_once_with(True)
        ctrl.config_speed_0x0903.assert_called_once_with(100)

    def test_enable_as_string_false_is_coerced(self):
        ctrl = make_controller()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.stop_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=100, acc_time=5000, enable="False")

        ctrl.Enable_JOG_Mode.assert_called_once_with(False)


class TestIsAlarmActive(unittest.TestCase):
    """Regression coverage for the 2026-09-18 real-hardware finding: this
    driver reports 0xFF (255), not just 0, for "no alarm" -- confirmed by
    checking the physical panel (showed "AL --", its own no-alarm display)
    right after a clear_alarm_12() call that left the register at 255.
    Before this, /alarm/clear's `after_code == 0` check reported "failed"
    on a clear that had actually succeeded."""

    def test_zero_is_not_active(self):
        self.assertFalse(is_alarm_active(0))

    def test_0xff_is_not_active(self):
        self.assertFalse(is_alarm_active(0xFF))
        self.assertFalse(is_alarm_active(255))

    def test_none_is_active(self):
        """None means "communication failure, status unknown" -- must
        never be treated as "safe"/"no alarm"."""
        self.assertTrue(is_alarm_active(None))

    def test_a_real_named_alarm_is_active(self):
        self.assertTrue(is_alarm_active(0x12))  # AL.12, Emergency stop

    def test_no_alarm_codes_contains_exactly_zero_and_0xff(self):
        self.assertEqual(NO_ALARM_CODES, {0, 0xFF})


class TestSoftwareMotionCompleteDetection(unittest.TestCase):
    """Regression coverage for the 2026-09-18 real-hardware finding:
    Read_Motion_Completed_Signal() (PF.PRCM, a PATH-execution status
    register) read as "already complete" from the very first poll of
    _read_continuously(), regardless of whether the motor had moved at
    all -- confirmed by live timing (reading_active flipped true->false in
    ~1.5s while current_angle barely changed). Replaced with a
    software-only check: track real encoder deltas, only declare
    completion after genuine motion followed by sustained stillness."""

    def _run_continuous_reading_with_sequence(self, ctrl, encoder_values, join_timeout=2):
        """Feeds encoder_values in order, then holds at the last value
        indefinitely (so the mock never raises StopIteration if the loop
        runs a few extra iterations before observing the stop event)."""
        values = list(encoder_values)

        def _side_effect():
            if values:
                return values.pop(0)
            return encoder_values[-1]

        ctrl.read_encoder_before_gear_ratio = MagicMock(side_effect=_side_effect)
        ctrl.delay_ms = MagicMock()  # no real sleeping -- deterministic, fast test
        ctrl.start_continuous_reading(interval=0.001)
        thread = ctrl.read_thread
        if thread is not None:
            thread.join(timeout=join_timeout)

    def test_auto_stops_after_motion_then_sustained_stillness(self):
        ctrl = make_controller()
        # Baseline read, then a real jump (motion), then it settles and
        # holds -- exactly the "moved, then arrived" pattern a real
        # position command produces.
        settled_value = STILL_THRESHOLD_PULSES + 1
        sequence = [0, settled_value] + [settled_value] * (STILL_COUNT_TO_COMPLETE + 5)

        completed_events = []
        ctrl.register_event_listener("on_motion_completed", lambda: completed_events.append(True))

        self._run_continuous_reading_with_sequence(ctrl, sequence)

        # stop_continuous_reading() resets _motion_seen/_still_count as part
        # of its own cleanup (both for a manual stop and this auto-stop
        # path), so they can't be inspected after the fact -- the real
        # evidence this was the *completion* auto-stop, not something else,
        # is: nothing external called stop_continuous_reading() here, yet
        # reading_active went false, on_motion_completed fired, and the
        # encoder settled at the value the sequence actually held at
        # (proving the full moved-then-stable pattern was consumed, not an
        # immediate/premature stop).
        self.assertFalse(ctrl.reading_active)
        self.assertEqual(completed_events, [True])
        self.assertEqual(ctrl.current_encoder, settled_value)

    def test_auto_stop_on_stillness_false_keeps_reading_active_through_a_pause(self):
        """Regression coverage for the 2026-09-18 finding: continuous JOG
        mode (enable_speed_ctrl()) is a continuous-run mode, not a discrete
        move -- pressing MOTION PAUSE makes the encoder go still on
        purpose, and that must not be mistaken for "the move finished" the
        way it legitimately is for pos_step_motion_test(). Without this
        flag, the auto-stop killed the background poll thread, which is
        also this mode's <1s drive keep-alive -- so the drive would then
        silently exit JOG mode, and the next MOTION START CW/CCW needed
        ENABLE SPEED CONTROL MODE pressed again first."""
        ctrl = make_controller()
        settled_value = STILL_THRESHOLD_PULSES + 1
        # Same "moved, then held still" pattern as the auto-stop test above,
        # but with far more settled reads than STILL_COUNT_TO_COMPLETE --
        # if auto-stop still fired here, it would have done so long before
        # this many iterations ran.
        sequence = [0, settled_value] + [settled_value] * (STILL_COUNT_TO_COMPLETE * 3)

        completed_events = []
        ctrl.register_event_listener("on_motion_completed", lambda: completed_events.append(True))
        ctrl.read_encoder_before_gear_ratio = MagicMock(side_effect=lambda: sequence.pop(0) if sequence else settled_value)
        ctrl.delay_ms = MagicMock()
        ctrl.start_continuous_reading(interval=0.001, auto_stop_on_stillness=False)
        try:
            deadline = time.time() + 2
            while sequence and time.time() < deadline:
                time.sleep(0.001)
            time.sleep(0.05)  # let a few more post-sequence iterations run
            self.assertTrue(ctrl.reading_active)
            self.assertEqual(completed_events, [])
        finally:
            ctrl.stop_continuous_reading()

    def test_does_not_auto_stop_if_encoder_never_moves(self):
        """This is the exact "ENABLE POS MODE alone" scenario: position
        mode is armed but no move (0x0907) is ever commanded, so the
        encoder legitimately never changes. Reading must keep running --
        auto-stopping here was the original bug."""
        ctrl = make_controller()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=500)
        ctrl.delay_ms = MagicMock(side_effect=lambda ms: time.sleep(0.001))
        ctrl.start_continuous_reading(interval=0.001)
        try:
            time.sleep(0.15)  # let well more than STILL_COUNT_TO_COMPLETE iterations run
            self.assertTrue(ctrl.reading_active)
            self.assertFalse(ctrl._motion_seen)
        finally:
            ctrl.stop_continuous_reading()
        self.assertFalse(ctrl.reading_active)

    def test_small_jitter_at_or_below_threshold_does_not_count_as_motion(self):
        ctrl = make_controller()
        base = 1000
        # Every delta is exactly STILL_THRESHOLD_PULSES -- the boundary
        # itself must NOT count as motion (condition is strictly >).
        jittery_sequence = [base, base + STILL_THRESHOLD_PULSES, base] * 10
        ctrl.read_encoder_before_gear_ratio = MagicMock(
            side_effect=jittery_sequence + [base] * 10
        )
        ctrl.delay_ms = MagicMock(side_effect=lambda ms: time.sleep(0.001))
        ctrl.start_continuous_reading(interval=0.001)
        try:
            time.sleep(0.15)
            self.assertTrue(ctrl.reading_active)
            self.assertFalse(ctrl._motion_seen)
        finally:
            ctrl.stop_continuous_reading()

    def test_start_continuous_reading_resets_detection_state(self):
        ctrl = make_controller()
        ctrl._motion_seen = True
        ctrl._still_count = 99
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1)
        ctrl.delay_ms = MagicMock(side_effect=lambda ms: time.sleep(0.001))
        ctrl.start_continuous_reading(interval=0.001)
        # Immediately after (re)starting, state must be fresh, not carried
        # over from whatever the previous session left behind.
        self.assertFalse(ctrl._motion_seen)
        self.assertEqual(ctrl._still_count, 0)
        ctrl.stop_continuous_reading()

    def test_calling_start_while_already_active_leaves_it_active(self):
        """Regression coverage for the 2026-09-18 finding: this used to
        stop the existing session and return, instead of ensuring reading
        was active for the caller that just asked to start it.
        pos_step_motion_test() calls start_continuous_reading() right
        before triggering the actual move (0x0907) -- if that killed the
        keep-alive poll thread instead of leaving it running, the drive's
        own 1-second communication timeout could silently exit test mode
        (and Servo-off) right as the move was supposed to start, i.e. the
        motor just wouldn't move."""
        ctrl = make_controller()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1)
        ctrl.delay_ms = MagicMock(side_effect=lambda ms: time.sleep(0.001))
        ctrl.start_continuous_reading(interval=0.001, auto_stop_on_stillness=False)
        try:
            time.sleep(0.02)
            thread_before = ctrl.read_thread
            self.assertTrue(ctrl.reading_active)

            # A second start_continuous_reading() call, as
            # pos_step_motion_test() makes -- must leave reading active
            # (same thread, not stopped), not stop it.
            ctrl.start_continuous_reading(interval=0.001)
            self.assertTrue(ctrl.reading_active)
            self.assertIs(ctrl.read_thread, thread_before)
        finally:
            ctrl.stop_continuous_reading()

    def test_second_call_refreshes_auto_stop_on_stillness_flag(self):
        """The second call's auto_stop_on_stillness must take effect even
        though the thread isn't restarted -- e.g. enable_speed_ctrl() (False)
        left reading active, and a later pos_step_motion_test() call
        (default True) needs its own move's completion to actually
        auto-stop."""
        ctrl = make_controller()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1)
        ctrl.delay_ms = MagicMock(side_effect=lambda ms: time.sleep(0.001))
        ctrl.start_continuous_reading(interval=0.001, auto_stop_on_stillness=False)
        try:
            ctrl.start_continuous_reading(interval=0.001, auto_stop_on_stillness=True)
            self.assertTrue(ctrl._auto_stop_on_stillness)
        finally:
            ctrl.stop_continuous_reading()


class TestReadServoState(unittest.TestCase):

    def test_bit0_set_returns_true(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = 0x01
            self.assertTrue(ctrl.read_servo_state())
        ctrl.modbus_client.build_read_message.assert_called_once_with(0x0200, 1)

    def test_bit0_clear_returns_false(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = 0x00
            self.assertFalse(ctrl.read_servo_state())

    def test_other_bits_set_but_bit0_clear_is_still_false(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = 0b1110
            self.assertFalse(ctrl.read_servo_state())

    def test_no_response_returns_none(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = None
        self.assertIsNone(ctrl.read_servo_state())


class TestReadMcOkStatus(unittest.TestCase):
    """MC_OK (CMDOK AND INP) reconstructed from the DO function-assignment
    registers (0x020C/0x020D) + DO_STATUS (0x0205), per the 2026-09-18
    manual research: this unit's DO1=INP, DO3=CMDOK at factory default, but
    the lookup is dynamic rather than hardcoded to those pins."""

    def _mock_reads(self, ctrl, do1_2_3_assignment, do4_5_6_assignment, do_status_value):
        """assignment values are already the packed 16-bit register value
        (as read from 0x020C/0x020D); do_status_value is DO_STATUS's raw
        value."""
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        patcher = patch("servo_control.ModbusRTUResponse")
        mock_cls = patcher.start()
        self.addCleanup(patcher.stop)
        mock_cls.return_value.get_value.side_effect = [
            do1_2_3_assignment, do4_5_6_assignment, do_status_value,
        ]
        return ctrl

    def _pack(self, fn1, fn2, fn3):
        return (fn1 & 0x1F) | ((fn2 & 0x1F) << 5) | ((fn3 & 0x1F) << 10)

    def test_factory_default_do1_inp_do3_cmdok_both_on(self):
        ctrl = make_controller()
        # DO1=INP(0x03), DO2=ZSP(0x08), DO3=CMDOK(0x09) -- factory default
        do1_2_3 = self._pack(0x03, 0x08, 0x09)
        # DO4=TLC(0x05), DO5=RD(0x01), DO6=ALM(0x02) -- factory default
        do4_5_6 = self._pack(0x05, 0x01, 0x02)
        # DO_STATUS: DO1 (bit0) and DO3 (bit2) both on
        do_status = (1 << 0) | (1 << 2)
        self._mock_reads(ctrl, do1_2_3, do4_5_6, do_status)

        self.assertTrue(ctrl.read_mc_ok_status())

    def test_inp_on_but_cmdok_off_is_false(self):
        ctrl = make_controller()
        do1_2_3 = self._pack(0x03, 0x08, 0x09)
        do4_5_6 = self._pack(0x05, 0x01, 0x02)
        do_status = (1 << 0)  # only DO1/INP on, DO3/CMDOK off
        self._mock_reads(ctrl, do1_2_3, do4_5_6, do_status)

        self.assertFalse(ctrl.read_mc_ok_status())

    def test_neither_on_is_false(self):
        ctrl = make_controller()
        do1_2_3 = self._pack(0x03, 0x08, 0x09)
        do4_5_6 = self._pack(0x05, 0x01, 0x02)
        self._mock_reads(ctrl, do1_2_3, do4_5_6, do_status_value=0)

        self.assertFalse(ctrl.read_mc_ok_status())

    def test_works_regardless_of_which_do_pins_inp_cmdok_are_assigned_to(self):
        """Must not hardcode DO1/DO3 -- if someone reassigns these to
        different pins, the lookup should follow."""
        ctrl = make_controller()
        # INP moved to DO4, CMDOK moved to DO6 this time.
        do1_2_3 = self._pack(0x01, 0x02, 0x08)  # RD, ALM, ZSP
        do4_5_6 = self._pack(0x03, 0x05, 0x09)  # INP, TLC, CMDOK
        do_status = (1 << 3) | (1 << 5)  # DO4 (bit3) and DO6 (bit5) on
        self._mock_reads(ctrl, do1_2_3, do4_5_6, do_status)

        self.assertTrue(ctrl.read_mc_ok_status())

    def test_inp_not_assigned_anywhere_returns_none(self):
        ctrl = make_controller()
        do1_2_3 = self._pack(0x01, 0x02, 0x09)  # RD, ALM, CMDOK -- no INP
        do4_5_6 = self._pack(0x05, 0x08, 0x0A)
        self._mock_reads(ctrl, do1_2_3, do4_5_6, do_status_value=0xFF)

        self.assertIsNone(ctrl.read_mc_ok_status())

    def test_no_response_on_first_read_returns_none(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = None
        self.assertIsNone(ctrl.read_mc_ok_status())


if __name__ == "__main__":
    unittest.main()
