"""
Unit tests for servo_control.py.

Two groups:
1. New logic introduced by the servo_comm_shihlin_unified merge: closed-loop
   diff_angle basis, EncoderPulseTracker integration, the abs()-based
   the drive's command-pulse range check, directional float_error accumulation.
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
    ServoController, PositionUnavailableError, MoveOutOfRangeError, is_alarm_active, NO_ALARM_CODES,
    STILL_THRESHOLD_PULSES, STILL_COUNT_TO_COMPLETE,
)
from modbus_utils import ModbusUtils
from modbus_rtu_client import ModbusRTUClient
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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=60)

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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=None)

        result = ctrl.pos_step_motion_by(target_pos=1000)

        self.assertEqual(result, 0.0)
        ctrl._execute_positioning.assert_not_called()

    def test_a_move_of_more_than_180_degrees_is_allowed(self):
        """There is no 180 degree limit in this application (2026-09-21)."""
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=0)

        target = int(ctrl.base_pulse_per_degree * 200)
        result = ctrl.pos_step_motion_by(target_pos=target)

        ctrl._execute_positioning.assert_called_once()
        self.assertEqual(ctrl._execute_positioning.call_args[0][0], target)
        self.assertAlmostEqual(result, 200.0, places=3)

    def test_a_large_move_in_the_negative_direction_is_allowed(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=0)

        target = -int(ctrl.base_pulse_per_degree * 720)   # two full output turns back
        result = ctrl.pos_step_motion_by(target_pos=target)

        ctrl._execute_positioning.assert_called_once()
        self.assertEqual(ctrl._execute_positioning.call_args[0][0], target)
        self.assertAlmostEqual(result, -720.0, places=3)

    def test_a_move_beyond_the_drives_pulse_register_is_refused(self):
        """0x0905/0x0906 hold 0..2^31-1 command pulses (manual, docs/en_manual.txt
        ~10416). More would be truncated into a wrong, shorter move -- that is a
        hardware limit, not an application limit."""
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=0)

        for target in (2**31, -(2**31)):
            with self.subTest(target=target):
                self.assertEqual(ctrl.pos_step_motion_by(target_pos=target), 0.0)
        ctrl._execute_positioning.assert_not_called()

    def test_the_largest_move_the_register_can_hold_is_still_sent(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=0)

        ctrl.pos_step_motion_by(target_pos=2**31 - 1)

        ctrl._execute_positioning.assert_called_once()



class TestPostStepMotionByClosedLoop(unittest.TestCase):
    """post_step_motion_by() must compute diff_angle from
    target_angle - current_angle, where current_angle is only ever set by
    the continuous-reading feedback loop -- never overwritten here (design
    doc §2.2 #4, the closed-loop decision)."""

    def test_diff_uses_target_minus_current_angle(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
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

    def test_relative_move_goes_by_the_amount_from_the_position_just_read(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 30.0

        ctrl.post_step_motion_by(angle=-12.5, speed_rpm=5, relative=True)

        self.assertAlmostEqual(ctrl.target_angle, 17.5)
        self.assertAlmostEqual(ctrl._execute_positioning.call_args[0][0], -12.5)

    def test_a_relative_move_of_more_than_180_degrees_is_allowed(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 100.0

        ctrl.post_step_motion_by(angle=200.0, relative=True)

        ctrl._execute_positioning.assert_called_once()
        self.assertAlmostEqual(ctrl._execute_positioning.call_args[0][0], 200.0)
        self.assertAlmostEqual(ctrl.target_angle, 300.0)

    def test_relative_move_refuses_when_the_position_is_unreadable(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=False)
        ctrl._execute_positioning = MagicMock()

        with self.assertRaises(PositionUnavailableError):
            ctrl.post_step_motion_by(angle=10.0, relative=True)
        ctrl._execute_positioning.assert_not_called()

    def test_a_large_positive_change_is_allowed_and_sends_the_right_pulses(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0

        ctrl.post_step_motion_by(angle=200.0)

        ctrl._execute_positioning.assert_called_once()
        angle, low, high = ctrl._execute_positioning.call_args[0][:3]
        self.assertAlmostEqual(angle, 200.0)
        self.assertEqual((high << 16) | low, int(ctrl.base_pulse_per_degree * 200))

    def test_a_large_negative_change_is_allowed_and_keeps_its_sign(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 220.0

        ctrl.post_step_motion_by(angle=0.0)   # e.g. HOME from 220 deg away

        ctrl._execute_positioning.assert_called_once()
        angle, low, high = ctrl._execute_positioning.call_args[0][:3]
        self.assertAlmostEqual(angle, -220.0)
        self.assertEqual((high << 16) | low, int(ctrl.base_pulse_per_degree * 220))

    def test_several_full_turns_are_allowed(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0
        ctrl.post_step_motion_by(angle=1080.0)
        ctrl._execute_positioning.assert_called_once()

    def test_a_move_beyond_the_pulse_register_is_refused_and_changes_no_state(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0
        ctrl.float_error, ctrl.accumulate_pulse = 0.25, 1000
        too_far = (2**31 + 1000) / ctrl.base_pulse_per_degree      # about 6144.4 deg

        with self.assertRaises(MoveOutOfRangeError):
            ctrl.post_step_motion_by(angle=too_far)
        with self.assertRaises(MoveOutOfRangeError):
            ctrl.post_step_motion_by(angle=-too_far)

        ctrl._execute_positioning.assert_not_called()
        self.assertEqual((ctrl.float_error, ctrl.accumulate_pulse), (0.25, 1000))

    def test_the_largest_move_the_register_can_hold_is_still_sent(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0
        ctrl.post_step_motion_by(angle=(2**31 - 2) / ctrl.base_pulse_per_degree)
        ctrl._execute_positioning.assert_called_once()

    def test_a_non_finite_angle_is_refused_before_anything_is_read_or_sent(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(angle=bad):
                with self.assertRaises(MoveOutOfRangeError):
                    ctrl.post_step_motion_by(angle=bad)
                with self.assertRaises(MoveOutOfRangeError):
                    ctrl.post_step_motion_by(angle=bad, relative=True)
        ctrl._refresh_current_angle_from_hardware.assert_not_called()
        ctrl._execute_positioning.assert_not_called()

    def test_out_of_range_is_a_value_error_so_existing_callers_keep_working(self):
        self.assertTrue(issubclass(MoveOutOfRangeError, ValueError))

    def test_directional_float_error_accumulation(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
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
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0
        ctrl.accumulate_pulse = 0

        ctrl.post_step_motion_by(angle=10.0)

        self.assertGreater(ctrl.accumulate_pulse, 0)


    def test_reads_the_real_position_before_computing_the_move(self):
        """Regression test for E1 (docs §7.E1): current_angle is stale (0.0)
        right after a process start. Real position 14.43 deg, target 20 deg:
        the move must be 5.57 deg, not 20."""
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0  # stale __init__ default

        def fake_refresh():
            ctrl.current_angle = 14.43
            return True

        ctrl._refresh_current_angle_from_hardware = MagicMock(side_effect=fake_refresh)

        ctrl.post_step_motion_by(angle=20.0)

        self.assertAlmostEqual(ctrl._execute_positioning.call_args[0][0], 5.57, places=4)

    def test_already_at_target_after_refresh_does_not_move(self):
        # The exact 2026-09-19 case: motor really at 14.43 = target.
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0

        def fake_refresh():
            ctrl.current_angle = 14.43
            return True

        ctrl._refresh_current_angle_from_hardware = MagicMock(side_effect=fake_refresh)

        ctrl.post_step_motion_by(angle=14.43)

        ctrl._execute_positioning.assert_not_called()

    def test_refuses_to_move_when_the_position_cannot_be_read(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.current_angle = 0.0
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=False)

        with self.assertRaises(PositionUnavailableError):
            ctrl.post_step_motion_by(angle=20.0)

        ctrl._execute_positioning.assert_not_called()

    def test_position_unavailable_is_a_value_error(self):
        # move_to_set_point()'s callers already turn ValueError into a 400.
        self.assertTrue(issubclass(PositionUnavailableError, ValueError))


class TestCancelContinuousReading(unittest.TestCase):

    def test_delegates_to_stop_continuous_reading_not_duplicated(self):
        ctrl = make_controller()
        ctrl.reading_active = True
        ctrl.stop_continuous_reading = MagicMock(wraps=ctrl.stop_continuous_reading)
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=1000)

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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=1000)

        ctrl.cancel_continuous_reading()

        ctrl.Enable_Position_Mode.assert_called_once_with(False)

    def test_updates_tracker_and_fires_on_cancel(self):
        ctrl = make_controller()
        ctrl.reading_active = True
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=1000)
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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=None)

        received = []
        ctrl.register_event_listener("on_cancel", lambda angle: received.append(angle))

        ctrl.cancel_continuous_reading()  # should not raise

        self.assertEqual(received, [])


class TestSetHomePosition(unittest.TestCase):

    def test_resets_tracker_consistently_with_saved_abs_home_pos(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.read_motor_feedback_pulses = MagicMock(return_value=777)

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
            ctrl.read_motor_feedback_pulses = MagicMock(return_value=1)
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


class TestSetPointRecording(unittest.TestCase):
    """SET POINT 1/2 record wherever the motor currently is -- they must
    never command a move. Persisted per-profile like abs_home_pos, via the
    same read-modify-write _save_config_value() helper."""

    def test_records_current_angle_without_moving(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.current_angle = 45.5
            ctrl.modbus_client.send_and_receive = MagicMock()

            result = ctrl.record_set_point(1)

            self.assertEqual(result, 45.5)
            self.assertEqual(ctrl.set_point_1, 45.5)
            self.assertEqual(ctrl.current_angle, 45.5)  # unchanged -- no move
            ctrl.modbus_client.send_and_receive.assert_not_called()

    def test_set_point_2_recorded_independently(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.current_angle = 12.0
            ctrl.record_set_point(1)
            ctrl.current_angle = 99.0

            ctrl.record_set_point(2)

            self.assertEqual(ctrl.set_point_1, 12.0)
            self.assertEqual(ctrl.set_point_2, 99.0)

    def test_invalid_set_point_number_raises(self):
        ctrl = make_controller()
        with self.assertRaises(ValueError):
            ctrl.record_set_point(3)

    def test_save_config_value_does_not_erase_other_keys(self):
        # Regression test: an earlier version of _save_config_value()
        # overwrote the whole config file with only {key: value}, which
        # would have silently erased abs_home_pos the moment a set point
        # was saved (or vice versa).
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.save_abs_home_pos(111)
            ctrl.current_angle = 22.5

            ctrl.record_set_point(1)

            import json
            with open(ctrl.config_file) as f:
                saved = json.load(f)
            self.assertEqual(saved["abs_home_pos"], 111)
            self.assertEqual(saved["set_point_1"], 22.5)

    def test_init_loads_existing_set_points_from_config_file(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            original_cwd = os.getcwd()
            os.chdir(tmp_dir)
            try:
                import json
                config_name = f"servo_config_{PROFILE_400W['name']}.json"
                with open(config_name, 'w') as f:
                    json.dump({"set_point_1": 30.0, "set_point_2": 60.0}, f)

                ctrl = make_controller()

                self.assertEqual(ctrl.set_point_1, 30.0)
                self.assertEqual(ctrl.set_point_2, 60.0)
            finally:
                os.chdir(original_cwd)

    def test_init_defaults_to_none_when_never_recorded(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            original_cwd = os.getcwd()
            os.chdir(tmp_dir)
            try:
                ctrl = make_controller()
                self.assertIsNone(ctrl.set_point_1)
                self.assertIsNone(ctrl.set_point_2)
            finally:
                os.chdir(original_cwd)


class TestMoveToSetPoint(unittest.TestCase):
    """MOVE TO SET POINT 1/2 commands an actual move to the previously
    recorded angle, via the same post_step_motion_by() path as HOME --
    distinct from SET POINT 1/2 (record_set_point()), which only records
    and never moves."""

    def test_moves_to_the_recorded_angle(self):
        ctrl = make_controller()
        ctrl.set_point_1 = 45.5
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl.post_step_motion_by = MagicMock()

        ctrl.move_to_set_point(1)

        ctrl.post_step_motion_by.assert_called_once_with(45.5, 5000, 10)

    def test_set_point_2_uses_its_own_value(self):
        ctrl = make_controller()
        ctrl.set_point_1 = 10.0
        ctrl.set_point_2 = 99.0
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl.post_step_motion_by = MagicMock()

        ctrl.move_to_set_point(2)

        ctrl.post_step_motion_by.assert_called_once_with(99.0, 5000, 10)

    def test_unreadable_position_refuses_the_move_end_to_end(self):
        """With the real post_step_motion_by(): a position that can't be read
        must surface as a ValueError and never reach the positioning code."""
        ctrl = make_controller()
        ctrl.set_point_1 = 45.5
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=False)
        ctrl._execute_positioning = MagicMock()

        with self.assertRaises(ValueError):
            ctrl.move_to_set_point(1)

        ctrl._execute_positioning.assert_not_called()

    def test_unrecorded_set_point_raises_without_moving(self):
        ctrl = make_controller()
        ctrl.set_point_1 = None
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl.post_step_motion_by = MagicMock()

        with self.assertRaises(ValueError):
            ctrl.move_to_set_point(1)

        ctrl.post_step_motion_by.assert_not_called()

    def test_invalid_set_point_number_raises(self):
        ctrl = make_controller()
        with self.assertRaises(ValueError):
            ctrl.move_to_set_point(3)


class TestRefreshCurrentAngleFromHardware(unittest.TestCase):

    def test_updates_current_angle_from_a_fresh_read(self):
        ctrl = make_controller()
        ctrl.abs_home_pos = 0
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=349525)

        result = ctrl._refresh_current_angle_from_hardware()

        self.assertTrue(result)
        self.assertEqual(ctrl.current_encoder, 349525)
        self.assertAlmostEqual(ctrl.current_angle, 1.0, places=2)

    def test_empty_response_leaves_current_angle_untouched(self):
        ctrl = make_controller()
        ctrl.current_angle = 42.0
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=None)

        result = ctrl._refresh_current_angle_from_hardware()

        self.assertFalse(result)
        self.assertEqual(ctrl.current_angle, 42.0)


class TestEncoderModeRegisters(unittest.TestCase):
    """PA23/PA28/PA30~PA33 register handling. No real drive involved: register
    I/O is a small in-memory model (writes update the value the next read
    returns), so read-back verification and handshakes behave realistically."""

    def _ctrl(self, values=None, reject_writes=(), uap_reads_before_zero=0):
        """values: initial register values by name. reject_writes: (name, value)
        pairs the fake drive refuses (as older firmware refuses PA23=2).
        uap_reads_before_zero: how many reads of PA30 return 1 after it is
        written before it reads 0 (the PA30 handshake)."""
        state = {"MCS": 2, "ABS": 0, "UAP": 0, "APST": 0, "APR": 0, "APP": 0}
        state.update(values or {})
        pending_uap = {"n": 0}
        ctrl = make_controller()
        ctrl.regs = state
        ctrl.write_log = []

        def fake_write(register, value):
            ctrl.write_log.append((register.name, value))
            if (register.name, value) in reject_writes:
                return False
            state[register.name] = value
            if register.name == "UAP":
                pending_uap["n"] = uap_reads_before_zero
            return True

        def fake_read(register, words=2, signed=False):
            if register.name == "UAP" and pending_uap["n"] > 0:
                pending_uap["n"] -= 1
                return 1
            if register.name == "UAP":
                state["UAP"] = 0
            return state[register.name]

        ctrl._write_parameter = MagicMock(side_effect=fake_write)
        ctrl._read_parameter = MagicMock(side_effect=fake_read)
        ctrl.delay_ms = MagicMock()
        return ctrl

    # ---- PA28 ----
    def test_read_pa28(self):
        self.assertEqual(self._ctrl({"ABS": 1}).read_PA28_Encoder_Mode(), 1)

    def test_write_pa28_verifies_by_readback(self):
        ctrl = self._ctrl({"MCS": 0})
        self.assertTrue(ctrl.write_PA28_Encoder_Mode(True))
        self.assertEqual(ctrl.regs["ABS"], 1)

    def test_write_pa28_fails_if_drive_rejects_it(self):
        ctrl = self._ctrl({"MCS": 0}, reject_writes=[("ABS", 1)])
        self.assertFalse(ctrl.write_PA28_Encoder_Mode(True))

    def test_write_pa28_does_not_flip_active_mode(self):
        # PA28 only takes effect after a power cycle; the software must keep
        # using the incremental path until refresh_encoder_mode() says so.
        ctrl = self._ctrl({"MCS": 0})
        ctrl.write_PA28_Encoder_Mode(True)
        self.assertFalse(ctrl.absolute_mode)

    def test_write_pa28_lifts_eeprom_protection_then_restores_it(self):
        # With PA23=2 the PA28 write would never reach the EEPROM and would be
        # lost at the very power cycle meant to apply it.
        ctrl = self._ctrl({"MCS": 2})
        self.assertTrue(ctrl.write_PA28_Encoder_Mode(True))
        names = [w for w in ctrl.write_log]
        self.assertEqual(names[0], ("MCS", 0))
        self.assertEqual(names[1], ("ABS", 1))
        self.assertEqual(names[-1], ("MCS", 2))
        self.assertEqual(ctrl.regs["MCS"], 2)

    def test_write_pa28_restores_protection_even_when_the_write_fails(self):
        ctrl = self._ctrl({"MCS": 2}, reject_writes=[("ABS", 1)])
        self.assertFalse(ctrl.write_PA28_Encoder_Mode(True))
        self.assertEqual(ctrl.regs["MCS"], 2)

    def test_write_pa28_refused_if_protection_cannot_be_lifted(self):
        ctrl = self._ctrl({"MCS": 2}, reject_writes=[("MCS", 0)])
        self.assertFalse(ctrl.write_PA28_Encoder_Mode(True))
        self.assertNotIn(("ABS", 1), ctrl.write_log)

    def test_write_pa28_refused_if_pa23_unreadable(self):
        ctrl = self._ctrl()
        ctrl.read_PA23_Memory_Write_Inhibit = MagicMock(return_value=None)
        self.assertFalse(ctrl.write_PA28_Encoder_Mode(True))
        self.assertEqual(ctrl.write_log, [])

    def test_refresh_encoder_mode_sets_flag(self):
        ctrl = self._ctrl({"ABS": 1})
        self.assertTrue(ctrl.refresh_encoder_mode())
        self.assertTrue(ctrl.absolute_mode)

    def test_refresh_encoder_mode_unreadable_keeps_previous_mode(self):
        ctrl = self._ctrl()
        ctrl.read_PA28_Encoder_Mode = MagicMock(return_value=None)
        ctrl.absolute_mode = True
        self.assertIsNone(ctrl.refresh_encoder_mode())
        self.assertTrue(ctrl.absolute_mode)

    # ---- PA23 EEPROM write protection ----
    def test_protection_already_on_writes_nothing(self):
        for value in (1, 2):
            ctrl = self._ctrl({"MCS": value})
            self.assertEqual(ctrl.ensure_eeprom_write_protection(), value)
            self.assertEqual(ctrl.write_log, [], "PA23=%d" % value)

    def test_unprotected_drive_gets_pa23_2_first(self):
        ctrl = self._ctrl({"MCS": 0})
        self.assertEqual(ctrl.ensure_eeprom_write_protection(), 2)
        self.assertEqual(ctrl.write_log, [("MCS", 2)])
        self.assertEqual(ctrl.eeprom_protection, 2)

    def test_old_firmware_rejecting_2_falls_back_to_1(self):
        ctrl = self._ctrl({"MCS": 0}, reject_writes=[("MCS", 2)])
        self.assertEqual(ctrl.ensure_eeprom_write_protection(), 1)
        self.assertEqual(ctrl.regs["MCS"], 1)

    def test_firmware_that_accepts_but_ignores_2_is_caught_by_readback(self):
        ctrl = self._ctrl({"MCS": 0})
        real_write = ctrl._write_parameter.side_effect

        def clamping_write(register, value):
            real_write(register, value)
            if register.name == "MCS" and value == 2:
                ctrl.regs["MCS"] = 0  # not stored
            return True

        ctrl._write_parameter = MagicMock(side_effect=clamping_write)
        self.assertEqual(ctrl.ensure_eeprom_write_protection(), 1)

    def test_protection_impossible_reports_zero_and_does_not_raise(self):
        ctrl = self._ctrl({"MCS": 0}, reject_writes=[("MCS", 2), ("MCS", 1)])
        self.assertEqual(ctrl.ensure_eeprom_write_protection(), 0)
        self.assertEqual(ctrl.eeprom_protection, 0)

    def test_unreadable_pa23_returns_none_and_writes_nothing(self):
        ctrl = self._ctrl()
        ctrl.read_PA23_Memory_Write_Inhibit = MagicMock(return_value=None)
        self.assertIsNone(ctrl.ensure_eeprom_write_protection())
        self.assertEqual(ctrl.write_log, [])

    def test_max_age_skips_a_recent_check(self):
        ctrl = self._ctrl({"MCS": 2})
        ctrl.ensure_eeprom_write_protection()
        reads_before = ctrl._read_parameter.call_count
        ctrl.ensure_eeprom_write_protection(max_age_s=30)
        self.assertEqual(ctrl._read_parameter.call_count, reads_before)

    def test_power_cycle_reverting_pa23_is_reapplied(self):
        ctrl = self._ctrl({"MCS": 1})
        ctrl.ensure_eeprom_write_protection()
        ctrl.regs["MCS"] = 0  # drive was power-cycled: PA23=1 reverts to 0
        self.assertEqual(ctrl.ensure_eeprom_write_protection(), 2)

    # ---- absolute position read (PA30 handshake, layout, validation) ----
    # Default layout (Chinese V1.07): PA32 = pulses, PA33 = signed revolutions.
    def test_absolute_position_combines_revolutions_and_pulses(self):
        ctrl = self._ctrl({"APR": 1000, "APP": -2})
        self.assertEqual(ctrl.read_absolute_position_pulses(), -2 * 4194304 + 1000)

    def test_absolute_position_layout_can_be_swapped_per_profile(self):
        ctrl = self._ctrl({"APR": -2, "APP": 1000})
        ctrl.abs_rev_register = "APR"
        self.assertEqual(ctrl.read_absolute_position_pulses(), -2 * 4194304 + 1000)

    def test_invalid_abs_rev_register_in_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            make_controller(dict(PROFILE_400W, abs_rev_register="PA99"))

    def test_wrong_layout_is_rejected_not_used(self):
        # Real (rev=3, pulses=1234567) read with the opposite layout looks
        # like 1234567 revolutions -> outside +-32768 -> refused.
        ctrl = self._ctrl({"APR": 3, "APP": 1234567})
        self.assertIsNone(ctrl.read_absolute_position_pulses())

    def test_pulse_word_beyond_one_revolution_is_rejected(self):
        ctrl = self._ctrl({"APR": 4194304, "APP": 0})
        self.assertIsNone(ctrl.read_absolute_position_pulses())
        ctrl = self._ctrl({"APR": -1, "APP": 0})
        self.assertIsNone(ctrl.read_absolute_position_pulses())

    def test_absolute_position_waits_for_pa30_to_return_to_zero(self):
        ctrl = self._ctrl({"APR": 5, "APP": 1}, uap_reads_before_zero=3)
        self.assertEqual(ctrl.read_absolute_position_pulses(), 4194304 + 5)
        self.assertEqual(ctrl.delay_ms.call_count, 3)

    def test_absolute_position_none_if_pa30_never_returns_to_zero(self):
        ctrl = self._ctrl({"APR": 5, "APP": 1}, uap_reads_before_zero=10 ** 6)
        self.assertIsNone(ctrl.read_absolute_position_pulses())

    def test_absolute_position_registers_not_read_before_handshake_completes(self):
        ctrl = self._ctrl({"APR": 5, "APP": 1}, uap_reads_before_zero=10 ** 6)
        ctrl.read_absolute_position_pulses()
        read_names = [c.args[0].name for c in ctrl._read_parameter.call_args_list]
        self.assertNotIn("APR", read_names)
        self.assertNotIn("APP", read_names)

    def test_absolute_position_refused_when_status_reports_fault(self):
        for bit in (0, 1, 2, 4):
            ctrl = self._ctrl({"APST": 1 << bit})
            self.assertIsNone(ctrl.read_absolute_position_pulses(), "bit %d" % bit)

    def test_absolute_position_none_when_a_read_fails(self):
        for name in ("APR", "APP", "APST"):
            ctrl = self._ctrl()
            real_read = ctrl._read_parameter.side_effect
            ctrl._read_parameter = MagicMock(
                side_effect=lambda register, words=2, signed=False, n=name:
                    None if register.name == n else real_read(register, words, signed))
            self.assertIsNone(ctrl.read_absolute_position_pulses(), name)

    def test_absolute_position_none_when_pa30_write_fails(self):
        ctrl = self._ctrl(reject_writes=[("UAP", 1)])
        self.assertIsNone(ctrl.read_absolute_position_pulses())

    def test_absolute_read_verifies_eeprom_protection_first(self):
        ctrl = self._ctrl({"MCS": 0})
        ctrl.read_absolute_position_pulses()
        self.assertEqual(ctrl.write_log[0], ("MCS", 2))
        self.assertLess(ctrl.write_log.index(("MCS", 2)), ctrl.write_log.index(("UAP", 1)))

    def test_pa30_rejects_invalid_mode(self):
        with self.assertRaises(ValueError):
            self._ctrl().write_PA30_Update_Abs_Position(3)

    def test_explain_helpers(self):
        self.assertEqual(PA.explain_APST(0), "normal")
        self.assertIn("battery low voltage", PA.explain_APST(0b10))
        self.assertIn("absolute position lost", PA.explain_APST(0b1))
        self.assertIn("incremental", PA.explain_ABS(0))
        self.assertIn("AL.24", PA.explain_ABS(1))
        self.assertIn("wears", PA.explain_MCS(0))
        self.assertIn("persists", PA.explain_MCS(2))


class TestAbsoluteModePositioning(unittest.TestCase):
    """Absolute mode changes only where the position reference comes from;
    incremental behavior (default) is covered by the tests above."""

    def _abs_ctrl(self, tracker_raw=1000, absolute_pulses=5000000, home=4000000):
        ctrl = make_controller()
        ctrl.absolute_mode = True
        ctrl.abs_home_pos_absolute = home
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=tracker_raw)
        ctrl.read_absolute_position_pulses = MagicMock(return_value=absolute_pulses)
        return ctrl

    def test_refresh_uses_absolute_reading_and_absolute_home(self):
        ctrl = self._abs_ctrl(absolute_pulses=5000000, home=4000000)
        self.assertTrue(ctrl._refresh_current_angle_from_hardware())
        self.assertEqual(ctrl.current_encoder, 5000000)
        self.assertAlmostEqual(ctrl.current_angle, 1000000 / ctrl.base_pulse_per_degree, places=4)

    def test_refresh_ignores_the_incremental_home(self):
        ctrl = self._abs_ctrl()
        ctrl.abs_home_pos = 123  # must not be used in absolute mode
        ctrl._refresh_current_angle_from_hardware()
        self.assertAlmostEqual(ctrl.current_angle, 1000000 / ctrl.base_pulse_per_degree, places=4)

    def test_refresh_fails_without_absolute_home(self):
        ctrl = self._abs_ctrl(home=None)
        self.assertFalse(ctrl._refresh_current_angle_from_hardware())

    def test_refresh_fails_when_absolute_position_untrustworthy(self):
        ctrl = self._abs_ctrl(absolute_pulses=None)
        self.assertFalse(ctrl._refresh_current_angle_from_hardware())

    def test_refresh_records_offset_for_the_continuous_loop(self):
        ctrl = self._abs_ctrl(tracker_raw=1000, absolute_pulses=5000000)
        ctrl._refresh_current_angle_from_hardware()
        # Loop later reads tracker value 1500 -> absolute scale 5000500.
        encoder, _ = ctrl._encoder_and_angle_for(ctrl._encoder_tracker.update(1500))
        self.assertEqual(encoder, 5000500)

    def test_loop_helper_is_unchanged_in_incremental_mode(self):
        ctrl = make_controller()
        ctrl.abs_home_pos = 1000
        encoder, angle = ctrl._encoder_and_angle_for(1000 + 349525)
        self.assertEqual(encoder, 1000 + 349525)
        self.assertAlmostEqual(angle, 1.0, places=2)

    def test_pos_step_motion_by_targets_the_absolute_scale(self):
        ctrl = self._abs_ctrl(absolute_pulses=5000000)
        ctrl._execute_positioning = MagicMock()
        ctrl.pos_step_motion_by(target_pos=5000000 + 1000)
        self.assertEqual(ctrl._execute_positioning.call_args[0][0], 1000)

    def test_initial_abs_home_refuses_without_an_absolute_home(self):
        ctrl = self._abs_ctrl(home=None)
        ctrl.pos_step_motion_by = MagicMock()
        self.assertFalse(ctrl.initial_abs_home())
        ctrl.pos_step_motion_by.assert_not_called()

    def test_set_home_captures_absolute_position_and_persists_it(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = self._abs_ctrl(absolute_pulses=7777777, home=None)
            ctrl.config_file = os.path.join(tmp_dir, ctrl.config_file)
            ctrl.delay_ms = MagicMock()

            ctrl.set_home_position()

            self.assertEqual(ctrl.abs_home_pos_absolute, 7777777)
            self.assertEqual(ctrl.current_angle, 0.0)
            import json
            with open(ctrl.config_file) as f:
                self.assertEqual(json.load(f)["abs_home_pos_absolute"], 7777777)
            self.assertEqual(ctrl.abs_home_pos, 62369153)  # incremental home untouched

    def test_set_home_refuses_when_absolute_position_unavailable(self):
        ctrl = self._abs_ctrl(absolute_pulses=None, home=None)
        ctrl.current_angle = 42.0
        ctrl.set_home_position()
        self.assertIsNone(ctrl.abs_home_pos_absolute)
        self.assertEqual(ctrl.current_angle, 42.0)


class TestFeedbackReadersAndGearRatio(unittest.TestCase):

    def test_translated_feedback_reads_0x0024_two_words_and_returns_value(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b"x"
        with patch("servo_control.ModbusRTUResponse") as response_cls:
            response_cls.return_value.get_value.return_value = 4242
            self.assertEqual(ctrl.read_motor_feedback_pulses_0x0024(), 4242)
        ctrl.modbus_client.build_read_message.assert_called_once_with(0x0024, 2)

    def test_translated_feedback_is_none_on_no_response(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = None
        self.assertIsNone(ctrl.read_motor_feedback_pulses_0x0024())

    def _gear_ctrl(self, cmx, cdv):
        ctrl = make_controller()
        values = {"CMX": cmx, "CDV": cdv}
        ctrl._read_parameter = MagicMock(side_effect=lambda register, *a, **k: values[register.name])
        return ctrl

    def test_unity_gear_ratio_is_recorded_ok(self):
        ctrl = self._gear_ctrl(1, 1)
        self.assertEqual(ctrl.check_electronic_gear_ratio(), (1, 1))
        self.assertTrue(ctrl.electronic_gear_unity)

    def test_equal_but_not_one_still_counts_as_unity(self):
        ctrl = self._gear_ctrl(4, 4)
        ctrl.check_electronic_gear_ratio()
        self.assertTrue(ctrl.electronic_gear_unity)

    def test_non_unity_ratio_is_flagged_and_warned_about(self):
        ctrl = self._gear_ctrl(2, 1)
        with self.assertLogs("servo_control", level="WARNING") as logs:
            ctrl.check_electronic_gear_ratio()
        self.assertFalse(ctrl.electronic_gear_unity)
        self.assertEqual(ctrl.electronic_gear, (2, 1))
        self.assertIn("2/1", logs.output[0])

    def test_unreadable_ratio_is_unknown_not_assumed_ok(self):
        ctrl = self._gear_ctrl(1, None)
        self.assertIsNone(ctrl.check_electronic_gear_ratio())
        self.assertIsNone(ctrl.electronic_gear_unity)

    def test_gear_ratio_never_written(self):
        ctrl = self._gear_ctrl(2, 1)
        ctrl._write_parameter = MagicMock()
        ctrl.check_electronic_gear_ratio()
        ctrl._write_parameter.assert_not_called()


class TestHomeSetSinceStart(unittest.TestCase):
    """The web UI reminds the operator to SET HOME after every start: the
    incremental counter restarts at drive power-on, so a saved home from an
    earlier run can't be trusted."""

    def test_false_until_home_is_set(self):
        self.assertFalse(make_controller().home_set_since_start)

    def test_incremental_set_home_marks_it(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.read_motor_feedback_pulses = MagicMock(return_value=500)
            ctrl.set_home_position()
            self.assertTrue(ctrl.home_set_since_start)

    def test_unreadable_encoder_does_not_mark_it_or_touch_state(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.current_angle = 42.0
            ctrl.read_motor_feedback_pulses = MagicMock(return_value=None)
            ctrl.set_home_position()
            self.assertFalse(ctrl.home_set_since_start)
            self.assertEqual(ctrl.current_angle, 42.0)

    def test_absolute_set_home_marks_it_only_on_success(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctrl = make_controller(tmp_dir=tmp_dir)
            ctrl.absolute_mode = True
            ctrl.delay_ms = MagicMock()
            ctrl.read_motor_feedback_pulses = MagicMock(return_value=1000)
            ctrl.read_absolute_position_pulses = MagicMock(return_value=None)
            ctrl.set_home_position()
            self.assertFalse(ctrl.home_set_since_start)
            ctrl.read_absolute_position_pulses = MagicMock(return_value=5000000)
            ctrl.set_home_position()
            self.assertTrue(ctrl.home_set_since_start)


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
    ("read_motor_feedback_pulses", 0x0000, 2),
    ("read_motor_feedback_pulses_0x0024", 0x0024, 2),
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

    def test_read_motor_feedback_pulses_returns_none_on_empty_response(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = None
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = None
            result = ctrl.read_motor_feedback_pulses()
        self.assertIsNone(result)

    def test_read_motor_feedback_pulses_returns_int(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        with patch("servo_control.ModbusRTUResponse") as mock_cls:
            mock_cls.return_value.get_value.return_value = 12345
            result = ctrl.read_motor_feedback_pulses()
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

    def test_rejects_the_prohibited_range_64_to_999(self):
        ctrl = make_controller()
        for value in (64, 500, 999):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    ctrl.write_PF82(value)

    def test_rejects_non_integers(self):
        ctrl = make_controller()
        for value in (2.5, "3", None, True):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    ctrl.write_PF82(value)

    def test_writes_the_requested_path_number_not_a_constant(self):
        """Regression: write_PF82() used to ignore its argument and always
        write 1 (so every call ran PATH#1)."""
        for value in (0, 1, 5, 63, 1000):
            with self.subTest(value=value):
                ctrl = make_controller()
                ctrl._write_parameter = MagicMock(return_value=True)
                self.assertTrue(ctrl.write_PF82(value))
                ctrl._write_parameter.assert_called_once_with(PF.PRCM, value)

    def test_reports_failure_when_the_drive_does_not_acknowledge(self):
        ctrl = make_controller()
        ctrl._write_parameter = MagicMock(return_value=False)
        self.assertFalse(ctrl.write_PF82(5))


class TestReadPosRelatedParameters(unittest.TestCase):

    def test_reads_each_expected_register_once(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        ctrl.delay_ms = MagicMock()  # skip the real 100ms sleep per register

        ctrl.Read_Pos_Related_Paremters()

        self.assertEqual(ctrl.modbus_client.build_read_message.call_count, 18)


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
        ctrl.speed_ctrl_action = MagicMock(side_effect=lambda v: call_order.append("stop"))
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
        # actually took effect on the drive. "stop" (0x0904=0) is a final,
        # explicit step added 2026-09-22: see speed_ctrl_action.assert
        # below for why.
        self.assertEqual(call_order, ["clear_alarm_12", "jog_mode", "accel", "speed", "stop"])
        # Regression coverage for a real-hardware bug (2026-09-22): 0x0904
        # (JOG_OPERATION) is sticky on this drive -- entering JOG mode does
        # NOT reset it, so a stale 1/2 (CW/CCW) left over from a MOTION
        # PAUSE that never landed made a single ENABLE SPEED CONTROL MODE
        # click resume rotation with no CW/CCW press this time. Every arm
        # must now explicitly stop first.
        ctrl.speed_ctrl_action.assert_called_once_with(0)
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
        ctrl.speed_ctrl_action = MagicMock()
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

    def test_enable_true_records_the_jog_speed_for_change_jog_speed_by(self):
        ctrl = make_controller()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl.config_speed_0x0903 = MagicMock()
        ctrl.config_acc_dec_0x0902 = MagicMock()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.speed_ctrl_action = MagicMock()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()

        ctrl.enable_speed_ctrl(speed_rpm=100, acc_time=5000, enable=True)

        self.assertEqual(ctrl.jog_speed_rpm, 100)

    def test_enable_false_clears_the_jog_speed(self):
        ctrl = make_controller()
        ctrl.Enable_JOG_Mode = MagicMock()
        ctrl.stop_continuous_reading = MagicMock()
        ctrl.delay_ms = MagicMock()
        ctrl.jog_speed_rpm = 100

        ctrl.enable_speed_ctrl(speed_rpm=200, acc_time=5000, enable=False)

        self.assertIsNone(ctrl.jog_speed_rpm)


class TestChangeJogSpeedBy(unittest.TestCase):
    """change_jog_speed_by() -- the web UI's Up/Down arrow-key +/-1 rpm
    nudge (also exposed to OSC as /jog_speed_adjust), distinct from
    enable_speed_ctrl() which sets an absolute starting speed."""

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
        ctrl.config_speed_0x0903.assert_called_once_with(0)

    def test_clamped_to_the_manual_documented_maximum(self):
        ctrl = make_controller()
        ctrl.jog_speed_rpm = 3000
        ctrl.config_speed_0x0903 = MagicMock()

        result = ctrl.change_jog_speed_by(1)

        self.assertEqual(result, 3000)
        ctrl.config_speed_0x0903.assert_called_once_with(3000)

    def test_after_motion_cancel_clears_jog_speed_it_raises_again(self):
        """Regression coverage for the merged ENABLE SPEED CONTROL MODE /
        MOTION CANCEL toggle button's off-path (app.py's "motionCancel"
        action): it must clear jog_speed_rpm itself (it does not call
        enable_speed_ctrl(enable=False)), or a stale value would let this
        method appear to succeed after JOG mode was actually torn down."""
        ctrl = make_controller()
        ctrl.jog_speed_rpm = 100
        ctrl.jog_speed_rpm = None  # what app.py's motionCancel action now does
        with self.assertRaises(RuntimeError):
            ctrl.change_jog_speed_by(1)


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

        ctrl.read_motor_feedback_pulses = MagicMock(side_effect=_side_effect)
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
        ctrl.read_motor_feedback_pulses = MagicMock(side_effect=lambda: sequence.pop(0) if sequence else settled_value)
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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=500)
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
        ctrl.read_motor_feedback_pulses = MagicMock(
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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=1)
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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=1)
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
        ctrl.read_motor_feedback_pulses = MagicMock(return_value=1)
        ctrl.delay_ms = MagicMock(side_effect=lambda ms: time.sleep(0.001))
        ctrl.start_continuous_reading(interval=0.001, auto_stop_on_stillness=False)
        try:
            ctrl.start_continuous_reading(interval=0.001, auto_stop_on_stillness=True)
            self.assertTrue(ctrl._auto_stop_on_stillness)
        finally:
            ctrl.stop_continuous_reading()

    def test_stop_continuous_reading_does_not_hang_forever_on_a_stuck_thread(self):
        """Regression test ported from servo_comm_shihlin's identical fix
        2026-09-22: thread_to_join.join() here had no timeout, so if the
        background thread ever got stuck for any reason (e.g. blocked
        inside a Modbus transaction), whichever caller is stopping it --
        possibly a Flask request holding hardware_lock.py's
        _hardware_busy_lock -- would hang forever with no way to recover
        short of restarting the process. Simulates a thread that never
        actually stops; asserts stop_continuous_reading() still returns
        (bounded by its own 5s join timeout) rather than blocking
        indefinitely."""
        ctrl = make_controller()
        ctrl.reading_active = True
        stuck_thread = MagicMock()
        stuck_thread.is_alive.return_value = True
        ctrl.read_thread = stuck_thread

        started = time.time()
        ctrl.stop_continuous_reading()
        elapsed = time.time() - started

        stuck_thread.join.assert_called_once_with(timeout=5.0)
        self.assertFalse(ctrl.reading_active)
        self.assertLess(elapsed, 1.0, "stop_continuous_reading() should not itself sleep/block "
                                       "beyond the mocked join() call")


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


class TestReadDoStatus(unittest.TestCase):
    """read_do_status(): DO1~DO6 ON/OFF (0x0205, bit0~5 = CN1_41~CN1_46) plus
    the function assigned to each pin (0x020C = DO1~3, 0x020D = DO4~6, five
    bits each). Read-only. Sample values are what the real drive returned on
    2026-09-21 (stationary, Servo OFF, no alarm): 0x0205=0x26, 0x020C=0x103,
    0x020D=0x825."""

    def _reads(self, ctrl, do1_2_3, do4_5_6, do_status):
        """Mocks the three 1-word reads in the order read_do_status() makes
        them (0x020C, 0x020D, 0x0205)."""
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b"not-empty"
        patcher = patch("servo_control.ModbusRTUResponse")
        mock_cls = patcher.start()
        self.addCleanup(patcher.stop)
        mock_cls.return_value.get_value.side_effect = [do1_2_3, do4_5_6, do_status]
        return mock_cls

    def test_real_drive_sample_2026_09_21(self):
        ctrl = make_controller()
        self._reads(ctrl, 0x103, 0x825, 0x26)

        do = ctrl.read_do_status()

        self.assertEqual([do[f"DO{n}"]["on"] for n in range(1, 7)],
                         [False, True, True, False, False, True])
        self.assertEqual([do[f"DO{n}"]["function_code"] for n in range(1, 7)],
                         [0x03, 0x08, 0x00, 0x05, 0x01, 0x02])
        self.assertEqual([do[f"DO{n}"]["function"] for n in range(1, 7)],
                         ["INP_SA", "ZSP", "unassigned", "TLC_VLC", "RD", "ALM"])

    def test_pins_are_cn1_41_to_46(self):
        ctrl = make_controller()
        self._reads(ctrl, 0, 0, 0)
        do = ctrl.read_do_status()
        self.assertEqual([do[f"DO{n}"]["pin"] for n in range(1, 7)],
                         ["CN1-41", "CN1-42", "CN1-43", "CN1-44", "CN1-45", "CN1-46"])

    def test_each_bit_maps_to_its_own_pin(self):
        for n in range(1, 7):
            with self.subTest(pin=n):
                ctrl = make_controller()
                self._reads(ctrl, 0, 0, 1 << (n - 1))
                do = ctrl.read_do_status()
                self.assertEqual([k for k, v in do.items() if v["on"]], [f"DO{n}"])

    def test_bits_above_bit5_are_ignored(self):
        ctrl = make_controller()
        self._reads(ctrl, 0, 0, 0xFFC0)  # only bits 6~15 set
        self.assertFalse(any(v["on"] for v in ctrl.read_do_status().values()))
        ctrl = make_controller()
        self._reads(ctrl, 0, 0, 0xFFFF)
        self.assertTrue(all(v["on"] for v in ctrl.read_do_status().values()))

    def test_function_field_is_five_bits_per_pin_from_bit0(self):
        ctrl = make_controller()
        # DO1=0x1F (top of range), DO2=0x01, DO3=0x02 packed into one word
        packed = 0x1F | (0x01 << 5) | (0x02 << 10)
        self._reads(ctrl, packed, 0, 0)
        do = ctrl.read_do_status()
        self.assertEqual([do[f"DO{n}"]["function_code"] for n in (1, 2, 3)], [0x1F, 0x01, 0x02])

    def test_unknown_function_code_keeps_the_code_and_has_no_name(self):
        ctrl = make_controller()
        self._reads(ctrl, 0x10, 0, 0)  # 0x10 is not in BitMapOutput
        pin = ctrl.read_do_status()["DO1"]
        self.assertEqual(pin["function_code"], 0x10)
        self.assertIsNone(pin["function"])

    def test_reads_exactly_the_three_registers_and_writes_nothing(self):
        ctrl = make_controller()
        self._reads(ctrl, 0, 0, 0)
        ctrl.read_do_status()
        reads = [c.args for c in ctrl.modbus_client.build_read_message.call_args_list]
        self.assertEqual(reads, [(0x020C, 1), (0x020D, 1), (0x0205, 1)])
        ctrl.modbus_client.build_write_message.assert_not_called()

    def test_no_response_returns_none_never_all_off(self):
        for failing_read in (0, 1, 2):
            with self.subTest(failing_read=failing_read):
                ctrl = make_controller()
                self._reads(ctrl, 0x103, 0x825, 0x26)
                responses = [b"x", b"x", b"x"]
                responses[failing_read] = None
                ctrl.modbus_client.send_and_receive.side_effect = responses
                self.assertIsNone(ctrl.read_do_status())

    def test_unparseable_reply_returns_none(self):
        ctrl = make_controller()
        mock_cls = self._reads(ctrl, 0x103, 0x825, 0x26)
        mock_cls.return_value.get_value.side_effect = [0x103, ValueError("bad CRC"), 0x26]
        self.assertIsNone(ctrl.read_do_status())

    def test_a_reply_without_a_value_returns_none(self):
        ctrl = make_controller()
        self._reads(ctrl, 0x103, None, 0x26)
        self.assertIsNone(ctrl.read_do_status())

    def test_exception_response_from_the_drive_returns_none(self):
        ctrl = make_controller()
        mock_cls = self._reads(ctrl, 0x103, 0x825, 0x26)
        # A Modbus exception frame raises (ModbusExceptionResponse is a ValueError).
        mock_cls.side_effect = ValueError("Modbus exception 0x02")
        self.assertIsNone(ctrl.read_do_status())

    def test_mc_ok_uses_the_same_read(self):
        """read_mc_ok_status() is now built on read_do_status()."""
        ctrl = make_controller()
        ctrl.read_do_status = MagicMock(return_value={
            "DO1": {"on": True, "function_code": 0x03}, "DO2": {"on": False, "function_code": 0x08},
            "DO3": {"on": True, "function_code": 0x09}, "DO4": {"on": False, "function_code": 0x05},
            "DO5": {"on": False, "function_code": 0x01}, "DO6": {"on": False, "function_code": 0x02},
        })
        self.assertTrue(ctrl.read_mc_ok_status())
        ctrl.read_do_status.assert_called_once()


class TestReadPosRelatedParemters(unittest.TestCase):
    """GET STATE VALUE ("getMsg") -- must decode each register via its
    explain_* helper (servo_p_register.py), not just log raw bytes. Values
    below are the manual's own documented defaults/worked examples (see
    docs/en_manual.txt), so the expected interpretation strings are the
    manual's, not reverse-engineered from the implementation."""

    def _mock_reads(self, ctrl, values):
        # Read order in Read_Pos_Related_Paremters(): STY, HMOV, PLSS,
        # ENR, PO1H, POL, SDI, ITST, MCOK, MCS, ABS, APST, APR, APP, CMX,
        # CDV, FBK_0000 (0x0000), FBK_0024 (0x0024).
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = b'not-empty'
        patcher = patch("servo_control.ModbusRTUResponse")
        mock_cls = patcher.start()
        self.addCleanup(patcher.stop)
        mock_cls.return_value.get_value.side_effect = values
        return ctrl

    def test_decodes_all_nine_registers_with_manual_defaults(self):
        ctrl = make_controller()
        # STY=0x1000 (factory default), HMOV=0x0000 (factory default),
        # PLSS=0x0312 (worked example: A/B phase, negative logic, 4Mpps),
        # ENR=10000 (factory default pulse/rev), PO1H=0, POL=0x0111,
        # SDI=0x0FFF (all DI communication-controlled),
        # ITST=0x0011 (manual's own worked example: DI1 and DI5 ON),
        # MCOK=0x0011 (hold + AL1B enabled).
        # MCS=2, ABS=0 (incremental), APST=0 (normal), APR=1234 (pulses in
        # the default layout), APP=-3 (signed revolutions).
        self._mock_reads(ctrl, [0x1000, 0x0000, 0x0312, 10000, 0, 0x0111,
                                 0x0FFF, 0x0011, 0x0011, 2, 0, 0, 1234, -3,
                                 1, 1, 1000, 2000])

        results = ctrl.Read_Pos_Related_Paremters()

        by_name = {entry["name"]: entry for entry in results}
        self.assertEqual(len(results), 18)
        self.assertIn("1:1", by_name["CMX"]["interpreted"])
        self.assertIn("all angle math", by_name["FBK_0000"]["interpreted"])
        # Translated is listed next to raw with the ratio between them, so the
        # real relationship (and which manual is right) can be read off the drive.
        self.assertIn("2.0000 x FBK_0000", by_name["FBK_0024"]["interpreted"])
        self.assertIn("persists", by_name["MCS"]["interpreted"])
        self.assertIn("incremental", by_name["ABS"]["interpreted"])
        self.assertEqual(by_name["APST"]["interpreted"], "normal")
        self.assertIn("1234 pulses", by_name["APR"]["interpreted"])
        self.assertIn("-3 rev", by_name["APP"]["interpreted"])
        self.assertIn("position", by_name["STY"]["interpreted"])
        self.assertIn("A/B phase pulse train", by_name["PLSS"]["interpreted"])
        self.assertEqual(by_name["ENR"]["value"], 10000)
        self.assertIn("pulses/rev", by_name["ENR"]["interpreted"])
        self.assertEqual(by_name["PO1H"]["interpreted"], "0 rev")
        self.assertIn("output division ratio", by_name["POL"]["interpreted"])
        self.assertIn("DI1", by_name["SDI"]["interpreted"])
        self.assertIn("DI12", by_name["SDI"]["interpreted"])
        # Manual's own worked example for ITST=0x0011: "DI1 and DI5 are ON".
        self.assertEqual(by_name["ITST"]["interpreted"], "Virtual ON: DI1, DI5")
        self.assertIn("held until next move", by_name["MCOK"]["interpreted"])
        self.assertIn("AL1B", by_name["MCOK"]["interpreted"])

    def test_no_response_reports_communication_failure_not_a_crash(self):
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.send_and_receive.return_value = None

        results = ctrl.Read_Pos_Related_Paremters()

        self.assertEqual(len(results), 18)
        for entry in results:
            self.assertIsNone(entry["value"])
            self.assertEqual(entry["interpreted"], "No response (communication failure)")


class TestWriteParameterFunctionCodes(unittest.TestCase):
    """_write_parameter(): function 0x06 first (verified on this drive for
    the equally 2-word PD16/PD25), 0x10 only if the drive rejects it with a
    Modbus exception."""

    @staticmethod
    def _frame(body):
        return body + ModbusUtils().calculate_crc(body)

    def _ctrl(self, replies):
        real = ModbusRTUClient(1, MagicMock())
        ctrl = make_controller()
        ctrl.modbus_client = MagicMock()
        ctrl.modbus_client.build_write_message = real.build_write_message
        ctrl.modbus_client.build_write_multiple_message = real.build_write_multiple_message
        ctrl.modbus_client.send_and_receive = MagicMock(side_effect=replies)
        return ctrl

    def _sent(self, ctrl):
        return [c.args[0] for c in ctrl.modbus_client.send_and_receive.call_args_list]

    OK_06 = None  # filled per test (an 0x06 reply echoes the request)

    def test_uses_function_0x06_when_the_drive_accepts_it(self):
        ok = self._frame(bytes([1, 0x06, 0x03, 0x2C, 0x00, 0x02]))
        ctrl = self._ctrl([ok])
        self.assertTrue(ctrl._write_parameter(PA.MCS, 2))
        sent = self._sent(ctrl)
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][1], 0x06)
        self.assertFalse(ctrl._prefer_multi_word_write)

    def test_falls_back_to_0x10_when_0x06_is_rejected_with_an_exception(self):
        rejected = self._frame(bytes([1, 0x86, 0x02]))
        ok10 = self._frame(bytes([1, 0x10, 0x03, 0x2C, 0x00, 0x02]))
        ctrl = self._ctrl([rejected, ok10])
        self.assertTrue(ctrl._write_parameter(PA.MCS, 2))
        sent = self._sent(ctrl)
        self.assertEqual([m[1] for m in sent], [0x06, 0x10])
        # value 2 as [low word, high word] = 0002 0000
        self.assertEqual(sent[1][6:11], bytes([0x04, 0x00, 0x02, 0x00, 0x00]))
        self.assertTrue(ctrl._prefer_multi_word_write)

    def test_after_a_successful_fallback_0x10_is_used_first(self):
        rejected = self._frame(bytes([1, 0x86, 0x02]))
        ok10 = self._frame(bytes([1, 0x10, 0x03, 0x2C, 0x00, 0x02]))
        ctrl = self._ctrl([rejected, ok10, ok10])
        ctrl._write_parameter(PA.MCS, 2)
        ctrl._write_parameter(PA.MCS, 1)
        self.assertEqual([m[1] for m in self._sent(ctrl)], [0x06, 0x10, 0x10])

    def test_a_value_wider_than_one_word_goes_straight_to_0x10(self):
        ok10 = self._frame(bytes([1, 0x10, 0x03, 0x2C, 0x00, 0x02]))
        ctrl = self._ctrl([ok10])
        self.assertTrue(ctrl._write_parameter(PA.MCS, 0x00030001))
        sent = self._sent(ctrl)
        self.assertEqual([m[1] for m in sent], [0x10])
        self.assertEqual(sent[0][7:11], bytes([0x00, 0x01, 0x00, 0x03]))  # low word, high word

    def test_both_rejected_reports_failure(self):
        rejected06 = self._frame(bytes([1, 0x86, 0x02]))
        rejected10 = self._frame(bytes([1, 0x90, 0x02]))
        ctrl = self._ctrl([rejected06, rejected10])
        self.assertFalse(ctrl._write_parameter(PA.MCS, 2))
        self.assertFalse(ctrl._prefer_multi_word_write)

    def test_no_response_is_a_dead_line_and_is_not_retried(self):
        ctrl = self._ctrl([None])
        self.assertFalse(ctrl._write_parameter(PA.MCS, 2))
        self.assertEqual(len(self._sent(ctrl)), 1)

    def test_a_corrupt_reply_is_not_retried_with_another_function(self):
        ctrl = self._ctrl([b"\x01\x06\x03\x2c\x00\x02\x00\x00"])  # bad CRC
        self.assertFalse(ctrl._write_parameter(PA.MCS, 2))
        self.assertEqual(len(self._sent(ctrl)), 1)

    def test_pa29_home_write_goes_through_the_same_checked_path(self):
        rejected = self._frame(bytes([1, 0x86, 0x02]))
        ok10 = self._frame(bytes([1, 0x10, 0x03, 0x38, 0x00, 0x02]))
        ctrl = self._ctrl([rejected, ok10])
        ctrl.write_PA29_Initial_Abs_Pos()
        self.assertEqual([m[1] for m in self._sent(ctrl)], [0x06, 0x10])


if __name__ == "__main__":
    unittest.main()
