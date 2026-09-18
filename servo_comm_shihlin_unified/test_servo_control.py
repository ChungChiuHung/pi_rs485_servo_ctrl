"""
Unit tests for the new logic introduced by the servo_comm_shihlin_unified
merge -- NOT a re-test of everything servo_control.py does. Mocks the
ServoController's own read/write helpers (read_encoder_before_gear_ratio,
_execute_positioning, the modbus client) rather than raw wire bytes, since
those wire-level behaviors are already covered by test_encoder_pulse_tracker.py
and the ModbusRTUClient/ModbusRTUResponse tests.

Covers (per the merge plan): EncoderPulseTracker integration in
pos_step_motion_by()/set_home_position()/cancel_continuous_reading(), the
closed-loop diff_angle basis in post_step_motion_by(), the abs()-based
180-degree guard in both motion functions, and directional float_error
accumulation.
"""
import os
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from servo_control import ServoController

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


if __name__ == "__main__":
    unittest.main()
