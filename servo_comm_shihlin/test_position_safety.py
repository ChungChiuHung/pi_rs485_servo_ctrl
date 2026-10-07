"""
Tests for the position/angle safety fixes ported from servo_comm_shihlin_unified
(read the real position before an absolute move, drive pulse-range check, electronic gear
check, SET HOME, write_PF82) -- all over the Modbus ASCII protocol this folder
keeps. The serial port is a mock; nothing touches hardware. Run from inside
this directory: python -m unittest test_position_safety
"""
import unittest
from unittest.mock import MagicMock, patch

from servo_control import ServoController, PositionUnavailableError, MoveOutOfRangeError, is_alarm_active
from servo_p_register import PA, PF

BASE_PULSE_PER_DEGREE = 349525.3333333333


def ascii_read_reply(value: int) -> bytes:
    """A Modbus ASCII read reply for one 2-word (32-bit) parameter: the drive
    sends the LOW word first (ModbusResponse.get_value swaps them). The LRC is
    not checked by the parser."""
    low, high = value & 0xFFFF, (value >> 16) & 0xFFFF
    return f":010304{low:04X}{high:04X}00\r\n".encode()


def make_controller():
    fake_serial = MagicMock()
    fake_serial.keep_running = True
    ctrl = ServoController(fake_serial)
    ctrl.modbus_client = MagicMock()
    return ctrl


class NoAlarmCodeTests(unittest.TestCase):
    def test_0xff_and_0_are_no_alarm(self):
        self.assertFalse(is_alarm_active(0xFF))
        self.assertFalse(is_alarm_active(0))

    def test_a_real_alarm_and_a_read_failure_are_active(self):
        self.assertTrue(is_alarm_active(0x12))
        self.assertTrue(is_alarm_active(None))


class RefreshAndAbsoluteMoveTests(unittest.TestCase):

    def test_the_angle_comes_from_the_drive_not_from_the_init_default(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 1000
        ctrl.current_angle = 0.0  # what a fresh process holds
        # The drive is really at home + 30 degrees.
        raw = 1000 + round(30 * BASE_PULSE_PER_DEGREE)
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=raw)

        ctrl.post_step_motion_by(angle=50.0, speed_rpm=5)

        self.assertAlmostEqual(ctrl.current_angle, 30.0, places=3)
        self.assertAlmostEqual(ctrl._execute_positioning.call_args[0][0], 20.0, places=3)

    def test_an_unreadable_position_refuses_to_move(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)

        with self.assertRaises(PositionUnavailableError):
            ctrl.post_step_motion_by(angle=10.0)
        ctrl._execute_positioning.assert_not_called()

    def test_a_read_that_raises_also_refuses_to_move(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(side_effect=ValueError("bad frame"))

        with self.assertRaises(PositionUnavailableError):
            ctrl.post_step_motion_by(angle=10.0)
        ctrl._execute_positioning.assert_not_called()

    def test_moves_of_more_than_180_degrees_are_allowed_both_ways(self):
        """No 180 degree limit (2026-09-21; the 2025-02-05 commit removed it from
        this folder and the unified port had wrongly put it back)."""
        for target, expected in ((200.0, 200.0), (-200.0, -200.0)):
            with self.subTest(target=target):
                ctrl = make_controller()
                ctrl._execute_positioning = MagicMock()
                ctrl.abs_home_pos = 0
                ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
                ctrl.post_step_motion_by(angle=target)
                ctrl._execute_positioning.assert_called_once()
                angle, low, high = ctrl._execute_positioning.call_args[0][:3]
                self.assertAlmostEqual(angle, expected)
                self.assertEqual((high << 16) | low, int(BASE_PULSE_PER_DEGREE * abs(expected)))

    def test_a_move_beyond_the_pulse_register_is_refused(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 0
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
        too_far = (2**31 + 1000) / BASE_PULSE_PER_DEGREE
        for target in (too_far, -too_far):
            with self.subTest(target=target):
                with self.assertRaises(MoveOutOfRangeError):
                    ctrl.post_step_motion_by(angle=target)
        ctrl._execute_positioning.assert_not_called()

    def test_a_non_finite_angle_is_refused(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
        for bad in (float("nan"), float("inf")):
            with self.subTest(angle=bad):
                with self.assertRaises(MoveOutOfRangeError):
                    ctrl.post_step_motion_by(angle=bad)
        ctrl._execute_positioning.assert_not_called()

    def test_a_small_move_is_allowed(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 0
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
        ctrl.post_step_motion_by(angle=90.0)
        ctrl._execute_positioning.assert_called_once()

    def test_the_move_uses_the_wraparound_tracker_not_the_raw_register(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.abs_home_pos = 0
        # First reading right below the 32-bit limit, then the register wraps.
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=4_294_967_000)
        ctrl._read_reference_encoder()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=100)
        # Unwrapped, the position is ~ +396 pulses past the limit, i.e. still
        # a huge angle from home 0 -- but the point is it is NOT the raw 100.
        self.assertGreater(ctrl._read_reference_encoder(), 4_294_967_000)


class PosStepMotionByTests(unittest.TestCase):

    def test_unreadable_position_does_not_crash_or_move(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)
        self.assertEqual(ctrl.pos_step_motion_by(target_pos=1000), 0.0)
        ctrl._execute_positioning.assert_not_called()

    def test_large_moves_are_allowed_in_both_directions(self):
        for target in (int(BASE_PULSE_PER_DEGREE * 200), -int(BASE_PULSE_PER_DEGREE * 200)):
            with self.subTest(target=target):
                ctrl = make_controller()
                ctrl._execute_positioning = MagicMock()
                ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
                self.assertAlmostEqual(ctrl.pos_step_motion_by(target_pos=target), target / BASE_PULSE_PER_DEGREE)
                ctrl._execute_positioning.assert_called_once()

    def test_a_move_beyond_the_pulse_register_is_refused(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
        for target in (2**31, -(2**31)):
            with self.subTest(target=target):
                self.assertEqual(ctrl.pos_step_motion_by(target_pos=target), 0.0)
        ctrl._execute_positioning.assert_not_called()

    def test_the_largest_move_the_register_can_hold_is_still_sent(self):
        ctrl = make_controller()
        ctrl._execute_positioning = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=0)
        ctrl.pos_step_motion_by(target_pos=2**31 - 1)
        ctrl._execute_positioning.assert_called_once()



class SetHomeTests(unittest.TestCase):

    def test_a_failed_read_changes_nothing(self):
        ctrl = make_controller()
        ctrl.save_abs_home_pos = MagicMock()
        ctrl.current_angle = 12.5
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)

        ctrl.set_home_position()

        self.assertEqual(ctrl.current_angle, 12.5)
        self.assertFalse(ctrl.home_set_since_start)
        ctrl.save_abs_home_pos.assert_not_called()

    def test_a_raising_read_changes_nothing(self):
        ctrl = make_controller()
        ctrl.save_abs_home_pos = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(side_effect=ValueError("bad frame"))
        ctrl.set_home_position()
        self.assertFalse(ctrl.home_set_since_start)
        ctrl.save_abs_home_pos.assert_not_called()

    def test_success_records_and_flags_home(self):
        ctrl = make_controller()
        ctrl.save_abs_home_pos = MagicMock()
        ctrl.delay_ms = MagicMock()
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=5000)

        ctrl.set_home_position()

        self.assertTrue(ctrl.home_set_since_start)
        self.assertEqual(ctrl.current_angle, 0.0)
        ctrl.save_abs_home_pos.assert_called_once_with(5000)

    def test_home_is_not_set_at_startup(self):
        self.assertFalse(make_controller().home_set_since_start)


class ElectronicGearRatioTests(unittest.TestCase):
    """Parsed from real-looking Modbus ASCII replies (low word first)."""

    def setUp(self):
        self.ctrl = make_controller()

    def replies(self, *values):
        self.ctrl.modbus_client.send_and_receive.side_effect = [ascii_read_reply(v) for v in values]

    def test_reads_pa06_and_pa07_and_accepts_one_to_one(self):
        self.replies(1, 1)
        self.assertEqual(self.ctrl.check_electronic_gear_ratio(), (1, 1))
        self.assertTrue(self.ctrl.electronic_gear_unity)
        addresses = [c.args[0] for c in self.ctrl.modbus_client.build_read_message.call_args_list]
        self.assertEqual(addresses, [PA.CMX.address, PA.CDV.address])

    def test_flags_a_ratio_that_is_not_one_to_one(self):
        self.replies(4, 1)
        self.assertEqual(self.ctrl.check_electronic_gear_ratio(), (4, 1))
        self.assertFalse(self.ctrl.electronic_gear_unity)

    def test_a_32_bit_value_is_read_low_word_first(self):
        self.replies(0x00010002, 0x00010002)
        self.assertEqual(self.ctrl.check_electronic_gear_ratio(), (0x00010002, 0x00010002))

    def test_no_reply_leaves_it_unknown_and_does_not_raise(self):
        self.ctrl.modbus_client.send_and_receive.return_value = None
        self.assertIsNone(self.ctrl.check_electronic_gear_ratio())
        self.assertIsNone(self.ctrl.electronic_gear_unity)

    def test_it_only_reads(self):
        self.replies(1, 1)
        self.ctrl.check_electronic_gear_ratio()
        self.ctrl.modbus_client.build_write_message.assert_not_called()

    def test_0x0024_returns_the_value_or_none(self):
        self.replies(123456)
        self.assertEqual(self.ctrl.read_encoder_after_gear_ratio(), 123456)
        self.ctrl.modbus_client.send_and_receive.side_effect = None
        self.ctrl.modbus_client.send_and_receive.return_value = None
        self.assertIsNone(self.ctrl.read_encoder_after_gear_ratio())


class WritePF82Tests(unittest.TestCase):

    def written_value(self, ctrl):
        return ctrl.modbus_client.build_write_message.call_args[0][1]

    def test_writes_the_requested_path_not_a_constant(self):
        ctrl = make_controller()
        ctrl.write_PF82(5)
        ctrl.modbus_client.build_write_message.assert_called_once_with(PF.PRCM.address, 5)

    def test_origin_return_and_stop_are_allowed(self):
        for value in (0, 63, 1000):
            with self.subTest(value=value):
                ctrl = make_controller()
                ctrl.write_PF82(value)
                self.assertEqual(self.written_value(ctrl), value)

    def test_osc_style_whole_number_float_is_accepted(self):
        ctrl = make_controller()
        ctrl.write_PF82(5.0)
        self.assertEqual(self.written_value(ctrl), 5)

    def test_prohibited_and_invalid_values_raise_and_send_nothing(self):
        for value in (64, 999, 1001, -1, 2.5, "3", True):
            with self.subTest(value=value):
                ctrl = make_controller()
                with self.assertRaises(ValueError):
                    ctrl.write_PF82(value)
                ctrl.modbus_client.build_write_message.assert_not_called()


if __name__ == "__main__":
    unittest.main()
