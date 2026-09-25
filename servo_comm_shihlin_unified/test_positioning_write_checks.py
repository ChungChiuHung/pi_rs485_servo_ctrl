"""
The positioning setup writes must not be fire-and-forget.

Found on the real drive (Art-Net interleave test, 2026-09-21): a move was
"commanded" but the motor never moved, with only a `No response received.`
warning -- _execute_positioning() threw away the result of every write. A lost
0x0907 trigger is a move that never happens; a lost 0x0905/0x0906/0x0903 write
would run the PREVIOUS distance or speed. Now every setup write must be
acknowledged (retried once) and read back before the trigger, and a failure
raises DriveCommunicationError before anything is started.
"""
import unittest
from unittest.mock import MagicMock, patch, call

from servo_control import DriveCommunicationError, ServoController
from test_servo_control import make_controller


def ctrl_with_client():
    ctrl = make_controller()
    ctrl.delay_ms = MagicMock()
    ctrl.modbus_client = MagicMock()
    ctrl.modbus_client.build_write_message.side_effect = lambda addr, val: ("w", addr, val)
    ctrl.modbus_client.build_read_message.side_effect = lambda addr, n: ("r", addr, n)
    return ctrl


class TestWriteRegisterChecked(unittest.TestCase):

    def test_acknowledged_write_is_sent_once(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = b"echo"
        with patch("servo_control.ModbusRTUResponse"):
            ctrl._write_register_checked(0x0903, 10, "speed")
        ctrl.modbus_client.send_and_receive.assert_called_once_with(("w", 0x0903, 10))

    def test_one_lost_reply_is_retried(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.side_effect = [None, b"echo"]
        with patch("servo_control.ModbusRTUResponse"):
            ctrl._write_register_checked(0x0903, 10, "speed")
        self.assertEqual(ctrl.modbus_client.send_and_receive.call_count, 2)

    def test_two_lost_replies_raise(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = None
        with self.assertRaises(DriveCommunicationError) as cm:
            ctrl._write_register_checked(0x0903, 10, "speed")
        self.assertEqual(ctrl.modbus_client.send_and_receive.call_count, 2)
        self.assertIn("speed", str(cm.exception))
        self.assertIn("no reply", str(cm.exception))

    def test_an_unparsable_or_exception_frame_counts_as_not_acknowledged(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = b"garbage"
        with patch("servo_control.ModbusRTUResponse", side_effect=ValueError("bad CRC")):
            with self.assertRaises(DriveCommunicationError) as cm:
                ctrl._write_register_checked(0x0903, 10, "speed")
        self.assertIn("bad CRC", str(cm.exception))


class TestWriteRegisterVerified(unittest.TestCase):

    def make(self, read_values):
        ctrl = ctrl_with_client()
        ctrl._write_register_checked = MagicMock()
        ctrl._read_register_word = MagicMock(side_effect=read_values)
        return ctrl

    def test_matching_readback_passes_without_a_rewrite(self):
        ctrl = self.make([2000])
        ctrl._write_register_verified(0x0902, 2000, "acc/dec")
        ctrl._write_register_checked.assert_called_once_with(0x0902, 2000, "acc/dec")

    def test_a_wrong_readback_rewrites_once_then_passes(self):
        ctrl = self.make([1000, 2000])
        ctrl._write_register_verified(0x0902, 2000, "acc/dec")
        self.assertEqual(ctrl._write_register_checked.call_count, 2)

    def test_readback_that_stays_wrong_raises_and_says_what_it_read(self):
        ctrl = self.make([1000, 1000])
        with self.assertRaises(DriveCommunicationError) as cm:
            ctrl._write_register_verified(0x0902, 2000, "acc/dec")
        self.assertIn("1000", str(cm.exception))
        self.assertIn("2000", str(cm.exception))

    def test_a_readback_with_no_reply_is_not_treated_as_a_match(self):
        ctrl = self.make([None, None])
        with self.assertRaises(DriveCommunicationError):
            ctrl._write_register_verified(0x0902, 2000, "acc/dec")

    def test_settle_time_is_waited_before_reading_back(self):
        ctrl = self.make([4])
        ctrl._write_register_verified(0x0901, 4, "mode", settle_ms=100)
        ctrl.delay_ms.assert_any_call(100)


class TestReadRegisterWord(unittest.TestCase):

    def test_returns_the_value(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = b"frame"
        with patch("servo_control.ModbusRTUResponse") as resp:
            resp.return_value.get_value.return_value = 4
            self.assertEqual(ctrl._read_register_word(0x0901), 4)
        ctrl.modbus_client.build_read_message.assert_called_once_with(0x0901, 1)

    def test_no_reply_is_none(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = None
        self.assertIsNone(ctrl._read_register_word(0x0901))

    def test_bad_frame_is_none(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = b"x"
        with patch("servo_control.ModbusRTUResponse", side_effect=ValueError("bad")):
            self.assertIsNone(ctrl._read_register_word(0x0901))


class TestExecutePositioningIsChecked(unittest.TestCase):

    def make(self):
        ctrl = make_controller()
        ctrl.delay_ms = MagicMock()
        ctrl.clear_alarm_12 = MagicMock()
        ctrl._write_register_verified = MagicMock()
        ctrl.pos_step_motion_test = MagicMock()
        return ctrl

    def test_every_setup_register_is_written_verified_in_the_manual_order(self):
        ctrl = self.make()
        ctrl._execute_positioning(angle=10, low_byte=0x1234, high_byte=0x0002, acc_dec_time=2000, speed_rpm=15)
        written = [(c.args[0], c.args[1]) for c in ctrl._write_register_verified.call_args_list]
        self.assertEqual(written, [(0x0901, 4), (0x0902, 2000), (0x0903, 15), (0x0905, 0x1234), (0x0906, 0x0002)])
        ctrl.pos_step_motion_test.assert_called_once_with(True)

    def test_alarm_is_cleared_before_the_mode_is_entered(self):
        ctrl = self.make()
        order = []
        ctrl.clear_alarm_12.side_effect = lambda: order.append("clear")
        ctrl._write_register_verified.side_effect = lambda addr, *a, **k: order.append(hex(addr))
        ctrl._execute_positioning(angle=10, low_byte=1, high_byte=0, acc_dec_time=100, speed_rpm=5)
        self.assertEqual(order[:2], ["clear", "0x901"])

    def test_the_mode_write_waits_for_the_drive_to_settle(self):
        ctrl = self.make()
        ctrl._execute_positioning(angle=10, low_byte=1, high_byte=0, acc_dec_time=100, speed_rpm=5)
        self.assertEqual(ctrl._write_register_verified.call_args_list[0].kwargs.get("settle_ms"), 100)

    def test_a_failed_setup_write_starts_nothing(self):
        for failing in (0x0901, 0x0902, 0x0903, 0x0905, 0x0906):
            with self.subTest(register=hex(failing)):
                ctrl = self.make()

                def fail_on(addr, *a, **k):
                    if addr == failing:
                        raise DriveCommunicationError(f"{hex(addr)} failed")
                ctrl._write_register_verified.side_effect = fail_on
                with self.assertRaises(DriveCommunicationError):
                    ctrl._execute_positioning(angle=10, low_byte=1, high_byte=0, acc_dec_time=100, speed_rpm=5)
                ctrl.pos_step_motion_test.assert_not_called()

    def test_negative_angle_runs_ccw(self):
        ctrl = self.make()
        ctrl._execute_positioning(angle=-10, low_byte=1, high_byte=0, acc_dec_time=100, speed_rpm=5)
        ctrl.pos_step_motion_test.assert_called_once_with(False)


class TestTriggerIsChecked(unittest.TestCase):

    def test_an_unanswered_trigger_raises_and_is_not_resent(self):
        """0x0907 is a trigger, not a setting: resending after a lost REPLY could
        start the move twice, so it is sent exactly once and the caller told."""
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = None
        with self.assertRaises(DriveCommunicationError):
            ctrl.pos_motion_start_0x0907(1)
        ctrl.modbus_client.send_and_receive.assert_called_once_with(("w", 0x0907, 1))

    def test_an_acknowledged_trigger_passes(self):
        ctrl = ctrl_with_client()
        ctrl.modbus_client.send_and_receive.return_value = b"echo"
        with patch("servo_control.ModbusRTUResponse"):
            ctrl.pos_motion_start_0x0907(2)

    def test_pos_step_motion_test_stops_the_polling_when_the_trigger_fails(self):
        ctrl = make_controller()
        ctrl.delay_ms = MagicMock()
        ctrl.start_continuous_reading = MagicMock()
        ctrl.stop_continuous_reading = MagicMock()
        ctrl.pos_motion_start_0x0907 = MagicMock(side_effect=DriveCommunicationError("no ack"))
        ctrl.reading_active = True
        with self.assertRaises(DriveCommunicationError):
            ctrl.pos_step_motion_test(CW=True)
        ctrl.stop_continuous_reading.assert_called()


class TestTriggerConfirmedAndRetried(unittest.TestCase):
    """Real drive, 2026-09-25 (COM4, 20 alternating 1 deg moves): 1 trigger in 20
    got no reply AND the motor did not move -- the drive never took it. So an
    unanswered trigger is checked against the encoder: moving means it did start
    (never resend), standing still means it is safe to send once more."""

    def make(self, encoder_moves_after_wait):
        ctrl = make_controller()
        ctrl.current_encoder = 1_000_000
        ctrl.start_continuous_reading = MagicMock()
        ctrl.stop_continuous_reading = MagicMock()
        ctrl.reading_active = True

        def wait(ms):
            if ms >= ctrl.TRIGGER_CONFIRM_WATCH_MS and encoder_moves_after_wait:
                ctrl.current_encoder += 50_000
        ctrl.delay_ms = MagicMock(side_effect=wait)
        return ctrl

    def test_an_answered_trigger_is_sent_once(self):
        ctrl = self.make(False)
        ctrl.pos_motion_start_0x0907 = MagicMock()
        ctrl.pos_step_motion_test(CW=True)
        ctrl.pos_motion_start_0x0907.assert_called_once_with(1)

    def test_unanswered_but_the_motor_is_moving_is_not_resent(self):
        ctrl = self.make(True)
        ctrl.pos_motion_start_0x0907 = MagicMock(side_effect=DriveCommunicationError("no ack"))
        ctrl.pos_step_motion_test(CW=False)          # must not raise
        ctrl.pos_motion_start_0x0907.assert_called_once_with(2)
        ctrl.stop_continuous_reading.assert_not_called()

    def test_unanswered_and_standing_still_is_sent_once_more(self):
        ctrl = self.make(False)
        ctrl.pos_motion_start_0x0907 = MagicMock(side_effect=[DriveCommunicationError("no ack"), None])
        ctrl.pos_step_motion_test(CW=True)
        self.assertEqual(ctrl.pos_motion_start_0x0907.call_args_list, [call(1), call(1)])
        ctrl.stop_continuous_reading.assert_not_called()

    def test_unanswered_twice_raises_and_stops_the_polling(self):
        ctrl = self.make(False)
        ctrl.pos_motion_start_0x0907 = MagicMock(side_effect=DriveCommunicationError("no ack"))
        with self.assertRaises(DriveCommunicationError):
            ctrl.pos_step_motion_test(CW=True)
        self.assertEqual(ctrl.pos_motion_start_0x0907.call_count, 2)
        ctrl.stop_continuous_reading.assert_called()

    def test_movement_is_judged_against_the_encoder_before_the_first_send(self):
        """A trigger that fails after the encoder already moved a little during the
        retry gap must not be resent: the baseline is taken before the first send."""
        ctrl = self.make(True)
        seen = []

        def fail_and_note(value):
            seen.append(ctrl.current_encoder)
            raise DriveCommunicationError("no ack")
        ctrl.pos_motion_start_0x0907 = MagicMock(side_effect=fail_and_note)
        ctrl.pos_step_motion_test(CW=True)
        self.assertEqual(len(seen), 1)


class TestPostStepMotionByRollsBackOnFailure(unittest.TestCase):

    def test_float_error_and_accumulate_pulse_are_restored_when_nothing_was_started(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl.current_angle = 0.0
        ctrl.float_error = 0.25
        ctrl.accumulate_pulse = 1000
        ctrl._execute_positioning = MagicMock(side_effect=DriveCommunicationError("no ack"))
        with self.assertRaises(DriveCommunicationError):
            ctrl.post_step_motion_by(10.0)
        self.assertEqual(ctrl.float_error, 0.25)
        self.assertEqual(ctrl.accumulate_pulse, 1000)

    def test_a_successful_move_still_accumulates(self):
        ctrl = make_controller()
        ctrl._refresh_current_angle_from_hardware = MagicMock(return_value=True)
        ctrl.current_angle = 0.0
        ctrl._execute_positioning = MagicMock()
        ctrl.post_step_motion_by(10.0)
        self.assertGreater(ctrl.accumulate_pulse, 0)


if __name__ == "__main__":
    unittest.main()
