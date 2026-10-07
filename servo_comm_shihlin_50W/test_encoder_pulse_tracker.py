import unittest

from encoder_pulse_tracker import EncoderPulseTracker, RAW_RANGE, HALF_RAW_RANGE


class TestEncoderPulseTracker(unittest.TestCase):

    def test_not_initialized_before_any_call(self):
        tracker = EncoderPulseTracker()
        self.assertFalse(tracker.is_initialized)
        with self.assertRaises(RuntimeError):
            _ = tracker.cumulative

    def test_first_update_seeds_without_computing_delta(self):
        tracker = EncoderPulseTracker()
        result = tracker.update(1000)
        self.assertEqual(result, 1000)
        self.assertEqual(tracker.cumulative, 1000)
        self.assertTrue(tracker.is_initialized)

    def test_reset_reseeds_explicitly(self):
        tracker = EncoderPulseTracker()
        tracker.update(1000)
        tracker.update(1500)
        self.assertEqual(tracker.cumulative, 1500)

        result = tracker.reset(42)
        self.assertEqual(result, 42)
        self.assertEqual(tracker.cumulative, 42)

    def test_normal_forward_motion_no_wrap(self):
        tracker = EncoderPulseTracker()
        tracker.reset(1000)
        self.assertEqual(tracker.update(1500), 1500)
        self.assertEqual(tracker.update(2000), 2000)

    def test_normal_backward_motion_no_wrap(self):
        tracker = EncoderPulseTracker()
        tracker.reset(2000)
        self.assertEqual(tracker.update(1500), 1500)
        self.assertEqual(tracker.update(1000), 1000)

    def test_forward_wraparound_keeps_increasing(self):
        # Raw counter approaches the top of its 32-bit range, then wraps
        # back to a small value while the motor kept moving forward.
        tracker = EncoderPulseTracker()
        tracker.reset(RAW_RANGE - 100)  # raw = 0xFFFFFF9C
        cumulative = tracker.update(50)  # wrapped: moved forward by 150 pulses
        self.assertEqual(cumulative, RAW_RANGE - 100 + 150)
        # Continuing forward after the wrap should keep accumulating normally.
        cumulative = tracker.update(550)
        self.assertEqual(cumulative, RAW_RANGE - 100 + 150 + 500)

    def test_backward_wraparound_keeps_decreasing(self):
        # Raw counter near zero, then wraps to near the top of its range
        # while the motor kept moving backward (negative direction).
        tracker = EncoderPulseTracker()
        tracker.reset(50)
        cumulative = tracker.update(RAW_RANGE - 100)  # moved backward by 150
        self.assertEqual(cumulative, 50 - 150)
        cumulative = tracker.update(RAW_RANGE - 600)
        self.assertEqual(cumulative, 50 - 150 - 500)

    def test_large_but_sub_half_range_delta_is_not_mistaken_for_wrap(self):
        tracker = EncoderPulseTracker()
        tracker.reset(0)
        big_forward_jump = HALF_RAW_RANGE - 1
        self.assertEqual(tracker.update(big_forward_jump), big_forward_jump)

    def test_exactly_half_range_delta_resolves_as_negative(self):
        # At the boundary, (new - old) % RAW_RANGE == HALF_RAW_RANGE is
        # ambiguous (could be +half or -half); this documents the tracker's
        # tie-breaking choice (treated as negative) so behavior is defined.
        tracker = EncoderPulseTracker()
        tracker.reset(0)
        cumulative = tracker.update(HALF_RAW_RANGE)
        self.assertEqual(cumulative, -HALF_RAW_RANGE)

    def test_raw_values_are_masked_to_32_bits(self):
        tracker = EncoderPulseTracker()
        tracker.reset(RAW_RANGE + 10)  # out-of-range input gets masked
        self.assertEqual(tracker.cumulative, 10)

    def test_multiple_wraps_across_many_updates(self):
        tracker = EncoderPulseTracker()
        tracker.reset(0)
        step = 100_000  # RAW_RANGE / step ~= 42950 updates per wrap
        raw = 0
        expected_cumulative = 0
        for _ in range(100_000):  # forces a couple of wraps of the 32-bit range
            raw = (raw + step) % RAW_RANGE
            expected_cumulative += step
            self.assertEqual(tracker.update(raw), expected_cumulative)


if __name__ == "__main__":
    unittest.main()
