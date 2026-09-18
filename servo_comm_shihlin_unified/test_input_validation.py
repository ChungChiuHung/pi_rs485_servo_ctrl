import unittest

from input_validation import validate_int_range


class TestValidateIntRange(unittest.TestCase):

    def test_in_range_int_passes(self):
        value, error = validate_int_range(100, 0, 3000, "speed_rpm")
        self.assertEqual(value, 100)
        self.assertIsNone(error)

    def test_whole_number_float_is_coerced_to_int(self):
        """JSON has no distinct int/float type -- {"pulses": 1920} and
        {"pulses": 1920.0} both arrive as a JS number, so a whole-number
        float must be accepted, not rejected as "not an int"."""
        value, error = validate_int_range(1920.0, 0, 2**31 - 1, "pulses")
        self.assertEqual(value, 1920)
        self.assertIsNone(error)

    def test_non_whole_float_is_rejected(self):
        value, error = validate_int_range(100.5, 0, 3000, "speed_rpm")
        self.assertIsNone(value)
        self.assertIn("whole number", error)

    def test_below_minimum_is_rejected(self):
        value, error = validate_int_range(-1, 0, 3000, "speed_rpm")
        self.assertIsNone(value)
        self.assertIn("between 0 and 3000", error)

    def test_above_maximum_is_rejected(self):
        value, error = validate_int_range(3001, 0, 3000, "speed_rpm")
        self.assertIsNone(value)
        self.assertIn("between 0 and 3000", error)

    def test_boundary_values_are_accepted(self):
        self.assertEqual(validate_int_range(0, 0, 3000, "speed_rpm"), (0, None))
        self.assertEqual(validate_int_range(3000, 0, 3000, "speed_rpm"), (3000, None))

    def test_non_numeric_string_is_rejected(self):
        value, error = validate_int_range("abc", 0, 3000, "speed_rpm")
        self.assertIsNone(value)
        self.assertIn("must be a number", error)

    def test_bool_is_rejected_even_though_it_is_an_int_subclass(self):
        """True/False are technically ints in Python -- must not silently
        pass through as 0/1."""
        value, error = validate_int_range(True, 0, 3000, "speed_rpm")
        self.assertIsNone(value)
        self.assertIn("must be a number", error)

    def test_none_is_rejected(self):
        value, error = validate_int_range(None, 0, 3000, "speed_rpm")
        self.assertIsNone(value)
        self.assertIn("must be a number", error)

    def test_error_message_includes_field_name(self):
        _, error = validate_int_range(-1, 0, 3000, "speed_rpm")
        self.assertIn("speed_rpm", error)


if __name__ == "__main__":
    unittest.main()
