import unittest

from motor_profile import resolve_profile, ProfileNotFoundError

SAMPLE_PROFILES = {
    "active_profile": "shihlin_400W",
    "encoder_pulses_per_rev": 4194304,
    "modbus_device_number": 1,
    "profiles": {
        "shihlin_400W": {"baud_rate": 115200, "gear_ratio": 30, "abs_home_pos": 62369153},
        "shihlin_50W": {"baud_rate": 9600, "gear_ratio": 10, "abs_home_pos": 1184347},
    },
}


class TestResolveProfile(unittest.TestCase):

    def test_400W_base_pulse_per_degree(self):
        resolved = resolve_profile(SAMPLE_PROFILES, "shihlin_400W")
        # 4194304 * 30 / 360 -- matches the historically hardcoded constant
        # (349525.3333333333) that used to live directly in servo_control.py.
        self.assertAlmostEqual(resolved["base_pulse_per_degree"], 349525.3333333333)
        self.assertEqual(resolved["baud_rate"], 115200)
        self.assertEqual(resolved["gear_ratio"], 30)
        self.assertEqual(resolved["abs_home_pos_default"], 62369153)
        self.assertEqual(resolved["name"], "shihlin_400W")

    def test_50W_base_pulse_per_degree(self):
        resolved = resolve_profile(SAMPLE_PROFILES, "shihlin_50W")
        # 4194304 * 10 / 360 -- matches the historically hardcoded constant
        # (116508.444445) from servo_comm_shihlin_50W/servo_control.py.
        self.assertAlmostEqual(resolved["base_pulse_per_degree"], 116508.44444444444)
        self.assertEqual(resolved["baud_rate"], 9600)
        self.assertEqual(resolved["gear_ratio"], 10)

    def test_unknown_profile_raises(self):
        with self.assertRaises(ProfileNotFoundError):
            resolve_profile(SAMPLE_PROFILES, "does_not_exist")

    def test_modbus_device_number_defaults_to_1_if_missing(self):
        profiles = {k: v for k, v in SAMPLE_PROFILES.items() if k != "modbus_device_number"}
        resolved = resolve_profile(profiles, "shihlin_400W")
        self.assertEqual(resolved["modbus_device_number"], 1)


if __name__ == "__main__":
    unittest.main()
