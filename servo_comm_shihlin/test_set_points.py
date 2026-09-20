"""
Set points and config persistence in ServoController (servo_config.json holds
abs_home_pos, set_point_1 and set_point_2 side by side). The serial port is a
mock; nothing touches hardware. Run from inside this directory:
python -m unittest test_set_points
"""
import json
import os
import tempfile
import unittest
from unittest.mock import MagicMock

from servo_control import ServoController, PositionUnavailableError


class ConfigTests(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.config_path = os.path.join(self.tmp.name, "servo_config.json")

    def make(self, initial=None):
        if initial is not None:
            with open(self.config_path, "w") as f:
                json.dump(initial, f)
        fake_serial = MagicMock()
        fake_serial.keep_running = True
        ctrl = ServoController(fake_serial)
        ctrl.CONFIG_FILE = self.config_path
        # __init__ already read the real servo_config.json; re-read from ours.
        ctrl.abs_home_pos = ctrl.load_abs_home_pos()
        ctrl.set_point_1 = ctrl._load_config_value("set_point_1")
        ctrl.set_point_2 = ctrl._load_config_value("set_point_2")
        ctrl.modbus_client = MagicMock()
        return ctrl

    def saved(self):
        with open(self.config_path) as f:
            return json.load(f)

    def test_saving_one_value_keeps_the_others(self):
        ctrl = self.make({"abs_home_pos": 1000, "set_point_1": 12.5, "set_point_2": 40.0})
        ctrl.save_abs_home_pos(2000)
        self.assertEqual(self.saved(), {"abs_home_pos": 2000, "set_point_1": 12.5, "set_point_2": 40.0})

    def test_a_missing_file_gives_defaults_and_is_created_on_save(self):
        ctrl = self.make()
        self.assertEqual(ctrl.abs_home_pos, 1184347)
        self.assertIsNone(ctrl.set_point_1)
        ctrl.save_abs_home_pos(5)
        self.assertEqual(self.saved(), {"abs_home_pos": 5})

    def test_set_points_survive_a_restart(self):
        ctrl = self.make({"abs_home_pos": 1000})
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=1000 + round(10 * 349525.3333333333))
        ctrl.record_set_point(2)
        restarted = self.make()
        self.assertAlmostEqual(restarted.set_point_2, 10.0, places=3)
        self.assertEqual(restarted.abs_home_pos, 1000)  # home was not lost either
        self.assertIsNone(restarted.set_point_1)

    def test_record_refuses_when_the_position_is_unreadable(self):
        ctrl = self.make({"abs_home_pos": 0})
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)
        with self.assertRaises(PositionUnavailableError):
            ctrl.record_set_point(1)
        self.assertIsNone(ctrl.set_point_1)
        self.assertNotIn("set_point_1", self.saved())

    def test_only_set_points_1_and_2_exist(self):
        ctrl = self.make()
        for n in (0, 3, "1"):
            with self.subTest(n=n):
                with self.assertRaises(ValueError):
                    ctrl.record_set_point(n)
                with self.assertRaises(ValueError):
                    ctrl.move_to_set_point(n)

    def test_move_to_an_unrecorded_set_point_never_moves(self):
        ctrl = self.make({"abs_home_pos": 0})
        ctrl._execute_positioning = MagicMock()
        with self.assertRaises(ValueError):
            ctrl.move_to_set_point(1)
        ctrl._execute_positioning.assert_not_called()

    def test_set_home_keeps_the_recorded_set_points(self):
        ctrl = self.make({"abs_home_pos": 0, "set_point_1": 33.0})
        ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=777)
        ctrl.delay_ms = MagicMock()
        ctrl.set_home_position()
        self.assertEqual(self.saved(), {"abs_home_pos": 777, "set_point_1": 33.0})


if __name__ == "__main__":
    unittest.main()
