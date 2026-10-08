"""
Tests for create_motor_profiles.py (start_server.bat creates
motor_profiles.json from the committed template when it is missing). Works in
a temporary folder; the real motor_profiles.json is never touched.
Run from inside this directory: python -m unittest test_create_motor_profiles
"""
import io
import json
import os
import shutil
import tempfile
import unittest
from contextlib import redirect_stdout

import create_motor_profiles
from motor_profile import load_active_profile, resolve_profile

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLE = os.path.join(HERE, "motor_profiles.example.json")


class CreateMotorProfilesTests(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.dir)
        self.example = os.path.join(self.dir, "motor_profiles.example.json")
        shutil.copy(EXAMPLE, self.example)
        self.target = os.path.join(self.dir, "motor_profiles.json")

    def create(self, **kwargs):
        with redirect_stdout(io.StringIO()) as out:
            code = create_motor_profiles.create(self.example, self.target, **kwargs)
        return code, out.getvalue()

    def load(self, path):
        with open(path) as f:
            return json.load(f)

    def test_default_is_an_exact_copy_of_the_template(self):
        code, text = self.create()
        self.assertEqual(code, 0)
        self.assertEqual(self.load(self.target), self.load(self.example))
        self.assertEqual(load_active_profile(self.target)["name"], "shihlin_400W")
        self.assertIn("default motor: shihlin_400W", text)
        self.assertIn("--profile shihlin_50W", text)  # how to pick the other motor

    def test_profile_option_changes_only_the_default_motor(self):
        code, _ = self.create(profile_name="shihlin_50W")
        self.assertEqual(code, 0)
        expected = self.load(self.example)
        expected["active_profile"] = "shihlin_50W"
        self.assertEqual(self.load(self.target), expected)

    def test_unknown_profile_option_creates_nothing(self):
        code, text = self.create(profile_name="shihlin_9000W")
        self.assertEqual(code, 1)
        self.assertIn("unknown profile", text)
        self.assertFalse(os.path.exists(self.target))

    def test_an_existing_file_is_never_overwritten(self):
        with open(self.target, "w") as f:
            f.write('{"tuned": "on this rig"}')
        code, text = self.create()
        self.assertEqual(code, 0)
        self.assertIn("left unchanged", text)
        with open(self.target) as f:
            self.assertEqual(f.read(), '{"tuned": "on this rig"}')

    def test_missing_template_creates_nothing(self):
        os.remove(self.example)
        code, text = self.create()
        self.assertEqual(code, 1)
        self.assertIn("cannot read the template", text)
        self.assertFalse(os.path.exists(self.target))

    def test_command_line(self):
        with redirect_stdout(io.StringIO()):
            self.assertEqual(create_motor_profiles.main(["x", "--bogus"]), 1)
            self.assertEqual(create_motor_profiles.main(["x", "--profile"]), 1)


class TemplateTests(unittest.TestCase):
    """The committed template is the default configuration a new PC gets."""

    def test_default_motor_is_the_400w_rig_and_every_profile_resolves(self):
        with open(EXAMPLE) as f:
            profiles = json.load(f)
        self.assertEqual(profiles["active_profile"], "shihlin_400W")
        for name in profiles["profiles"]:
            with self.subTest(profile=name):
                self.assertGreater(resolve_profile(profiles, name)["base_pulse_per_degree"], 0)

    def test_template_is_not_gitignored_but_the_real_file_is(self):
        with open(os.path.join(HERE, "..", ".gitignore")) as f:
            ignored = f.read()
        self.assertIn("/servo_comm_shihlin_unified/motor_profiles.json", ignored)
        self.assertNotIn("motor_profiles.example.json", ignored)


if __name__ == "__main__":
    unittest.main()
