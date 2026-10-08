"""
Tests for check_requirements.py (the launcher's "install only if needed"
check). Installed versions are faked; nothing is installed or downloaded.
Run from inside this directory: python -m unittest test_check_requirements
"""
import os
import tempfile
import unittest
from unittest.mock import patch

import check_requirements
from check_requirements import PackageNotFoundError, problems, version_tuple


def fake_versions(installed):
    def lookup(name):
        if name not in installed:
            raise PackageNotFoundError(name)
        return installed[name]
    return patch.object(check_requirements, "installed_version", side_effect=lookup)


class ProblemsTests(unittest.TestCase):

    def test_all_met(self):
        with fake_versions({"pyserial": "3.5", "flask": "3.0.3", "python-osc": "1.9.0"}):
            self.assertEqual(problems(["pyserial>=3.5\n", "flask>=2.3\n", "python-osc>=1.7.0\n"]), [])

    def test_missing_package(self):
        with fake_versions({"pyserial": "3.5"}):
            self.assertEqual(problems(["pyserial>=3.5", "flask>=2.3"]), ["flask is not installed"])

    def test_too_old(self):
        with fake_versions({"flask": "2.2.5"}):
            self.assertEqual(problems(["flask>=2.3"]), ["flask 2.2.5 is older than 2.3"])

    def test_version_compares_numerically_not_as_text(self):
        with fake_versions({"flask": "2.10.0"}):
            self.assertEqual(problems(["flask>=2.9"]), [])

    def test_name_without_version_only_needs_to_be_installed(self):
        with fake_versions({"flask": "0.1"}):
            self.assertEqual(problems(["flask"]), [])

    def test_comments_blank_lines_and_markers_are_ignored(self):
        with fake_versions({"flask": "3.0"}):
            self.assertEqual(problems(["# comment\n", "\n", "flask>=2.3  # web UI\n",
                                       "flask>=2.3; python_version >= '3.8'\n"]), [])

    def test_unparseable_line_means_install(self):
        with fake_versions({"flask": "3.0"}):
            self.assertEqual(len(problems(["flask==3.0"])), 1)

    def test_version_tuple(self):
        self.assertEqual(version_tuple("3.1.2rc1"), (3, 1, 2))
        self.assertEqual(version_tuple("1.7.0"), (1, 7, 0))


class MainTests(unittest.TestCase):

    def run_main(self, text, installed):
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
            f.write(text)
        self.addCleanup(os.remove, f.name)
        with fake_versions(installed), patch("builtins.print"):
            return check_requirements.main(["check_requirements.py", f.name])

    def test_exit_codes(self):
        self.assertEqual(self.run_main("flask>=2.3\n", {"flask": "3.0"}), 0)
        self.assertEqual(self.run_main("flask>=2.3\n", {}), 1)

    def test_unreadable_file_means_install(self):
        with patch("builtins.print"):
            self.assertEqual(check_requirements.main(["x", "no_such_file.txt"]), 1)

    def test_real_requirements_file_parses(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "requirements_pc.txt"), encoding="utf-8") as f:
            lines = f.readlines()
        with fake_versions({"pyserial": "99", "flask": "99", "python-osc": "99"}):
            self.assertEqual(problems(lines), [])


if __name__ == "__main__":
    unittest.main()
