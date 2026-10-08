"""
Tests for check_pc_setup.py (start_server.bat's setup report and "install
only if needed" check). Installed versions and serial ports are faked;
nothing is installed, downloaded or opened.
Run from inside this directory: python -m unittest test_check_pc_setup
"""
import io
import os
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

import check_pc_setup
from check_pc_setup import PackageNotFoundError, package_status, version_tuple


def fake_versions(installed):
    def lookup(name):
        if name not in installed:
            raise PackageNotFoundError(name)
        return installed[name]
    return patch.object(check_pc_setup, "installed_version", side_effect=lookup)


ALL_INSTALLED = {"pyserial": "3.5", "flask": "3.0.3", "python-osc": "1.9.0"}


class PackageStatusTests(unittest.TestCase):

    def statuses(self, lines, installed):
        with fake_versions(installed):
            return [(row[0], row[1]) for row in package_status(lines)]

    def test_all_met(self):
        self.assertEqual(self.statuses(["pyserial>=3.5\n", "flask>=2.3\n"], ALL_INSTALLED),
                         [("OK", "pyserial"), ("OK", "flask")])

    def test_missing_and_too_old(self):
        self.assertEqual(self.statuses(["pyserial>=3.5", "flask>=2.3"], {"flask": "2.2.5"}),
                         [("MISSING", "pyserial"), ("TOO OLD", "flask")])

    def test_version_compares_numerically_not_as_text(self):
        self.assertEqual(self.statuses(["flask>=2.9"], {"flask": "2.10.0"}), [("OK", "flask")])

    def test_name_without_version_only_needs_to_be_installed(self):
        self.assertEqual(self.statuses(["flask"], {"flask": "0.1"}), [("OK", "flask")])

    def test_comments_blank_lines_and_markers_are_ignored(self):
        lines = ["# comment\n", "\n", "flask>=2.3  # web UI\n",
                 "flask>=2.3; python_version >= '3.8'\n"]
        self.assertEqual(self.statuses(lines, ALL_INSTALLED), [("OK", "flask"), ("OK", "flask")])

    def test_exact_pin_and_compatible_release(self):
        self.assertEqual(self.statuses(["flask==3.0.3", "pyserial==3.4", "python-osc~=1.8"], ALL_INSTALLED),
                         [("OK", "flask"), ("WRONG VERSION", "pyserial"), ("OK", "python-osc")])

    def test_unparseable_line_is_unchecked(self):
        self.assertEqual(self.statuses(["flask>3.0"], ALL_INSTALLED), [("UNCHECKED", "flask>3.0")])

    def test_version_tuple(self):
        self.assertEqual(version_tuple("3.1.2rc1"), (3, 1, 2))
        self.assertEqual(version_tuple("1.7.0"), (1, 7, 0))

    def test_real_requirements_file_parses_and_has_no_pi_only_package(self):
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "requirements_pc.txt"), encoding="utf-8") as f:
            lines = f.readlines()
        with fake_versions({"pyserial": "99", "flask": "99", "python-osc": "99"}):
            rows = package_status(lines)
        self.assertEqual({row[0] for row in rows}, {"OK"})
        self.assertNotIn("rpi.gpio", {row[1].lower() for row in rows})


class ReportTests(unittest.TestCase):

    def run_report(self, text, installed, ports=(), python=(3, 9, 13), flags=()):
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
            f.write(text)
        self.addCleanup(os.remove, f.name)
        out = io.StringIO()
        with fake_versions(installed), \
                patch.object(check_pc_setup, "serial_ports", return_value=ports), \
                patch.object(check_pc_setup.sys, "version_info", python), \
                redirect_stdout(out):
            code = check_pc_setup.main(["check_pc_setup.py", *flags, f.name])
        return code, out.getvalue()

    def test_unchecked_line_needs_install_but_passes_after_pip(self):
        # Otherwise the re-check after a successful pip install would fail forever.
        self.assertEqual(self.run_report("flask>3.0\n", ALL_INSTALLED)[0], 1)
        code, text = self.run_report("flask>3.0\n", ALL_INSTALLED, flags=("--installed",))
        self.assertEqual(code, 0)
        self.assertIn("left to pip", text)

    def test_installed_flag_does_not_hide_a_missing_package(self):
        self.assertEqual(self.run_report("flask>=2.3\n", {}, flags=("--installed",))[0], 1)

    def test_exit_0_when_ready_and_ports_are_listed(self):
        code, text = self.run_report("flask>=2.3\n", ALL_INSTALLED, ports=[("COM4", "USB-SERIAL CH340 (COM4)")])
        self.assertEqual(code, 0)
        self.assertIn("COM4", text)
        self.assertIn("RPi.GPIO is for the Raspberry Pi only", text)

    def test_exit_1_and_reason_when_a_package_is_missing(self):
        code, text = self.run_report("flask>=2.3\n", {})
        self.assertEqual(code, 1)
        self.assertRegex(text, r"MISSING\s+flask")

    def test_exit_1_when_python_is_too_old(self):
        code, text = self.run_report("flask>=2.3\n", ALL_INSTALLED, python=(3, 7, 9))
        self.assertEqual(code, 1)
        self.assertIn("TOO OLD", text)

    def test_no_ports_gives_a_driver_hint(self):
        _, text = self.run_report("flask>=2.3\n", ALL_INSTALLED, ports=[])
        self.assertIn("Device Manager", text)

    def test_ports_wait_for_pyserial(self):
        _, text = self.run_report("flask>=2.3\n", ALL_INSTALLED, ports=None)
        self.assertIn("shown once pyserial is installed", text)

    def test_unreadable_requirements_file_means_install(self):
        with redirect_stdout(io.StringIO()):
            self.assertEqual(check_pc_setup.main(["x", "no_such_file.txt"]), 1)


if __name__ == "__main__":
    unittest.main()
