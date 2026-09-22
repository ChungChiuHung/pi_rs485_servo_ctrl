"""
Tests for hardware_lock.py, added 2026-09-22 alongside the fix for a
real-hardware report: a fast keyboard tap on a JOG arrow key fires
motionStart_CW/CCW (keydown) immediately followed by motionPause (keyup),
close enough together that motionPause could arrive while motionStart_CW/CCW
was still holding the busy-lock and get rejected with 429 -- silently
dropping the stop, since nothing retries it once the key has been released.
STOP_ACTIONS now wait (bounded) for the lock instead of being rejected.

No app.py import (it opens a real serial connection at module load time) --
a minimal Flask app exercises the decorator directly, per the module's own
"unit-testable without importing app.py" design note.
"""
import threading
import time
import unittest

from flask import Flask, jsonify

import hardware_lock
from hardware_lock import hardware_serialized, run_when_idle, STOP_ACTIONS


def make_test_app():
    app = Flask(__name__)

    @app.route('/action', methods=['POST'])
    @hardware_serialized
    def action():
        return jsonify({"status": "success"})

    return app


class HardwareSerializedTests(unittest.TestCase):

    def setUp(self):
        self.app = make_test_app()
        self.client = self.app.test_client()
        # Defensive: a prior test failure must not leave the module-level
        # lock held and break every test after it.
        if hardware_lock._hardware_busy_lock.locked():
            hardware_lock._hardware_busy_lock.release()

    def tearDown(self):
        if hardware_lock._hardware_busy_lock.locked():
            hardware_lock._hardware_busy_lock.release()

    def test_stop_actions_are_exactly_motion_pause_and_motion_cancel(self):
        """Pin the set explicitly -- adding/removing a stop-type action
        changes real hardware-safety behavior and should be a deliberate,
        visible diff here, not silently inherited by an unrelated action."""
        self.assertEqual(STOP_ACTIONS, frozenset({"motionPause", "motionCancel"}))

    def test_non_stop_action_is_rejected_immediately_when_busy(self):
        hardware_lock._hardware_busy_lock.acquire()
        start = time.monotonic()
        response = self.client.post('/action', json={"action": "servoOn"})
        elapsed = time.monotonic() - start
        self.assertEqual(response.status_code, 429)
        self.assertLess(elapsed, 0.1)  # rejected immediately, did not wait

    def test_stop_action_waits_and_succeeds_once_the_lock_is_released(self):
        hardware_lock._hardware_busy_lock.acquire()

        def release_soon():
            time.sleep(0.2)
            hardware_lock._hardware_busy_lock.release()

        threading.Thread(target=release_soon, daemon=True).start()

        start = time.monotonic()
        response = self.client.post('/action', json={"action": "motionPause"})
        elapsed = time.monotonic() - start

        self.assertEqual(response.status_code, 200)
        self.assertGreaterEqual(elapsed, 0.15, "should have actually waited for the lock, not returned immediately")

    def test_motion_cancel_also_waits(self):
        hardware_lock._hardware_busy_lock.acquire()

        def release_soon():
            time.sleep(0.15)
            hardware_lock._hardware_busy_lock.release()

        threading.Thread(target=release_soon, daemon=True).start()
        response = self.client.post('/action', json={"action": "motionCancel"})
        self.assertEqual(response.status_code, 200)

    def test_stop_action_still_429s_if_the_wait_exceeds_the_timeout(self):
        original_timeout = hardware_lock.STOP_ACTION_WAIT_TIMEOUT_S
        hardware_lock.STOP_ACTION_WAIT_TIMEOUT_S = 0.1
        try:
            hardware_lock._hardware_busy_lock.acquire()
            try:
                response = self.client.post('/action', json={"action": "motionPause"})
                self.assertEqual(response.status_code, 429)
            finally:
                hardware_lock._hardware_busy_lock.release()
        finally:
            hardware_lock.STOP_ACTION_WAIT_TIMEOUT_S = original_timeout

    def test_non_json_body_is_treated_as_not_a_stop_action(self):
        """Other routes decorated with @hardware_serialized (e.g.
        /encoder_mode/adopt) don't send an 'action' field at all -- must not
        crash and must not wait."""
        hardware_lock._hardware_busy_lock.acquire()
        try:
            response = self.client.post('/action', data="not json", content_type="text/plain")
            self.assertEqual(response.status_code, 429)
        finally:
            hardware_lock._hardware_busy_lock.release()

    def test_missing_action_field_is_treated_as_not_a_stop_action(self):
        hardware_lock._hardware_busy_lock.acquire()
        try:
            response = self.client.post('/action', json={})
            self.assertEqual(response.status_code, 429)
        finally:
            hardware_lock._hardware_busy_lock.release()


class RunWhenIdleTests(unittest.TestCase):
    """Unchanged by this fix -- background housekeeping must still never
    wait, regardless of what STOP_ACTIONS does for /action requests."""

    def tearDown(self):
        if hardware_lock._hardware_busy_lock.locked():
            hardware_lock._hardware_busy_lock.release()

    def test_runs_when_idle(self):
        ran = run_when_idle(lambda: None)
        self.assertTrue(ran)

    def test_skips_when_busy(self):
        hardware_lock._hardware_busy_lock.acquire()
        try:
            ran = run_when_idle(lambda: None)
            self.assertFalse(ran)
        finally:
            hardware_lock._hardware_busy_lock.release()


if __name__ == "__main__":
    unittest.main()
