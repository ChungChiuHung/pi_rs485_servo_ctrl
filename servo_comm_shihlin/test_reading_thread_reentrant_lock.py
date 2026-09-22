"""
Regression test for a real deadlock found via the web UI 2026-09-22:
ENABLE POS MODE leaves ServoController.reading_active True; the next
POS TEST START CW/CCW click calls pos_step_motion_test() ->
start_continuous_reading(), which -- because reading_active is already
True -- calls stop_continuous_reading() from *inside* its own
`with self.lock:` block. With a plain threading.Lock() (non-reentrant)
that call hangs forever (the thread deadlocks trying to re-acquire a lock
it already holds), which also never releases hardware_lock.py's
_hardware_busy_lock, so every other action then gets rejected with 429
until the process is restarted. Fixed by making ServoController.lock an
RLock (matching servo_comm_shihlin_unified, which already uses one).

Mocked serial port; nothing touches hardware. Run from inside this
directory: python -m unittest test_reading_thread_reentrant_lock
"""
import threading
import unittest
from unittest.mock import MagicMock

from servo_control import ServoController


def make_controller():
    fake_serial = MagicMock()
    fake_serial.keep_running = True
    ctrl = ServoController(fake_serial)
    ctrl.modbus_client = MagicMock()
    # The background thread's own reads should just fail fast and retry --
    # irrelevant to this test, which is only about the lock, but must not
    # block on real timing.
    ctrl.read_encoder_before_gear_ratio = MagicMock(return_value=None)
    return ctrl


class ReadingThreadReentrantLockTests(unittest.TestCase):

    def test_start_continuous_reading_while_already_active_does_not_deadlock(self):
        ctrl = make_controller()

        ctrl.start_continuous_reading(interval=0.01)
        self.assertTrue(ctrl.reading_active)

        # This is the exact call POS TEST START CW/CCW makes (via
        # pos_step_motion_test()) right after ENABLE POS MODE has already
        # called start_continuous_reading() once. Run it on its own thread
        # so a regression (a real deadlock) fails this test instead of
        # hanging the whole test process; the thread is left running in
        # that failure case, but it's a daemon so it won't block process exit.
        result = {}

        def call_again():
            ctrl.start_continuous_reading(interval=0.01)
            result["returned"] = True

        t = threading.Thread(target=call_again, daemon=True)
        t.start()
        t.join(timeout=2.0)

        self.assertFalse(t.is_alive(), "start_continuous_reading() deadlocked "
                                        "(self.lock must be an RLock)")
        self.assertTrue(result.get("returned"))
        # Toggle behavior is unchanged: a second call while active stops it.
        self.assertFalse(ctrl.reading_active)

    def test_stop_continuous_reading_reentrant_from_start(self):
        """Same lock, called the other direction: stop_continuous_reading()
        itself must not deadlock either (it also takes self.lock)."""
        ctrl = make_controller()
        ctrl.start_continuous_reading(interval=0.01)

        t = threading.Thread(target=ctrl.stop_continuous_reading, daemon=True)
        t.start()
        t.join(timeout=2.0)

        self.assertFalse(t.is_alive(), "stop_continuous_reading() deadlocked")
        self.assertFalse(ctrl.reading_active)


if __name__ == "__main__":
    unittest.main()
