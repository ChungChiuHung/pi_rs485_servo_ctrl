"""
Tests the hardware_serialized decorator's concurrency behavior in
isolation -- does NOT exercise /action or /alarm/clear themselves (that
would need real hardware writes), and does NOT import app.py (which opens
a real serial connection at module load time). hardware_lock.py has no
serial/hardware dependency at all, so this suite runs anywhere, no port
required.
"""
import threading
import time
import unittest

from flask import Flask

from hardware_lock import hardware_serialized, run_when_idle

# A throwaway Flask app purely to give jsonify() (used inside
# hardware_serialized's busy-response path) the application context it
# needs -- unrelated to the real app.py.
_test_app = Flask(__name__)


class TestHardwareSerializedDecorator(unittest.TestCase):
    """Real-hardware timing tests (check_response_timing.py) found the
    driver handles rapid consecutive reads fine -- the actual risk from a
    user double-clicking a button is our own app running two
    hardware-touching requests concurrently, not the driver choking on
    speed. hardware_serialized (hardware_lock.py) is the fix: a second request that
    arrives while one is already in flight is rejected (429) immediately,
    never queued to run later out of order.
    """

    def test_second_concurrent_call_is_rejected_not_queued(self):
        call_log = []
        release_first_call = threading.Event()

        @hardware_serialized
        def slow_dummy():
            call_log.append("start")
            release_first_call.wait(timeout=2)
            call_log.append("end")
            return "ok", 200

        results = {}

        def worker(key):
            with _test_app.app_context():
                results[key] = slow_dummy()

        t1 = threading.Thread(target=worker, args=("first",))
        t1.start()

        # Give the first call time to acquire the lock and start "working".
        deadline = time.time() + 1
        while not call_log and time.time() < deadline:
            time.sleep(0.005)
        self.assertEqual(call_log, ["start"], "first call should be in flight")

        second_call_start = time.time()
        worker("second")  # runs on this thread while t1 is still blocked
        second_call_duration = time.time() - second_call_start

        release_first_call.set()
        t1.join(timeout=2)

        self.assertEqual(results["second"][1], 429)
        self.assertLess(
            second_call_duration, 0.2,
            "the busy response must return immediately, not wait for the "
            "first call to finish (that would be queuing, not rejecting)"
        )
        self.assertEqual(results["first"], ("ok", 200))
        self.assertEqual(call_log, ["start", "end"])

    def test_sequential_calls_both_succeed(self):
        @hardware_serialized
        def quick_dummy():
            return "ok", 200

        with _test_app.app_context():
            first = quick_dummy()
            second = quick_dummy()

        self.assertEqual(first, ("ok", 200))
        self.assertEqual(second, ("ok", 200))


class TestRunWhenIdle(unittest.TestCase):

    def test_runs_when_nothing_is_in_flight(self):
        ran = []
        self.assertTrue(run_when_idle(lambda: ran.append(1)))
        self.assertEqual(ran, [1])

    def test_skips_without_waiting_while_a_request_is_in_flight(self):
        release = threading.Event()
        started = threading.Event()

        @hardware_serialized
        def slow():
            started.set()
            release.wait(timeout=2)
            return "ok", 200

        def worker():
            with _test_app.app_context():
                slow()

        t = threading.Thread(target=worker)
        t.start()
        self.assertTrue(started.wait(timeout=1))
        ran = []
        self.assertFalse(run_when_idle(lambda: ran.append(1)))
        self.assertEqual(ran, [])
        release.set()
        t.join(timeout=2)
        self.assertTrue(run_when_idle(lambda: ran.append(1)))


if __name__ == "__main__":
    unittest.main()
