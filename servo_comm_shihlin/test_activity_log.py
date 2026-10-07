import logging
import unittest

from activity_log import ActivityLog, ActivityLogHandler


class TestActivityLog(unittest.TestCase):

    def test_add_and_get_since_zero_returns_everything(self):
        log = ActivityLog()
        log.add("info", "app", "first")
        log.add("info", "app", "second")

        entries = log.get_since(0)

        self.assertEqual([e["message"] for e in entries], ["first", "second"])

    def test_get_since_returns_only_newer_entries(self):
        log = ActivityLog()
        log.add("info", "app", "first")
        first_id = log.get_since(0)[0]["id"]
        log.add("info", "app", "second")
        log.add("info", "app", "third")

        entries = log.get_since(first_id)

        self.assertEqual([e["message"] for e in entries], ["second", "third"])

    def test_ids_are_strictly_increasing(self):
        log = ActivityLog()
        log.add("info", "app", "a")
        log.add("info", "app", "b")

        ids = [e["id"] for e in log.get_since(0)]

        self.assertEqual(ids, sorted(ids))
        self.assertEqual(len(ids), len(set(ids)))

    def test_maxlen_drops_oldest_entries(self):
        log = ActivityLog(maxlen=3)
        for i in range(5):
            log.add("info", "app", f"msg{i}")

        entries = log.get_since(0)

        self.assertEqual([e["message"] for e in entries], ["msg2", "msg3", "msg4"])

    def test_entry_has_level_and_logger_name(self):
        log = ActivityLog()
        log.add("warning", "servo_control", "something odd")

        entry = log.get_since(0)[0]

        self.assertEqual(entry["level"], "warning")
        self.assertEqual(entry["logger"], "servo_control")


class TestActivityLogHandler(unittest.TestCase):

    def _make_logger_with_handler(self, activity_log, name):
        logger = logging.getLogger(name)
        logger.setLevel(logging.DEBUG)
        handler = ActivityLogHandler(activity_log)
        logger.addHandler(handler)
        self.addCleanup(logger.removeHandler, handler)
        return logger

    def test_logging_call_is_captured(self):
        log = ActivityLog()
        logger = self._make_logger_with_handler(log, "test_activity_log.capture")

        logger.info("Art-Net: position-mode move to 40.00 deg at 50 rpm")

        entries = log.get_since(0)
        self.assertEqual(len(entries), 1)
        self.assertEqual(entries[0]["level"], "info")
        self.assertIn("position-mode move", entries[0]["message"])

    def test_werkzeug_access_logs_are_filtered_out(self):
        """Noise to someone watching this feed for "what did my last
        command actually do" -- not an HTTP access log."""
        log = ActivityLog()
        logger = self._make_logger_with_handler(log, "werkzeug")

        logger.info('GET /status HTTP/1.1" 200 -')

        self.assertEqual(log.get_since(0), [])

    def test_error_level_is_preserved(self):
        log = ActivityLog()
        logger = self._make_logger_with_handler(log, "test_activity_log.errors")

        logger.error("No response reading servo state (0x0200).")

        entries = log.get_since(0)
        self.assertEqual(entries[0]["level"], "error")


if __name__ == "__main__":
    unittest.main()
