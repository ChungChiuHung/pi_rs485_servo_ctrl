"""
In-memory activity log surfaced in the web UI, for a user who isn't
watching the server's own console/terminal output. Taps into the
existing `logging` calls already made throughout servo_control.py,
osc_server.py, artnet_server.py, and app.py -- every one of those
already logs a meaningful message for each action (e.g. "Art-Net:
position-mode move to 40.00 deg at 50 rpm (channel 4 rising edge...)"),
so capturing them here needs no changes to that already hardware-tested
code.
"""
import logging
import threading
import time
from collections import deque
from itertools import count


class ActivityLog:
    """Thread-safe, fixed-size ring buffer of {id, time, level, logger,
    message} entries, with incremental fetch via get_since(last_id) so
    the web UI can poll for only what's new instead of re-fetching
    everything on every tick."""

    def __init__(self, maxlen: int = 300):
        self._entries = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._next_id = count(1)

    def add(self, level: str, logger_name: str, message: str) -> None:
        with self._lock:
            self._entries.append({
                "id": next(self._next_id),
                "time": time.time(),
                "level": level,
                "logger": logger_name,
                "message": message,
            })

    def get_since(self, last_id: int = 0) -> list:
        with self._lock:
            return [e for e in self._entries if e["id"] > last_id]


class ActivityLogHandler(logging.Handler):
    """Bridges Python's logging system into an ActivityLog. Attach to the
    root logger to capture every existing logging.info/warning/error call
    across the codebase. Filters out werkzeug's per-request access logs
    (e.g. "GET /status 200") -- noise to someone watching this feed to
    see what their last command actually did, not an HTTP access log."""

    def __init__(self, activity_log: ActivityLog):
        super().__init__()
        self.activity_log = activity_log
        self.addFilter(lambda record: not record.name.startswith("werkzeug"))

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = self.format(record)
        except Exception:
            message = record.getMessage()
        self.activity_log.add(record.levelname.lower(), record.name, message)
