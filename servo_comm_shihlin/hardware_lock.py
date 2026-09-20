"""
Serializes hardware-touching Flask endpoints against each other.

(Copied from servo_comm_shihlin_unified. The timing figures below were
measured there over Modbus RTU; this folder speaks ASCII and has not been
timed. The lock only serializes web requests against each other -- the
continuous-reading thread is not covered by it.)

Real-hardware timing tests (check_response_timing.py) found the driver
itself handles back-to-back Modbus RTU requests fine -- ~15-20ms round trip
with no rate-related failures down to 0ms spacing. So the actual risk from
a user rapidly clicking a command button isn't "too fast for the driver":
it's two overlapping requests from THIS app interleaving their writes/reads
on the same serial line, or a second click flipping a toggle-like action
(e.g. ServoController.start_continuous_reading(), which stops reading if
it's already active) before the first click's effect has taken hold.

hardware_serialized rejects (429) a request that arrives while another is
already in flight, rather than queuing it -- queuing would let a stale
double-click fire later, out of order, which is exactly the kind of
surprise this is meant to prevent.

Split into its own module (no serial/hardware imports) specifically so it's
unit-testable without importing app.py, which opens a real serial
connection at module load time.
"""
import threading
from functools import wraps

from flask import jsonify

_hardware_busy_lock = threading.Lock()


def run_when_idle(fn) -> bool:
    """Runs fn() only if no hardware-serialized request is in flight; never
    waits. Returns True if it ran. For background housekeeping (e.g. the
    EEPROM-protection guard) that must not delay or interleave with a user's
    command."""
    if not _hardware_busy_lock.acquire(blocking=False):
        return False
    try:
        fn()
        return True
    finally:
        _hardware_busy_lock.release()


def hardware_serialized(f):
    @wraps(f)
    def decorated_function(*args, **kwargs):
        if not _hardware_busy_lock.acquire(blocking=False):
            return jsonify({
                "status": "error",
                "message": "Another command is still in progress. Please wait and try again.",
            }), 429
        try:
            return f(*args, **kwargs)
        finally:
            _hardware_busy_lock.release()
    return decorated_function
