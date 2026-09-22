"""
Serializes hardware-touching Flask endpoints against each other.

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

Exception: STOP_ACTIONS (see below). Found 2026-09-22 via a real-hardware
report -- a fast keyboard tap on a JOG arrow key fires motionStart_CW/CCW
on keydown immediately followed by its own keyup's motionPause, close
enough together that motionPause could arrive while motionStart_CW/CCW was
still in flight and get flatly rejected with 429. Unlike a stale
double-click, a dropped STOP is never retried by anything (the key has
already been released) and leaves the motor running with no other way to
stop it. So a stop-type action WAITS (bounded) for the lock instead of
being rejected outright -- "queuing" is exactly the right behavior for a
stop, since it is always correct to run as soon as the bus is free, never
"stale".

Split into its own module (no serial/hardware imports) specifically so it's
unit-testable without importing app.py, which opens a real serial
connection at module load time.
"""
import threading
from functools import wraps

from flask import jsonify, request

_hardware_busy_lock = threading.Lock()

# /action requests naming one of these must never be silently dropped by
# the busy-lock -- see the module docstring. motionPause is the direct
# fix for the reported bug; motionCancel is the same class of action (also
# stops motion, also has no other retry path once fired) and is included
# for the same reason.
STOP_ACTIONS = frozenset({"motionPause", "motionCancel"})
# How long a stop-type action will wait for the lock before giving up and
# answering 429 -- comfortably above any single action's real duration
# (typically 100-300ms, per check_response_timing.py), so this almost
# never actually triggers; it exists only to bound the wait, not because
# waiting this long is expected.
STOP_ACTION_WAIT_TIMEOUT_S = 2.0


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
        # request.get_json() is cached by Flask/Werkzeug, so peeking at the
        # action name here does not stop the wrapped view from reading the
        # body itself afterward. silent=True: a non-JSON/absent body (any
        # non-/action route using this decorator) just means "not a stop
        # action", not a 400 before the real view even runs. RuntimeError:
        # this decorator is also applied to plain functions exercised
        # outside any real HTTP request (see test_app_concurrency.py's
        # app_context()-only tests) -- request itself is unavailable there,
        # which must fall back to "not a stop action", not crash.
        try:
            body = request.get_json(silent=True) or {}
        except RuntimeError:
            body = {}
        is_stop_action = body.get('action') in STOP_ACTIONS
        acquired = _hardware_busy_lock.acquire(
            blocking=is_stop_action,
            timeout=STOP_ACTION_WAIT_TIMEOUT_S if is_stop_action else -1)
        if not acquired:
            return jsonify({
                "status": "error",
                "message": "Another command is still in progress. Please wait and try again.",
            }), 429
        try:
            return f(*args, **kwargs)
        finally:
            _hardware_busy_lock.release()
    return decorated_function
