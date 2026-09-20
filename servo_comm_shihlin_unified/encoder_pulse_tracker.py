"""
Wraparound-safe tracking for the raw 32-bit "motor feedback pulses" counter
(Modbus address 0x0000, 2-word register; see also 0x0024, "translated motor
feedback pulses" -- same width, same issue).

Background: docs/servo_comm_shihlin_merge_design.md §2.4 confirmed this
register wraps silently every 2**32 raw pulses (~1024 motor revolutions,
~34.1 output revolutions at gear ratio 30, ~102.4 at gear ratio 10), with no
alarm or status flag from the driver -- ModbusResponse.get_value() parses it
as a plain unsigned int.from_bytes(...) and the rest of servo_control.py
(diff_angle, pos_step_motion_by, etc.) assumes it's monotonic.

The originally planned fix (§2.4 "Plan B") was to read PA32(APR)/PA33(APP)
instead, which cover the full ±32767-revolution range -- but that requires
PA28 (ABS) to be 1 (absolute mode). On 2026-09-18, real-hardware confirmation
(via RS-485 RTU and independently via Shihlin's SHServo_Soft over USB) found
PA28 == 0: this driver is deliberately operated as an incremental motor
(manual p.17, PA28 description), not absolute mode. PA32/PA33 are therefore
not valid here, and Plan B does not apply as-is.

This module is the alternative: unwrap the existing 0x0000/0x0024 32-bit
counter in software, using the fact that between two polls (100ms apart in
the existing continuous-reading loop) the motor cannot physically move
anywhere close to half of the 2**32 range -- so any observed raw delta can
be unambiguously resolved to a signed short-way delta, and accumulated into
a cumulative pulse count that never silently wraps.

This is read-side-only: it does not send anything to the driver and is not
wired into any motion/write path. It only transforms values already read
by the existing read_motor_feedback_pulses()/read_motor_feedback_pulses_0x0024().
"""
import logging

logger = logging.getLogger(__name__)

RAW_RANGE = 1 << 32
HALF_RAW_RANGE = 1 << 31


class EncoderPulseTracker:
    """Unwraps a wrapping 32-bit unsigned pulse counter into a cumulative,
    signed pulse count that is safe to use in diff/angle calculations.

    Only valid within one continuous run: it has no knowledge of wraps that
    may have happened before the first reset()/update() call (e.g. across a
    program restart while the motor kept moving). Call reset() at the same
    points the existing code already re-captures abs_home_pos (servo_on(),
    set_home_position()) so both stay consistent with each other.
    """

    def __init__(self):
        self._last_raw = None
        self._cumulative = None

    def reset(self, raw_value: int) -> int:
        """Seed (or re-seed) tracking at raw_value. Returns the new cumulative
        value (equal to raw_value immediately after reset)."""
        raw_value &= 0xFFFFFFFF
        self._last_raw = raw_value
        self._cumulative = raw_value
        logger.debug("EncoderPulseTracker reset to raw=%s", raw_value)
        return self._cumulative

    def update(self, raw_value: int) -> int:
        """Feed the latest raw register reading. Returns the updated
        cumulative pulse count. The first call (or the first call after
        reset() has never been called) seeds tracking instead of computing
        a delta, since there is no prior sample to diff against."""
        raw_value &= 0xFFFFFFFF

        if self._last_raw is None:
            return self.reset(raw_value)

        delta = (raw_value - self._last_raw) % RAW_RANGE
        if delta >= HALF_RAW_RANGE:
            delta -= RAW_RANGE
            logger.debug(
                "EncoderPulseTracker: wrap detected (raw %s -> %s), resolved delta=%s",
                self._last_raw, raw_value, delta
            )

        self._cumulative += delta
        self._last_raw = raw_value
        return self._cumulative

    @property
    def cumulative(self) -> int:
        if self._cumulative is None:
            raise RuntimeError(
                "EncoderPulseTracker.update()/reset() has not been called yet"
            )
        return self._cumulative

    @property
    def is_initialized(self) -> bool:
        return self._cumulative is not None
