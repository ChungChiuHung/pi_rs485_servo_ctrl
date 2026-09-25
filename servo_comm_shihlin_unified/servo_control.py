import time
import logging
import json
import math
import threading
from typing import Union, Callable
from threading import Thread, Event
from serial import SerialException
from modbus_rtu_client import ModbusRTUClient
from modbus_rtu_response import ModbusRTUResponse, ModbusExceptionResponse
from encoder_pulse_tracker import EncoderPulseTracker
from servo_utility import ServoUtility
from servo_control_registers import ServoControlRegistry
from status_bit_map import DI_Function_Code, BitMapOutput
from servo_p_register import Register, PA, PC, PD, PE, PF

PA.init_registers()
PC.init_registers()
PD.init_registers()
PE.init_registers()
PF.init_registers()


# Configure logging
logger = logging.getLogger(__name__)

# Values the "Current alarm" register (0x0100) reports when there is no
# active alarm. docs/en_manual.txt only documents 0 ("(3) Alarm
# information", ~line 10220), but real-hardware testing (2026-09-18) found
# this driver reports 0xFF (255) after a successful AL.12 clear -- verified
# by checking the physical panel, which showed "AL --" (its own "nothing to
# display" state) at that exact moment, not any alarm code. The full alarm
# table (docs/en_manual.txt ~line 10464-10537) tops out at AL.64, so 0xFF
# isn't a real numbered alarm either way. Treat both as "no alarm" rather
# than assuming 0 is the only valid value -- see is_alarm_active().
NO_ALARM_CODES = frozenset({0, 0xFF})


# Software motion-complete detection (see _read_continuously()). A per-poll
# encoder delta at or below this many pulses counts as "not moving". The
# original value (10) was calibrated against a stationary, torque-off
# encoder's ~1-2 pulse noise floor -- but a real positioning-test move,
# once it arrives and HOLDS the target under active closed-loop torque,
# dithers considerably more than that (confirmed live 2026-09-19: a real
# completed +90deg move sat at up to +/-45 pulses/poll indefinitely). With
# the old threshold, that hold-dither was misread as continuous "motion",
# _still_count never reached STILL_COUNT_TO_COMPLETE, and the auto-stop
# never fired -- reading_active (and the drive's test mode/Servo-on state)
# stayed on forever after every real move. Raised well above the observed
# dither, and still far below any real intended move (the smallest
# practical move, ~1 degree, is on the order of 1000 pulses).
STILL_THRESHOLD_PULSES = 200
# Consecutive "not moving" polls required before declaring motion complete.
# At the loop's ~100-150ms per-iteration pace this is roughly 1-1.5 seconds
# of confirmed stillness -- long enough that the brief pause between
# _execute_positioning()'s config writes and the actual 0x0907 start trigger
# can't be mistaken for "already done" the way the old PF.PRCM check was.
STILL_COUNT_TO_COMPLETE = 12

# PA30 (UAP) handshake: after writing PA30=1 the drive latches the encoder
# into PA31~PA33 and sets PA30 back to 0 when the data is ready to read
# (Chinese manual V1.07). Poll up to ABS_UPDATE_MAX_POLLS * ABS_UPDATE_POLL_MS.
ABS_UPDATE_MAX_POLLS = 50
ABS_UPDATE_POLL_MS = 20
# Absolute-position plausibility limits (revolution word is a signed 16-bit
# range; the pulse word is one revolution of the 22-bit encoder).
ABS_REV_MIN, ABS_REV_MAX = -32768, 32767


# Alarms relevant to the absolute-encoder system (manual §8.1, alarm table
# ~line 10498+). Keyed by the code as displayed on the drive (AL.2A -> 0x2A):
# the alarm registers (0x0100~0x010A) are plain hex, not BCD -- Chinese V1.07
# manual p.260: 0x00FF = no alarm, 0x0001 = AL.01, 0x0012 = AL.12 (and the
# alarm list itself contains letters, e.g. AL.0A, which BCD cannot express).
ABSOLUTE_SYSTEM_ALARM_NAMES = {
    0x24: "Encoder type error (PA28=1 with an incremental motor, or absolute position lost)",
    0x29: "Encoder error 5 (motor revolutions outside -32768~32767)",
    0x2A: "ABS encoder abnormal 1 (expected once after first PA28=1: power-cycle the drive again)",
    0x2C: "ABS encoder abnormal 3 (expected after AL.2A: set PA29=1 or home-return to initialise)",
    0x2D: "Encoder battery voltage low (replace the backup battery)",
}


class PositionUnavailableError(ValueError):
    """The drive's current position could not be read (or, in absolute mode,
    cannot be trusted), so a position-relative move was refused rather than
    computed from a stale angle."""


# Command pulses the drive can be given for one positioning move: registers
# 0x0905 (low word) and 0x0906 (high word) hold 0..(2^31-1) (manual,
# docs/en_manual.txt ~10416). More would be truncated into a wrong, shorter
# move. This is a hardware limit; the application itself sets no limit on how
# far one move may go (the old 180 degree guard was removed 2026-09-21 -- it came
# from an earlier application's requirement).
MAX_POSITIONING_PULSES = 2**31 - 1


class MoveOutOfRangeError(ValueError):
    """The requested move cannot be expressed in the drive's command-pulse
    register (0..2^31-1), or the angle is not a finite number. Nothing was
    sent. A ValueError, so callers that already handle refused moves cope."""


def alarm_name(alarm_code):
    """Human-readable name for the alarms this module knows about, else None."""
    return ABSOLUTE_SYSTEM_ALARM_NAMES.get(alarm_code)


def is_alarm_active(alarm_code) -> bool:
    """True if alarm_code represents an active alarm. alarm_code=None
    (communication failure) is deliberately NOT "no alarm" -- callers must
    treat None as unknown/unsafe, same as read_current_alarm_code() itself
    documents."""
    if alarm_code is None:
        return True
    return alarm_code not in NO_ALARM_CODES


class ServoController:
    """Merged servo_comm_shihlin / servo_comm_shihlin_50W controller.

    Communication is Modbus RTU only (ModbusRTUClient/ModbusRTUResponse) --
    this driver's confirmed PC22 setting is RTU, not ASCII; see
    docs/servo_comm_shihlin_merge_design.md's 2026-09-18 hardware
    confirmation note. servo_comm_shihlin/servo_comm_shihlin_50W's
    ModbusASCIIClient cannot talk to this driver at all.

    Position tracking uses EncoderPulseTracker (wraparound-safe unwrapping
    of the raw 0x0000/0x0024 32-bit register) rather than PA32/PA33, since
    PA28 was confirmed 0 (incremental mode) on real hardware -- see
    docs/servo_comm_shihlin_merge_design.md §2.4 "Plan C".
    """

    def __init__(self, serial_port, profile: dict):
        self.serial_port = serial_port
        self.profile = profile
        self.base_pulse_per_degree = profile["base_pulse_per_degree"]
        # Per-profile config file: abs_home_pos values differ by orders of
        # magnitude between motors (see motor_profiles.json), so a captured
        # home position for one motor must never leak into another profile.
        self.config_file = f"servo_config_{profile['name']}.json"

        self.modbus_client = ModbusRTUClient(
            device_number=profile["modbus_device_number"],
            serial_port_manager=serial_port,
        )

        self.read_thread: Union[Thread, None] = None
        self.read_thread_stop_event = threading.Event()
        self.reading_active = False
        # RLock (not Lock): start_continuous_reading() can call
        # stop_continuous_reading() from inside its own `with self.lock`
        # block when reading is already active -- a plain Lock would
        # deadlock there. RLock also lets cancel_continuous_reading() safely
        # re-enter after calling stop_continuous_reading().
        self.lock = threading.RLock()
        self.stop_event = Event()
        self.response = ""
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.target_angle = 0.0
        self.current_encoder = 0
        self.previous_encoder = 0
        self._encoder_tracker = EncoderPulseTracker()
        self.float_error = 0.0
        self.accumulate_pulse = 0
        self.on_initial_home = False
        # Software motion-complete detection state -- see _read_continuously()'s
        # comment for why this replaced a PF.PRCM-based check.
        self._motion_seen = False
        self._still_count = 0
        # None/0 = not currently running in either direction (reversal
        # guard in speed_ctrl_action() allows the next action freely); 1/2 =
        # currently commanded CW/CCW (only a matching repeat or an explicit
        # stop is allowed next -- see speed_ctrl_action()'s comment).
        self._last_motion_direction = None
        # The JOG speed (0x0903) last written by enable_speed_ctrl(), so
        # change_jog_speed_by() has a baseline to nudge from. None = JOG/
        # speed-control mode is not currently armed.
        self.jog_speed_rpm = None
        self._auto_stop_on_stillness = True
        self.abs_home_pos = self.load_abs_home_pos()
        # Set Point 1/2 (degrees) -- recorded, not moved-to: SET POINT 1/2
        # save wherever the motor currently is, they don't command a move.
        # Per-profile, like abs_home_pos (see _load_config_value()'s
        # docstring). None means "never recorded for this profile".
        self.set_point_1 = self._load_config_value("set_point_1")
        self.set_point_2 = self._load_config_value("set_point_2")
        # Encoder mode the drive is ACTUALLY running in (PA28 == 1). False
        # (incremental, PA28 == 0) matches the confirmed state of this
        # project's drive and keeps every existing code path unchanged; it
        # only becomes True via refresh_encoder_mode() reading PA28 == 1
        # from the drive. PA28 takes effect only after a drive power cycle
        # (manual: PA28 is a (*) parameter), so this is deliberately NOT set
        # by write_PA28_Encoder_Mode() itself.
        self.absolute_mode = False
        # Absolute-mode home reference, in absolute encoder pulses
        # (PA32*pulses_per_rev + PA33). Kept separate from abs_home_pos,
        # which is in the incremental tracker's scale -- the two scales are
        # unrelated (the incremental counter restarts at power-on, the
        # absolute one does not), so one must never be used as the other.
        self.abs_home_pos_absolute = self._load_config_value("abs_home_pos_absolute")
        # absolute_pulses - incremental_tracker_cumulative, captured whenever
        # both are read together. Lets the 100ms continuous-reading loop
        # (which only reads the cheap raw counter) report positions in the
        # absolute scale without 4 extra Modbus transactions per poll.
        self._absolute_offset = None
        # Which of PA32/PA33 holds the signed REVOLUTION count (the other one
        # holds pulses within a revolution). The Chinese V1.07 manual says
        # PA33 = revolutions / PA32 = pulses; the English manuals say the
        # opposite -- unverified on this drive, so it is overridable per
        # profile ("abs_rev_register": "APR" | "APP" in motor_profiles.json).
        # A wrong choice is caught by the plausibility checks in
        # read_absolute_position_pulses() rather than yielding a bad position.
        self.abs_rev_register = profile.get("abs_rev_register", "APP")
        if self.abs_rev_register not in ("APR", "APP"):
            raise ValueError(f"abs_rev_register must be 'APR' or 'APP', got {self.abs_rev_register!r}")
        # Last known PA23 (EEPROM write-inhibit) value; None = never read.
        self.eeprom_protection = None
        self._eeprom_checked_at = None
        # Set once a function-0x10 parameter write succeeds after 0x06 was
        # rejected -- see _write_parameter().
        self._prefer_multi_word_write = False
        # Electronic gear ratio (PA06/PA07): (cmx, cdv), None until checked;
        # electronic_gear_unity is True/False/None (unknown).
        self.electronic_gear = None
        self.electronic_gear_unity = None
        # True once SET HOME has succeeded since this process started. The
        # incremental encoder counter restarts at every drive power-on, so a
        # saved home from an earlier run can't be trusted until the operator
        # sets it again (the web UI prompts for this on every start).
        self.home_set_since_start = False
        self._event_listeners = {
            "on_motion_completed": [],
            "on_moving": [],
            "on_cancel": [],
        }

    def _load_config_dict(self) -> dict:
        try:
            with open(self.config_file, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            logging.warning(f"Config file {self.config_file} not found; using defaults.")
            return {}
        except json.JSONDecodeError as e:
            logging.error(f"Error parsing configuration file: {e}.")
            return {}

    def _load_config_value(self, key: str, default=None):
        return self._load_config_dict().get(key, default)

    def _save_config_value(self, key: str, value) -> None:
        """Read-modify-write the whole config dict, not just {key: value} --
        this file holds several independent values (abs_home_pos,
        set_point_1, set_point_2), and a previous version of this method
        overwrote the entire file with only its own key, silently erasing
        whatever else had been saved there."""
        config = self._load_config_dict()
        config[key] = value
        try:
            with open(self.config_file, 'w') as file:
                json.dump(config, file)
            logging.info(f"Saved {key}: {value} to {self.config_file}")
        except Exception as e:
            logging.error(f"Error saving {key}: {e}")

    def load_abs_home_pos(self) -> int:
        return self._load_config_value("abs_home_pos", self.profile["abs_home_pos_default"])

    def save_abs_home_pos(self, abs_home_pos: int):
        """Persists the incremental-mode home AND makes it the home in use now.
        It used to write the file only, so the polling loop kept computing angles
        from the old home until the app was restarted (real drive 2026-09-21:
        after SET HOME the display still said 102.49 deg, and HOME / MOVE TO SET
        POINT would have driven the motor back to the old zero)."""
        self._save_config_value("abs_home_pos", abs_home_pos)
        self.abs_home_pos = abs_home_pos

    def record_set_point(self, n: int) -> float:
        """Persists the CURRENTLY TRACKED angle as Set Point 1 or 2 --
        does not move the motor. Returns the recorded angle."""
        if n not in (1, 2):
            raise ValueError(f"Set Point must be 1 or 2, got {n!r}.")
        with self.lock:
            angle = self.current_angle
        self._save_config_value(f"set_point_{n}", angle)
        setattr(self, f"set_point_{n}", angle)
        logging.info(f"Set Point {n} recorded: {angle} deg")
        return angle

    def move_to_set_point(self, n: int, acc_dec_time: int = 5000, speed_rpm: int = 10) -> None:
        """Commands a move to the previously recorded Set Point 1 or 2 (see
        record_set_point()) via the same closed-loop post_step_motion_by()
        path as HOME. Raises ValueError if n isn't 1/2, or if that set
        point has never been recorded -- callers must not fall back to a
        default target (e.g. 0) in that case."""
        if n not in (1, 2):
            raise ValueError(f"Set Point must be 1 or 2, got {n!r}.")
        target_angle = getattr(self, f"set_point_{n}")
        if target_angle is None:
            raise ValueError(f"Set Point {n} has not been recorded yet.")
        # post_step_motion_by() reads the real position itself (and raises
        # PositionUnavailableError, a ValueError, if it can't).
        logging.info(f"Moving to Set Point {n}: {target_angle} deg")
        self.post_step_motion_by(target_angle, acc_dec_time, speed_rpm)

    def register_event_listener(self, event_name: str, callback: Callable):
        """Register a callback for a specific event."""
        if event_name not in self._event_listeners:
            raise ValueError(f"Event {event_name} is not supported.")
        if not callable(callback):
            raise ValueError("Callback must be callable.")
        self._event_listeners[event_name].append(callback)

    def unregister_event_listener(self, event_name: str, callback: callable):
        """Unregister a callback for a specific event."""
        if event_name not in self._event_listeners:
            raise ValueError(f"Event {event_name} is not supported.")
        try:
            self._event_listeners[event_name].remove(callback)
        except ValueError:
            logging.info(f"Callback not found for event {event_name}.")

    def _notify_event_listeners(self, event_name, *args, **kwargs):
        """Notify all registered callbacks for a specific event."""
        for callback in self._event_listeners.get(event_name, []):
            try:
                callback(*args, **kwargs)
            except Exception as e:
                logger.error(f"Error in event listener '{event_name}': {e}")

    def delay_ms(self, milliseconds: int) -> None:
        time.sleep(milliseconds / 1000.0)

    def print_byte_array_as_spaced_hex(self, byte_array, data_name) -> None:
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        logger.info(f"{data_name}: {hex_string}")

    # default address = 0x0205
    def start_continuous_reading(self, interval: float = 0.1, auto_stop_on_stillness: bool = True) -> None:
        with self.lock:
            if self.reading_active:
                # Already running -- e.g. enable_speed_ctrl() left it active
                # (auto_stop_on_stillness=False keeps it running through a
                # pause), and enablePosMode/pos_step_motion_test() then also
                # call this expecting reading to be active afterward.
                # Previously this stopped the existing session and RETURNED
                # instead of ensuring one was running -- pos_step_motion_test()
                # would go on to trigger 0x0907 immediately after, with the
                # keep-alive poll thread just killed, so the drive's own
                # 1-second communication timeout could silently exit test
                # mode (and Servo-off) right as the move was supposed to
                # start. Refresh this call's session state in place instead
                # of tearing down and restarting the thread -- reading stays
                # active throughout, no gap for the drive to time out in.
                self._motion_seen = False
                self._still_count = 0
                self._auto_stop_on_stillness = auto_stop_on_stillness
                logging.info("Continuous reading already active; refreshed session state.")
                return

            self.read_thread_stop_event.clear()
            # Fresh motion-complete detection state per reading session --
            # see _read_continuously()'s comment.
            self._motion_seen = False
            self._still_count = 0
            # False for continuous JOG/speed-control mode (enable_speed_ctrl()):
            # that mode runs until explicitly stopped/cancelled, so a
            # deliberate pause (encoder goes still) must not be mistaken for
            # "the move finished" and tear the session down. True (default)
            # for discrete positioning-test moves, where stillness really
            # does mean the move completed.
            self._auto_stop_on_stillness = auto_stop_on_stillness
            self.read_thread = threading.Thread(target=self._read_continuously, args=(interval,))
            self.reading_active = True
            self.read_thread.start()
            logging.info("Continuous reading started.")

    def stop_continuous_reading(self) -> None:
        with self.lock:
            if not self.reading_active:
                logging.warning("Continuous reading is not active; skipping stop.")
                return

            self.reading_active = False
            self.read_thread_stop_event.set()
            thread_to_join = self.read_thread
            self.read_thread = None
            self._motion_seen = False
            self._still_count = 0
            self._last_motion_direction = None
            if self.on_initial_home:
                self.on_initial_home = False

        # Join OUTSIDE self.lock. _read_continuously() takes self.lock
        # itself for its wire I/O -- if a caller on a different thread held
        # the lock here while blocking on join(), and the background
        # thread happened to be waiting to acquire that same lock to reach
        # its next read_thread_stop_event check, neither side could ever
        # make progress (found via a stress test that reliably hit this
        # ordering -- not just a theoretical race).
        if thread_to_join and threading.current_thread() is not thread_to_join:
            # Bounded, defense-in-depth: this join() itself had no timeout,
            # so if the background thread ever got stuck for some other
            # reason (e.g. blocked inside a Modbus transaction), whichever
            # caller is stopping it -- possibly a Flask request holding
            # hardware_lock.py's _hardware_busy_lock -- would hang forever
            # with no way to recover short of restarting the process. A
            # stuck thread is leaked rather than joined in that case;
            # logged so it's visible. Ported from servo_comm_shihlin's
            # identical fix (2026-09-22), applied there after a receive()
            # loop with no absolute deadline caused exactly this.
            thread_to_join.join(timeout=5.0)
            if thread_to_join.is_alive():
                logging.error(
                    "Background reading thread did not stop within 5s; "
                    "continuing without it (it will be abandoned)."
                )

        logging.info("Motion Completed Signal Reading Stopped.")
        self._notify_event_listeners("on_motion_completed")
        self.stop_event.set()

    def cancel_continuous_reading(self) -> None:
        """Like stop_continuous_reading(), but also takes one more encoder
        reading afterward and fires on_cancel with the resulting angle.
        Used by the OSC/Art-Net "cancel loop" control (design doc §2.2 #1).
        Calls stop_continuous_reading() instead of duplicating its body
        (servo_comm_shihlin_50W's version was a copy-paste of it).

        Also explicitly exits whatever test mode is active (CTRL_MODE_SEL ->
        0x0000), matching the web UI's MOTION CANCEL (stop_continuous_reading()
        + Enable_Position_Mode(False)) -- without this, OSC/Art-Net's cancel
        only stopped the keep-alive poll thread and relied on the drive's own
        ~1s communication-timeout to fall out of test mode on its own,
        instead of exiting immediately and deterministically. Confirmed live
        via OSC 2026-09-19: /cancel_loop left CTRL_MODE_SEL in its prior
        state (e.g. still 3, JOG test) until the timeout caught up a moment
        later. Enable_Position_Mode(False) is the documented generic "quit
        test mode" write regardless of which mode (JOG or Positioning) was
        active -- see docs/en_manual.txt's Step 6/Step 7 for each.
        """
        self.stop_continuous_reading()
        self.Enable_Position_Mode(False)
        self.delay_ms(50)
        if self._refresh_current_angle_from_hardware():
            self._notify_event_listeners("on_cancel", self.current_angle)

    def _refresh_current_angle_from_hardware(self) -> bool:
        """One-shot fresh encoder read that brings self.current_angle/
        current_encoder up to date immediately, instead of waiting for the
        continuous-reading background thread (the only other writer of
        these fields). Needed before computing a RELATIVE move (e.g.
        move_to_set_point()) right after a process restart: current_angle
        still holds its __init__ default (0.0) until that thread has run
        at least once, and a relative move computed against a stale value
        lands at the wrong absolute position -- confirmed live 2026-09-19
        (a move-to-14.43deg landed at 28.86deg, exactly double, because
        current_angle was still 0.0 post-restart). Returns False (and
        leaves current_angle untouched) on a communication failure."""
        with self.lock:
            encoder = self._read_reference_encoder()
            if encoder is None:
                logging.warning("_refresh_current_angle_from_hardware: no usable position reading.")
                return False
            home = self._active_home_pos()
            if home is None:
                logging.warning(
                    "Absolute mode is active but no absolute home position has been set "
                    "(abs_home_pos_absolute); press SET HOME first. Position not updated."
                )
                return False
            self.current_encoder = encoder
            logging.info(f"Current Encoder Value: {self.current_encoder}")
            self.current_angle = round((encoder - home) / self.base_pulse_per_degree, 4)
            logging.info(f"Current Angle: {self.current_angle}")
            return True

    def _active_home_pos(self):
        """Home reference in the scale of the ACTIVE encoder mode. None in
        absolute mode until set_home_position() has captured one -- the
        incremental abs_home_pos is a different scale and must not stand in."""
        return self.abs_home_pos_absolute if self.absolute_mode else self.abs_home_pos

    def _read_reference_encoder(self):
        """Fresh position in the active reference scale, or None on any
        failure. Incremental mode: the wraparound-tracked raw counter (the
        original behavior). Absolute mode: PA32/PA33 absolute pulses, and the
        incremental-tracker offset is re-synced from this same moment so the
        cheap continuous-reading loop can keep reporting in the absolute
        scale."""
        raw = self.read_motor_feedback_pulses()
        if raw is None:
            return None
        with self.lock:
            tracker_value = self._encoder_tracker.update(raw)
            if not self.absolute_mode:
                return tracker_value
        absolute_pulses = self.read_absolute_position_pulses()
        if absolute_pulses is None:
            return None
        with self.lock:
            self._absolute_offset = absolute_pulses - tracker_value
        return absolute_pulses

    def _encoder_and_angle_for(self, tracker_value: int):
        """(encoder, angle) for a tracker value read by the continuous loop.
        Identical to the original computation in incremental mode."""
        if (self.absolute_mode and self._absolute_offset is not None
                and self.abs_home_pos_absolute is not None):
            encoder = tracker_value + self._absolute_offset
            home = self.abs_home_pos_absolute
        else:
            encoder = tracker_value
            home = self.abs_home_pos
        return encoder, round((encoder - home) / self.base_pulse_per_degree, 4)

    def _read_continuously(self, interval: float) -> None:
        # Software motion-complete detection instead of Read_Motion_Completed_Signal()
        # (PF.PRCM): real-hardware testing (2026-09-18) found PF.PRCM is a
        # PATH-execution status register, unrelated to the raw-pulse
        # positioning workflow this loop actually monitors
        # (pos_step_motion_test()/_execute_positioning(), driven by
        # 0x0905/0x0906/0x0907 -- not PF82 PATH execution). It read as
        # "already complete" from the very first poll regardless of whether
        # the motor had moved at all, so the old 6-consecutive-completed
        # check triggered a false auto-stop within ~1.5s of every
        # start_continuous_reading() call, cutting off current_angle
        # feedback while a real move might still be in progress.
        #
        # Instead: track the raw per-poll encoder delta. Once the encoder
        # has genuinely moved (delta above STILL_THRESHOLD_PULSES, safely
        # above the ~1-2 pulse jitter seen on a stationary encoder) at least
        # once, require STILL_COUNT_TO_COMPLETE consecutive polls back below
        # that threshold before declaring the motion complete and
        # auto-stopping. If the encoder never moves at all (e.g. "ENABLE
        # POS MODE" alone, which arms position mode without commanding a
        # move via 0x0907), reading is deliberately left running rather
        # than auto-stopped -- stop it explicitly (MOTION CANCEL /
        # stop_continuous_reading()) instead.
        #
        # This loop's poll cadence (`interval`, plus the fixed 100ms retry
        # delay on a failed read) also does double duty as the drive's
        # required test-mode keep-alive: 0x0907 puts the drive into
        # "positioning test operation" mode, which the manual documents as
        # auto Servo-Off + force-exiting test mode after any communication
        # gap over 1 second -- see pos_step_motion_test()'s comment. Keep
        # this loop's timing well under that 1s ceiling.
        previous_encoder_for_stillness = None

        while not self.read_thread_stop_event.is_set():
            if not self.serial_port.keep_running:
                logger.info("Reconnection attempts stopped.")
                break

            # Read encoder position. Holds self.lock across the wire I/O
            # (not just the state writes below) so a concurrent call to a
            # method that also talks to the modbus client from another
            # thread (e.g. a web UI status-polling endpoint calling
            # read_current_alarm_code()) can't interleave its
            # request/response with this loop's on the same serial line.
            # RTU has no built-in transaction ID to tell interleaved
            # responses apart.
            try:
                with self.lock:
                    encoder = self.read_motor_feedback_pulses()
            except Exception as e:
                logger.warning(f"Failed to read encoder ({e}); retrying...")
                self.delay_ms(interval * 1000)
                continue

            if encoder is None:
                logger.warning("Empty encoder response; retrying...")
                self.delay_ms(interval * 1000)
                continue

            # Process valid encoder reading (unwrapped -- see
            # encoder_pulse_tracker.py / docs/servo_comm_shihlin_merge_design.md
            # §2.4 "Plan C" for why the raw 0x0000 register can't be trusted
            # directly). current_angle/current_encoder are only ever updated
            # here, from real feedback -- never overwritten by the command
            # path (closed-loop basis for post_step_motion_by(), design doc
            # §2.2 #4).
            with self.lock:
                tracker_value = self._encoder_tracker.update(encoder)
                self.current_encoder, diff_angle = self._encoder_and_angle_for(tracker_value)
                logger.info(f"Current Encoder Value: {self.current_encoder} (raw: {encoder})")
                self.current_angle = diff_angle
            logger.info(f"Diff Angle: {diff_angle}")
            self._notify_event_listeners("on_moving", diff_angle)

            if previous_encoder_for_stillness is not None:
                delta = abs(self.current_encoder - previous_encoder_for_stillness)
                if delta > STILL_THRESHOLD_PULSES:
                    self._motion_seen = True
                    self._still_count = 0
                else:
                    self._still_count += 1
            previous_encoder_for_stillness = self.current_encoder

            if self._auto_stop_on_stillness and self._motion_seen and self._still_count >= STILL_COUNT_TO_COMPLETE:
                logger.info(
                    f"Motion complete: encoder stable for {self._still_count} consecutive reads."
                )
                # Diagnostic cross-check only -- does the driver's own
                # official MC_OK signal agree with our software-only
                # detection? Doesn't affect the stop decision either way;
                # None (comm failure, or INP/CMDOK not currently assigned
                # to any DO) just gets logged as "unknown", not treated as
                # a disagreement.
                try:
                    mc_ok = self.read_mc_ok_status()
                    logger.info(f"MC_OK cross-check at auto-stop: {mc_ok}")
                except Exception as e:
                    logger.warning(f"MC_OK cross-check failed (non-fatal): {e}")
                self.stop_continuous_reading()
                break

            # loop delay
            self.delay_ms(interval * 1000)

    def read_PA01_Ctrl_Mode(self):
        logging.info(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        message = self.modbus_client.build_read_message(PA.STY.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def _read_parameter(self, register, word_length: int = 2, signed: bool = False):
        """Reads one PA/PD register and returns its int value, or None on a
        communication/parse failure (never a guess -- callers treat None as
        "unknown")."""
        message = self.modbus_client.build_read_message(register.address, word_length)
        try:
            response = self.modbus_client.send_and_receive(message)
            if response is None:
                logger.error(f"No response reading {register.name}.")
                return None
            return ModbusRTUResponse(response).get_value(signed=signed)
        except Exception as e:
            logger.error(f"Failed to read {register.name}: {e}")
            return None

    def _write_parameter(self, register, value: int) -> bool:
        """Writes one PA/PD register value. Every parameter group is 2 words
        (32-bit) in the manual, but function 0x06 (single word) is what has
        been verified on this drive -- PD16/PD25, also 2-word parameters, are
        written that way on every servo on/off. For our values (all fit in
        16 bits) 0x06 sets the whole value, so it goes first. If the drive
        answers with a Modbus EXCEPTION (it rejects 0x06 for this register),
        the write is retried once with function 0x10 as [low word, high word]
        and, if that works, 0x10 is used for all later writes. A missing
        answer is a dead line, not a rejected frame, and is not retried.
        Callers verify important writes by reading back."""
        low, high = value & 0xFFFF, (value >> 16) & 0xFFFF
        attempts = [
            ("0x06", lambda: self.modbus_client.build_write_message(register.address, value)),
            ("0x10", lambda: self.modbus_client.build_write_multiple_message(
                register.address, [low, high])),
        ]
        if high:
            attempts = attempts[1:]  # does not fit one word: 0x06 cannot carry it
        elif self._prefer_multi_word_write:
            attempts.reverse()
        for label, build in attempts:
            try:
                response = self.modbus_client.send_and_receive(build())
                if response is None:
                    logger.error(f"No response writing {register.name}.")
                    return False
                ModbusRTUResponse(response)  # validates CRC / exception frames
            except ModbusExceptionResponse as e:
                logger.warning(f"Drive rejected function {label} write of {register.name}: {e}")
                continue
            except Exception as e:
                logger.error(f"Failed to write {register.name}: {e}")
                return False
            if label == "0x10" and not self._prefer_multi_word_write:
                self._prefer_multi_word_write = True
                logger.warning("Function 0x06 was rejected but 0x10 worked: using 0x10 for parameter writes.")
            return True
        logger.error(f"Failed to write {register.name}: rejected by the drive with both 0x06 and 0x10.")
        return False

    def read_PA23_Memory_Write_Inhibit(self):
        return self._read_parameter(PA.MCS)

    def _set_pa23(self, value: int) -> bool:
        """Writes PA23 and verifies it by reading it back."""
        if not self._write_parameter(PA.MCS, value):
            return False
        return self.read_PA23_Memory_Write_Inhibit() == value

    def ensure_eeprom_write_protection(self, max_age_s: float = 0.0):
        """Makes sure PA23 (MCS) inhibits EEPROM writes, and returns its
        value (0/1/2), or None if it could not be read.

        Why: this app writes PD16/PD25 (DI control source / virtual DI state)
        on nearly every button, OSC and Art-Net action, and the parameters
        live in an EEPROM rated for ~100,000 writes (AL.0F when exhausted).
        The manual says communication-driven parameter writes should run with
        PA23 = 2 (persists across power-off; firmware >= 106) or, on older
        firmware, PA23 = 1 (reverts to 0 at every power-off, so it must be
        re-applied after each boot -- which is why this is called at connect
        time and then periodically, not just once).

        Only reads when PA23 is already 1/2 (no EEPROM wear). When it is 0
        it tries 2 first, then 1 (older firmware rejects or ignores 2), and
        verifies each attempt by reading back. max_age_s > 0 skips the check
        if one was done that recently. Never raises; a failure is logged and
        reported as the (unprotected) value read."""
        now = time.monotonic()
        if max_age_s and self._eeprom_checked_at is not None \
                and now - self._eeprom_checked_at < max_age_s:
            return self.eeprom_protection
        current = self.read_PA23_Memory_Write_Inhibit()
        self._eeprom_checked_at = now
        if current is None:
            logger.warning("Could not read PA23; EEPROM write protection status unknown.")
            return None
        if current in (1, 2):
            self.eeprom_protection = current
            return current
        for candidate in (2, 1):
            if self._set_pa23(candidate):
                self.eeprom_protection = candidate
                logger.info(
                    f"EEPROM write protection enabled (PA23={candidate})"
                    + ("" if candidate == 2 else
                       " -- older firmware: reverts at power-off, re-applied automatically"))
                return candidate
        self.eeprom_protection = 0
        logger.error(
            "Could not enable EEPROM write protection (PA23 stays 0): every PD16/PD25 "
            "write is wearing the EEPROM (~100,000 write life). Set PA23 from the panel."
        )
        return 0

    def read_PA28_Encoder_Mode(self):
        """PA28 (ABS): 0 = incremental, 1 = absolute. None if unreadable."""
        return self._read_parameter(PA.ABS)

    def write_PA28_Encoder_Mode(self, absolute: bool) -> bool:
        """Writes PA28 and verifies it by reading it back. HARDWARE-AFFECTING
        -- read before calling:
          * Only takes effect after the drive is power-cycled (PA28 is a (*)
            parameter, manual p.88); this method does NOT change
            self.absolute_mode for that reason.
          * absolute=True on a drive whose motor has no absolute encoder
            raises AL.24 (encoder type error). Even with the right motor the
            absolute system needs its backup battery (SDH-BAT-SET), or the
            position is lost at every power-off.
          * First-time absolute init is a multi-step sequence: write 1,
            power-cycle (AL.2A appears), power-cycle again (AL.2C appears),
            then clear it with PA29=1 (write_PA29_Initial_Abs_Pos()) or a
            home-return -- manual §8.1 "System initialization".
        Returns True only if the write succeeded AND read back as requested."""
        requested = 1 if absolute else 0
        # With PA23 = 1/2 NO parameter reaches the EEPROM, so PA28 would be
        # lost at the very power cycle that is supposed to apply it. Lift the
        # protection for this one write and restore it afterwards.
        protection = self.read_PA23_Memory_Write_Inhibit()
        if protection is None:
            logger.error("PA28 not written: PA23 unreadable, cannot guarantee the value would persist.")
            return False
        try:
            if protection != 0 and not self._set_pa23(0):
                logger.error("PA28 not written: could not lift PA23 EEPROM protection (it would not persist).")
                return False
            if not self._write_parameter(PA.ABS, requested):
                return False
            readback = self.read_PA28_Encoder_Mode()
        finally:
            self.ensure_eeprom_write_protection()
        if readback != requested:
            logger.error(f"PA28 write not confirmed: wrote {requested}, read back {readback}.")
            return False
        logger.warning(
            f"PA28 set to {requested} ({'absolute' if absolute else 'incremental'}). "
            "Power-cycle the drive for it to take effect."
        )
        return True

    def refresh_encoder_mode(self):
        """Reads PA28 from the drive and updates self.absolute_mode. Returns
        the mode as True/False, or None if PA28 couldn't be read (in which
        case self.absolute_mode is left as it was). Call at connect time and
        after a power cycle."""
        value = self.read_PA28_Encoder_Mode()
        if value is None:
            return None
        with self.lock:
            self.absolute_mode = (value == 1)
            if not self.absolute_mode:
                self._absolute_offset = None
        logger.info(f"Encoder mode: {'absolute' if self.absolute_mode else 'incremental'} (PA28={value}).")
        return self.absolute_mode

    def read_PA31_Abs_Position_Status(self):
        return self._read_parameter(PA.APST)

    def write_PA30_Update_Abs_Position(self, mode: int = 1) -> bool:
        """PA30 (UAP): 1 = refresh PA31~PA33 from the encoder; 2 = also reset
        the position command to the current position (clears position error
        -- more invasive, not used by the position-read path)."""
        if mode not in (1, 2):
            raise ValueError(f"PA30 mode must be 1 or 2, got {mode!r}.")
        return self._write_parameter(PA.UAP, mode)

    def read_PA32_Abs_Revolutions(self):
        return self._read_parameter(PA.APR, 2, signed=True)

    def read_PA33_Encoder_ABS_Pos(self):
        return self._read_parameter(PA.APP, 2)

    def _update_absolute_registers(self) -> bool:
        """PA30 = 1, then poll until the drive sets PA30 back to 0 -- only then
        are PA31~PA33 valid. False on a failed write or a handshake timeout."""
        if not self.write_PA30_Update_Abs_Position(1):
            return False
        for _ in range(ABS_UPDATE_MAX_POLLS):
            if self._read_parameter(PA.UAP) == 0:
                return True
            self.delay_ms(ABS_UPDATE_POLL_MS)
        logger.warning("PA30 did not return to 0: absolute position registers not updated.")
        return False

    def read_absolute_position_pulses(self):
        """Absolute encoder position in pulses (revolutions * pulses_per_rev +
        pulses), or None if it can't be trusted: PA30 handshake fails, PA31
        reports lost position / low battery / overflow / coordinate system
        not set, a read fails, or a value is outside its plausible range
        (which is how a wrong abs_rev_register layout shows up). Only
        meaningful when PA28 == 1.

        The PA30 write is a parameter write, so PA23 protection is verified
        first (throttled) -- otherwise every position read would wear the
        EEPROM."""
        self.ensure_eeprom_write_protection(max_age_s=30)
        if not self._update_absolute_registers():
            return None
        status = self.read_PA31_Abs_Position_Status()
        if status is None:
            return None
        faults = PA.decode_APST(status)
        if faults:
            logger.warning(f"Absolute position not trustworthy (PA31={status:#06x}): {'; '.join(faults)}")
            return None
        rev_register, pulse_register = (
            (PA.APP, PA.APR) if self.abs_rev_register == "APP" else (PA.APR, PA.APP))
        revolutions = self._read_parameter(rev_register, 2, signed=True)
        pulses = self._read_parameter(pulse_register, 2)
        if revolutions is None or pulses is None:
            return None
        pulses_per_rev = self.profile["encoder_pulses_per_rev"]
        if not (ABS_REV_MIN <= revolutions <= ABS_REV_MAX) or not (0 <= pulses < pulses_per_rev):
            logger.error(
                f"Implausible absolute position ({rev_register.name}={revolutions} rev, "
                f"{pulse_register.name}={pulses} pulses): PA32/PA33 layout probably wrong "
                f"(abs_rev_register={self.abs_rev_register!r}); position rejected."
            )
            return None
        return revolutions * pulses_per_rev + pulses

    def write_PA01_Ctrl_Mode(self):
        logger.info(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 1, 0)
        message = self.modbus_client.build_write_message(
            PA.STY.address, config_value)
        try:
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusRTUResponse(response)
            logger.info(response_object)
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def write_PA29_Initial_Abs_Pos(self):
        logger.info("Address of PA29: Initial Absolute Position")
        if self._write_parameter(PA.CAP, 1):
            logger.info("Initial Absolute Position Set!")

    def write_PD_16_Enable_DI_Control(self):
        logger.info(f"Address of PD{PD.SDI.no} {PD.SDI.name}: {hex(PD.SDI.address)}")
        config_value = ServoUtility.config_hex_with(0, 0xF, 0xF, 0xF)
        message = self.modbus_client.build_write_message(
            PD.SDI.address, config_value)
        try:
            response = self.modbus_client.send_and_receive(message)
            logger.info(f"Response Message: {response}")
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def read_PD_16(self):
        logger.info(f"Address of PD{PD.SDI.no} {PD.SDI.name}: {hex(PD.SDI.address)}")
        message = self.modbus_client.build_read_message(PD.SDI.address, 1)
        try:
            response = self.modbus_client.send_and_receive(message)
            logger.info(f"Response Message: {response}")
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def read_0x0206_To_0x020B(self):
        logger.info(f"Address of 0x0206: Read Value")
        message = self.modbus_client.build_read_message(0x0206, 6)
        try:
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusRTUResponse(response)
            logger.info(response_object)

            # ModbusRTUResponse exposes raw data_bytes (no hex-string .data
            # list like ModbusResponse/ASCII does) -- decode word-by-word.
            cnt = 1
            data_bytes = response_object.data_bytes
            for i in range(0, len(data_bytes), 2):
                word_value = int.from_bytes(data_bytes[i:i + 2], byteorder='big')
                logger.info(f"Original data value: {word_value}")
                for code in DI_Function_Code:
                    if code.value == word_value:
                        logger.info(f"DI{cnt} :{code.name}")
                        cnt += 1
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def write_PD_25(self):
        logger.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logger.info(response_object)

    def read_PD_25(self):
        logger.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        message = self.modbus_client.build_read_message(PD.ITST.address, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logger.info(response_object)

    def clear_alarm(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        self.response = self.modbus_client.send_and_receive(message)
        logging.info(f"Clear Alarm!:{self.response}")

    def servo_on(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        logging.info(f"Servo On:{response}")

    def clear_alarm_12(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        logging.info(f"Clear Alarm 12:{response}")

    def read_current_alarm_code(self):
        """Read the 'Current alarm' monitor register (0x0100, 1 word,
        read-only). Returns the raw int code -- use is_alarm_active() to
        interpret it (this driver reports 0xFF, not just 0, for "no alarm";
        see NO_ALARM_CODES's comment), or None on a communication/parse
        failure (never assume None means "no alarm" -- is_alarm_active(None)
        is deliberately True).

        Locked (self.lock) so a concurrent status poll from the web UI can't
        interleave its request/response with the continuous-reading loop's
        own traffic on the same serial line -- see the comment in
        _read_continuously().
        """
        with self.lock:
            message = self.modbus_client.build_read_message(0x0100, 1)
            response = self.modbus_client.send_and_receive(message)
        if response is None:
            logger.error("No response reading current alarm code (0x0100).")
            return None
        try:
            return ModbusRTUResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to parse current-alarm response: {e}")
            return None

    # CN1 pin of each digital output (manual, DO status register 0x0205:
    # bit0~bit5 = DO1~DO6 = CN1_41~CN1_46).
    DO_CN1_PINS = {1: "CN1-41", 2: "CN1-42", 3: "CN1-43", 4: "CN1-44", 5: "CN1-45", 6: "CN1-46"}

    def read_do_status(self):
        """Live state of DO1~DO6, read-only (function 0x03 only).

        Reads the function assigned to each pin (0x020C = DO1~DO3, 0x020D =
        DO4~DO6, five bits per pin: bit4~0, bit9~5, bit14~10; see manual
        ~line 10177-10192) and then the ON/OFF status register 0x0205
        (bit0~bit5 = DO1~DO6 = CN1_41~CN1_46, ~line 10105-10120). Returns

            {"DO1": {"pin": "CN1-41", "on": bool,
                     "function_code": int, "function": str | None}, ... "DO6": {...}}

        where "function" is the BitMapOutput name (e.g. "INP_SA", "RD",
        "ALM"), "unassigned" for code 0, or None for a code this project does
        not name. Returns None -- never a dict of guessed OFF values -- if any of
        the three reads gets no usable reply, so callers cannot mistake a
        communication failure for "all outputs off".

        "on" is the raw bit of 0x0205. The contact polarity setting PD27 (DOD)
        is not consulted, so on a pin whose contact is defined inverted "on"
        may not mean the pin is conducting (docs §9.1 T4).

        Locked (self.lock, re-entrant) so a status poll cannot interleave with
        the continuous-reading loop's traffic. Each call is three serial
        transactions -- do not poll it at a high rate on the Pi 3 B.
        """
        with self.lock:
            values = []
            for address in (0x020C, 0x020D, ServoControlRegistry.DO_STATUS.address):
                message = self.modbus_client.build_read_message(address, 1)
                response = self.modbus_client.send_and_receive(message)
                if response is None:
                    logger.error(f"No response reading DO registers ({hex(address)}).")
                    return None
                try:
                    value = ModbusRTUResponse(response).get_value()
                except Exception as e:
                    logger.error(f"Failed to parse the reply for {hex(address)}: {e}")
                    return None
                if value is None:
                    logger.error(f"Reply for {hex(address)} carried no value.")
                    return None
                values.append(int(value))
        do1_2_3, do4_5_6, do_status = values

        function_names = {member.value: member.name for member in BitMapOutput}
        result = {}
        for pin in range(1, 7):
            packed = do1_2_3 if pin <= 3 else do4_5_6
            code = (packed >> (5 * ((pin - 1) % 3))) & 0x1F
            result[f"DO{pin}"] = {
                "pin": self.DO_CN1_PINS[pin],
                "on": bool(do_status & (1 << (pin - 1))),
                "function_code": code,
                "function": "unassigned" if code == 0 else function_names.get(code),
            }
        return result

    def read_mc_ok_status(self):
        """Cross-check for the software-only motion-complete detection in
        _read_continuously(): reconstructs the official MC_OK signal
        (CMDOK AND INP -- docs/en_manual.txt ~line 1598, 8447-8450) from
        read_do_status(): finds the DO pins that currently have those two
        functions assigned and checks their ON state. Confirmed 2026-09-18 that
        this unit's DO1-DO6 are at Pt-mode factory defaults (DO1=INP,
        DO3=CMDOK among others -- manual ~line 1790-1809), but the assignment is
        read dynamically rather than hardcoded, so it keeps working if that is
        ever reconfigured.

        Returns True/False, or None if either function currently isn't
        assigned to any DO pin, or on a communication/parse failure (never
        assume None means "not complete" -- this is a cross-check only, the
        real auto-stop decision in _read_continuously() does not depend on it).
        """
        do = self.read_do_status()
        if do is None:
            return None

        def pin_with(function_code):
            return next((name for name, info in do.items()
                         if info["function_code"] == function_code), None)

        inp_pin = pin_with(BitMapOutput.INP_SA.value)
        cmdok_pin = pin_with(BitMapOutput.CMDOK.value)
        if inp_pin is None or cmdok_pin is None:
            logger.warning(
                f"Cannot compute MC_OK: INP assigned to {inp_pin}, CMDOK assigned to "
                f"{cmdok_pin} (need both assigned to some DO pin)."
            )
            return None
        return do[inp_pin]["on"] and do[cmdok_pin]["on"]

    def clear_alarm_via_register(self):
        """Official 'Alarm clearance' register (0x0130): writing 0x1EA5
        clears the current alarm directly. Unlike clear_alarm_12(), this
        does NOT touch the DI control source (PD16) or the virtual EMG
        DI bit (PD25/ITST) -- see docs/en_manual.txt, "(4) Alarm
        clearance" (~line 10230).
        """
        message = self.modbus_client.build_write_message(0x0130, 0x1EA5)
        response = self.modbus_client.send_and_receive(message)
        logging.info(f"Clear alarm via 0x0130 register: {response}")
        return response

    def servo_off(self):
        logger.info("Servo Off, Alarm 12 ON!")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        logger.info(f"Servo Off:{response}")
        self.delay_ms(100)

    def read_PD_01(self):
        logger.info(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        message = self.modbus_client.build_read_message(PD.DIA1.address, 2)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(self.response)
        logging.info(response_object)

    def write_PD_01(self):
        logging.info(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(
            PD.DIA1.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def write_PD_02(self):
        logging.info(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        config_value = 1
        message = self.modbus_client.build_write_message(
            PD.DI1.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def read_PD_02(self):
        logging.info(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        message = self.modbus_client.build_read_message(PD.DI1.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def read_PD_08(self):
        logging.info(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_read_message(PD.DI7.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def write_PD_08(self):
        logging.info(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_write_message(PD.DI7.address, 0x02F)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def read_servo_state(self):
        """Read the 'Servo ready status' bit (0x0200, bit0; docs/en_manual.txt
        ~line 10201): 0 = Servo OFF, 1 = Servo ON. Returns bool, or None on
        a communication/parse failure (never assume None means "off").
        """
        with self.lock:
            message = self.modbus_client.build_read_message(0x0200, 1)
            response = self.modbus_client.send_and_receive(message)
        if response is None:
            logger.error("No response reading servo state (0x0200).")
            return None
        try:
            value = ModbusRTUResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to parse servo-state response: {e}")
            return None
        return bool(value & 0x01)

    def read_control_mode(self):
        message = self.modbus_client.build_read_message(0x0201, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def read_alarm_msg(self):
        message = self.modbus_client.build_read_message(0x0100, 11)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(self.response)
        logging.info(response_object)

    def read_test_mode_0x0901(self):
        """Read CTRL_MODE_SEL (0x0901): 0=idle/normal, 2=DO forced output,
        3=JOG test, 4=Positioning test. Returns the raw int, or None on a
        communication/parse failure. Diagnostic only -- lets /status show
        whether the drive is actually latched into the mode a caller just
        tried to enter, instead of trusting the write alone (this driver
        has twice now been found to silently accept a 0x0901 write at the
        wire level without it taking effect, when its precondition wasn't
        met -- see enable_speed_ctrl()/_execute_positioning()'s comments).
        """
        message = self.modbus_client.build_read_message(0x0901, 1)
        response = self.modbus_client.send_and_receive(message)
        if response is None:
            logger.error("No response reading CTRL_MODE_SEL (0x0901).")
            return None
        try:
            return ModbusRTUResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to parse CTRL_MODE_SEL response: {e}")
            return None

    def read_PF82(self):
        logging.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
        message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(self.response)
        logger.info(response_object.get_value())

    def write_PF82(self, execute_PATH_value: int = 0) -> bool:
        """Writes PF82 (PRCM, "PR trigger register"): 0 = execute origin
        return, 1~63 = execute PATH#1~PATH#63, 1000 = stop; 64~999 is
        prohibited by the manual (docs/en_manual.txt:6123-6133). Returns True
        if the drive acknowledged the write. HARDWARE-AFFECTING: starts a
        real move. Not wired to any web/OSC/Art-Net action.

        (An earlier version ignored its argument and always wrote 1, so every
        call ran PATH#1.)"""
        value = execute_PATH_value
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError("execute_PATH_value must be an integer.")
        if not (0 <= value <= 63 or value == 1000):
            raise ValueError(
                "execute_PATH_value must be 0 (origin return), 1~63 (PATH#), or 1000 (stop); "
                "64~999 is prohibited by the manual."
            )
        logger.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {hex(PF.PRCM.address)} <- {value}")
        return self._write_parameter(PF.PRCM, value)

    # Registers whose value is a packed bitfield/mask (manual pp.83-90,
    # 105-108), not a plain measurement -- decoded via the matching
    # explain_* classmethod on PA/PD (servo_p_register.py) rather than
    # shown as a raw integer.
    _EXPLAIN_FN_BY_REGISTER_NAME = {
        "STY": PA.explain_STY,
        "HMOV": PA.explain_HMOV,
        "PLSS": PA.explain_PLSS,
        "POL": PA.explain_POL,
        "SDI": PD.explain_SDI,
        "ITST": PD.explain_ITST,
        "MCOK": PD.explain_MCOK,
        "ABS": PA.explain_ABS,
        "MCS": PA.explain_MCS,
        "APST": PA.explain_APST,
    }
    # 32-bit (2-word) registers; everything else here is read as one word.
    _READ_WORDS_BY_REGISTER_NAME = {"APR": 2, "APP": 2, "CMX": 2, "CDV": 2,
                                    "FBK_0000": 2, "FBK_0024": 2}

    def Read_Pos_Related_Paremters(self) -> list:
        """Reads a fixed diagnostic set of PA/PD *parameter* registers --
        mostly configuration bitfields, not live motion feedback -- and
        decodes each one that has a known bit layout (see
        _EXPLAIN_FN_BY_REGISTER_NAME) so "GET STATE VALUE" surfaces actual
        meaning instead of raw undecoded bytes. Read-only; never writes."""
        read_address_array = [PA.STY, PA.HMOV, PA.PLSS,
                               PA.ENR, PA.PO1H, PA.POL,
                               PD.SDI, PD.ITST, PD.MCOK,
                               PA.MCS, PA.ABS, PA.APST, PA.APR, PA.APP,
                               PA.CMX, PA.CDV,
                               # Status-monitor registers, listed side by side so
                               # the raw/translated relationship can be read off
                               # the drive (the manuals disagree on 0x0024).
                               Register(0, "FBK_0000", "Motor feedback pulses, address 0x0000", 0, 0x0000),
                               Register(0, "FBK_0024", "Motor feedback pulses, address 0x0024", 0, 0x0024)]

        results = []
        raw_feedback = None
        for address in read_address_array:
            logger.info(f"Read {address.no}: {address.name}: {hex(address.address)}")
            message = self.modbus_client.build_read_message(
                address.address, self._READ_WORDS_BY_REGISTER_NAME.get(address.name, 1))
            response = self.modbus_client.send_and_receive(message)
            entry = {
                "no": address.no,
                "name": address.name,
                "description": address.description,
                "address": hex(address.address),
                "value": None,
                "interpreted": None,
            }
            if response is None:
                entry["interpreted"] = "No response (communication failure)"
            else:
                try:
                    value = ModbusRTUResponse(response).get_value(
                        signed=address.name == self.abs_rev_register)
                    entry["value"] = value
                    explain_fn = self._EXPLAIN_FN_BY_REGISTER_NAME.get(address.name)
                    if explain_fn:
                        entry["interpreted"] = explain_fn(value)
                    elif address.name == "ENR":
                        entry["interpreted"] = f"{value} pulses/rev (or division ratio, per POL's z-bit)"
                    elif address.name == "PO1H":
                        entry["interpreted"] = f"{value} rev"
                    elif address.name in ("CMX", "CDV"):
                        entry["interpreted"] = (
                            f"{value} (this app assumes CMX = CDV, i.e. 1:1)")
                    elif address.name == "FBK_0000":
                        raw_feedback = value
                        entry["interpreted"] = (
                            f"{value} pulses -- the register all angle math here is built on "
                            "(English manual: raw count; Chinese p.305: after electronic gear)")
                    elif address.name == "FBK_0024":
                        ratio = (f"; {value / raw_feedback:.4f} x FBK_0000"
                                 if raw_feedback else "")
                        entry["interpreted"] = (
                            f"{value} pulses{ratio} (English manual: after electronic gear; "
                            "Chinese p.305: before). At CMX/CDV = 1:1 both addresses should "
                            "match; read a moment after FBK_0000, so they may differ slightly "
                            "while the shaft moves)")
                    elif address.name in ("APR", "APP"):
                        unit = "rev" if address.name == self.abs_rev_register else "pulses"
                        entry["interpreted"] = (
                            f"{value} {unit} (as configured by abs_rev_register="
                            f"{self.abs_rev_register!r}; meaningful only when PA28 = 1)")
                except Exception as e:
                    logger.error(f"Failed to parse {address.name} response: {e}")
                    entry["interpreted"] = f"Parse error: {e}"
            logger.info(f"{address.name} = {entry['value']} ({entry['interpreted']})")
            results.append(entry)
            self.delay_ms(100)
        return results

    def Read_Motion_Completed_Signal(self) -> bool:
        try:
            message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
            self.response = self.modbus_client.send_and_receive(message)
            response_object = ModbusRTUResponse(self.response)
            return response_object.get_value() != 0
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
            return False
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")
            return False

    def Enable_Position_Mode(self, enable=True):
        address = ServoControlRegistry.CTRL_MODE_SEL.value
        config_value = 0x0000
        if enable == True:
            config_value = 0x0004
        message = self.modbus_client.build_write_message(address, config_value)
        # send_and_receive (not the old fire-and-forget send()) so the
        # driver's write-echo gets drained here instead of sitting unread
        # in the input buffer, where it would silently concatenate onto a
        # later, unrelated response -- see modbus_rtu_client.py's
        # _infer_expected_length() docstring for the bug this fixes.
        self.modbus_client.send_and_receive(message)

    def Enable_JOG_Mode(self, enable=True):
        address = ServoControlRegistry.CTRL_MODE_SEL.value
        config_value = 0x0000
        if enable:
            config_value = 0x0003
        message = self.modbus_client.build_write_message(address, config_value)
        self.response = self.modbus_client.send_and_receive(message)

    def config_acc_dec_0x0902(self, acc_dec_time):
        config_value = acc_dec_time
        message = self.modbus_client.build_write_message(0x0902, config_value)
        # See Enable_Position_Mode()'s comment: drains the write echo
        # instead of leaving it unread for a later transaction to inherit.
        self.modbus_client.send_and_receive(message)

    def config_speed_0x0903(self, speed_rpm):
        config_value = speed_rpm
        message = self.modbus_client.build_write_message(0x0903, config_value)
        self.modbus_client.send_and_receive(message)

    def config_pulses_0x0905_low_byte(self, low_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_L.value
        config_value = low_byte
        message = self.modbus_client.build_write_message(address, config_value)
        self.modbus_client.send_and_receive(message)

    def config_pulses_0x0906_high_byte(self, high_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_H.value
        config_value = high_byte
        message = self.modbus_client.build_write_message(address, config_value)
        self.modbus_client.send_and_receive(message)

    def read_0x0905_low_byte(self):
        message = self.modbus_client.build_read_message(0x0905, 1)
        response = self.modbus_client.send_and_receive(message)
        return response

    def read_0x0906_high_byte(self):
        message = self.modbus_client.build_read_message(0x0906, 1)
        response = self.modbus_client.send_and_receive(message)
        return response

    def pos_motion_start_0x0907(self, value):
        config_value = value
        message = self.modbus_client.build_write_message(0x0907, config_value)
        self.modbus_client.send_and_receive(message)

    def read_motor_feedback_pulses(self):
        """0x0000, 2 words, unsigned 32-bit that wraps every 2**32 pulses
        (see EncoderPulseTracker). All angle math here is built on this
        register at 4194304 pulses per motor revolution -- which is only
        correct when the electronic gear ratio is 1:1 (see
        check_electronic_gear_ratio()).

        The manuals disagree on whether this address is before or after the
        electronic gear ratio: the English manual (and Chinese p.256) list it
        plainly as "Motor feedback pulses"; Chinese V1.07 p.305 labels it
        "(電子齒輪比後)" (after gear) and recommends 0x0024 (before gear)
        instead. At 1:1 the two addresses should read the same, so the
        ambiguity does not matter for as long as the ratio stays 1:1;
        read_motor_feedback_pulses_0x0024() and GET STATE VALUE exist to
        confirm that on the drive."""
        message = self.modbus_client.build_read_message(0x0000, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)
        encoder_value = response_object.get_value()
        if encoder_value is not None:
            return int(encoder_value)
        return None

    def read_motor_feedback_pulses_0x0024(self):
        """0x0024, 2 words. English manual: "Translated motor feedback
        pulses" (multiplied by the electronic gear ratio, panel FPH.O/FPL.O).
        Chinese V1.07 p.256/p.305: "(電子齒輪比前)" -- before the gear ratio --
        and the address that manual RECOMMENDS for reading position. The
        manuals contradict each other; named by address for that reason. NOT
        used for any position math (switching would rescale the saved home);
        GET STATE VALUE lists it next to 0x0000 so the relationship can be
        read off the drive. None on any failure."""
        message = self.modbus_client.build_read_message(0x0024, 2)
        try:
            response = self.modbus_client.send_and_receive(message)
            value = ModbusRTUResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to read motor feedback pulses (0x0024): {e}")
            return None
        return None if value is None else int(value)

    def check_electronic_gear_ratio(self):
        """Reads PA06 (CMX) / PA07 (CDV) and records whether the electronic
        gear ratio is 1:1. Returns (cmx, cdv), or None if unreadable.

        Why it matters: positioning commands (0x0905/0x0906 pulses) are
        multiplied by CMX/CDV before they reach the motor, while every angle
        here is computed from encoder pulses at 4194304/rev. The two only
        agree at 1:1, so anything else would make moves and readings
        disagree -- and, if the ratio is not 1:1, which of 0x0000/0x0024 is
        the pre-gear count would suddenly matter too (the manuals disagree,
        see read_motor_feedback_pulses()). Read-only; warns but never blocks.
        (PC32~PC34 hold alternative ratios selected by DI; not checked.)"""
        cmx = self._read_parameter(PA.CMX)
        cdv = self._read_parameter(PA.CDV)
        if cmx is None or cdv is None:
            self.electronic_gear = None
            self.electronic_gear_unity = None
            logger.warning("Could not read PA06/PA07; electronic gear ratio unverified.")
            return None
        self.electronic_gear = (cmx, cdv)
        self.electronic_gear_unity = (cmx == cdv)
        if not self.electronic_gear_unity:
            logger.warning(
                f"Electronic gear ratio is CMX/CDV = {cmx}/{cdv}, not 1:1. Angle and "
                "positioning math assume 1:1 (command pulses = encoder pulses); "
                "moves will not match the displayed angle. Set PA06 = PA07 on the drive."
            )
        return self.electronic_gear

    def pos_step_motion_test(self, CW=True):
        # pos_motion_start_0x0907 (0x0907, "Positioning test operation") is
        # a TEST-MODE trigger. Per the SDE manual (confirmed 2026-09-18):
        # while the drive is in Forced-DO / JOG-test / positioning-test
        # mode, ANY communication gap over 1 second makes it auto Servo-Off
        # and force-exit test mode -- mid-motion if that's when the gap
        # happens. This is why start_continuous_reading() is called BEFORE
        # triggering 0x0907, not after: its ~100-150ms poll cycle (well
        # under the 1s limit) is what keeps the drive from timing out for
        # as long as continuous reading stays active. If anything ever
        # changes _read_continuously()'s poll interval or retry/backoff
        # timing, it must stay well under 1s or real in-progress moves can
        # get cut off by the drive itself, not just by our own software.
        #
        # NOTE: deliberately NOT stopping a stale reading_active session
        # here first (servo_comm_shihlin's own version of this method does,
        # after finding a *different* bug there -- its start_continuous_reading()
        # still had the old toggle-off-if-already-active behavior this
        # project already moved away from). Doing that here would defeat
        # the fix described in start_continuous_reading()'s own comment:
        # tearing down and restarting the keep-alive thread reopens exactly
        # the >1s communication gap that lets the drive auto-exit test
        # mode mid-move.
        self.start_continuous_reading()
        self.delay_ms(100)
        if CW == True:
            self.pos_motion_start_0x0907(1)
        else:
            self.pos_motion_start_0x0907(2)

    def pos_step_motion_by(self, target_pos: int = 0, acc_dec_time=5000, speed_rpm=10):
        # Route the live read through the same EncoderPulseTracker instance
        # used by _read_continuously()/cancel_continuous_reading(), not the
        # raw register directly -- target_pos (typically self.abs_home_pos)
        # is itself a tracker-derived cumulative value, so comparing it
        # against a raw, wrapping reading would silently break once a wrap
        # has occurred (see design doc §2.4 "Plan C").
        # In absolute mode target_pos is in the absolute scale (see
        # _active_home_pos()) and _read_reference_encoder() returns the
        # matching absolute reading; in incremental mode it is exactly the
        # tracker-derived value described above.
        current_pos = self._read_reference_encoder()
        if current_pos is None:
            logging.warning("pos_step_motion_by: no usable position reading; not moving.")
            return 0.0
        with self.lock:
            self.current_encoder = current_pos
        logger.info(f"Current Encoder Value: {current_pos}")

        diff_pulses = target_pos - current_pos

        if abs(diff_pulses) > MAX_POSITIONING_PULSES:
            logging.warning(
                f"pos_step_motion_by: {abs(diff_pulses)} pulses is more than the drive's "
                f"command-pulse register holds ({MAX_POSITIONING_PULSES}); not moving."
            )
            return 0.0

        logging.info(f"Diff Pulses: {diff_pulses}")
        move_pulses = abs(diff_pulses)

        low_byte = move_pulses & 0xFFFF
        high_byte = (move_pulses >> 16) & 0xFFFF

        self._execute_positioning(diff_pulses, low_byte, high_byte, acc_dec_time, speed_rpm)

        angle_rotated = diff_pulses / self.base_pulse_per_degree
        logging.info(f"Angle Rotated: {angle_rotated}")
        return angle_rotated

    def post_step_motion_by(self, angle: float = 0.0, acc_dec_time: int = 5000, speed_rpm: int = 10,
                            relative: bool = False):
        """Moves to the absolute `angle` (degrees from home) by commanding the
        RELATIVE difference from the drive's current angle. With
        relative=True, `angle` is instead the amount to move BY, measured
        from the position just read from the drive (Art-Net channels 13-16).
        Every caller (web
        HOME, MOVE TO SET POINT, OSC /set_point, Art-Net ch4) needs that
        difference computed from the REAL position: current_angle is
        otherwise only updated by the continuous-reading thread, so right
        after a process start it still holds its __init__ default (0.0) and a
        move computed from it lands at the wrong angle -- seen live
        2026-09-19 (target 14.43 deg landed at 28.86 deg). So the position is
        read from the drive first. Raises PositionUnavailableError, without
        moving, if it can't be read or (absolute mode) can't be trusted.

        There is no limit on how far one move may go (the 180 degree guard was
        removed 2026-09-21). Only what the drive cannot represent is refused:
        MoveOutOfRangeError (a ValueError) for an angle that is not a finite
        number, or a move of more than 2^31-1 command pulses (about 6144 deg of
        output shaft at 30:1). Nothing is read or sent in that case."""
        if not math.isfinite(angle):
            raise MoveOutOfRangeError(f"angle must be a finite number, got {angle!r}.")
        if not self._refresh_current_angle_from_hardware():
            raise PositionUnavailableError(
                "Could not read the current position from the drive; refusing to move "
                "(a move computed from a stale position would land in the wrong place)."
            )
        with self.lock:
            self.previous_angle = self.current_angle
            self.target_angle = self.current_angle + angle if relative else angle
            diff_angle = self.target_angle - self.current_angle

        logger.info(
            f"Performing motion: Target Angle={self.target_angle}, "
            f"Previous Angle={self.previous_angle}"
        )

        if diff_angle != 0.0:
            total_pulse = self.base_pulse_per_degree * abs(diff_angle)
            if total_pulse > MAX_POSITIONING_PULSES:
                # Before any state (float_error, accumulate_pulse) is touched.
                raise MoveOutOfRangeError(
                    f"A move of {diff_angle:.1f} deg is {total_pulse:.0f} command pulses; the drive "
                    f"holds at most {MAX_POSITIONING_PULSES} (about "
                    f"{MAX_POSITIONING_PULSES / self.base_pulse_per_degree:.0f} deg). Nothing was sent."
                )
            integer_pulse = int(total_pulse)
            fractional_pulse = total_pulse - integer_pulse

            # Directional float-error accumulation (servo_comm_shihlin_50W's
            # fix -- the original's equivalent block was dead code, wrapped
            # in a triple-quoted no-op). Keeps sub-pulse rounding error from
            # silently drifting in one direction over many small moves.
            if diff_angle > 0:
                self.float_error += fractional_pulse
            else:
                self.float_error -= fractional_pulse

            if abs(self.float_error) >= 1.0:
                integer_error = int(self.float_error)
                integer_pulse += integer_error
                self.float_error -= integer_error

            self.accumulate_pulse += integer_pulse

            low_byte = integer_pulse & 0xFFFF
            high_byte = (integer_pulse >> 16) & 0xFFFF

            logger.info(f"Motion Pulses: {integer_pulse}, float_error: {self.float_error}")

            self._execute_positioning(diff_angle, low_byte, high_byte, acc_dec_time, speed_rpm)

    def _execute_positioning(self, angle, low_byte, high_byte, acc_dec_time, speed_rpm):
        # Manual Step 1 for Positioning test (docs/en_manual.txt:10390),
        # identical to JOG test's own Step 1 (see enable_speed_ctrl()'s
        # comment, confirmed 2026-09-18 on real hardware): the drive only
        # accepts entering this mode "without any alarm occurrence or Servo
        # ON activated". Without this, SET POINT 1/2, HOME, and any other
        # caller of this method would silently fail to actually enter
        # position-test mode whenever Servo was already ON -- the 0x0901
        # write would be accepted at the wire level but never take effect,
        # so the pulse/speed/trigger writes that follow would have no
        # effect either.
        self.clear_alarm_12()
        self.delay_ms(100)
        self.Enable_Position_Mode(True)
        self.delay_ms(100)
        self.config_acc_dec_0x0902(acc_dec_time)
        self.delay_ms(50)
        self.config_speed_0x0903(speed_rpm)
        self.delay_ms(50)
        self.config_pulses_0x0905_low_byte(low_byte)
        self.delay_ms(50)
        self.config_pulses_0x0906_high_byte(high_byte)
        self.delay_ms(50)

        if angle > 0:
            logger.info("Running Servo CW")
            self.pos_step_motion_test(True)
        else:
            logger.info("Running Servo CCW")
            self.pos_step_motion_test(False)

    def enable_speed_ctrl(self, speed_rpm=100, acc_time=5000, enable=True):
        # Per the SDE manual's "JOG test" procedure (docs/en_manual.txt,
        # confirmed 2026-09-18): entering JOG mode means writing 0x0003 to
        # CTRL_MODE_SEL (0x0901), THEN setting accel/decel (0x0902) and
        # speed (0x0903) -- only then is 0x0904 (JOG_OPERATION, written by
        # speed_ctrl_action()) meaningful. This previously called
        # Enable_Position_Mode() instead of Enable_JOG_Mode(), so the drive
        # was never actually switched into JOG mode and speed_rpm/acc_time
        # were silently ignored on the enable=True path (the one every
        # caller -- the web UI button, OSC, Art-Net -- actually uses).
        # OSC clients can send `enable` as a plain string ("True"/"False")
        # rather than a native OSC boolean -- "True" == True is False in
        # Python, so without this coercion a string "True" silently took
        # the wrong (disable) branch with zero error/warning. Confirmed
        # live via OSC 2026-09-19: /set_continous_motion 100,5000,"True"
        # armed nothing.
        if isinstance(enable, str):
            enable = enable.strip().lower() in ("true", "1", "on", "yes")

        if enable:
            # Manual Step 1 for JOG test (docs/en_manual.txt:10347): the
            # drive only accepts entering JOG mode "without any alarm
            # occurrence or Servo ON activated". Deliberately calling
            # clear_alarm_12() here rather than servo_off() -- servo_off()
            # zeroes the DI7 bit that keeps Alarm 12 suppressed (see its own
            # "Alarm 12 ON!" log line), which would trade "Servo ON blocks
            # JOG mode" for "Alarm 12 blocks JOG mode". clear_alarm_12()
            # (already used as servoOn's own first step) sets Servo OFF
            # while keeping that bit set, satisfying both halves of Step 1
            # at once.
            self.clear_alarm_12()
            self.delay_ms(100)
            # Manual step order is mode-entry (Step 2) THEN accel/speed
            # (Steps 3-4) -- confirmed 2026-09-18 as load-bearing, not just
            # documentation style, by the Step 1 precondition above (writing
            # 0x0901 while Servo was still ON was silently accepted at the
            # wire level but never took effect on the drive). Setting
            # 0x0903 before the drive is actually in JOG mode risked the
            # same failure mode -- the speed value never taking hold,
            # regardless of what was typed into the UI.
            self.Enable_JOG_Mode(True)
            self.delay_ms(100)
            self.config_acc_dec_0x0902(acc_time)
            self.delay_ms(100)
            self.set_jog_speed(speed_rpm)
            self.delay_ms(100)
            # Explicit stop (0x0904=0) as the LAST step of arming -- found
            # 2026-09-22: 0x0904 (JOG_OPERATION) is a sticky register on this
            # drive, not reset by (re-)entering JOG mode. If a previous
            # session left it at 1/2 (CW/CCW) -- e.g. a MOTION PAUSE that
            # never landed (keyup missed while the browser tab lost focus,
            # a dropped request) -- simply re-arming JOG mode resumed
            # rotation immediately, with no CW/CCW ever explicitly pressed
            # this time. Forcing 0x0904=0 here guarantees every arm ends in
            # a definite stopped state regardless of leftover register
            # state. speed_ctrl_action() (not a bare write) so
            # _last_motion_direction is reset too, letting the very next
            # CW/CCW press through without needing an extra MOTION PAUSE.
            self.speed_ctrl_action(0)
            self.delay_ms(100)
            # auto_stop_on_stillness=False: JOG mode is continuous-run, not
            # a discrete move -- pressing MOTION PAUSE (speed_ctrl_action(0))
            # is a deliberate hold, not "the move finished", so the
            # stillness-based auto-stop built for pos_step_motion_test()
            # must not tear this session down. If it did, the background
            # poll thread (also this mode's <1s keep-alive, see
            # _read_continuously()'s comment) would die, the drive would
            # silently exit JOG mode after the timeout, and the next MOTION
            # START CW/CCW would need ENABLE SPEED CONTROL MODE pressed
            # again first to re-enter JOG mode.
            self.start_continuous_reading(0.1, auto_stop_on_stillness=False)
        else:
            self.Enable_JOG_Mode(False)
            self.delay_ms(100)
            # Disabling continuous motion must stop the keep-alive polling
            # too -- this previously fell through to the same
            # start_continuous_reading() call as the enable=True branch,
            # leaving reading_active=True (and the background thread
            # running indefinitely) even after an explicit "disable"
            # request. Confirmed live via OSC 2026-09-19: /set_continous_motion
            # ...,False left reading_active=True.
            self.stop_continuous_reading()
            self.clear_jog_speed()

    # Manual's documented range for 0x0903 (docs/en_manual.txt:10367-10373).
    JOG_SPEED_MIN_RPM = 0
    JOG_SPEED_MAX_RPM = 3000

    def set_jog_speed(self, speed_rpm) -> int:
        """Sets the JOG speed (0x0903) to an absolute value, clamped to the
        manual's documented range, and records it in self.jog_speed_rpm --
        the single source of truth change_jog_speed_by() and the web UI's
        /status polling use to report "the actual running speed", regardless
        of which input source (web ENABLE SPEED CONTROL MODE / arrow keys,
        OSC /set_continous_motion or /jog_speed_adjust, Art-Net Channel 1)
        set it last. Always call this (not a bare config_speed_0x0903())
        for a JOG speed that should be tracked -- config_speed_0x0903() is
        also used for unrelated things (e.g. positioning-test speed) that
        must NOT overwrite this. Returns the resulting speed."""
        new_speed = max(self.JOG_SPEED_MIN_RPM, min(self.JOG_SPEED_MAX_RPM, int(speed_rpm)))
        self.config_speed_0x0903(new_speed)
        self.jog_speed_rpm = new_speed
        return new_speed

    def clear_jog_speed(self) -> None:
        """Forgets the tracked JOG speed -- call whenever JOG mode is torn
        down (enable_speed_ctrl(enable=False), MOTION CANCEL) so a stale
        value doesn't let change_jog_speed_by() appear to succeed, or
        /status report a running speed, for a mode that is no longer
        active."""
        self.jog_speed_rpm = None

    def change_jog_speed_by(self, delta_rpm: int) -> int:
        """Nudges the running JOG speed by delta_rpm (e.g. +1/-1 from an
        arrow-key press) instead of setting an absolute value -- see
        set_jog_speed(). Requires enable_speed_ctrl() to have been called
        with enable=True first -- raises RuntimeError rather than silently
        guessing a starting speed if it hasn't (or if MOTION CANCEL/
        enable=False has since torn the mode down). Returns the resulting
        speed."""
        if self.jog_speed_rpm is None:
            raise RuntimeError(
                "change_jog_speed_by: JOG/speed-control mode is not active "
                "(press ENABLE SPEED CONTROL MODE first) -- nothing to adjust."
            )
        return self.set_jog_speed(self.jog_speed_rpm + delta_rpm)

    # 0: Stop
    # 1: CW
    # 2: CCW
    def speed_ctrl_action(self, action_value):
        # Fail-safe: refuse to jump straight from one direction to the
        # other while the motor is still running that way -- an abrupt
        # reversal without stopping first can shock the mechanism. Requires
        # an explicit action_value=0 (MOTION PAUSE) in between. Only guards
        # a direct 1<->2 switch; 0 (stop) and repeating the same direction
        # are always allowed.
        if action_value in (1, 2) and self._last_motion_direction in (1, 2) \
                and action_value != self._last_motion_direction:
            logging.warning(
                f"Refusing direct direction reversal (currently "
                f"{self._last_motion_direction}, requested {action_value}) "
                "-- send MOTION PAUSE (action_value=0) first."
            )
            return False

        if action_value == 0:
            logging.info("Servo Stop!")
        elif action_value == 1:
            logging.info("Servo CW")
        elif action_value == 2:
            logging.info("Servo CCW")
        else:
            logging.info("Error Config.")
        self.delay_ms(100)
        address = 0x0904
        message = self.modbus_client.build_write_message(address, action_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

        if action_value in (0, 1, 2):
            self._last_motion_direction = action_value
        return True

    def set_home_position(self):
        if self.absolute_mode:
            self._set_home_position_absolute()
            return
        raw_encoder = self.read_motor_feedback_pulses()
        if raw_encoder is None:
            logger.error("set_home_position: encoder unreadable; home NOT set.")
            return
        with self.lock:
            self.current_angle = 0.0
            self.previous_angle = 0.0
            self.target_angle = 0.0
            self.previous_encoder = self.current_encoder
            self.float_error = 0.0
            self.accumulate_pulse = 0
            self.current_encoder = self._encoder_tracker.reset(raw_encoder)
        self.delay_ms(100)
        self.save_abs_home_pos(self.current_encoder)
        self.home_set_since_start = True
        logger.info("home position set!!!")

    def _set_home_position_absolute(self):
        """Absolute-mode SET HOME: captures the current absolute encoder
        position (PA32/PA33) as the home reference. Refuses (changes
        nothing) if the absolute position can't be trusted."""
        raw_encoder = self.read_motor_feedback_pulses()
        absolute_pulses = self.read_absolute_position_pulses()
        if raw_encoder is None or absolute_pulses is None:
            logger.error("set_home_position: absolute position unavailable; home NOT set.")
            return
        with self.lock:
            self.current_angle = 0.0
            self.previous_angle = 0.0
            self.target_angle = 0.0
            self.float_error = 0.0
            self.accumulate_pulse = 0
            tracker_value = self._encoder_tracker.reset(raw_encoder)
            self._absolute_offset = absolute_pulses - tracker_value
            self.previous_encoder = self.current_encoder
            self.current_encoder = absolute_pulses
        self.abs_home_pos_absolute = absolute_pulses
        self._save_config_value("abs_home_pos_absolute", absolute_pulses)
        self.home_set_since_start = True
        logger.info("absolute home position set!!!")

    def initial_abs_home(self) -> bool:
        if self.on_initial_home:
            logging.info("Initial absolute home now is running.")
            return False

        home_pos = self._active_home_pos()
        if home_pos is None:
            logging.warning("initial_abs_home: absolute mode is active but no absolute home is set; not moving.")
            return False

        try:
            self.stop_event.clear()
            self.on_initial_home = True

            speed_rpm = 12
            angle_rotated = self.pos_step_motion_by(home_pos, 5000, speed_rpm)

            time_per_revolution = 60 / speed_rpm
            timeout = 1.2 * (angle_rotated / 360) * time_per_revolution

            logging.info(f"Estimate Timeout: {timeout} seconds for angle")

            start_time = time.time()
            while not self.stop_event.is_set():
                if time.time() - start_time > timeout:
                    logging.info("Timeout reached while waiting for stop process.")
                    break
                self.delay_ms(100)

            if self.stop_event.is_set():
                logging.info("Stop process completed successfully.")
                self.on_initial_home = False
                return True
            else:
                logging.warning("Operation timed out.")
                self.on_initial_home = False
                return False
        except Exception as e:
            logging.error(f"Error in initial_abs_home: {e}")
            self.on_initial_home = False
            return False
