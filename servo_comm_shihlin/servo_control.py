import time
import logging
import json
import math
import threading
from typing import Union, Callable
from threading import Thread, Event, Lock
from serial import SerialException
from modbus_ascii_client import ModbusASCIIClient
from modbus_response import ModbusResponse
from encoder_pulse_tracker import EncoderPulseTracker
from servo_utility import ServoUtility
from servo_control_registers import ServoControlRegistry
from status_bit_map import DI_Function_Code
from servo_p_register import PA, PC, PD, PE, PF

PA.init_registers()
PC.init_registers()
PD.init_registers()
PE.init_registers()
PF.init_registers()


# Configure logging
logger = logging.getLogger(__name__)

# The alarm monitor register (0x0100) is plain hex, and this driver reports
# 0xFF (255) -- shown as "AL --" on its panel -- not 0 for "no alarm"
# (confirmed on real hardware 2026-09-18, servo_comm_shihlin_unified). Treat
# both as "no alarm"; None (communication failure) is NOT "no alarm".
NO_ALARM_CODES = frozenset({0, 0xFF})

# Per-poll encoder delta at or below this many pulses counts as "not moving"
# for the motion-complete auto-stop in _read_continuously(). Ported from
# servo_comm_shihlin_unified (same constant, same rationale: calibrated
# above a stationary encoder's few-pulse noise floor and the larger dither
# a real move exhibits while holding its target under closed-loop torque)
# after real-hardware testing on this project's own COM4 rig (2026-09-22)
# found the old PF.PRCM-based check (Read_Motion_Completed_Signal()) falsely
# declared a positioning-test move "complete" long before the encoder had
# actually finished moving -- see _read_continuously()'s comment.
STILL_THRESHOLD_PULSES = 200
# Consecutive "not moving" polls required before declaring motion complete.
STILL_COUNT_TO_COMPLETE = 12


def is_alarm_active(alarm_code) -> bool:
    if alarm_code is None:
        return True
    return alarm_code not in NO_ALARM_CODES


class PositionUnavailableError(ValueError):
    """The drive's current position could not be read, so a position-relative
    move was refused rather than computed from a stale angle."""


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


class ServoController:
    CONFIG_FILE = "servo_config.json"

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.modbus_client = ModbusASCIIClient.get_instance(
            device_number=1, serial_port_manager=serial_port
            )
        self.read_thread: Union[Thread, None] = None
        self.read_thread_stop_event = threading.Event()
        self.reading_active = False
        # RLock (not Lock): start_continuous_reading() can call
        # stop_continuous_reading() from inside its own `with self.lock`
        # block when reading is already active (see its comment) -- a plain
        # Lock deadlocks there, confirmed live 2026-09-22 via the web UI:
        # ENABLE POS MODE leaves reading_active True, so the next POS TEST
        # START CW/CCW click hung forever inside start_continuous_reading(),
        # never releasing hardware_lock.py's _hardware_busy_lock either --
        # every other button then got rejected (429) until the server was
        # restarted, while the original background thread kept polling.
        # Ported from servo_comm_shihlin_unified, which already uses RLock
        # here for the same reason.
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
        self.on_initial_home = False
        self.completed_tag = False
        self.completed_cnt = 0
        # Motion-complete auto-stop state for _read_continuously() -- see its
        # comment and start_continuous_reading()'s auto_stop_on_stillness.
        self._motion_seen = False
        self._still_count = 0
        self._auto_stop_on_stillness = True
        #self.abs_home_pos = 1184347
        self.abs_home_pos = self.load_abs_home_pos()
        # Recorded by SET POINT 1/2 (degrees from home, None = never recorded)
        # and persisted next to abs_home_pos in servo_config.json.
        self.set_point_1 = self._load_config_value("set_point_1")
        self.set_point_2 = self._load_config_value("set_point_2")
        # PA06/PA07 as (cmx, cdv), and whether they are 1:1 (None = unread):
        # see check_electronic_gear_ratio().
        self.electronic_gear = None
        self.electronic_gear_unity = None
        # True once SET HOME has succeeded since this process started: the
        # incremental encoder counter restarts at every drive power-on, so a
        # home saved by an earlier run may not match the shaft.
        self.home_set_since_start = False
        # on_alarm is notified only from existing on-demand alarm reads (see
        # read_current_alarm_code()) -- deliberately not backed by a new
        # polling thread; this board (Pi 3 B) has no CPU headroom to spare
        # for an always-on alarm poll (see CLAUDE.md §2).
        self._event_listeners = {"on_motion_completed": [], "on_moving": [], "on_alarm": []}
        # None/0 = not currently running in either direction (reversal guard
        # in speed_ctrl_action() allows the next action freely); 1/2 =
        # currently commanded (whichever this drive's speed_ctrl_action()
        # convention maps to CW/CCW) -- only a matching repeat or an
        # explicit stop is allowed next. Ported from servo_comm_shihlin_unified.
        self._last_motion_direction = None
        # The JOG speed (0x0903) last written by enable_speed_ctrl(), so
        # change_jog_speed_by() has a baseline to nudge from. None = JOG/
        # speed-control mode is not currently armed. Ported from
        # servo_comm_shihlin_unified.
        self.jog_speed_rpm = None

    def _load_config_dict(self) -> dict:
        try:
            with open(self.CONFIG_FILE, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            logging.warning("Config file not found.")
            return {}
        except json.JSONDecodeError as e:
            logging.error(f"Error parsing configuration file: {e}.")
            return {}

    def _load_config_value(self, key: str, default=None):
        return self._load_config_dict().get(key, default)

    def _save_config_value(self, key: str, value) -> None:
        """Read-modify-write the whole config dict: this file holds several
        independent values (abs_home_pos, set_point_1, set_point_2), and
        rewriting it with only one key would silently erase the others (the
        old save_abs_home_pos() did exactly that)."""
        config = self._load_config_dict()
        config[key] = value
        try:
            with open(self.CONFIG_FILE, 'w') as file:
                json.dump(config, file)
            logging.info(f"Saved {key}: {value} to {self.CONFIG_FILE}")
        except Exception as e:
            logging.error(f"Error saving {key}: {e}")

    def load_abs_home_pos(self) -> int:
        return self._load_config_value("abs_home_pos", 1184347)

    def save_abs_home_pos(self, abs_home_pos: int):
        self._save_config_value("abs_home_pos", abs_home_pos)

    def record_set_point(self, n: int) -> float:
        """Persists the drive's CURRENT angle as Set Point 1 or 2 -- does not
        move the motor. The position is read from the drive first (the
        tracked current_angle is only kept fresh by the reading thread, so
        right after a start it is 0.0); if it cannot be read, nothing is
        recorded and PositionUnavailableError is raised. Returns the angle."""
        if n not in (1, 2):
            raise ValueError(f"Set Point must be 1 or 2, got {n!r}.")
        if not self._refresh_current_angle_from_hardware():
            raise PositionUnavailableError(
                "Could not read the current position from the drive; Set Point not recorded."
            )
        with self.lock:
            angle = self.current_angle
        self._save_config_value(f"set_point_{n}", angle)
        setattr(self, f"set_point_{n}", angle)
        logging.info(f"Set Point {n} recorded: {angle} deg")
        return angle

    def move_to_set_point(self, n: int, acc_dec_time: int = 5000, speed_rpm: int = 10) -> None:
        """Commands a move to the previously recorded Set Point 1 or 2 via the
        same closed-loop post_step_motion_by() path as HOME. Raises ValueError
        if n isn't 1/2, or if that set point has never been recorded -- a
        default target (e.g. 0) must never be used in that case."""
        if n not in (1, 2):
            raise ValueError(f"Set Point must be 1 or 2, got {n!r}.")
        target_angle = getattr(self, f"set_point_{n}")
        if target_angle is None:
            raise ValueError(f"Set Point {n} has not been recorded yet.")
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
                self.stop_continuous_reading()
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
            # does mean the move completed. Ported from
            # servo_comm_shihlin_unified.
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
            if self.read_thread and threading.current_thread() is not self.read_thread:
                # Bounded, defense-in-depth: the real fix for the 2026-09-22
                # web UI freeze was ModbusASCIIClient.receive()'s own
                # unbounded loop (see its comment), but this join() had no
                # timeout either, so ANY future way for the background
                # thread to get stuck would hang whichever caller (a Flask
                # request, holding hardware_lock.py's _hardware_busy_lock)
                # is trying to stop it -- forever, with no way to recover
                # short of restarting the process. A stuck thread is leaked
                # rather than joined in that case; logged so it's visible.
                self.read_thread.join(timeout=5.0)
                if self.read_thread.is_alive():
                    logging.error(
                        "Background reading thread did not stop within 5s; "
                        "continuing without it (it will be abandoned)."
                    )

            self.read_thread = None
            self.completed_cnt = 0
            self.completed_tag = False
            if self.on_initial_home:
                self.on_initial_home = False
            logging.info("Motion Completed Signal Reading Stopped.")
            self._notify_event_listeners("on_motion_completed")
            self.stop_event.set()
            

    def _read_continuously(self, interval: float) -> None:
        # Software motion-complete detection instead of
        # Read_Motion_Completed_Signal() (PF.PRCM) -- ported from
        # servo_comm_shihlin_unified after real-hardware testing there
        # (2026-09-18) found PF.PRCM is a PATH-execution status register,
        # unrelated to the raw-pulse positioning workflow this loop actually
        # monitors (pos_step_motion_test()/_execute_positioning(), driven by
        # 0x0905/0x0906/0x0907 -- not PF82 PATH execution).
        #
        # Confirmed live on this project's own COM4 rig (2026-09-22): the old
        # check declared a commanded move "complete" after ~3s while the
        # encoder had barely moved at all. Root cause was two-layered --
        # Enable_Position_Mode()/config_acc_dec_0x0902()/config_speed_0x0903()/
        # config_pulses_0x0905_low_byte()/config_pulses_0x0906_high_byte()/
        # pos_motion_start_0x0907() used to fire-and-forget (.send(), never
        # .send_and_receive()), so their write-echoes sat undrained in the
        # serial input buffer; the next receive() (this loop's first
        # Read_Motion_Completed_Signal() call) then scooped up all of them
        # concatenated together, ModbusResponse parsed the leading (garbled)
        # WRITE echo instead of the intended READ response, and
        # get_value() on a write-shaped response returns None -- "None != 0"
        # is True, so completed_tag was wrongly True from the very first
        # poll. The six methods above now drain their own echo via
        # send_and_receive() (see their comments), but PF.PRCM was never a
        # reliable completion signal for this workflow to begin with, so the
        # check itself is replaced too: track the raw per-poll encoder
        # delta, and once the encoder has genuinely moved (delta above
        # STILL_THRESHOLD_PULSES, safely above a stationary encoder's few-
        # pulse jitter) at least once, require STILL_COUNT_TO_COMPLETE
        # consecutive polls back below that threshold before declaring the
        # motion complete and auto-stopping. If the encoder never moves at
        # all, reading is deliberately left running rather than auto-stopped
        # -- stop it explicitly (MOTION CANCEL / stop_continuous_reading())
        # instead. auto_stop_on_stillness (see start_continuous_reading())
        # disables this entirely for continuous JOG/speed-control mode,
        # where a deliberate pause must not be mistaken for "done".
        #
        # An earlier version of this comment (2026-09-22, during this same
        # investigation) concluded 0x0907 never auto-stops and added a
        # software target-tracked active stop on top of this stillness
        # check. That was wrong: docs/en_manual.txt §4.5.3(2) ("Positioning
        # operation", the official Shihlin PC software's version of this
        # same feature) documents the drive stopping on its own "after
        # moving the command route set by the user", and manual (10)'s "0:
        # pause" wording is that section's Pause button (interrupt
        # mid-route), not evidence 1/2 run forever. Confirmed 2026-09-22:
        # the official software's own JOG and Positioning operation tests,
        # run repeatedly against this same drive, worked correctly every
        # time. The real reason this project's own 0x0907 triggers were
        # never producing motion was a precondition bug (see
        # _execute_positioning()'s comment), not a stopping-mechanism bug --
        # the active target-tracking layer was reverted.
        base_pulse_per_degree = 349525.3333333333
        previous_encoder_for_stillness = None

        while not self.read_thread_stop_event.is_set():
            if not self.serial_port.keep_running:
                logger.info("Reconnection attempts stopped.")
                break

            # Read encoder position
            try:
                encoder = self.read_encoder_before_gear_ratio()
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
            # §2.4 for why the raw 0x0000 register can't be trusted directly)
            self.current_encoder = self._encoder_tracker.update(encoder)
            logger.info(f"Current Encoder Value: {self.current_encoder} (raw: {encoder})")
            diff_angle = round((self.current_encoder - self.abs_home_pos) / base_pulse_per_degree, 4)
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

            if (self._auto_stop_on_stillness and self._motion_seen
                    and self._still_count >= STILL_COUNT_TO_COMPLETE):
                logger.info(
                    f"Motion complete: encoder stable for {self._still_count} consecutive reads."
                )
                self.stop_continuous_reading()
                break

            # loop delay
            self.delay_ms(interval * 1000)

    def read_PA01_Ctrl_Mode(self):
        logging.info(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        message = self.modbus_client.build_read_message(PA.STY.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)
    
    def read_PA33_Encoder_ABS_Pos(self):
        message = self.modbus_client.build_read_message(0x0340, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logger.info(response_object)

    def write_PA01_Ctrl_Mode(self):
        logger.info(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 1, 0)
        message = self.modbus_client.build_write_message(
            PA.STY.address, config_value)
        try:
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(response)
            logger.info(response_object)
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def write_PA29_Initial_Abs_Pos(self):
        logger.info("Address of PA29: Initial Absolute Position")
        message = self.modbus_client.build_write_message(0x0338, 1)
        try:
            response = self.modbus_client.send_and_receive(message)
            #response_object = ModbusResponse(response)
            logger.info(f"Initial Absolute Position Set!:{response}")
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")
        

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
            #print(f"Build Read Message: {message}")
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
            response_object = ModbusResponse(response)
            logger.info(response_object)

            cnt = 1
            for data in response_object.data:
                logger.info(f"Original data value: {data}")
                for code in DI_Function_Code:
                    if code.value == int(data, 16):
                        logger.info(f"DI{cnt} :{code.name}")
                        cnt += 1
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    # 內部位置命令 p129
    # Pos 1:  000000 PE01/PE02
    # Pos 2:  000001 PE03/PE04
    # ...
    # Pos 63: 111111 PF29/PF30


    # Communication control DI on/off [Pt Mode]
    # 0x0040 (DI7) EMG
    # 0000 0  0  0  0 0 1 0 0 0 0 0 0
    #      12 11 10 9 8 7 6 5 4 3 2 1
    #      12 11 10 9 8 7 6 5 4 3 2 1
    # 0x0041 (DI7 + DI1) SON
    # 0000 0  0  0  0 0 1 0 0 0 0 0 1
    # 0x0341 (DI11 + DI10 + DI7 + DI1) : (LSN, LSP, EMG, SON)
    # 0000 0  0  1  1 0 1 0 0 0 0 0 1
    def write_PD_25(self):
        logger.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")

        config_value = ServoUtility.config_hex_with(0, 0, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)

        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logger.info(response_object)

    def read_PD_25(self):
        logger.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        message = self.modbus_client.build_read_message(PD.ITST.address, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logger.info(response_object)

    def clear_alarm(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        self.response = self.modbus_client.send_and_receive(message)
        #response_object = ModbusResponse(self.response)
        logging.info(f"Clear Alarm!:{self.response}")

    def servo_on(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        #response_object = ModbusResponse(response)
        logging.info(f"Servo On:{response}")


    def clear_alarm_12(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        #response_object = ModbusResponse(response)
        logging.info(f"Clear Alarm 12:{response}")

    def read_current_alarm_code(self):
        """Read the 'Current alarm' monitor register (0x0100, 1 word,
        read-only). 0 means no alarm active; nonzero is the active alarm
        code. See docs/en_manual.txt, "(3) Alarm information" (~line 10220).

        Returns the raw int code, or None on a communication/parse failure
        (never assume None means "no alarm").
        """
        message = self.modbus_client.build_read_message(0x0100, 1)
        response = self.modbus_client.send_and_receive(message)
        if response is None:
            logger.error("No response reading current alarm code (0x0100).")
            return None
        try:
            code = ModbusResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to parse current-alarm response: {e}")
            return None
        # Piggyback on this existing on-demand read rather than a new poll --
        # every caller (clear-alarm endpoint, JOG/positioning preconditions,
        # etc.) already does this round trip.
        self._notify_event_listeners("on_alarm", code)
        return code

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
        # print(
        #    f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        logger.info("Servo Off, Alarm 12 ON!")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        #response_object = ModbusResponse(response)
        logger.info(f"Servo Off:{response}")
        self.delay_ms(100)

    # Pos mode 0x0000 0x0000
    def read_PD_01(self):
        logger.info(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        message = self.modbus_client.build_read_message(PD.DIA1.address, 2)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(self.response)
        logging.info(response_object)

    # 0x0000, 0x0000
    def write_PD_01(self):
        logging.info(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(
            PD.DIA1.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    # Config DI Function
    # Pos mode 0x0001 , 0x0000
    def write_PD_02(self):
        logging.info(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        config_value = 1
        message = self.modbus_client.build_write_message(
            PD.DI1.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    # 0x0001 0x0000


    def read_PD_02(self):
        logging.info(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        message = self.modbus_client.build_read_message(PD.DI1.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    # initial 0x0012 0x0000 DI7


    def read_PD_08(self):
        logging.info(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_read_message(PD.DI7.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def write_PD_08(self):
        logging.info(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_write_message(PD.DI7.address, 0x02F)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def read_servo_state(self):
        message = self.modbus_client.build_read_message(0x0200, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def read_control_mode(self):
        message = self.modbus_client.build_read_message(0x0201, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def read_alarm_msg(self):
        message = self.modbus_client.build_read_message(0x0100, 11)
        self.response = self.modbus_client.send_and_receive(message)
        # print(f"Response Message: {response}")
        response_object = ModbusResponse(self.response)
        logging.info(response_object)

    def read_test_mode_0x0901(self):
        """Read CTRL_MODE_SEL (0x0901): 0=idle/normal, 2=DO forced output,
        3=JOG test, 4=Positioning test. Returns the raw int, or None on a
        communication/parse failure. Diagnostic (/status) and the
        ENABLE POS MODE / ENABLE SPEED CONTROL MODE mutual-exclusion guard
        (_reject_if_other_mode_active() in app.py) both rely on this
        actually returning a value -- it used to only log and fall through
        to an implicit `None` return unconditionally, so neither could ever
        tell JOG from Positioning from idle."""
        message = self.modbus_client.build_read_message(0x0901, 1)
        response = self.modbus_client.send_and_receive(message)
        if response is None:
            logger.error("No response reading CTRL_MODE_SEL (0x0901).")
            return None
        try:
            return ModbusResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to parse CTRL_MODE_SEL response: {e}")
            return None

    # PR (procedure) sequence control
    def read_PF82(self):
        logging.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
        message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(self.response)
        logger.info(response_object.get_value())

    def write_PF82(self, execute_PATH_value: int = 0):
        """Writes PF82 (PRCM, "PR trigger register"): 0 = execute origin
        return, 1~63 = execute PATH#1~PATH#63, 1000 = stop; 64~999 is
        prohibited by the manual. HARDWARE-AFFECTING: starts a real move.
        (An earlier version ignored its argument and always wrote 1, so every
        call ran PATH#1.)"""
        value = execute_PATH_value
        if isinstance(value, float) and value.is_integer():
            value = int(value)  # OSC delivers 5.0 for 5
        if not isinstance(value, int) or isinstance(value, bool):
            raise ValueError("execute_PATH_value must be an integer.")
        if not (0 <= value <= 63 or value == 1000):
            raise ValueError(
                "execute_PATH_value must be 0 (origin return), 1~63 (PATH#), or 1000 (stop); "
                "64~999 is prohibited by the manual."
            )
        logging.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address} <- {value}")
        message = self.modbus_client.build_write_message(PF.PRCM.address, value)
        self.response = self.modbus_client.send_and_receive(message)

        # Process the response using ModbusResponse
        try:
            response_object = ModbusResponse(self.response)
            logging.info(f"Parsed Mobus Response: {response_object}")
        except Exception as e:
            logging.info(f"An unexpected error occurred: {e}")

    # Read Position Control related parameters


    def Read_Pos_Related_Paremters(self):
        read_address_array = [PA.STY, PA.HMOV, PA.PLSS,
                              PA.ENR, PA.PO1H, PA.POL,
                              PD.SDI, PD.ITST, PD.MCOK]
        # PD28 MCOK
        # PD16 SDI
        # PD25 ITST

        # PA01, 2 3, 6, 7, 13, 15
        # PA02, ATUM: Gain tuning mode option
        # PA03, ATUL: Auto-tuning response level setting
        # PA06, CMX : Electronic gear numerator
        # PA07, CDV : Electronic gear denominator
        # PA13, PLSS: Command pulse option
        # PA15, CRSHA: Motor crash protection (time)

        for address in read_address_array:
            logger.info(f"Read {address.no}: {address.name}: {hex(address.address)}")
            message = self.modbus_client.build_read_message(address.address, 1)
            self.response = self.modbus_client.send_and_receive(message)
            logger.info(self.response)
            self.delay_ms(100)


    def Read_Motion_Completed_Signal(self) -> bool:
        try:
            message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
            self.response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(self.response)
            return response_object.get_value() != 0
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
            return False
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")
            return False

    # Position Control Test Mode
    def Enable_Position_Mode(self, enable=True):
        address = ServoControlRegistry.CTRL_MODE_SEL.value
        config_value = 0x0000
        # print(f"Address of {address}")
        if enable == True:
            config_value = 0x0004

        message = self.modbus_client.build_write_message(address, config_value)
        # send_and_receive (not the old fire-and-forget send()) so the
        # drive's write-echo gets drained here instead of sitting unread in
        # the serial input buffer, where it would silently concatenate onto
        # a later, unrelated response -- ModbusASCIIClient.receive() has no
        # per-transaction framing/resync, so it returns whatever bytes are
        # sitting in the buffer, echo included. Confirmed live 2026-09-22:
        # this is what made _read_continuously()'s first
        # Read_Motion_Completed_Signal() call misparse a garbled multi-frame
        # response as an instant "motion complete" -- see that method's
        # comment.
        self.modbus_client.send_and_receive(message)


    def Enable_JOG_Mode(self, enable=True):
        address = ServoControlRegistry.CTRL_MODE_SEL.value
        config_value = 0x0000
        if enable:
            config_value = 0x0003

        message = self.modbus_client.build_write_message(address, config_value)
        self.response = self.modbus_client.send_and_receive(message)

    def config_acc_dec_0x0902(self, acc_dec_time):
        # print(f"Address 0x0902, 1 word")
        config_value = acc_dec_time
        message = self.modbus_client.build_write_message(0x0902, config_value)
        # See Enable_Position_Mode()'s comment: drains the write echo
        # instead of leaving it unread for a later transaction to inherit.
        self.modbus_client.send_and_receive(message)


    def config_speed_0x0903(self, speed_rpm):
        # print(f"Address 0x0903, 1 word")
        config_value = speed_rpm
        message = self.modbus_client.build_write_message(0x0903, config_value)
        # See Enable_Position_Mode()'s comment.
        self.modbus_client.send_and_receive(message)


    def config_pulses_0x0905_low_byte(self, low_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_L.value
        # print(f"Address {address}, 1 word")
        config_value = low_byte
        message = self.modbus_client.build_write_message(address, config_value)
        # See Enable_Position_Mode()'s comment.
        self.modbus_client.send_and_receive(message)


    def config_pulses_0x0906_high_byte(self, high_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_H.value
        # print(f"Address {address}, 1 word")
        config_value = high_byte
        message = self.modbus_client.build_write_message(address, config_value)
        # See Enable_Position_Mode()'s comment.
        self.modbus_client.send_and_receive(message)

    def read_0x0905_low_byte(self):
        # print(f"Address 0x0905, 1 word")
        message = self.modbus_client.build_read_message(0x0905, 1)
        # print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        return response

    def read_0x0906_high_byte(self):
        # print(f"Address 0x0906, 1 word")
        message = self.modbus_client.build_read_message(0x0906, 1)
        # print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        return response

    def pos_motion_start_0x0907(self, value):
        # print(f"Address 0x0907, 1 word")
        config_value = value
        message = self.modbus_client.build_write_message(0x0907, config_value)
        # See Enable_Position_Mode()'s comment.
        self.modbus_client.send_and_receive(message)

    def read_encoder_before_gear_ratio(self):
        message = self.modbus_client.build_read_message(0x0000, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)
        encoder_value = response_object.get_value()
        if encoder_value is not None:
            return int(encoder_value)
        return None

    def read_encoder_after_gear_ratio(self):
        """0x0024. The English manual calls it the gear-translated count; the
        Chinese V1.07 manual calls it the pre-gear one -- they contradict, so
        no position math uses it (at a 1:1 gear ratio it should equal
        0x0000). Returns the value, or None on any failure."""
        message = self.modbus_client.build_read_message(0x0024, 2)
        try:
            response = self.modbus_client.send_and_receive(message)
            value = ModbusResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to read the 0x0024 feedback counter: {e}")
            return None
        return None if value is None else int(value)

    def _read_parameter(self, register):
        """A 2-word PA/PD/PE/PF parameter, or None on any failure."""
        message = self.modbus_client.build_read_message(register.address, 2)
        try:
            response = self.modbus_client.send_and_receive(message)
            value = ModbusResponse(response).get_value()
        except Exception as e:
            logger.error(f"Failed to read the parameter at {hex(register.address)}: {e}")
            return None
        return None if value is None else int(value)

    def check_electronic_gear_ratio(self):
        """Reads PA06 (CMX) / PA07 (CDV) and records whether the electronic
        gear ratio is 1:1. Returns (cmx, cdv), or None if unreadable.

        Positioning commands (0x0905/0x0906 pulses) are multiplied by CMX/CDV
        before they reach the motor, while every angle here is computed from
        encoder pulses at 4194304/rev; the two only agree at 1:1. Read-only;
        warns but never blocks."""
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

    def _read_reference_encoder(self):
        """Fresh wraparound-tracked encoder value, or None on any failure."""
        try:
            raw = self.read_encoder_before_gear_ratio()
        except Exception as e:
            logger.warning(f"Failed to read the encoder ({e}).")
            return None
        if raw is None:
            return None
        with self.lock:
            return self._encoder_tracker.update(raw)

    def _refresh_current_angle_from_hardware(self) -> bool:
        """One-shot fresh encoder read that brings current_angle/
        current_encoder up to date immediately, instead of waiting for the
        continuous-reading thread (the only other writer of these fields).
        Right after a process start current_angle still holds its __init__
        default (0.0), and a move computed against it lands in the wrong
        place -- seen live 2026-09-19 (a move to 14.43 deg landed at 28.86 deg).
        Returns False, leaving current_angle untouched, on a read failure."""
        encoder = self._read_reference_encoder()
        if encoder is None:
            logging.warning("_refresh_current_angle_from_hardware: no usable position reading.")
            return False
        with self.lock:
            self.current_encoder = encoder
            self.current_angle = round((encoder - self.abs_home_pos) / 349525.3333333333, 4)
        return True

    def pos_step_motion_test(self, CW=True):
        # Explicitly stop any stale reading session first rather than
        # relying on start_continuous_reading()'s own toggle-if-already-
        # active behavior: confirmed live 2026-09-22 via the web UI that
        # ENABLE POS MODE leaves reading_active True, so this call's own
        # start_continuous_reading() would just toggle that session OFF and
        # return -- meaning the move triggered by pos_motion_start_0x0907()
        # below actually runs with NO active monitoring thread at all (the
        # web UI's /status then shows reading_active=false while the motor
        # is still physically moving). This makes sure a fresh session is
        # always running for THIS move.
        if self.reading_active:
            self.stop_continuous_reading()
        self.start_continuous_reading()
        self.delay_ms(100)
        if CW == True:
            self.pos_motion_start_0x0907(1)
        else:
            self.pos_motion_start_0x0907(2)

    def pos_test_step(self, cw: bool, degrees: float = 0.5, acc_dec_time: int = 200,
                       speed_rpm: int = 10) -> None:
        """One fixed-size nudge in the given direction, via the same
        reliable post_step_motion_by()/_execute_positioning() path as
        HOME/Set Point (clear_alarm_12() precondition satisfied and the
        full 0x0901/0x0902/0x0903/0x0905/0x0906 setup redone every call,
        stillness-based completion) -- not the old bare 0x0907 trigger
        (pos_step_motion_test(), still used by the legacy
        ENABLE POS MODE + POS TEST START CW/CCW web UI flow this replaces
        the trigger of). Confirmed live 2026-09-22: a second bare 0x0907
        trigger, sent without redoing that setup, produced no real motion
        at all -- most likely because whatever makes positioning-test mode
        accept the drive is only satisfied at the moment 0x0901 is written,
        and a later bare trigger with no fresh 0x0901 write doesn't
        re-satisfy it. Redoing the full sequence every press (exactly like
        HOME/Set Point already did) fixed that for those actions, so the
        same pattern is used here instead of trying to keep the two-step
        ENABLE POS MODE + bare-trigger flow working.

        cw picks the sign relative to the CURRENT angle (True: +degrees,
        False: -degrees) -- matches the direction _execute_positioning()
        logs as "CW" for a positive diff_angle. Which physical direction
        that actually is on this drive is unconfirmed (see
        _execute_positioning()'s own comment on the 1/2 mapping).
        Raises PositionUnavailableError, without moving, if the current
        position can't be read."""
        if not self._refresh_current_angle_from_hardware():
            raise PositionUnavailableError(
                "Could not read the current position from the drive; refusing to move."
            )
        delta = degrees if cw else -degrees
        self.post_step_motion_by(self.current_angle + delta, acc_dec_time, speed_rpm)

    def pos_step_motion_by(self, target_pos: int = 0, acc_dec_time=5000, speed_rpm=10):
        base_pulse_per_degree = 349525.3333333333
        # Get Current Encoder Value through the wraparound tracker: target_pos
        # (typically abs_home_pos) is itself a tracker-derived cumulative
        # value, so comparing it with a raw, wrapping reading would break
        # once a wrap has occurred.
        current_pos = self._read_reference_encoder()
        if current_pos is None:
            logging.warning("pos_step_motion_by: no usable position reading; not moving.")
            return 0.0
        logger.info(f"Current Encoder Value: {current_pos}")
        # Set Target Encoder Value
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

        self._execute_positioning(target_pos, low_byte, high_byte, acc_dec_time, speed_rpm)

        # Calculate and return the angle rotated
        angle_rotated = diff_pulses / base_pulse_per_degree
        logging.info(f"Angle Rotated: {angle_rotated}")
        return angle_rotated


    def post_step_motion_by(self, angle: float = 0.0, acc_dec_time: int = 5000, speed_rpm: int =10):
        # 125829120 pulse/rev
        # 349525 + 1/3 pulse/degree
        # 125829120 pulse/rev
        # 349525 + 1/3 pulse/degree
        base_pulse_per_degree = 349525.3333333333

        # Moves to the absolute `angle` by commanding the difference from the
        # position READ FROM THE DRIVE now (see
        # _refresh_current_angle_from_hardware()); refuses, without moving,
        # if it cannot be read. There is no limit on how far one move may go
        # (the 180 degree guard is gone -- 2026-09-21, and 2025-02-05 in this
        # folder before it was ported back by mistake); only a move the drive
        # cannot represent (more than 2^31-1 command pulses, or a non-finite
        # angle) raises MoveOutOfRangeError.
        if not math.isfinite(angle):
            raise MoveOutOfRangeError(f"angle must be a finite number, got {angle!r}.")
        if not self._refresh_current_angle_from_hardware():
            raise PositionUnavailableError(
                "Could not read the current position from the drive; refusing to move "
                "(a move computed from a stale position would land in the wrong place)."
            )
        self.previous_angle = self.current_angle
        self.target_angle = angle
        diff_angle = self.target_angle - self.current_angle

        logger.info(f"Performing motion: Target Angle={self.target_angle}, Previous Angle={self.previous_angle}")

        if diff_angle != 0.0:
            total_pulse = base_pulse_per_degree * abs(diff_angle)
            if total_pulse > MAX_POSITIONING_PULSES:
                raise MoveOutOfRangeError(
                    f"A move of {diff_angle:.1f} deg is {total_pulse:.0f} command pulses; the drive "
                    f"holds at most {MAX_POSITIONING_PULSES}. Nothing was sent."
                )
            integer_pulse = int(total_pulse)
            fractional_pulse = total_pulse - integer_pulse

            '''
            # Accumulate fractional part
            self.float_error += fractional_pulse


            if abs(self.float_error) >= 1.0:
                integer_error = int(self.float_error)
                integer_pulse += integer_error
                self.float_error -= integer_error
            '''

            low_byte = integer_pulse & 0xFFFF
            high_byte = (integer_pulse >> 16) & 0xFFFF

            logger.info(f"Motion Pulses: {integer_pulse}, float_error: {self.float_error}")

            target_encoder = self.current_encoder + (integer_pulse if diff_angle > 0 else -integer_pulse)
            self._execute_positioning(target_encoder, low_byte, high_byte, acc_dec_time, speed_rpm)

    def _execute_positioning(self, target_encoder, low_byte, high_byte, acc_dec_time, speed_rpm):
        # Manual (10) Step 1 for Positioning test (docs/en_manual.txt:10390),
        # identical wording to JOG test's own Step 1 (see
        # enable_speed_ctrl()'s comment): the drive only accepts entering
        # this mode "without any alarm occurrence or Servo ON activated".
        # Confirmed live 2026-09-22 on this project's own COM4 rig: every
        # positioning-test attempt after servo_on() had already been called
        # produced zero real motion (encoder stayed within its noise floor)
        # while the write echoes all came back looking normal -- exactly
        # the "accepted at the wire level but silently ignored" symptom this
        # precondition predicts. Ported from servo_comm_shihlin_unified
        # (commit dc67fda, confirmed on real hardware), which found the same
        # bug for this same _execute_positioning() call. See
        # clear_alarm_12()'s own comment for why that method (not
        # servo_off()) is used to reach "Servo OFF" here.
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

        # Direction is decided from the actual encoder vs. target -- not
        # from the caller's angle/pulse sign -- so both call sites
        # (pos_step_motion_by()'s raw-pulse target, post_step_motion_by()'s
        # degree target) go through one consistent source of truth. Which of
        # 1/2 is really CW vs CCW on this drive is unconfirmed (manual (10)
        # Step 6 says "1: forward (CCW)", "2: reverse (CW)"; this code
        # previously assumed the opposite for logging purposes only).
        if target_encoder > self.current_encoder:
            logger.info("Running Servo CW")
            self.pos_step_motion_test(True)
        else:
            logger.info("Running Servo CCW")
            self.pos_step_motion_test(False)

    def enable_speed_ctrl(self, speed_rpm=100, acc_time=5000, enable=True):
        # Manual (9) Step 1 for JOG test (docs/en_manual.txt:10390 has the
        # identical wording for Positioning test): the drive only accepts
        # entering JOG mode "without any alarm occurrence or Servo ON
        # activated". Ported from servo_comm_shihlin_unified (commit
        # 3ae9f18, confirmed on real hardware) after this session's own
        # COM4 testing showed writes into position-test mode having no
        # effect while Servo was already ON -- see _execute_positioning()'s
        # comment, which has the same fix for the positioning-test path.
        # Deliberately clear_alarm_12(), not servo_off(): servo_off() zeroes
        # the DI bit that keeps Alarm 12 suppressed (see its own "Alarm 12
        # ON!" log line), trading "Servo ON blocks JOG mode" for "Alarm 12
        # blocks JOG mode". clear_alarm_12() sets Servo OFF while keeping
        # that bit set, satisfying both halves of Step 1 at once.
        #
        # enable/acc_time ported from servo_comm_shihlin_unified 2026-09-22:
        # this method previously had no way to leave JOG mode at all (every
        # caller only ever entered it) and no accel/decel control (always
        # whatever the drive already had configured).
        if isinstance(enable, str):
            enable = enable.strip().lower() in ("true", "1", "on", "yes")

        if enable:
            self.clear_alarm_12()
            self.delay_ms(100)
            self.Enable_JOG_Mode(True)
            self.delay_ms(100)
            self.config_acc_dec_0x0902(acc_time)
            self.delay_ms(100)
            self.set_jog_speed(speed_rpm)
            self.delay_ms(100)
            # Explicit stop (0x0904=0) as the LAST step of arming -- found
            # 2026-09-22 on servo_comm_shihlin_unified: 0x0904 (JOG_OPERATION)
            # is a sticky register on this drive family, not reset by
            # (re-)entering JOG mode. If a previous session left it at 1/2
            # (a direction) -- e.g. a MOTION PAUSE that never landed --
            # simply re-arming JOG mode resumed rotation immediately, with
            # no direction ever explicitly pressed this time. Forcing
            # 0x0904=0 here guarantees every arm ends in a definite stopped
            # state regardless of leftover register state.
            # speed_ctrl_action() (not a bare write) so _last_motion_direction
            # is reset too, letting the very next direction press through
            # without needing an extra MOTION PAUSE first.
            self.speed_ctrl_action(0)
            self.delay_ms(100)
            # auto_stop_on_stillness=False: this is continuous JOG/speed-control
            # mode, which runs until explicitly stopped -- a deliberate pause
            # must not be misread as "the move finished" and tear the reading
            # session down. See start_continuous_reading()'s comment.
            self.start_continuous_reading(0.1, auto_stop_on_stillness=False)
        else:
            self.Enable_JOG_Mode(False)
            self.delay_ms(100)
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
        of which input source (web ENABLE SPEED CONTROL MODE / arrow keys)
        set it last. Always call this (not a bare config_speed_0x0903())
        for a JOG speed that should be tracked -- config_speed_0x0903() is
        also used for unrelated things (e.g. positioning-test speed) that
        must NOT overwrite this. Returns the resulting speed. Ported from
        servo_comm_shihlin_unified."""
        new_speed = max(self.JOG_SPEED_MIN_RPM, min(self.JOG_SPEED_MAX_RPM, int(speed_rpm)))
        self.config_speed_0x0903(new_speed)
        self.jog_speed_rpm = new_speed
        return new_speed

    def clear_jog_speed(self) -> None:
        """Forgets the tracked JOG speed -- call whenever JOG mode is torn
        down (enable_speed_ctrl(enable=False), MOTION CANCEL) so a stale
        value doesn't let change_jog_speed_by() appear to succeed, or
        /status report a running speed, for a mode that is no longer
        active. Ported from servo_comm_shihlin_unified."""
        self.jog_speed_rpm = None

    def change_jog_speed_by(self, delta_rpm: int) -> int:
        """Nudges the running JOG speed by delta_rpm (e.g. +1/-1 from an
        arrow-key press) instead of setting an absolute value -- see
        set_jog_speed(). Requires enable_speed_ctrl() to have been called
        with enable=True first -- raises RuntimeError rather than silently
        guessing a starting speed if it hasn't (or if MOTION CANCEL/
        enable=False has since torn the mode down). Returns the resulting
        speed. Ported from servo_comm_shihlin_unified."""
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
        # are always allowed. Ported from servo_comm_shihlin_unified.
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
        response_object = ModbusResponse(response)
        logging.info(response_object)

        if action_value in (0, 1, 2):
            self._last_motion_direction = action_value
        return True

    def set_home_position(self):
        # Read first: a failed read must change nothing (it used to zero the
        # angle and then crash on reset(None)).
        try:
            raw_encoder = self.read_encoder_before_gear_ratio()
        except Exception as e:
            logger.error(f"set_home_position: encoder unreadable ({e}); home NOT set.")
            return
        if raw_encoder is None:
            logger.error("set_home_position: encoder unreadable; home NOT set.")
            return
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.target_angle = 0.0
        self.previous_encoder = self.current_encoder
        self.current_encoder = self._encoder_tracker.reset(raw_encoder)
        self.delay_ms(100)
        self.float_error = 0.0
        self.save_abs_home_pos(self.current_encoder)
        self.home_set_since_start = True
        logger.info("home position set!!!")

    def initial_abs_home(self) -> bool:
        if self.on_initial_home:
            logging.info("Initial absolute home now is running.")
            return False
        
        try:
            self.stop_event.clear()
            self.on_initial_home = True
            
            speed_rpm = 12
            angle_rotated = self.pos_step_motion_by(self.abs_home_pos, 5000, speed_rpm)
            
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

