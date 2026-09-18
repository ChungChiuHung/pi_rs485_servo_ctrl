import time
import logging
import json
import threading
from typing import Union, Callable
from threading import Thread, Event
from serial import SerialException
from modbus_rtu_client import ModbusRTUClient
from modbus_rtu_response import ModbusRTUResponse
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
        self.completed_tag = False
        self.completed_cnt = 0
        self.abs_home_pos = self.load_abs_home_pos()
        self._event_listeners = {
            "on_motion_completed": [],
            "on_moving": [],
            "on_cancel": [],
        }

    def load_abs_home_pos(self) -> int:
        default = self.profile["abs_home_pos_default"]
        try:
            with open(self.config_file, 'r') as file:
                config = json.load(file)
            return config.get("abs_home_pos", default)
        except FileNotFoundError:
            logging.warning(f"Config file {self.config_file} not found; using profile default.")
            return default
        except json.JSONDecodeError as e:
            logging.error(f"Error parsing configuration file: {e}.")
            return default

    def save_abs_home_pos(self, abs_home_pos: int):
        try:
            with open(self.config_file, 'w') as file:
                json.dump({"abs_home_pos": abs_home_pos}, file)
            logging.info(f"Saved abs_home_pos: {abs_home_pos} to {self.config_file}")
        except Exception as e:
            logging.error(f"Error saving abs_home_pos: {e}")

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
    def start_continuous_reading(self, interval: float = 0.1) -> None:
        with self.lock:
            if self.reading_active:
                self.stop_continuous_reading()
                return

            self.read_thread_stop_event.clear()
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
                self.read_thread.join()

            self.read_thread = None
            self.completed_cnt = 0
            self.completed_tag = False
            if self.on_initial_home:
                self.on_initial_home = False
            logging.info("Motion Completed Signal Reading Stopped.")
            self._notify_event_listeners("on_motion_completed")
            self.stop_event.set()

    def cancel_continuous_reading(self) -> None:
        """Like stop_continuous_reading(), but also takes one more encoder
        reading afterward and fires on_cancel with the resulting angle.
        Used by the OSC/Art-Net "cancel loop" control (design doc §2.2 #1).
        Calls stop_continuous_reading() instead of duplicating its body
        (servo_comm_shihlin_50W's version was a copy-paste of it)."""
        self.stop_continuous_reading()
        with self.lock:
            self.delay_ms(50)
            raw_encoder = self.read_encoder_before_gear_ratio()
            if raw_encoder is None:
                logging.warning("cancel_continuous_reading: empty encoder response.")
                return
            self.current_encoder = self._encoder_tracker.update(raw_encoder)
            logging.info(f"Current Encoder Value: {self.current_encoder} (raw: {raw_encoder})")
            self.current_angle = round(
                (self.current_encoder - self.abs_home_pos) / self.base_pulse_per_degree, 4
            )
            logging.info(f"Current Angle: {self.current_angle}")
            self._notify_event_listeners("on_cancel", self.current_angle)

    def _read_continuously(self, interval: float) -> None:
        while not self.read_thread_stop_event.is_set():
            if not self.serial_port.keep_running:
                logger.info("Reconnection attempts stopped.")
                break

            # 1) Read "motion completed" flag
            # Holds self.lock across the wire I/O (not just the state
            # writes below) so a concurrent call to a method that also
            # talks to the modbus client from another thread (e.g. a web UI
            # status-polling endpoint calling read_current_alarm_code())
            # can't interleave its request/response with this loop's on the
            # same serial line. RTU has no built-in transaction ID to tell
            # interleaved responses apart.
            try:
                with self.lock:
                    self.completed_tag = self.Read_Motion_Completed_Signal()
            except Exception as e:
                logger.warning(f"Failed to read motion-completed signal ({e}); retrying...")
                self.delay_ms(interval * 1000)
                continue

            # small inter-read delay
            self.delay_ms(100)

            # 2) Read encoder position
            try:
                with self.lock:
                    encoder = self.read_encoder_before_gear_ratio()
            except Exception as e:
                logger.warning(f"Failed to read encoder ({e}); retrying...")
                self.delay_ms(interval * 1000)
                continue

            if encoder is None:
                logger.warning("Empty encoder response; retrying...")
                self.delay_ms(interval * 1000)
                continue

            # 3) Process valid encoder reading (unwrapped -- see
            # encoder_pulse_tracker.py / docs/servo_comm_shihlin_merge_design.md
            # §2.4 "Plan C" for why the raw 0x0000 register can't be trusted
            # directly). current_angle/current_encoder are only ever updated
            # here, from real feedback -- never overwritten by the command
            # path (closed-loop basis for post_step_motion_by(), design doc
            # §2.2 #4).
            with self.lock:
                self.current_encoder = self._encoder_tracker.update(encoder)
                logger.info(f"Current Encoder Value: {self.current_encoder} (raw: {encoder})")
                diff_angle = round(
                    (self.current_encoder - self.abs_home_pos) / self.base_pulse_per_degree, 4
                )
                self.current_angle = diff_angle
            logger.info(f"Diff Angle: {diff_angle}")
            self._notify_event_listeners("on_moving", diff_angle)

            # 4) Check for motion-complete bursts
            if self.completed_tag:
                self.completed_cnt += 1
                if self.completed_cnt > 6:
                    logger.info(f"Motion Completed Signal Detected: {self.completed_cnt}")
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

    def read_PA33_Encoder_ABS_Pos(self):
        message = self.modbus_client.build_read_message(0x0340, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logger.info(response_object)

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
        message = self.modbus_client.build_write_message(0x0338, 1)
        try:
            response = self.modbus_client.send_and_receive(message)
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
        message = self.modbus_client.build_read_message(0x0200, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

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
        message = self.modbus_client.build_read_message(0x0901, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def read_PF82(self):
        logging.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
        message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(self.response)
        logger.info(response_object.get_value())

    def write_PF82(self, execute_PATH_value: int = 0):
        """Writes and controls the PATH execution.

        Parameters:
            execute_PATH_value (int): The PATH number to execute (1~63)
        """
        logging.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
        if execute_PATH_value < 0 or execute_PATH_value > 9999:
            raise ValueError("execute_PATH_value must be between 0 and 9999.")
        if execute_PATH_value >= 64 and execute_PATH_value < 1000:
            logging.info("Value out of acceptable range.")

        message = self.modbus_client.build_write_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)

        try:
            response_object = ModbusRTUResponse(self.response)
            logging.info(f"Parsed Mobus Response: {response_object}")
        except Exception as e:
            logging.info(f"An unexpected error occurred: {e}")

    def Read_Pos_Related_Paremters(self):
        read_address_array = [PA.STY, PA.HMOV, PA.PLSS,
                               PA.ENR, PA.PO1H, PA.POL,
                               PD.SDI, PD.ITST, PD.MCOK]

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

    def read_encoder_before_gear_ratio(self):
        message = self.modbus_client.build_read_message(0x0000, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)
        encoder_value = response_object.get_value()
        if encoder_value is not None:
            return int(encoder_value)
        return None

    def read_encoder_after_gear_ratio(self):
        message = self.modbus_client.build_read_message(0x0024, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusRTUResponse(response)
        logging.info(response_object)

    def pos_step_motion_test(self, CW=True):
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
        raw_pos = self.read_encoder_before_gear_ratio()
        if raw_pos is None:
            logging.warning("pos_step_motion_by: empty encoder response; not moving.")
            return 0.0
        with self.lock:
            self.current_encoder = self._encoder_tracker.update(raw_pos)
            current_pos = self.current_encoder
        logger.info(f"Current Encoder Value: {current_pos}")

        diff_pulses = target_pos - current_pos

        if abs(diff_pulses) >= self.base_pulse_per_degree * 180:
            logging.info(
                "Target position change is 180 degrees or more; refusing to "
                "move (see docs/servo_comm_shihlin_merge_design.md §2.2 #7)."
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

    def post_step_motion_by(self, angle: float = 0.0, acc_dec_time: int = 5000, speed_rpm: int = 10):
        with self.lock:
            self.previous_angle = self.current_angle
            self.target_angle = angle
            diff_angle = self.target_angle - self.current_angle

        logger.info(
            f"Performing motion: Target Angle={self.target_angle}, "
            f"Previous Angle={self.previous_angle}"
        )

        if abs(diff_angle) >= 180:
            logger.warning(
                f"Target angle change ({diff_angle} deg) is 180 degrees or "
                "more; refusing to move (see "
                "docs/servo_comm_shihlin_merge_design.md §2.2 #7)."
            )
            return

        if diff_angle != 0.0:
            total_pulse = self.base_pulse_per_degree * abs(diff_angle)
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
        if enable == True:
            self.Enable_Position_Mode(False)
        else:
            self.config_speed_0x0903(speed_rpm)
            self.delay_ms(100)
            self.config_acc_dec_0x0902(acc_time)
            self.delay_ms(100)
            self.Enable_Position_Mode(True)
        self.delay_ms(100)
        self.start_continuous_reading(0.1)

    # 0: Stop
    # 1: CW
    # 2: CCW
    def speed_ctrl_action(self, action_value):
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

    def set_home_position(self):
        with self.lock:
            self.current_angle = 0.0
            self.previous_angle = 0.0
            self.target_angle = 0.0
            self.previous_encoder = self.current_encoder
            self.float_error = 0.0
            self.accumulate_pulse = 0
        raw_encoder = self.read_encoder_before_gear_ratio()
        with self.lock:
            self.current_encoder = self._encoder_tracker.reset(raw_encoder)
        self.delay_ms(100)
        self.save_abs_home_pos(self.current_encoder)
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
