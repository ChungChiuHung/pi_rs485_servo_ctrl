import struct
import serial
import time
import threading
import logging
from typing import Union

from modbus_utils import ModbusUtils
from modbus_command_code import CmdCode
from modbus_response import ModbusResponse
from servo_control_registers import ServoControlRegistry
from serial_port_manager import SerialPortManager

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class ModbusASCIIClient:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, device_number = None, serial_port_manager: SerialPortManager = None):
        """Singleton instance creation."""
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(ModbusASCIIClient, cls).__new__(cls)
        return cls._instance
    
    @classmethod
    def get_instance(cls, device_number = None, serial_port_manager = None):
        instance = cls()
        if not hasattr(isinstance, '_is_initialized'):
            instance._initialize(device_number, serial_port_manager)
        return instance    

    def _initialize(self, device_number: int, serial_port_manager: SerialPortManager):
        """Initialize the Modbus ASCII client."""
        if hasattr(self, '_is_initialized') and self._is_initialized:
            # The client is a singleton, so a reconnect (a new
            # SerialPortManager after the port was missing or the profile
            # changed) must re-bind it: otherwise it keeps talking through the
            # old, closed manager forever.
            if serial_port_manager is not None and serial_port_manager is not self.serial_port_manager:
                self.serial_port_manager = serial_port_manager
                logger.info("ModbusASCIIClient re-bound to a new serial port manager.")
            return

        if device_number is None or serial_port_manager is None:
            raise ValueError("Device number and serial port manager must be provided.")
        
        self.device_number = device_number
        self.serial_port_manager = serial_port_manager
        self.lrc = ModbusUtils()
        # The most recent frame sent / received, for the web UI's "Last RS-485
        # transaction" panel (see format_frame()).
        self.last_sent = None
        self.last_received = None
        # Serializes send_and_receive() across threads. RS-485 is a shared
        # half-duplex bus and Flask handles requests concurrently -- e.g. the
        # web UI's status poll landing while an /action handler is mid-move,
        # or (confirmed live 2026-09-22 on this project's own COM4 rig) the
        # continuous-reading background thread's own encoder polling racing
        # the main thread's pos_motion_start_0x0907() trigger write, which
        # start_continuous_reading() deliberately starts just before sending.
        # Without this lock the two threads' send()/receive() calls can
        # interleave on the wire: one thread's write can land between
        # another thread's request and response, or reset_input_buffer() in
        # send() can wipe out a response a different thread was still
        # waiting for. Ported from servo_comm_shihlin_unified's
        # ModbusRTUClient._transaction_lock, which was added there after
        # real production logs showed CRC mismatches from exactly this kind
        # of collision (2026-09-18).
        self._transaction_lock = threading.Lock()
        self._is_initialized = True
        logger.info("ModbusASCIIClient initialized.")

    def build_read_message(self, servo_control_registry: int, word_length: int) -> bytes:
        address = servo_control_registry
        data = struct.pack('>HH', address, word_length)
        return self._build_message(CmdCode.READ_DATA.value, data)

    def build_write_message(self, servo_control_registry: int, data: int) -> bytes:
        address = servo_control_registry
        # print(f"Address : {struct.pack('>H',address)}")
        new_data = struct.pack('>HH', address, data)
        return self._build_message(CmdCode.WRITE_DATA.value, new_data)

    def _build_message(self, command_code, data):
        adr = f'{self.device_number:02X}'
        cmd = f'{command_code:02X}'
        data_hex = data.hex().upper()
        message_without_lrc = f':{adr}{cmd}{data_hex}'
        lrc = self.lrc.calclulate_lrc(bytes.fromhex(message_without_lrc[1:]))
        full_message = f'{message_without_lrc}{lrc:02X}\r\n'
        return full_message.encode('utf-8')

    def send_and_receive(self, message: bytes, expected_length: int = None, timeout:float = 0.1) -> Union[bytes, None]:
        with self._transaction_lock:
            try:
                self.send(message)
                if expected_length is None:
                    expected_length = self._infer_expected_length(message)
                return self.receive(expected_length, timeout)
            except Exception as e:
                logger.error(f"Error in send_and_receive: {e}")
                return None

    @staticmethod
    def _infer_expected_length(message: bytes) -> Union[int, None]:
        """The exact ASCII response length a request calls for, computed
        from the request itself, so receive() can stop the instant that
        many bytes have arrived instead of waiting for a "quiet period"
        with nothing more coming in. No call site had ever passed
        expected_length before this (it defaulted to None everywhere), so
        every single Modbus transaction in this project was paying that
        quiet-period wait -- confirmed live 2026-09-22 that this, not the
        code's explicit delay_ms() pacing, was the dominant cost behind a
        ~1.9s positioning-test button press (each of its ~7 transactions
        was costing ~100-250ms on its own). Ported from
        servo_comm_shihlin_unified's ModbusRTUClient._infer_expected_length(),
        adapted to ASCII framing:
          - WRITE_DATA (0x06): the drive always echoes back the exact
            request frame, a fixed ":"+ADR(2)+CMD(2)+ADDR(4)+DATA(4)+LRC(2)
            +CRLF(2) = 17 bytes.
          - READ_DATA (0x03): ":"+ADR(2)+CMD(2)+BYTECOUNT(2)+DATA(4*N)+
            LRC(2)+CRLF(2) = 11 + 4*word_count bytes, where word_count is
            the request's own word-length field.
        Returns None (falls back to the timeout-based "quiet period" wait)
        for any other command or a malformed/undecodable message -- never
        guesses.
        """
        try:
            text = message.decode('ascii')
        except (UnicodeDecodeError, AttributeError):
            return None
        if len(text) < 13 or text[0] != ':':
            return None
        cmd = text[3:5]
        if cmd == f'{CmdCode.WRITE_DATA.value:02X}':
            return 17
        if cmd == f'{CmdCode.READ_DATA.value:02X}':
            try:
                word_length = int(text[9:13], 16)
            except ValueError:
                return None
            return 11 + 4 * word_length
        return None

    @staticmethod
    def format_frame(frame) -> str:
        """A Modbus ASCII frame as readable text (":010300010002F9"), without
        the trailing CR/LF; "" when there is none yet."""
        if not frame:
            return ""
        if isinstance(frame, (bytes, bytearray)):
            frame = bytes(frame).decode('ascii', errors='replace')
        return frame.strip()

    def send(self, message):
        if self.ensure_connection():
            self.last_sent = message
            try:
                serial_instance = self.serial_port_manager.get_serial_instance()
                # Defensive: discard any bytes still sitting unread from an
                # earlier transaction (e.g. one that timed out, or a stray
                # echo from before this lock existed) before this new
                # request's response can arrive and get concatenated onto
                # that leftover data. Safe under _transaction_lock -- no
                # other thread can be mid-transaction when this runs.
                serial_instance.reset_input_buffer()
                serial_instance.write(message)
                logger.debug(f"Message sent: {message}")
            except serial.SerialException as e:
                logger.error(f"Failed to send message due to serial error: {e}")
            except Exception as e:
                logger.error(f"Unexpected error occurred: {e}")

    def receive(self, expected_length: int = None, timeout: float = 0.1) -> Union[bytes, None]:
        if not self.ensure_connection():
            # print("Connection is not stablished.")
            logger.warning("Connection not established.")
            return None

        response = bytearray()
        start_time = time.time()
        # Absolute deadline, independent of the "quiet period" timer below:
        # that timer resets every time in_waiting is nonzero (see the loop),
        # so a continuous trickle of incoming bytes -- noise, an echo, a
        # slow/garbled response -- can keep this loop running forever with
        # no sleep, holding _transaction_lock the whole time. Confirmed
        # live 2026-09-22 via the web UI: repeated POS TEST START CCW
        # clicks froze the whole server for 6+ minutes -- both the
        # continuous-reading background thread's own polling AND every new
        # /action request stopped making any progress at the exact same
        # moment, consistent with one receive() call never returning and
        # every other Modbus user then blocking on the same lock forever.
        hard_deadline = start_time + timeout + 1.0

        try:
            while True:
                if time.time() > hard_deadline:
                    logger.error(
                        "receive() hit its hard deadline without going quiet; "
                        "giving up on this transaction."
                    )
                    break
                if time.time() - start_time > timeout:
                    if not self.serial_port_manager.get_serial_instance().in_waiting:
                        break
                bytes_to_read = self.serial_port_manager.get_serial_instance().in_waiting
                if bytes_to_read:
                    response.extend(self.serial_port_manager.get_serial_instance().read(bytes_to_read or 1))
                    if expected_length and len(response) >= expected_length:
                        break
                    start_time = time.time()
                else:
                    # Nothing to read yet: this was a tight busy-loop with
                    # no sleep at all, polling in_waiting as fast as the
                    # interpreter could go (confirmed live 2026-09-22 --
                    # this same loop was also the site of the "can run
                    # forever" hang above). A short sleep only on the
                    # "still waiting" path cuts CPU usage dramatically
                    # without adding latency to actually draining a
                    # response: once bytes start arriving, this branch
                    # isn't taken and they're read immediately.
                    time.sleep(0.001)

            if response:
                logger.debug(f"Response received: {response}")
                self.last_received = bytes(response)
                return response
            else:
                logger.warning("No response received.")
                return None
            
        except serial.SerialException as e:
            logger.error(f"Failed to receive message due to serial error: {e}")
        except Exception as e:
            logger.error(f"Unexpected error occurred while receiving message: {e}")
        
        return None
            
    def ensure_connection(self) -> bool:
        if not self.serial_port_manager.get_serial_instance():
            logger.warning("Serial instance not available. Attempting to reconnect...")
            if not self.serial_port_manager.connect():
                logger.error("Failed to establish serial connection.")
                return False
        return True

    def parse_response(self, response: Union[bytes, bytearray]) -> dict:
        if isinstance(response, (bytes, bytearray)):
            response = response.decode('utf-8')
        
        if response[0] != ':' or response[-2:] != '\r\n':
            raise ValueError("Invalid response: Does not start with ':'")
        
        response = response[1:-2]
        adr = response[0:2]
        cmd = response[2:4]
        cmd_value = int(cmd, 16)

        if cmd_value == CmdCode.READ_DATA.value:
            return self._parse_read_response(response)
        elif cmd_value == CmdCode.WRITE_DATA.value:
            return self._parse_write_response(response) 
        elif cmd_value == CmdCode.WRITE_MULTI_DATA.value:
            return self._parse_write_multi_response(response)
        else:
            raise ValueError(f"Unsupported comand code: {cmd}")
        
    def _parse_read_response(self, response: str) -> dict:
        """Parse a Modbus read response."""
        data_count = response[4:6]
        data_length = int(data_count, 16) * 2
        data = response[6:6 + data_length]
        data_addresses = [data[i:i + 4] for i in range(0, len(data), 4)]
        lrc = response[6 + data_length:6 + data_length + 2]

        return {
            "STX": ':',
            "ADR": response[0:2],
            "CMD": response[2:4],
            "Data Count": data_count,
            "Data": data_addresses,
            "LRC": lrc,
            "End1": '\r',
            "End0": '\n'
        }
    
    def _parse_write_response(self, response: str) -> dict:
        return {
            "STX": ':',
            "ADR": response[0:2],
            "CMD": response[2:4],
            "Start Address": response[4:8],
            "Data Content": response[8:12],
            "LRC": response[-2:],
            "End1": '\r',
            "End0": '\n'
        }
    
    def _parse_write_multi_response(self, response: str) -> dict:
        return {
            "STX": ':',
            "ADR": response[0:2],
            "CMD": response[2:4],
            "Start Address": response[4:8],
            "Data Count or Content": response[8:12],
            "LRC": response[-2:],
            "End1": '\r',
            "End0": '\n'
        }

    def set_di_control_source(self, control_bits):
        data = struct.pack('>H', control_bits)
        message = self.build_write_message(ServoControlRegistry.SEL_DI_CONTROL_SOURCE, data)
        self.send(message)

    def set_di_state(self, state_bits):
        data = struct.pack('>H', state_bits)
        message = self.build_write_message(ServoControlRegistry.POS_EXE_MODE, data)
        self.send(message)

    def set_acc_dec_time(self, time_ms):
        if not 0 <= time_ms <= 20000:
            raise ValueError("Acceleration/deceleration time out of range. (0~20000 ms)")
        data = struct.pack('>H', time_ms)
        message = self.build_write_message(ServoControlRegistry.POS_SET_ACC, data)
        self.send(message)

    def set_jog_speed(self, speed_rpm):
        if not 0 <= speed_rpm <= 3000:
            raise ValueError("JOG speed out of range (0~3000 rpm).")
        data = struct.pack('>H', speed_rpm)
        message = self.build_write_message(ServoControlRegistry.JOG_SPEED, data)
        self.send(message)

    def set_command_pulses(self, pulses):
        if not 0 <= pulses < 2 **31:
            raise ValueError("Command pulses out of range (0 to 2^31 -1).")
        high = (pulses >> 16) & 0xFFFF
        low = pulses & 0xFFFF
        data = struct.pack('>HH', low, high)
        message = self.build_write_message(ServoControlRegistry.POS_PULSES_CMD_1, data)
        self.send(message)

    def start_positioning_peration(self, direction):
        if direction not in [0, 1, 2]:
            raise ValueError("Invalid direction code (must be 0, 1, or 2).")
        data = struct.pack('>H', direction)
        message = self.build_write_message(ServoControlRegistry.POS_EXE_MODEe, data)
        self.send(message)

    def exit_positioning_mode(self):
        data = struct.pack('>H', 0x0000)
        message = self.build_write_message(ServoControlRegistry.DO_OUTPUT, data)
        self.send(message)