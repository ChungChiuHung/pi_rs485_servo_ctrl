import struct
import threading
import time
import logging
from typing import Union

import serial

from modbus_utils import ModbusUtils
from modbus_command_code import CmdCode
from serial_port_manager import SerialPortManager

logger = logging.getLogger(__name__)


class ModbusRTUClient:
    """Modbus RTU (binary framing + CRC16) counterpart to ModbusASCIIClient.

    Ref: servo_comm_shihlin/modbus_rtu_client.py, adapted here to take raw
    int register addresses (like ModbusASCIIClient.build_read_message) so
    it's a drop-in swap for callers such as absolute_mode_check.py -- the
    original in servo_comm_shihlin instead requires a ServoControlRegistry
    enum member, which doesn't cover PA/PC/PD-group addresses like PA28.

    Exists to test the hypothesis that the physical driver's PC22 protocol
    option is set to a Modbus RTU variant (6/7/8) rather than the ASCII
    variant (0-5) this project otherwise assumes -- see
    docs/servo_comm_shihlin_merge_design.md and docs/en_manual.txt SS9.2.
    """

    def __init__(self, device_number: int, serial_port_manager: SerialPortManager):
        self.device_number = device_number
        self.serial_port_manager = serial_port_manager
        self.crc = ModbusUtils()
        # Serializes send_and_receive() across threads. RS-485 is a shared
        # half-duplex bus and Flask handles requests concurrently -- e.g.
        # the web UI's /status poll (every 2s) landing while an /action
        # handler is mid-sequence through several writes. ServoController's
        # own self.lock doesn't cover most write methods (Enable_Position_Mode,
        # config_*, clear_alarm_12, etc. never acquire it), so two threads'
        # send()/receive() calls could interleave on the wire: one thread's
        # reset_input_buffer() (in send()) could wipe out a response another
        # thread was still waiting for, or a response could be read by the
        # wrong thread's receive() entirely. Confirmed 2026-09-18 via real
        # production logs showing CRC mismatches during exactly this
        # collision (an /action sequence running concurrently with /status's
        # polling reads). Locking here, at the lowest level every call site
        # funnels through, fixes it everywhere at once rather than requiring
        # every current and future ServoController method to remember to
        # take a lock itself.
        self._transaction_lock = threading.Lock()
        # Raw bytes of the most recent transaction, for diagnostics (the web
        # UI's "RS-485 Send/Receive" boxes). Written only from inside send()/
        # receive(), both always called under _transaction_lock via
        # send_and_receive() -- see format_hex() for how callers display these.
        self.last_sent: Union[bytes, None] = None
        self.last_received: Union[bytes, None] = None

    @staticmethod
    def format_hex(data: Union[bytes, None]) -> str:
        """Space-separated uppercase hex, e.g. b'\\x01\\x03' -> '01 03'.
        Empty string for None/empty (no traffic yet, or the last attempt
        got no response) -- never a stale or fabricated value."""
        return ' '.join(f'{b:02X}' for b in data) if data else ''

    def build_read_message(self, address: int, word_length: int) -> bytes:
        data = struct.pack('>H', word_length)
        return self._build_message(CmdCode.READ_DATA.value, address, data)

    def build_write_message(self, address: int, data: int) -> bytes:
        packed = struct.pack('>H', data)
        return self._build_message(CmdCode.WRITE_DATA.value, address, packed)

    def build_write_multiple_message(self, address: int, words) -> bytes:
        """Function 0x10 ("write data, multiple words"): start address, word
        count, byte count, then the words, each big-endian. For a 32-bit
        parameter the drive's word order is [low word][high word] (the same
        order reads come back in)."""
        words = list(words)
        payload = struct.pack('>H', len(words)) + struct.pack('B', len(words) * 2)
        payload += b''.join(struct.pack('>H', w & 0xFFFF) for w in words)
        return self._build_message(CmdCode.WRITE_MULTI_DATA.value, address, payload)

    def _build_message(self, command_code: int, address: int, data: bytes) -> bytes:
        adr = struct.pack('B', self.device_number)
        cmd = struct.pack('B', command_code)
        start_address = struct.pack('>H', address)
        message_without_crc = adr + cmd + start_address + data
        crc = self.crc.calculate_crc(message_without_crc)
        return message_without_crc + crc

    @staticmethod
    def expected_read_response_length(word_length: int) -> int:
        """addr(1) + func(1) + byte_count(1) + data(word_length*2) + crc(2)."""
        return 5 + word_length * 2

    @staticmethod
    def expected_write_response_length() -> int:
        """A write-single-register echo mirrors the request's shape:
        addr(1) + func(1) + address(2) + data(2) + crc(2)."""
        return 8

    def _infer_expected_length(self, message: bytes) -> Union[int, None]:
        """Works out how many bytes a response to `message` should be, from
        the message itself (function code, and for reads, the word_length
        already encoded in it) -- so callers don't have to compute and pass
        expected_length by hand at each of the ~40 call sites in
        servo_control.py. Returns None if the message is too short to
        contain a function code (caller falls back to timeout-based
        receive()).

        This matters beyond convenience: without a known expected_length,
        receive() has no way to tell "this response is complete" from "more
        bytes might still be coming", so it just keeps accumulating
        whatever arrives until the line goes quiet. If an earlier
        transaction left an unread response sitting in the input buffer
        (e.g. a write whose driver-echo was never drained), that leftover
        gets silently concatenated onto this transaction's real response --
        see docs/servo_comm_shihlin_merge_design.md's note on the
        concatenated-frame bug this was written to fix.
        """
        if len(message) < 6:
            return None
        command_code = message[1]
        if command_code == CmdCode.READ_DATA.value:
            word_length = struct.unpack('>H', message[4:6])[0]
            return self.expected_read_response_length(word_length)
        if command_code in (CmdCode.WRITE_DATA.value, CmdCode.WRITE_MULTI_DATA.value):
            return self.expected_write_response_length()
        return None

    def send_and_receive(self, message: bytes, expected_length: int = None,
                          timeout: float = 0.5) -> Union[bytes, None]:
        with self._transaction_lock:
            try:
                self.send(message)
                if expected_length is None:
                    expected_length = self._infer_expected_length(message)
                return self.receive(expected_length, timeout)
            except Exception as e:
                logger.error(f"Error in send_and_receive: {e}")
                return None

    def send(self, message: bytes) -> None:
        if self.ensure_connection():
            try:
                serial_instance = self.serial_port_manager.get_serial_instance()
                # Defensive: discard any bytes still sitting unread from an
                # earlier transaction (e.g. a fire-and-forget write whose
                # echo was never drained) before this new request's
                # response can arrive and get concatenated onto that
                # leftover data.
                serial_instance.reset_input_buffer()
                logger.debug(f"Message sent: {message.hex()}")
                serial_instance.write(message)
                self.last_sent = message
            except serial.SerialException as e:
                logger.error(f"Failed to send message due to serial error: {e}")
            except Exception as e:
                logger.error(f"Unexpected error occurred: {e}")

    def receive(self, expected_length: int = None, timeout: float = 0.5) -> Union[bytes, None]:
        if not self.ensure_connection():
            logger.warning("Connection not established.")
            return None

        response = bytearray()
        start_time = time.time()

        try:
            while True:
                if time.time() - start_time > timeout:
                    if not self.serial_port_manager.get_serial_instance().in_waiting:
                        break
                bytes_to_read = self.serial_port_manager.get_serial_instance().in_waiting
                if bytes_to_read:
                    # Cap the read at what's still needed for THIS response
                    # when the length is known -- in_waiting can already
                    # hold more than one frame's worth of bytes (e.g. if
                    # this call was scheduled late and a subsequent,
                    # unrelated response had time to arrive too), and
                    # reading all of in_waiting unconditionally would pull
                    # that next frame's bytes into this one's response
                    # even though len(response) >= expected_length gets
                    # checked right after -- by then it's already too late,
                    # the extra bytes are already mixed in.
                    if expected_length:
                        bytes_to_read = min(bytes_to_read, expected_length - len(response))
                    response.extend(self.serial_port_manager.get_serial_instance().read(bytes_to_read or 1))
                    if expected_length and len(response) >= expected_length:
                        break
                    start_time = time.time()

            if response:
                logger.debug(f"Response received: {response.hex()}")
                self.last_received = bytes(response)
                return bytes(response)
            logger.warning("No response received.")
            self.last_received = None
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
