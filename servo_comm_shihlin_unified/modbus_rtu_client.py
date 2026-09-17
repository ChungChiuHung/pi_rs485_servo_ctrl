import struct
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

    def build_read_message(self, address: int, word_length: int) -> bytes:
        data = struct.pack('>H', word_length)
        return self._build_message(CmdCode.READ_DATA.value, address, data)

    def build_write_message(self, address: int, data: int) -> bytes:
        packed = struct.pack('>H', data)
        return self._build_message(CmdCode.WRITE_DATA.value, address, packed)

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

    def send_and_receive(self, message: bytes, expected_length: int = None,
                          timeout: float = 0.5) -> Union[bytes, None]:
        try:
            self.send(message)
            return self.receive(expected_length, timeout)
        except Exception as e:
            logger.error(f"Error in send_and_receive: {e}")
            return None

    def send(self, message: bytes) -> None:
        if self.ensure_connection():
            try:
                logger.debug(f"Message sent: {message.hex()}")
                self.serial_port_manager.get_serial_instance().write(message)
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
                    response.extend(self.serial_port_manager.get_serial_instance().read(bytes_to_read or 1))
                    if expected_length and len(response) >= expected_length:
                        break
                    start_time = time.time()

            if response:
                logger.debug(f"Response received: {response.hex()}")
                return bytes(response)
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
