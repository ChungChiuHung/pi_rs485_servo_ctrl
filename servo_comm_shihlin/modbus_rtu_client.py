import struct
from modbus_utils import ModbusUtils
from modbus_command_code import CmdCode

class ModbusRTUClient:
    def __init__(self, device_number):
        self.device_number = device_number
        self.crc = ModbusUtils()

    def build_message(self, command_code: CmdCode, servo_control_registry, word_length):
        adr = struct.pack('B', self.device_number)
        cmd = struct.pack('B', command_code.value)
        start_address = struct.pack('>H', servo_control_registry.value[0])
        data = struct.pack('>H', word_length)

        message_without_crc = adr + cmd + start_address + data

        crc = self.crc.calculate_crc(message_without_crc)
        full_message = message_without_crc + crc

        return full_message


        