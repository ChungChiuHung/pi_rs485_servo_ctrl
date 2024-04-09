import struct
import serial
from modbus_utils import ModbusUtils
from modbus_command_code import CmdCode
from servo_control_registers import ServoControlRegistry
from serial_port_manager import SerialPortManager

class ModbusRTUClient:
    def __init__(self, device_number, serial_port_manager: SerialPortManager):
        self.device_number = device_number
        self.serial_port_manager = serial_port_manager
        self.crc = ModbusUtils()

    
    def build_read_message(self, servo_control_registry, word_length):
        return self._build_message(CmdCode.READ_DATA, servo_control_registry, word_length)
    
    def build_write_message(self, servo_control_registry, data):
        message = self._build_message(CmdCode.WRITE_DATA, servo_control_registry, len(data) // 2, data)
        return message

    def _build_message(self, command_code: CmdCode, servo_control_registry: ServoControlRegistry, word_length= None, data = None):
        adr = struct.pack('B', self.device_number)
        cmd = struct.pack('B', command_code.value)
        start_address = struct.pack('>H', servo_control_registry.value[0])

        if command_code in [CmdCode.WRITE_DATA, CmdCode.WRITE_MULTI_DATA] and data is not None:
            data_section = data
            message_without_crc = adr + cmd + start_address + data_section
        else:
            word_length_section = struct.pack('>H', word_length)
            message_without_crc = adr + cmd + start_address + word_length_section

        crc = self.crc.calculate_crc(message_without_crc)
        full_message = message_without_crc + crc

        return full_message
    
    def send(self, message):
        if not self.serial_port_manager.get_serial_instance():
            print("Serial instance not available. Attempting to reconnect...")
            self.serial_port_manager.connect()
            if not self.serial_port_manager.get_serial_instance():
                print("Failed to establish serial connection.")
                return
        try:
            self.serial_port_manager.get_serial_instance().write(message)
            print("Message sent:", message.hex())
        except serial.SerialException as e:
            print(f"Failed to send message due to serial error: {e}")
        except Exception as e:
            print(f"Unexpected error occurred while sending message: {e}")
    
    def receive(self, expected_length = None):
        if not self.serial_port_manager.get_serial_instance():
            print("Serial instance not avaiable. Attempting to reconnect...")
            self.serial_port_manager.connect()
            if not self.serial_port_manager.get_serial_instance():
                print("Failed to establish serial connection.")
                return None
        try:
            response = bytearray()
            if expected_length:
                while len(response) < expected_length:
                    bytes_to_read = self.serial_port_manager.get_serial_instance().inWaiting()
                    if bytes_to_read:
                        response.extend(self.serial_port_manager.get_serial_instance().read(bytes_to_read))
            else:
                response = self.serial_port_manager.get_serial_instance().read.all()

            if not response:
                print("No response received.")
                return None
            
            print("Response received:", response.hex())
            return response

        except serial.SerialException as e:
            print(f"Failed to receive message due to serial error: {e}")
        except Exception as e:
            print(f"Unexpected error occurred while receiving message: {e}")
            return None