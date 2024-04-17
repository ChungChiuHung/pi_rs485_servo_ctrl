import struct
import serial
import time
from modbus_utils import ModbusUtils
from modbus_command_code import CmdCode
from servo_control_registers import ServoControlRegistry
from serial_port_manager import SerialPortManager

class ModbusASCIIClient:
    def __init__(self, device_number, serial_port_manager: SerialPortManager):
        self.device_number = device_number
        self.serial_port_manager = serial_port_manager
        self.lrc = ModbusUtils()

    def build_read_message(self, servo_control_registry, word_length):
        address = servo_control_registry
        data = struct.pack('>HH', address, word_length)
        return self._build_message(CmdCode.READ_DATA.value, data)

    def build_write_message(self, servo_control_registry, data):
        address = servo_control_registry
        print(f"Address : {struct.pack('>H',address)}")
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

    def send_and_receive(self, message, expected_length = None, timeout = 0.1):
        self.send(message)
        return self.receive(expected_length, timeout)

    def send(self, message):
        if self.ensure_connection():
            try:
                self.serial_port_manager.get_serial_instance().write(message)
                print("Message sent:", message)
            except serial.SerialException as e:
                print(f"Failed to send message due to serial error: {e}")
            except Exception as e:
                print(f"Unexpected error occurred: {e}")

    def receive(self, expected_length=None, timeout=0.1):
        if self.ensure_connection():
            try:
                response = bytearray()
                start_time = time.time()

                while True:
                    if expected_length and len(response) >= expected_length:
                        break
                    if not expected_length and time.time() - start_time > timeout:
                        if not self.serial_port_manager.get_serial_instance().in_waiting:
                            break

                    bytes_to_read = self.serial_port_manager.get_serial_instance().in_waiting
                    if bytes_to_read:
                        response.extend(self.serial_port_manager.get_serial_instance().read(bytes_to_read or 1))
                        start_time = time.time()

                if response:
                    print("Response received:", response)
                    print("Parsing Response: ", self.parse_response(response))
                    return response
                else:
                    print("No response received.")
            except serial.SerialException as e:
                print(f"Failed to receive message due to serial error: {e}")
            except Exception as e:
                print(f"Unexpected error occurred while receiving message: {e}")
                return None
            
    def ensure_connection(self):
        if not self.serial_port_manager.get_serial_instance():
            print("Serial instance not available. Attempting to reconnect...")
            if not self.serial_port_manager.connect():
                print("Failed to establish serial connection.")
                return False
        return True
    
    def parse_response(self, response):
        if isinstance(response, (bytes, bytearray)):
            response = response.decode('utf-8')
        
        if response[0] != ':' or response[-2:] != '\r\n':
            raise ValueError("Invalid response: Does not start with ':'")
        
        response = response[1:-2]

        adr = response[0:2]
        cmd = response[2:4]

        cmd_value = int(cmd, 16)

        if cmd_value == CmdCode.READ_DATA.value:
            data_count = response[4:6]
            data_length = int(data_count, 16) * 2

            data_start_idx = 6
            data_end_idx = 6 + data_length
            data = response[data_start_idx:data_end_idx]

            data_addresses = [data[i:i+4] for i in range(0, len(data), 4)]

            lrc = response[data_end_idx:data_end_idx+2]

            parsed_response ={
                "STX": ':',
                "ADR":adr,
                "CMD":cmd,
                "Data Count":data_count,
                "Data":data_addresses,
                "LRC":lrc,
                "End1": '\r',
                "End0": '\n'
            }
            return parsed_response
        
        elif cmd == CmdCode.WRITE_DATA.value:
            start_address = response[4:8]  # 2 bytes for start address, 4 hex digits
            data_content = response[8:12]  # 2 bytes for data count or data content, 4 hex digits

            lrc = response[-2:]  # 1 byte for LRC, 2 hex digits

            parsed_response = {
                "STX": ':',
                "ADR": adr,
                "CMD": cmd,
                "Start Address": start_address,
                "Data Content": data_content,
                "LRC": lrc,
                "End1": '\r',
                "End0": '\n'
            }
            return parsed_response
            
        elif cmd == CmdCode.WRITE_MULTI_DATA.value:
            start_address = response[4:8]  # 2 bytes for start address, 4 hex digits
            data_count = response[8:12]  # 2 bytes for data count or data content, 4 hex digits

            lrc = response[-2:]  # 1 byte for LRC, 2 hex digits

            parsed_response = {
                "STX": ':',
                "ADR": adr,
                "CMD": cmd,
                "Start Address": start_address,
                "Data Count or Content": data_count,
                "LRC": lrc,
                "End1": '\r',
                "End0": '\n'
            }
            return parsed_response
        else:
            raise ValueError(f"Unsupported comand code: {cmd}")

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