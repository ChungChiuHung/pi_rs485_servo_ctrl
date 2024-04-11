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
        if self.ensure_connection():
            try:
                self.serial_port_manager.get_serial_instance().write(message)
                print("Message sent:", message.hex())
            except serial.SerialException as e:
                print(f"Failed to send message due to serial error: {e}")
            except Exception as e:
                print(f"Unexpected error occurred: {e}")
    
    def receive(self, expected_length = None):
        if self.ensure_connection():
            try:
                response = bytearray()
                while expected_length and len(response) < expected_length:
                    bytes_to_read = self.serial_port_manager.get_serial_instance().inWaiting()
                    response.extend(self.serial_port_manager.get_serial_instance().read(bytes_to_read or 1))
                if not expected_length:
                    response = self.serial_port_manager.get_serial_instance().read.all()

                if response:
                    print("Response received:", response.hex())
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
            print("Serial instance not available. Attempting to reconncect...")
            self.serial_port_manager.connect()
            if not self.serial_port_manager.get_serial_instance():
                print("Failed to establish serial connection.")
                return False
        return True
        
    def set_di_control_source(self, control_bits):
        data = struct.pack('>H', control_bits)
        message = self.build_write_message(ServoControlRegistry.SEL_DI_CONTROL_SOURCE, data)
        self.send(message)

    def set_di_state(self, state_bits):
        data = struct.pack('>H', state_bits)
        message = self.build_write_message(ServoControlRegistry.DI_PIN_CONTROL, data)
        self.send(message)

    def receive_servo_status(self):
        # Placeholder for method to read servo status
        return 0x0001
    
    def set_positioning_test_mode(self):
        self.send(self.build_write_message(ServoControlRegistry.DO_OUTPUT, struct.pack('>H', 0x0004)))

    def set_acc_dec_time(self, time_ms):
        if not 0 <= time_ms <=20000:
            raise ValueError("Acceleration/deceleration time out of range. (0~20000 ms).")
        self.send(self.build_write_message(ServoControlRegistry.POS_SET_ACC, struct.pack('>H', time_ms)))

    def set_jog_speed(self, speed_rpm):
        if not 0 <= speed_rpm <= 3000:
            raise ValueError("JOG speed out of range (0~3000 rpm)")
        self.send(self.build_write_message(ServoControlRegistry.JOG_SPEED, struct.pack('>H', speed_rpm)))

    def set_command_pulses(self, pulses):
        if not 0 <= pulses < 2**31:
            raise ValueError("Commnad pulses out of range (0 to 2^31 -1).")
        high = (pulses >> 16) & 0xFFFF
        low = pulses & 0xFFFF
        self.send(self.build_write_message(ServoControlRegistry.POS_PULSES_CMD_1, struct.pack('>H', low)))
        self.send(self.build_write_message(ServoControlRegistry.POS_PULSES_CMD_2, struct.pack('>H', high)))
    
    def start_positioning_operation(self, direction):
        if direction not in [0, 1, 2]:
            raise ValueError("Invalid direction code (must be 0, 1, or 2).")
        self.send(self.build_write_message(ServoControlRegistry.POS_EXE_MODE, struct.pack('>H', direction)))

    def exit_positioning_mode(self):
        self.send(self.build_write_message(ServoControlRegistry.DO_OUTPUT, struct.pack('>H', 0x0000)))

    def positioning(self, acc_dec_time, jog_speed, command_pulses, direction):
        servo_status = self.receive_servo_status()
        if servo_status != 0x0001:
            print("Servo is not ON or there is an alarm.")
            return False
        self.set_positioning_test_mode()
        self.set_acc_dec_time(acc_dec_time)
        self.set_jog_speed(jog_speed)
        self.set_command_pulses(command_pulses)
        self.start_positioning_operation(direction)
        self.exit_positioning_mode()

        print("Positioning commad sequence completed successfully.")
        return True