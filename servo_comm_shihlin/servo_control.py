import time
import struct
from serial import SerialException
from modbus_ascii_client import ModbusASCIIClient
from servo_control_registers import ServoControlRegistry

class ServoController:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.modbus_client = ModbusASCIIClient(device_number=1, serial_port_manager= serial_port)

    def delay_ms(self, milliseconds):
      time.sleep(milliseconds / 1000.0)

    def print_byte_array_as_spaced_hex(self,byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def send_command_and_wait_for_response(self, command, description, read_timeout=0.1):
        try:
            self.modbus_client.send(command)
            time.sleep(read_timeout)
            response = self.modbus_client.receive()
            if response:
                print(f"{description} response received: {response.hex()}")
            else:
                print(f"Timeout waiting for {description} response.")
            return response
        except SerialException as e:
            print(f"Error during communication: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        return None
    
    def parse_logic_io(self, logic_io_bytes):
        io_status= {}
        return io_status
    
    def send_servo_command(self, command_code, data=b'', bitmap=None, value=None, response_delay=0):
        if bitmap is not None and value is not None:
            command = self.modbus_client.build_write_message(bitmap, struct.pack('>H', value))
        else:
            command = self.modbus_client.build_write_message(command_code, data)
        self.send_command_and_wait_for_response(command, f"{command_code.name}", response_delay)

    def monitor_end_status(self, status_code, interval=0.5, duration=10):
        end_time = time.time() + duration
        while time.time() < end_time and self.monitoring_active:
            status_command = self.modbus_client.build_read_message(status_code, 4)
            status = self.send_command_and_wait_for_response(status_command, "Monitoring Status", interval)
            if status:
                print("Status: ", status.hex())
            time.sleep(interval)

    def execute_motion_start_sequence(self, commands):
        self.monitoring_active = True
        print(f"execute_motion {commands}")

    def start_motion_sequence(self, commands):
        self.monitoring_active = True
        print(f"execute_motion {commands}")

    def stop_motion_sequence(self):
        self.monitoring_active = False

    def execute_motion_sequence(self, commands):
        print(f"execute_motion {commands}")
        

    def enable_control_DI(self, x, y, z, u):
        if any(not(0 <= a <15) for a in [x,y,z,u]):
            raise ValueError("All inputs must be within the range 0 to 15 (inclusive).") 
        value = (x << 12) | (y << 8)| (z << 4) | u

        print(f"Config value: {value}")


    

