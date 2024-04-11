import time
import struct
import threading
from serial import SerialException
from status_bit_map import BitMapOutput
from modbus_rtu_client import ModbusRTUClient

class ServoController:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.modbus_client = ModbusRTUClient(device_number=1, serial_port_manager= serial_port)
        self.monitoring_active = False
        self.motion_thread = None

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
        value = int.from_bytes(logic_io_bytes, byteorder='big', signed=False)
        for bit in BitMapOutput:
            if isinstance(bit.value, tuple):
                mask = (1 << (bit.value[1] - bit.value[0] + 1)) - 1
                io_status[bit.name] = (value >> bit.value[0]) & mask
            else:
                io_status[bit.name] = bool(value &(1 << bit.value))
            
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
        thread_args = (commands,)
        self.motion_thread = threading.Thread(target=self.execute_motion_sequence, args = thread_args)
        self.motion_thread.start()

    def start_motion_sequence(self, commands):
        self.monitoring_active = True
        thread_args = (commands,)
        self.motion_thread = threading.Thread(target=self.execute_motion_sequence, args = thread_args)
        self.motion_thread.start()

    def stop_motion_sequence(self):
        self.monitoring_active = False
        if self.motion_thread and self.motion_thread.is_alive():
            self.motion_thread.join()

    def execute_motion_sequence(self, commands):
        for command in commands:
            if not self.monitoring_active:
                break
            try:
                if 'data' in command:
                    data = command['data']
                else:
                    data = b''
                print(f"The command: {command['data']}")
                self.send_servo_command(command['code'], command.get('data', b''))
            except Exception as e:
                print(f"An error occurred: {e}")
                break
        print("Motion sequence completed or stopped.")


    

