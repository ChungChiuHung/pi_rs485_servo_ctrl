import time
import json
import serial
from time import sleep
from colorama import Fore, init
from cal_cmd_response_time import CmdDelayTime
from command_code import CmdCode
from servo_params import ServoParams
from status_bit_mapping import BitMap, BitMapOutput

class SerialPortHandler:
    def __init__(self, baud_rate=57600, timeout=1):
        self.serial_port = None
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.available_ports = ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]

    
    def get_baud_rate(self):
        return self.serial_port.baudrate

    def open_serial(self):
        for port in self.available_ports:
            try:
                self.serial_port = serial.Serial(port, self.baud_rate, timeout=self.timeout)
                self.serial_port.bytesize = serial.EIGHTBITS
                self.serial_port.parity = serial.PARITY_NONE
                self.serial_port.stopbits = serial.STOPBITS_ONE
                print(f"Successfully connected to {port}")
                return True
            except (OSError, serial.SerialException):
                continue
        return False

    def close_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")

    def write(self, data):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(data)

    def read(self, timeout_sec):
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            if self.serial_port and self.serial_port.in_waiting:
                return self.serial_port.read(self.serial_port.in_waiting)
        return b''

class SerialCommunication:
    def __init__(self, port_handler, command_timeout=1):
           init(autoreset=True)
           self.port_handler = port_handler
           self.command_timeout = command_timeout
           self.last_send_message = b''
           self.last_received_message = b''

    def delay_ms(self, milliseconds):
      time.sleep(milliseconds / 1000.0)

    def print_byte_array_as_spaced_hex(self,byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def send_command_and_wait_for_response(self, command, command_description):
        try:
            self.last_send_message = command
            print(f"Sending {command_description}:")
            self.print_byte_array_as_spaced_hex(command, command_description)
            self.ser_port.write(command)

            result = b''
            start_time = time.time()
            while time.time()-start_time < self.command_timeout:
                elapsed_time = time.time() - start_time
                progress = elapsed_time / self.command_timeout
                progress_bar = self.create_progress_bar(progress)
                print(f"\r{command_description} progress: {progress_bar}", end="")

                incoming_data = self.port_handler.read()
                if incoming_data:
                    response += incoming_data
                    break

            if response:
                print(f"\n{command_description} response received")
                self.last_received_message = response
                self.print_byte_array_as_spaced_hex(response, "Response")
            else:
                print(f"\n{command_description} time out waiting for response.")
            return response
        
        except serial.SerialException as e:
            print(f"Error sending command: {e}")
            return None
    

    @staticmethod
    def create_progress_bar(progress, bar_length=30):
        filled_length = int(round(bar_length * progress))
        bar = Fore.GREEN + '█' * filled_length + Fore.RED + '█' * (bar_length - filled_length)
        return bar
    
    #IOStatusFetcher:  
    def parse_logic_io(self, logic_io_bytes):
        io_status= {}
        value = int.from_bytes(logic_io_bytes, byteorder='big')

        for bit in BitMapOutput:
            if isinstance(bit.value, tuple):
                mask = (1 << (bit.value[1] - bit.value[0] + 1)) - 1
                io_status[bit.name] = (value >> bit.value[0]) & mask
            else:
                io_status[bit.name] = bool(value &(1 << bit.value))
            
        return io_status
    
    def get_output_io_status(self):
        get_output_io = ServoParams.GET_OUTPUT_IO
        response_recieved = False
        result, response_recieved = self.send_command_and_wait_for_response(get_output_io, 
                                                                            "GET_IO_STATES")

        if result and len(result) >= 6:
            logic_io_bytes = result[4:8]
            parsed_status = self.parse_logic_io(logic_io_bytes)
            return json.dumps(parsed_status), response_recieved
        
        response_recieved = True

        return (json.dumps({"error" : "No response or invalid response length"}), response_recieved)
    
    def last_send_message(self):
        return ' '.join(f"{byte:02X}" for byte in self._last_send_message)

    def last_received_message(self):
        return ' '.join(f"{byte:02X}" for byte in self._last_received_message)
    
    def get_serial_port(self):
        return self._serial_port
    
    def close(self):
        self._serial_port.close