import time
import serial
from colorama import Fore, init
from servo_params import ServoParams
from status_bit_mapping import  BitMapOutput

# Initialize colorama for pretty console outputs
init(autoreset=True)

class SerialCommunication:
    def __init__(self, baud_rate=57600, timeout=1, command_timeout=1):
           self.baud_rate = baud_rate
           self.timeout = timeout
           self.command_timeout = command_timeout
           self.serial_port = None
           self.available_ports = ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]
           self.last_send_message = b''
           self.last_received_message = b''

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
        print("Failed to open any serial port.")
        return False
    
    def close_serial(self):
        if self.serial_port:
            self.serial_port.close()
            print("Serial port closed.")

    @staticmethod
    def print_byte_array_as_spaced_hex(self,byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def send_command_and_wait_for_response(self, command, command_description):
        if not self.serial_port:
            print("Serial port is not open.")
            return None
        
        self.last_send_message = command
        print(f"Sending {command_description}:")
        self.print_byte_array_as_spaced_hex(command, command_description)

        try:
            self.serial_port.write(command)
            response = b''
            start_time = time.time()
            while time.time()-start_time < self.command_timeout:
                if self.serial_port.in_waiting > 0:
                    response += self.serial_port.read(self.serial_port.in_waiting)
                elapsed_time = time.time() -start_time
                self.create_progress_bar(elapsed_time/self.command_timeout)

            self.last_received_message = response

            if response:
                print(f"\n{command_description} response received")
                self.print_byte_array_as_spaced_hex(response, "Response")
            else:
                print(f"\n{command_description} time out waiting for response.")
            return response

        except serial.SerialException as e:
            print(f"Error during communication: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        
        finally:
            pass

        return response
     
    def create_progress_bar(progress, bar_length=30):
        filled_length = int(round(bar_length * progress))
        bar = Fore.GREEN + '█' * filled_length + Fore.RED + '█' * (bar_length - filled_length)
        return bar
    
    @property
    def last_send_message(self):
        return ' '.join(f"{byte:02X}" for byte in self._last_send_message)

    @property
    def last_received_message(self):
        return ' '.join(f"{byte:02X}" for byte in self._last_received_message)
    