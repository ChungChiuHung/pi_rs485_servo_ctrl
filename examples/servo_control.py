import json
import time
from colorama import Fore, init
from cal_cmd_response_time import CmdDelayTime
from crc import CRC16CCITT
from servo_params import ServoParams
from set_servo_io_status import SetServoIOStatus
from status_bit_mapping import BitMap, BitMapOutput

class ServoCntroller:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.cal_command_time_delay = CmdDelayTime(self.serial_port.baud_rate)
        self.last_send_message = b''
        self.last_received_message = b''

    @staticmethod
    def delay_ms(self, milliseconds):
      time.sleep(milliseconds / 1000.0)

    @staticmethod
    def print_byte_array_as_spaced_hex(self,byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def find_start_marker(self, start_marker):
        previous_byte = None
        while True:
            byte = self.serial_port.read(1)
            if not byte:
                return False
            if previous_byte is not None and previous_byte + byte == start_marker:
                return True
            previous_byte = byte

    def send_command_and_wait_for_response(self, command, command_description, read_timeout=0.1):
        if not self.serial_port:
            print("Serial port is not open.")
            return None
        
        self.last_send_message = command
        print(f"Sending {command_description}:")
        self.print_byte_array_as_spaced_hex(command, command_description)

        try:
            self.serial_port.write(command)
            response = b''

            command_transmission_time_ms = self.cal_command_time_delay.calculate_transmission_time_ms(command)
            total_timeout = command_transmission_time_ms + read_timeout  
            start_time = time.time()

            while time.time()-start_time <  total_timeout :
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

        except self.serial_port.SerialException as e:
            print(f"Error during communication: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        
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
   