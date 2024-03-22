import time
from time import sleep
from cal_cmd_response_time import CmdDelayTime

class SerialCommunication:
    def __init__(self) -> None:
           pass

    def delay_ms(self, milliseconds):
      seconds = milliseconds / 1000.0 # Convert milliseconds to seconds
      sleep(seconds)

    def print_byte_array_as_spaced_hex(self,byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def send_command_and_wait_for_response(self, ser_port, command, command_description, delay_before_read=50, wait_respnose_timeout_sec=1):
        """
        Send a command to the serial port and wait for a response.
        :param ser_port: The serial port instance.
        :param command: The command bytes to send.
        :param command_description: Description of the command for logging.
        :param total_timeout: Total timeout in milliseconds to wait for a response.
        :return: Ther esponse bytes read from the serail port
        """
        print(f"Start Sending Command:{command_description}")
        self.print_byte_array_as_spaced_hex(command, f"{command_description}:")
        ser_port.write(command)

        print("Waiting for response...")
        self.delay_ms(delay_before_read)
        current_baud_rate = ser_port.baudrate
        start_time = time.time()

        cmd_delay_time = CmdDelayTime(current_baud_rate)
        cmd_delay_time.calculate_transmission_time_ms(command)    
 
        result = b''
        
        while True:
            elapsed_time = time.time() - start_time
            remaining_time = wait_respnose_timeout_sec - elapsed_time
            if remaining_time <= 0:
                print("Response waiting time expired.")
                break
            print("Remaining time: {remaining_time:.2f} seconds")
            waiting_bytes = ser_port.inWaiting()
            if waiting_bytes > 0:
                result += ser_port.read(waiting_bytes)
                break
            self.delay_ms(delay_before_read) # Check periodically

        print("Original Data: ", result)
        self.print_byte_array_as_spaced_hex(result, f"{command_description} Response hex: ")

        return result


    