import time
from time import sleep
from colorama import Fore, Style, init
from cal_cmd_response_time import CmdDelayTime

class SerialCommunication:
    def __init__(self,ser_port,delay_before_read, wait_response_timeout_sec):
           init(autoreset=True)
           self.ser_port = ser_port
           self.delay_before_read = delay_before_read
           self.wait_response_timeout_sec = wait_response_timeout_sec

    def delay_ms(self, milliseconds):
      seconds = milliseconds / 1000.0 # Convert milliseconds to seconds
      sleep(seconds)

    def print_byte_array_as_spaced_hex(self,byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def send_command_and_wait_for_response(self, command, command_description):
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
        self.ser_port.write(command)

        print("Waiting for response...")
        self.delay_ms(self.delay_before_read)
        current_baud_rate = self.ser_port.baudrate

        bar_length = 30 # Define the progress bar length
        start_time = time.time()

        cmd_delay_time = CmdDelayTime(current_baud_rate)
        cmd_delay_time.calculate_transmission_time_ms(command)
        response_received = False
 
        result = b''
        
        while True:
            elapsed_time = time.time() - start_time
            remaining_time = self.wait_response_timeout_sec - elapsed_time
            if remaining_time <= 0:
                break

            filled_length = int(round(bar_length * (elapsed_time / self.wait_response_timeout_sec)))
            bar = Fore.GREEN + '█' * filled_length + Fore.RED + '█' * (bar_length - filled_length)

            # Print the progress bar with remaining time
            print(f"\rRemaining time: {max(0, remaining_time):.2f} seconds [{bar}]", end='', flush=True)

            waiting_bytes = self.ser_port.inWaiting()
            if waiting_bytes > 0:
                result += self.ser_port.read(waiting_bytes)
                break
            self.delay_ms(self.delay_before_read) # Check periodically

        # Keep the bar at 100% after finishing the countdown
        bar = Fore.GREEN + '█' * bar_length
        print(f"\rRemaining time: 0.00 seconds [{bar}]", end='', flush=True)

        print("\nResponse received:")
        self.print_byte_array_as_spaced_hex(result, f"{command_description} Response hex: ")

        response_received = True

        return (result , response_received)


    