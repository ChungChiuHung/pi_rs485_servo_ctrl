import json
import time
import threading
from serial import SerialException
from cal_cmd_response_time import CmdDelayTime
from servo_params import ServoParams
from status_bit_mapping import BitMap, BitMapOutput
from servo_command_code import CmdCode
from servo_serial_protocol_handler import SerialProtocolHandler

class ServoController:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.cal_command_time_delay = CmdDelayTime(self.serial_port.baudrate)
        self.command_format = SerialProtocolHandler()
        self._last_send_message = b''
        self._last_received_message = b''
        self.monitoring_active = False
        self.motion_thread = None

    def delay_ms(self, milliseconds):
      time.sleep(milliseconds / 1000.0)

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

    def send_command_and_wait_for_response(self, command, description, read_timeout=0.1):
        if not self.serial_port:
            print("Serial port is not open.")
            return None
        
        print(f"Sending {description}: {command.hex()}")
        self._last_send_message = command
        start_time = time.time()
        # self.print_byte_array_as_spaced_hex(command, description)

        response = b''
        try:
            self.serial_port.write(command)
            command_transmission_time_ms = self.cal_command_time_delay.calculate_transmission_time_ms(command)
            total_timeout = command_transmission_time_ms + read_timeout  
            start_time = time.time()

            while time.time()-start_time <  total_timeout:
                if self.serial_port.in_waiting > 0:
                    response += self.serial_port.read(self.serial_port.in_waiting)

                #elapsed_time = time.time() -start_time
                #print(self.create_progress_bar(elapsed_time/total_timeout), end='\r')

            self._last_received_message = response

            if response:
                print(f"\n{description} response received: {response.hex()}")
            else:
                print(f"\nTimeout waiting for {description} response.")
            return response

        except SerialException as e:
            print(f"Error during communication: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        
        return response
    
    @property
    def last_send_message(self):
        return ' '.join(f"{byte:02X}" for byte in self._last_send_message)

    @property
    def last_received_message(self):
        return ' '.join(f"{byte:02X}" for byte in self._last_received_message)

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


    def send_servo_command(self, command_code, data=b'', bitmap=None, value=None, response_delay=0.05):
        if bitmap is not None and value is not None:
            command_packet = self.command_format.construct_packet(1, command_code, data, bitmap, value, is_response=False)
        else:
            command_packet = self.command_format.construct_packet(1, command_code, data, is_response=False)
        self.send_command_and_wait_for_response(command_packet, f"{command_code.name}", response_delay)

    def monitor_end_status(self):
        print("Monioring 'MEND' status...")
        while self.monitoring_active:
            #response = self.send_servo_command(CmdCode.GET_STATE_VALUE_4, b'\x01\x28')
            command_code = CmdCode.GET_STATE_VALUE_4
            get_io_output_state = self.command_format.construct_packet(1,command_code, b'\x01\x28', is_response=False)
            response = self.send_command_and_wait_for_response(get_io_output_state, f"{command_code.name}", 0.05)

            if response:
                parsed_response = self.command_format.response_parser(CmdCode.GET_STATE_VALUE_4, response)
                data = json.loads(parsed_response)
                if not data['bit_statuses']['MEND']:
                    print("MEND is false, continuing...")
                else:
                    print("MEND is true, breaking the loop.")
                    break
            else:
                print("Failed to receive a valid response. Retrying...")

    def execute_motion_start_sequence(self, points):
        print("Executing motion start sequence...")
        # SET_PARM_2 command
        self.send_servo_command(CmdCode.SET_PARAM_2, b'\x00\x09\x00\x01')
        # SERVO ON
        self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, b'\x01\x20\x00\x00\x00\x01\x00\x00\x00\x01')

        for point in points:
            print(f"POINT {point}")
            self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, bitmap=BitMap.SEL_NO, value=point)

            print("START")
            self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, bitmap=BitMap.START1, value=0)
            self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, bitmap=BitMap.START1, value=1)
            # Immediately after setting start motion to 1, monitor "MEND" status
            self.monitor_end_status()

    def execute_motion_stop_sequence(self):
        print("Executing motion stop sequence...")
        print(f"Selecting Home POS")
        self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, bitmap=BitMap.SEL_NO, value=1)
        print("Homing...")
        self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, bitmap=BitMap.START1, value=0)
        self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, bitmap=BitMap.START1, value=1)
        # Immediately after setting start motion to 1, monitor "MEND" status
        self.monitor_end_status()
        self.send_servo_command(CmdCode.SET_STATE_VALUE_WITHMASK_4, b'\x01\x20\x00\x00\x00\x00\x00\x00\x00\x01')

    def stop_monitoring(self):
        self.monitoring_active = False

    def execute_motion_sequence_thread(self, points):
        while self.monitoring_active:
            self.execute_motion_start_sequence(points)
            time.sleep(0.1)

    def start_motion_sequence(self, points):
        self.monitoring_active = True
        if self.motion_thread is not None and self.motion_thread.is_alive():
            self.stop_motion_sequence()

        self.motion_thread = threading.Thread(target=self.execute_motion_sequence_thread, args=(points,))
        self.motion_thread.start()

    def stop_motion_sequence(self):
        self.monitoring_active = False
        if self.motion_thread:
            self.motion_thread.join()

        self.execute_motion_stop_sequence()

    

