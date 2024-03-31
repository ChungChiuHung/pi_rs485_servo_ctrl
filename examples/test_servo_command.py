import json
import random
import time
from servo_serial_protocol_handler import SerialProtocolHandler
from servo_command_code import CmdCode
from status_bit_mapping import BitMap, BitMapOutput
from crc import CRC16CCITT

command_code = CmdCode.NOP
command_format = SerialProtocolHandler()
nop_command = command_format.construct_packet(1,command_code, b'', is_response=True)
print(f"{command_code.name} Command: ", nop_command.hex())

nop_response = command_format.construct_packet(1,command_code, b'', is_response=False)
print(f"{command_code.name} Response: ", nop_response.hex())

command_code = CmdCode.GET_PARAM_2
get_param_2 = command_format.construct_packet(1, command_code, b'\x00\x20', is_response=False)
print(f"{command_code.name} Command:", get_param_2.hex())

get_param_2 = command_format.construct_packet(1, command_code, b'\x00\x20', is_response=True)
print(f"{command_code.name} Response:", get_param_2.hex())

command_code = CmdCode.GET_PARAM_4
get_param_4 = command_format.construct_packet(1, command_code, b'\x02\x8B', is_response=False)
print(f"{command_code.name} Command:", get_param_4.hex())

get_param_4 = command_format.construct_packet(1, command_code, b'\x02\x8B', is_response=True)
print(f"{command_code.name} Response:", get_param_4.hex())

#=========================================
command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
get_io_input_command = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 3, False)
print(f"{command_code.name} Command:", get_io_input_command)
print(f"{command_code.name} Command:", get_io_input_command.hex())

#=========================================
# SET_PARAM_2 command        
command_code = CmdCode.SET_PARAM_2
command_format = SerialProtocolHandler()
set_param_2_command = command_format.construct_packet(1,command_code, b'\x00\x09\x00\x01', is_response=False)
print(f"{command_code.name} Command: ", set_param_2_command.hex())

# SERVO_ON Command
command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
command_format = SerialProtocolHandler()
servo_on_command = command_format.construct_packet(1,command_code, b'\x01\x20\x00\x00\x00\x01\x00\x00\x00\x01', is_response=False)
print(f"{command_code.name} Command: ", servo_on_command.hex())

# Config multiple bits status
def set_multiple_bit_statuses(*args):
    data = 0x00000000  # Initialize data
    
    for status, value in args:
        if isinstance(status.value, tuple):  # If the status represents a range
            start_bit, end_bit = status.value
            bit_length = end_bit - start_bit + 1
            mask = (1 << bit_length) - 1
            data |= ((value & mask) << start_bit)
        else:  # If the status represents a single bit
            if value:  # Only set the bit if the value is True
                data |= (1 << status.value)
    
    return data

def generate_random_bit_statuses():
    statuses = [
        (BitMapOutput.MEND, random.choice([True, False])),
        (BitMapOutput.PAUSED, random.choice([True, False])),
        (BitMapOutput.SREDY, random.choice([True, False])),
        (BitMapOutput.SERVO, random.choice([True, False])),
        (BitMapOutput.POINT_SELECT, random.randint(0, 15))  # Assuming 4 bits for POINT_SELECT
    ]
    return statuses

# Generate a Response From Logic IO Output
def generate_bytes():
    header = [0x26, 0x01, 0x80, 0x11]
    data = 0x00000000  # Starting point for data bytes

    data_value = set_multiple_bit_statuses(*generate_random_bit_statuses())

    data_bytes = data_value.to_bytes(4, byteorder='big')

    crc = CRC16CCITT()

    error_detection = crc.calculate_crc(bytes(header) + data_bytes)

    final_bytes = bytes(header) + data_bytes + bytes(error_detection)

    return final_bytes




byte_sequence = generate_bytes()
print(byte_sequence.hex(' '))

command_code = CmdCode.GET_PARAM_4
result = command_format.response_parser(command_code, byte_sequence)
print(result)





# Initialize a counter or a condition to avoid infinite loops for this example
max_attempts = 30  # Set a reasonable limit for attempts
attempts = 0

while attempts < max_attempts:
    # In a real scenario, you'd fetch or receive new JSON data here
    # data = fetch_json_data() or similar function
    time.sleep(0.5)
    command_code = CmdCode.GET_PARAM_4
    result = command_format.response_parser(command_code, byte_sequence)
    # print(result)
    # Convert the JSON string to a Python dictionary
    data = json.loads(result)
    # Check if "MEND" is false
    if not data['bit_statuses']['MEND']:
        print("MEND is false, continuing...")
        # Insert logic here to wait or fetch new data before the next iteration
        # For example, you might have a time delay or a call to receive new data
        
        attempts += 1  # Increment attempts to simulate data change and avoid infinite loop in this example
    else:
        print("MEND is true, breaking the loop.")
        break
