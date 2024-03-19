from message_generator import MessageGenerator
from base_msg_generator import BaseMsgGenerator
from command_code import CmdCode
from servo_params import ServoParams
from cal_cmd_response_time import CommAnalyzer
from status_bit_mapping import StatusBitPositions
from set_servo_io_status import SetServoIOStatus
import math

def print_byte_array_as_spaced_hex(byte_array, data_name):
    """
    Print a byte array as a spaced hex string.

    :param byte_array: The byte array to be printed.
    """
    hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
    print(f"Hex representation {data_name}: {hex_string}")

def decimal_to_2_bytes(number):
        if not (0 <= number <= 65535):
            raise ValueError("Number out of range. Must be between 0 and 65535.")
        return number.to_bytes(2, byteorder='big')

def test_byte_operation():
    print("=== Test Byte Operation ===")
    print(CmdCode.SET_STATE_VALUE_WITHMASK_4)
    print(CmdCode.SET_STATE_VALUE_WITHMASK_4.name)
    print(CmdCode.SET_STATE_VALUE_WITHMASK_4.value)

    bit_array = [CmdCode.SET_STATE_VALUE_WITHMASK_4.value, CmdCode.GET_PARAM_2.value]

    print_byte_array_as_spaced_hex(bit_array, "bit array")

    int_value = 123455
    byte_length = math.ceil(int_value.bit_length()/8)
    dynamic_byte_array = int_value.to_bytes(byte_length, byteorder='big')

    print("The hex value: ",dynamic_byte_array)
    print_byte_array_as_spaced_hex(dynamic_byte_array, "dynamic byte array")

def test_message_generator():
    print("=== Test BaseMsgGenerator ===")
    base_msg_generator = BaseMsgGenerator()
    # Generate a message.
    # Replace the following example values with your actual values.
    protocol_id = 1 # 1 : Single Master Protocol.
    destination_address = 1
    dir_bit = 0 # 0:Command Message, 1:Response Message  
    #toggle_bit = 0x00 # Set 0 and 1 alternatively every time the host controller sends a command message.
                      # The amplifer sends back the same value in the response message
    error_code = 0 # fixed value (Command Message)
    cmd_group = 0 # 0 : amplifier communications command (Fixed)
    cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value # Page 12- List of Commands
    parameter_data = [0x04, 0x05]  # Example parameter data.Page 12- List of Commands

    message = base_msg_generator.generate_message(protocol_id, destination_address, dir_bit, error_code, cmd_group, cmd_code, parameter_data)

    # Now 'message' contains your structured message including the CRC.
    # print("Generated Message:", message.hex())
    print_byte_array_as_spaced_hex(message, "message")

def test_get_parameter_code():
    message_generator = BaseMsgGenerator()
    nop_param_code = message_generator.get_parameter_code(CmdCode.NOP)

    #param_group = [0x00, 0x24]
    #write_value = [0x09, 0xC4]
    param_group = decimal_to_2_bytes(36)
    write_value = decimal_to_2_bytes(2500)
    set_param_2_code = message_generator.get_parameter_code(CmdCode.SET_PARAM_2,
                                                            param_group = param_group,
                                                            write_value = write_value)
    protocol_id = 1 # 1 : Single Master Protocol.
    destination_address = 1
    dir_bit = 0 # 0:Command Message, 1:Response Message  
    # toggle_bit = 0x00 # Set 0 and 1 alternatively every time the host controller sends a command message.
                      # The amplifer sends back the same value in the response message
    error_code = 0 # fixed value (Command Message)
    cmd_group = 0 # 0 : amplifier communications command (Fixed)
    cmd_code = CmdCode.SET_PARAM_2.value # Page 12- List of Commands
    parameter_data = set_param_2_code
    full_message = message_generator.generate_message(protocol_id, destination_address, dir_bit, error_code, cmd_group, cmd_code, parameter_data)
    

    print_byte_array_as_spaced_hex(nop_param_code, "NOP")
    print_byte_array_as_spaced_hex(set_param_2_code, "parameter code")
    print_byte_array_as_spaced_hex(full_message, "message")

def test_decimal_to_2bytes():
     print("======  decimal to 2-bytes ======")
     params_no = 32
     bytes = decimal_to_2_bytes(params_no)
     print_byte_array_as_spaced_hex(bytes, "Parameter Group ")


if __name__ == "__main__":
    test_byte_operation()
    test_message_generator()
    test_get_parameter_code()
    test_decimal_to_2bytes()
    