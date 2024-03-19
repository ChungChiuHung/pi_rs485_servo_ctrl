from crc import CRC16CCITT
from command_code import CmdCode

class BaseMsgGenerator:
    def __init__(self):
        self.crc_calculator = CRC16CCITT()
        self.toggle_bit = 0 # Initialize toggle bit

    def get_parameter_code(self, command_code_enum,return_as_str=False, **kwargs):
        # Initialize an empty parameter_code list
        parameter_code = []
        # Check command code type and construct parameter code accordingly
        if command_code_enum in [CmdCode.NOP, CmdCode.UNLOCK_PARAM_ALL]:
            # These commands do not require additional parameters
            parameter_code = []
        elif command_code_enum == CmdCode.SET_PARAM_2:
            # Expecting param_group and write_value, both should be 2 bytes
            param_group = kwargs.get('param_group', [])
            write_value = kwargs.get('write_value', [])
            if not (len(param_group) == 2 and len(write_value) == 2):
                raise ValueError("Incorrect parameter length for SET_PARAM_2.")
            parameter_code = param_group + write_value
        elif command_code_enum == CmdCode.SET_PARAM_4:
            # Expecting param_group 2 bytes and write_value 4 bytes
            param_group = kwargs.get('param_group', [])
            write_value = kwargs.get('write_value', [])
            if not (len(param_group) == 2 and len(write_value) == 4):
                raise ValueError("Incorrect parameter length for SET_PARAM_4.")
            parameter_code = param_group + write_value
        elif command_code_enum in [CmdCode.SAVE_PARAM_ALL, CmdCode.GET_PARAM_2, CmdCode.GET_PARAM_4, CmdCode.GET_STATE_VALUE_2, CmdCode.GET_STATE_VALUE_4, CmdCode.CLEAR_EA05_DATA]:
            # These commands expect a single parameter that should be 2 bytes
            parameter = kwargs.get('parameter', [])
            if not (len(parameter) == 2):
                raise ValueError(f"Incorrect parameter length for {command_code_enum.name}.")
            parameter_code = parameter
        elif command_code_enum == CmdCode.READ_EA05_DATA:
            # Expecting alarm_info and fixed, both 2 bytes each
            alarm_info = kwargs.get('alarm_info', [])
            fixed = kwargs.get('fixed', [])
            if not (len(alarm_info) == 2 and len(fixed) == 2):
                raise ValueError("Incorrect parameter length for READ_EA05_DATA.")
            parameter_code = alarm_info + fixed
        elif command_code_enum == CmdCode.READ_EA05_DATA_EX:
            # Expecting alarm, single_turn_data, and multi_turn_data, 2 bytes each
            alarm = kwargs.get('alarm', [])
            single_turn_data = kwargs.get('single_turn_data', [])
            multi_turn_data = kwargs.get('multi_turn_data', [])
            if not (len(alarm) == 2 and len(single_turn_data) == 2 and len(multi_turn_data) == 2):
                raise ValueError("Incorrect parameter length for READ_EA05_DATA_EX.")
            parameter_code = alarm + single_turn_data + multi_turn_data
        elif command_code_enum == CmdCode.SET_STATE_VALUE_WITHMASK_4:
            # Expecting status_number 2 bytes, status_value 4 bytes, and mask 4 bytes
            status_number = kwargs.get('status_number', [])
            status_value = kwargs.get('status_value', [])
            mask = kwargs.get('mask', [])
            if not (len(status_number) == 2 and len(status_value) == 4 and len(mask) == 4):
                raise ValueError("Incorrect parameter length for SET_STATE_VALUE_WITHMASK_4.")
            parameter_code = status_number + status_value + mask
        else:
            # If an unrecognized command code is provided, raise an error
            raise NotImplementedError(f"Command {command_code_enum.name} is not implemented.")

        # Return the parameter code as a list or a hex string based on return_as_str flag
        if return_as_str:
            return ' '.join(format(x, '02X') for x in parameter_code)
        else:
            return parameter_code


    def generate_message(self, protocol_id, destination_address,
                         dir_bit, error_code,
                         cmd_group, cmd_code, parameter_data):
        # A: Protocol Header
        # the data length is the length of C + D + E. 
        data_length = 1 + 1 + len(parameter_data)
        protocol_header = ((protocol_id & 0b111)<<5) | (data_length & 0x1F)

        # B: Destination Address (8-bit)

        # C: Control Code
        # dir_bit is 1 bit, toggole_bit is 1 bit, and error_code is 4 bits.
        control_code = (dir_bit << 7) | (self.toggle_bit << 6) | (error_code & 0x0F)
        self.toggle_bit = 1 - self.toggle_bit # Flip the toggle bit for the next message

        # D: Command Code
        # cmd_group is 2 bits and cmd_code is 6 bits
        command_code = ((cmd_group & 0b11) << 6) | (cmd_code & 0x3F)

        # E: Parameter Code and Response Data
        # A byte array 0~29 bytes
        

        # Combine parts A to E
        message = bytes([protocol_header, destination_address, control_code, command_code]) + bytes(parameter_data)

        # F: CRC (Error detection)
        crc = self.crc_calculator.calculate_crc(message)

        return message + crc