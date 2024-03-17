from enum import Enum

class CmdCode(Enum):
    NOP = 0x00
    GET_PARAM_2 = 0x04
    GET_PARAM_4 = 0x05
    SET_PARAM_2 = 0x07
    SET_PARAM_4 = 0x08
    UNLOCK_PARAM_ALL = 0x0A
    SAVE_PARAM_ALL = 0x0B
    GET_STATE_VALUE_2 = 0x10
    GET_STATE_VALUE_4 = 0x11
    READ_EA05_DATA = 0x1E
    CLEAR_EA05_DATA = 0x1F
    READ_EA05_DATA_EX = 0x62
    SET_STATE_VALUE_WITHMASK_4 = 0x66

class CRC16CCITT:
    def __init__(self, poly=0x1021, init_val=0xFFFF):
        self.poly = poly
        self.init_val = init_val

    def calculate_crc(self, data):
        crc = self.init_val
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ self.poly
                else:
                    crc <<= 1
            crc &= 0xFFFF
        return crc.to_bytes(2, 'big')

class MessageGenerator:
    def __init__(self, destination_address, control_code):
        self.destination_address = destination_address
        self.control_code = control_code

    def get_protocol_header(self, parameter_length):
        return (1 + 1 + parameter_length) + 0x20

    def append_crc(self, command_bytes):
        crc_calculator = CRC16CCITT()
        return command_bytes + crc_calculator.calculate_crc(command_bytes)

    def generate_command(self, command_code, parameter_code, return_as_str):
        protocol_header = self.get_protocol_header(len(parameter_code))
        command_bytes = bytes([protocol_header, self.destination_address, self.control_code, command_code]) + bytes(parameter_code)
        command_with_crc = self.append_crc(command_bytes)
        return ' '.join(format(byte, '02X') for byte in command_with_crc) if return_as_str else command_with_crc

    def get_command(self, command_code_enum, return_as_str=True, **kwargs):
        match command_code_enum:
            case CmdCode.NOP | CmdCode.UNLOCK_PARAM_ALL:
                parameter_code = []
            case CmdCode.SET_PARAM_2:
                param_group = kwargs.get('param_group', [])
                write_value = kwargs.get('write_value', [])
                if not (len(param_group) == 2 and len(write_value) == 2):
                    raise ValueError("Incorrect parameter length for SET_PARAM_2.")
                parameter_code = param_group + write_value
            case CmdCode.SET_PARAM_4:
                param_group = kwargs.get('param_group', [])
                write_value = kwargs.get('write_value', [])
                if not (len(param_group) == 2 and len(write_value) == 4):
                    raise ValueError("Incorrect parameter length for SET_PARAM_4.")
                parameter_code = param_group + write_value
            case CmdCode.SAVE_PARAM_ALL | CmdCode.GET_PARAM_2 | CmdCode.GET_PARAM_4 | CmdCode.GET_STATE_VALUE_2 | CmdCode.GET_STATE_VALUE_4 | CmdCode.CLEAR_EA05_DATA:
                parameter = kwargs.get('parameter', [])
                if not (len(parameter) == 2):
                    raise ValueError(f"Incorrect parameter length for {command_code_enum.name}.")
                parameter_code = parameter
            case CmdCode.READ_EA05_DATA:
                alarm_info = kwargs.get('alarm_info', [])
                fixed = kwargs.get('fixed', [])
                if not (len(alarm_info) == 2 and len(fixed) == 2):
                    raise ValueError("Incorrect parameter length for READ_EA05_DATA.")
                parameter_code = alarm_info + fixed
            case CmdCode.READ_EA05_DATA_EX:
                alarm = kwargs.get('alarm', [])
                single_turn_data = kwargs.get('single_turn_data', [])
                multi_turn_data = kwargs.get('multi_turn_data', [])
                if not (len(alarm) == 2 and len(single_turn_data) == 2 and len(multi_turn_data) == 2):
                    raise ValueError("Incorrect parameter length for READ_EA05_DATA_EX.")
                parameter_code = alarm + single_turn_data + multi_turn_data
            case CmdCode.SET_STATE_VALUE_WITHMASK_4:
                status_number = kwargs.get('status_number', [])
                status_value = kwargs.get('status_value', [])
                mask = kwargs.get('mask', [])
                if not (len(status_number) == 2 and len(status_value) == 4 and len(mask) == 4):
                    raise ValueError("Incorrect parameter length for SET_STATE_VALUE_WITHMASK_4.")
                parameter_code = status_number + status_value + mask
            case _:
                raise NotImplementedError(f"Command {command_code_enum.name} is not implemented.")

        return self.generate_command(command_code_enum.value, parameter_code, return_as_str)
