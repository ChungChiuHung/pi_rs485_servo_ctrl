from crc import CRC16CCITT
from examples.servo_command_code import CmdCode
from set_servo_io_status import SetServoIOStatus

class BaseMsgGenerator:
    def __init__(self):
        self.crc_calculator = CRC16CCITT()
        self.toggle_bit = 0 # Initialize toggle bit

    def generate_message(self, protocol_id, destination_address,
                         dir_bit, error_code,
                         cmd_code, parameter_data):
        # A: Protocol Header
        # the data length is the length of C + D + E. 
        data_length = 1 + 1 + len(parameter_data)
        protocol_header = ((protocol_id & 0b111)<<5) | (data_length & 0x1F)
        print("Protocol Header: ",hex(protocol_header))

        # B: Destination Address (8-bit)

        # C: Control Code
        # dir_bit is 1 bit, toggole_bit is 1 bit, and error_code is 4 bits.
        control_code = (dir_bit << 7) | (self.toggle_bit << 6) | (error_code & 0x0F)
        self.toggle_bit = 1 - self.toggle_bit # Flip the toggle bit for the next message
        print("Control Code: ", hex(control_code))

        # D: Command Code
        # cmd_group is 2 bits and cmd_code is 6 bits
        command_code = cmd_code

        # E: Parameter Code and Response Data
        # A byte array 0~29 bytes
        print("Parameter Code: ",parameter_data)
        

        # Combine parts A to E
        message = bytes([protocol_header, destination_address, control_code, command_code]) + bytes(parameter_data)

        # F: CRC (Error detection)
        crc = self.crc_calculator.calculate_crc(message)

        return message + crc


class MessageCommander:
    def __init__(self, protocol_id, destination_address, dir_bit, error_code):
        self.cmd_generator = BaseMsgGenerator()
        self.set_bit_status = SetServoIOStatus()
        self.protocol_id = protocol_id
        self.destination_address = destination_address
        self.dir_bit = dir_bit
        self.error_code = error_code
    
    def generate_set_state_cmd(self, status_bit, status_value):
        cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
        parameter_data = self.set_bit_status(status_bit, status_value)
        return self.cmd_generator.generate_message(
            self.protocol_id,
            self.destination_address,
            self.dir_bit,
            self.error_code,
            cmd_code,
            parameter_data
        )