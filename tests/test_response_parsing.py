from test_crc import CRC16CCITT
from test_status_bit_mapping import BitMap
from test_command_code import CmdCode
import json

class ResponseMsgParser:
    def __init__(self):
        self.crc_calculator=CRC16CCITT()

    def print_byte_array_as_spaced_hex(self, byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def parse_bit_statuses(self, parameter_data):
        bit_statuses = {}
        value = int.from_bytes(parameter_data, byteorder='big')

        # Check each bit status
        for status in BitMap:
            if isinstance(status.value, tuple):
                bit_range = status.value
                mask = (1<<(bit_range[1]-bit_range[0] + 1)) -1
                bit_statuses[status.name] = (value >> bit_range[0]) & mask
            else:
                # Single-bit field
                bit_statuses[status.name] = bool(value & (1<< status.value))
        
        return bit_statuses

    def parse_message(self, message):
        if len(message) < 6:
            raise ValueError("Invalid message length")
        
        protocol_header = message[0]
        destination_address = message[1]
        control_code = message[2]
        command_code = message[3] # Extract command code from the message
        parameter_data = message[4:-2] # Everything before the last 2 bytes
                                       # Parameters/Response Data
        received_crc = message[-2:]

        message_without_crc = message[:-2]
        expected_crc = self.crc_calculator.calculate_crc(message_without_crc)

        if received_crc != expected_crc:
            raise json.dumps({"error": "CRC mismatch"})
        
        if control_code != 0x80:
            raise json.dumps({"error":"Invalid control code for response"})

        # Return parsed message components
        response_data = {
            'protocol_header':protocol_header,
            'destination_address': destination_address,
            'control_code': control_code,
            'command_code': command_code,
            'parameter_data': list(parameter_data),
        }

        if command_code == CmdCode.GET_STATE_VALUE_4.value:
            response_data['bit_statuses']  = self.parse_bit_statuses(parameter_data)

        return json.dumps(response_data, indent=4)
    

    
    def generate_response_message(self, protocol_id, destination_address, cmd_code, parameter_data):
        if not (0 <= protocol_id <= 0x07):
            raise ValueError("Invalid protocol ID: must be in 0x00-0x07 range")

        if len(parameter_data) > 29:
            raise ValueError("Parameter data too long: must be 0 to 29 bytes")

        data_length = 1 + len(parameter_data)  # Length of C + E
        protocol_header = ((protocol_id & 0b111) << 5) | (data_length & 0x1F)
        control_code = 0x80  # Fixed value for response messages
        command_code = cmd_code

        # Combine parts A, B, C, and E
        message = bytes([protocol_header, destination_address, control_code, command_code]) + bytes(parameter_data)

        # F: CRC (Error Detection)
        crc = self.crc_calculator.calculate_crc(message)

        full_message = message + crc

        self.print_byte_array_as_spaced_hex(full_message, "Response Message:")

        return full_message