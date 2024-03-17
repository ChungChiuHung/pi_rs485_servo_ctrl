import binascii
from enum import Enum

class ProtocolID(Enum):
    RESERVED = 0
    SINGLE_MASTER_PROTOCOL = 1

    @staticmethod
    def get_protocol_id(protocol_header):
        protocol_id_bits = int(protocol_header, 16) >> 5
        return ProtocolID.SINGLE_MASTER_PROTOCOL if protocol_id_bits == 1 else ProtocolID.RESERVED

class CRC16CCITT:
    def __init__(self, poly=0x1021, init_val=0xFFFF):
        self.poly = poly
        self.init_val = init_val

    def append_crc(self, data):
        crc = self.init_val
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ self.poly
                else:
                    crc <<= 1
            crc &= 0xFFFF
        return data + crc.to_bytes(2, 'big')

class MessageGenerator:
    def __init__(self, protocol_header, destination_address, control_code, command_code, parameters):
        """
        Initialize the message generator with the specified components.

        Parameters:
        protocol_header (str): The protocol header hex.
        destination_address (str): The destination address hex.
        control_code (str): The control code hex.
        command_code (str): The command code hex.
        parameters (list): The array of parameter/response data hex values.
        """
        self.protocol_header = protocol_header
        self.protocol_id = ProtocolID.get_protocol_id(protocol_header)

        self.destination_address = destination_address
        self.control_code = control_code
        self.command_code = command_code
        self.parameters = parameters

    def get_data_length_code(self):
        data_length_bits = int(self.protocol_header, 16) & 0x1F # Extract 4th to 0th bits
        data_length = max(2, min(data_length_bits, 31)) # Ensure the data length is within the 2-31 bytes range
        return data_length
        
    def generate_message(self):
        header_data = bytes.fromhex(f"{self.protocol_header}{self.destination_address}")
        data = bytes.fromhex(f"{self.control_code}{self.command_code}") + b''.join(bytes.fromhex(param) for param in self.parameters)
        combined_data = header_data + data

        crc_calculator = CRC16CCITT()
        full_message_with_crc = crc_calculator.append_crc(combined_data)

        #return [self.protocol_header, self.destination_address, self.control_code, self.command_code] + self.parameters + error_detection
        return full_message_with_crc
    
    def print_message_hex(self):
        message_bytes = self.generate_message()
        hex_message = ' '.join([f"{byte:02X}" for byte in message_bytes])
        print(hex_message)