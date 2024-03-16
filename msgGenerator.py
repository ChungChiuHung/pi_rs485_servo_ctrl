import binascii

class MessageGenerator:
    def __init__(self, protocol_header, destination_address, control_code, command_code, parameters):
        self.protocol_header = protocol_header
        self.destination_address = destination_address
        self.control_code = control_code
        self.command_code = command_code
        self.parameters = parameters

    def crc16_ccitt(self, hex_data):
        data = bytes.fromhex(hex_data)
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1)^0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF # Trim CRC to 16 bits
        return crc
    
    def generate_message(self):
        # Combine the header and data parts
        header_data = f"{self.protocol_header}{self.destination_address}"
        data = f"{self.control_code}{self.command_code}{''.join(self.parameters)}"
        combined_data = f"{header_data}{data}"

        # Calculate CRC-16-CCITT for error detection
        crc = self.crc16_ccitt(combined_data)
        error_detection = [hex(crc >> 8)[2:0].zfill(2), hex(crc & 0xFF)[2:].zfill(2)]

        # Construct the full message
        return [self.protocol_header, self.destination_address, self.control_code, self.command_code] + self.parameters + error_detection