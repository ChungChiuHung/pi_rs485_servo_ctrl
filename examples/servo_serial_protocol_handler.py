from crc import CRC16CCITT

class SerialPotocolHandler:
    PROTOCOL_ID = 1 # Protocol ID is always 1: Single Master Protocol

    def __init__(self, destination_address_range=(1,31)):
        self.destination_address_range = destination_address_range
        self.crc = CRC16CCITT()

    @staticmethod
    def construct_header(data_length, destination_address):
        if not (0 <= data_length <= 31):
            raise ValueError("Data length must be between 0 and 31 bytes.")
        if not (1 <= destination_address <= 31):
            raise ValueError("Destination address must be between 1 and 31.")
        
        # Constructing the header byte
        header_first_byte = (SerialPotocolHandler.PROTOCOL_ID <<5) | data_length
        header_second_byte = destination_address

        return bytes([header_first_byte, header_second_byte])
    
    def construct_packet(self, destination_address, command_code, data, is_response=False):
        data_length = len(data) + 2 # +1 for the command/response flag byte
                                    # +1 for the command_code
        header = self.construct_header(data_length, destination_address) 
        
        # Set or clear the 7th bit of the first data byte based on message type
        command_response_flag_byte = (0x80 if is_response else 0x00)
        command_code_byte = command_code.value
        modified_data = bytes([command_response_flag_byte, command_code_byte]) + data

        error_detection = self.crc.calculate_crc(header + modified_data)

        return header + modified_data + error_detection