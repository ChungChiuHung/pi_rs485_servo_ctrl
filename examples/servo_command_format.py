class CommandFormat:
    PROTOCOL_ID = 1 # Protocol ID is always 1: Single Master Protocol

    def __init__(self, destination_address_range=(1,31)):
        self.destination_address_range = destination_address_range

    @staticmethod
    def construct_header(data_length, destination_address):
        if not (0 <= data_length <= 31):
            raise ValueError("Data length must be between 0 and 31 bytes.")
        if not (1 <= destination_address <= 31):
            raise ValueError("Destination address must be between 1 and 31.")
        
        # Constructing the header byte
        header_first_byte = (CommandFormat.PROTOCOL_ID <<5) | data_length
        header_second_byte = destination_address
        return bytes([header_first_byte, header_second_byte])
    
    def construct_command(self, destination_address, data):
        if len(data) != 2:
            raise ValueError("NOP command data must be 2 bytes")
        
        header = self.construct_header(len(data), destination_address)

        error_detection = bytes([sum(data)%256])

        return header + data + error_detection