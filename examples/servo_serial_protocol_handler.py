from crc import CRC16CCITT
from servo_command_code import CmdCode

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
    
    def construct_packet(self, destination_address, command_code, data='b', status_name=None, value=None, is_response=False):
        if status_name is not None and value is not None:
            modified_data = self.set_bit_status(status_name, value)
        else:
            modified_data = data

        # Prepend the command code to the modified_data
        modified_data_with_command = bytes([command_code.value]) + modified_data

        # Calculate data length, adding 1 for the command/response floag byte
        data_length = len(modified_data_with_command) + 1

        # Always add 1 to data length for the command/response flag byte
        header = self.construct_header(data_length, destination_address) 
        
        # Set or clear the 7th bit of the first data byte based on message type
        command_response_flag_byte = (0x80 if is_response else 0x00)
        payload = bytes([command_response_flag_byte]) + modified_data_with_command

        error_detection = self.crc.calculate_crc(header + payload)

        return header + payload + error_detection
    
    def set_bit_status(self, status_name, value):
        status_numb = 288
        status_value = 0
        mask_value = 0

        if isinstance(status_name.value,tuple):
            bit_range = status_name.value
            if not(0 <= value <2 ** (bit_range[1] - bit_range[0] + 1)):
                raise ValueError(f"Value for {status_name.name} must be withing the appropriate range.")
            status_value |= (value << bit_range[0])
            mask_value |= ((2 ** (bit_range[1] - bit_range[0] + 1) -1) << bit_range[0])
        else:
            if value not in [0,1]:
                raise ValueError(f"Value for {status_name.name} must be 0 or 1.")
            status_value |= (value << status_name.value)
            mask_value |=(1<<status_name.value)

        # Convert status number, value, and mask to byte array
        status_no_bytes = status_numb.to_bytes(2, byteorder='big')
        status_bytes = status_value.to_bytes(4, byteorder='big')
        mask_bytes = mask_value.to_bytes(4, byteorder='big')

        return status_no_bytes + status_bytes + mask_bytes

