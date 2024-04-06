from modbus_utils import ModbusUtils

class ModbusProtocolHandler:
    def __init__(self, destination_address_range=(1,31)):
        self.destination_address_range = destination_address_range
        self.modbus_utils = ModbusUtils()

    def construct_data_packet(self, ADR, CMD, data, address):
        byte_length = len(data)
        crc_bytes = self.modbus_utils.calculate_crc(data)
        data_packet = ADR + CMD + byte_length, address, crc_bytes
    
        