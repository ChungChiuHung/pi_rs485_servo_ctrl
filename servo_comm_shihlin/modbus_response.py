from enum import Enum
from typing import Union
from modbus_command_code import CmdCode

class ModbusResponse:
    def __init__(self, response):
        if isinstance(response, (bytes, bytearray)):
            response = response.decode('utf-8', errors='replace')

        if len(response) < 5 or response[0] != ':' or response[-2:] != '\r\n':
            raise ValueError("Invalid response: Does not start with ':' or does not end with '\\r\\n'")

        self.stx = ':'
        self.end1 = '\r'
        self.end0 = '\n'
        
        response = response[1:-2]
        self.adr = response[:2]
        self.cmd = response[2:4]

        try:
            self.cmd_value = int(self.cmd, 16)
        except ValueError:
            raise ValueError(f"Invalid command code: {self.cmd}")

        if self.cmd_value == CmdCode.WRITE_DATA.value:
            self._parse_write_data(response)
        elif self.cmd_value == CmdCode.WRITE_MULTI_DATA.value:
            self._parse_write_multi_data(response)
        elif self.cmd_value == CmdCode.READ_DATA.value:
            self._parse_read_data(response)
        else:
            raise ValueError(f"Unsupported command code: {self.cmd}")
        
    def _parse_write_data(self, response):
        """Parse WRITE_DATA command response."""
        self.start_address = response[4:8]
        self.data_content = bytes.fromhex(response[8:12])
        self.lrc = response[-2:]

    def _parse_write_multi_data(self, response):
        """Parse WRITE_MULTI_DATA command response."""
        self.start_address = response[4:8]
        self.data_count = int(response[8:12], 16)
        self.lrc = response[-2:]
    
    def _parse_read_data(self, response):
        """Parse READ_DATA command response"""
        self.data_count = int(response[4:6], 16)
        data_length = self.data_count * 2
        data_start_idx = 6
        data_end_idx = data_start_idx + data_length

        self.data = [response[i:i+2] for i in range(data_start_idx, data_end_idx, 2)]
        self.data_bytes = bytes.fromhex(''.join(self.data))
        self.lrc = response[data_end_idx:data_end_idx+2]

    def __str__(self):
        base_info = (f"Modbus Response:\n"
                     f"  STX: {self.stx}\n"
                     f"  Address: {self.adr}\n"
                     f"  Command: {self.cmd}\n")

        additional_info = ""
        if hasattr(self, 'start_address'):
            additional_info += f" Start Address: {self.start_address}\n"

        if hasattr(self, 'data_bytes'):
            data_str = ', '.join(f"{b:02X}" for b in self.data_bytes)
            decimal_value = self.get_value()
            scaled_value = self.get_scaled_value()

            additional_info += (f"  Data Count: {self.data_count}\n"
                               f"  Data: [{data_str}]\n"
                               f"  Decimal Value: {decimal_value}\n"
                               f"  Scaled Value: {scaled_value}\n"
            )
        else:
            additional_info += " Data: Not applicable for write commands\n"

        additional_info += f" LRC: {self.lrc}\n"
        return base_info + additional_info
    
    def get_value(self) -> Union[int, None]:
        if not hasattr(self, 'data_bytes'):
            raise ValueError("No data available to extract value.")
        
        high_byte = self.data_bytes[2:4]
        low_byte = self.data_bytes[0:2]

        return int.from_bytes(high_byte + low_byte, byteorder='big')
           

    def get_scaled_value(self) -> Union[float, None]:
        if hasattr(self, 'data_bytes'):
            decimal_value = self.get_value()
            return round(decimal_value * 0.0001, 1)
        return None