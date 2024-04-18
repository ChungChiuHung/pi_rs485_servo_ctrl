from enum import Enum
from modbus_command_code import CmdCode

class ModbusResponse:
    def __init__(self, response):
        if isinstance(response, (bytes, bytearray)):
            response = response.decode('utf-8')
        
        if response[0] != ':' or response[-2:] != '\r\n':
            raise ValueError("Invalid response: Does not start with ':' or does not end with '\\r\\n'")
        
        self.stx = ':'
        self.end1 = '\r'
        self.end0 = '\n'
        
        response = response[1:-2]
        self.adr = response[0:2]
        self.cmd = response[2:4]
        self.cmd_value = int(self.cmd, 16)

        self.start_address = response[4:8]  # Common extraction for start address

        if self.cmd_value in (CmdCode.WRITE_DATA.value, CmdCode.WRITE_MULTI_DATA.value):
            self.data_count = response[8:12]  # Common extraction for data count/content for writes
            self.lrc = response[-2:]  # Common LRC extraction
            self.data = None  # Write commands might not have immediate data following
        elif self.cmd_value == CmdCode.READ_DATA.value:
            self.data_count = response[8:10]
            data_length = int(self.data_count, 16) * 2
            data_start_idx = 10
            data_end_idx = data_start_idx + data_length
            self.data = [response[i:i+4] for i in range(data_start_idx, data_end_idx, 4)]
            self.lrc = response[data_end_idx:data_end_idx+2]
        else:
            raise ValueError(f"Unsupported command code: {self.cmd}")

    def __str__(self):
        base_info = (f"Modbus Response:\n"
                     f"  STX: {self.stx}\n"
                     f"  Address: {self.adr}\n"
                     f"  Command: {self.cmd} (Cmd Value: {self.cmd_value})\n"
                     f"  Start Address: {self.start_address}\n")
        
        if self.data:
            data_str = ', '.join(self.data)
            additional_info = (f"  Data Count: {self.data_count}\n"
                               f"  Data: [{data_str}]\n")
        else:
            additional_info = (f"  Data Count or Content: {self.data_count}\n")
        
        return base_info + additional_info + (f"  LRC: {self.lrc}\n"
                                              f"  End1 (CR): {self.end1}\n"
                                              f"  End0 (LF): {self.end0}")
