import json
import time
from status_bit_mapping import BitMapOutput
from servo_params import ServoParams

class IOStatusFetcher:
    def __init__(self, ser_port):
        self.ser_port = ser_port

    def send_command(self, command):
        self.ser_port.write(command)
        time.sleep(0.05)
    
    def read_response(self):
        deadline = time.time() + 1 # 1-second timeout
        while time.time() < deadline:
            if self.ser_port.inWaiting() >0:
                return self.ser_port.read(self.ser_port.inWaiting())
            return None # No response received within the tieout peroid
        
    def parse_logic_io(self, logic_io_bytes):
        io_status= {}
        value = int.from_bytes(logic_io_bytes, byteorder='big')

        for bit in BitMapOutput:
            if isinstance(bit.value, tuple):
                mask = (1 << (bit.value[1] - bit.value[0] + 1)) - 1
                io_status[bit.name] = (value >> bit.value[0]) & mask
            else:
                io_status[bit.name] = bool(value &(1 << bit.value))
            
        return io_status
    
    def get_output_io_status(self):
        get_output_io = ServoParams.GET_OUTPUT_IO
        self.send_command(bytes(get_output_io))

        response = self.read_response()
        if response and len(response) >= 6:
            logic_io_bytes = response[4:8]
            parsed_status = self.parse_logic_io(logic_io_bytes)
            return json.dumps(parsed_status)
        
        return json.dumps({"error" : "No response or invalid response length"})
    

class IOStatusFetcher_Test:
    def __init__(self):
        pass

    def send_command(self, command):
        print(f"Command sent: {command}")

    def parse_logic_io(self, logic_io_bytes):
        io_status = {}
        value = int.from_bytes(logic_io_bytes, byteorder='big')

        for bit in BitMapOutput:
            if isinstance(bit.value, tuple):
                mask = (1 << (bit.value[1] - bit.value[0] + 1)) - 1
                io_status[bit.name] = (value >> bit.value[0]) & mask
            else:
                io_status[bit.name] = bool(value & (1 << bit.value))
            
        return io_status
    
    def get_output_io_status(self, input_response):
        get_output_io = [0x24, 0x01, 0x00, 0x11, 0x01, 0x28, 0x75, 0xE0]  # Assuming GET_OUTPUT_IO defined
        self.send_command(bytes(get_output_io))

        response = input_response
        if response and len(response) >= 6:
            logic_io_bytes = response[4:8]
            parsed_status = self.parse_logic_io(logic_io_bytes)
            return json.dumps(parsed_status, indent=4)
        
        return json.dumps({"error": "No response or invalid response length"})

