import json
import time
from serial_communication import SerialCommunication
from servo_params import ServoParams
from set_servo_io_status import SetServoIOStatus
from status_bit_mapping import BitMap, BitMapOutput

class ServoCntroller:

    @staticmethod
    def delay_ms(self, milliseconds):
      time.sleep(milliseconds / 1000.0)

    #IOStatusFetcher:  
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
        response_recieved = False
        result, response_recieved = self.send_command_and_wait_for_response(get_output_io, 
                                                                            "GET_IO_STATES")

        if result and len(result) >= 6:
            logic_io_bytes = result[4:8]
            parsed_status = self.parse_logic_io(logic_io_bytes)
            return json.dumps(parsed_status), response_recieved
        
        response_recieved = True

        return (json.dumps({"error" : "No response or invalid response length"}), response_recieved)
   