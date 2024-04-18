import time
import struct
import inspect
from serial import SerialException
from modbus_ascii_client import ModbusASCIIClient
from modbus_response import ModbusResponse
from servo_utility import ServoUtility
from servo_control_registers import ServoControlRegistry
from status_bit_map import DI_Function_Code
from servo_p_register import PA, PC, PD, PE

PA.init_registers()
PC.init_registers()
PD.init_registers()
PE.init_registers()

class ServoController:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.modbus_client = ModbusASCIIClient(device_number=1, serial_port_manager= serial_port)

    def delay_ms(self, milliseconds):
      time.sleep(milliseconds / 1000.0)

    def print_byte_array_as_spaced_hex(self,byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def send_command_and_wait_for_response(self, command, description, read_timeout=0.1):
        try:
            self.modbus_client.send(command)
            time.sleep(read_timeout)
            response = self.modbus_client.receive()
            if response:
                print(f"{description} response received: {response.hex()}")
            else:
                print(f"Timeout waiting for {description} response.")
            return response
        except SerialException as e:
            print(f"Error during communication: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
        return None
    
    def parse_logic_io(self, logic_io_bytes):
        io_status= {}
        return io_status
    
    def send_servo_command(self, command_code, data=b'', bitmap=None, value=None, response_delay=0):
        if bitmap is not None and value is not None:
            command = self.modbus_client.build_write_message(bitmap, struct.pack('>H', value))
        else:
            command = self.modbus_client.build_write_message(command_code, data)
        self.send_command_and_wait_for_response(command, f"{command_code.name}", response_delay)

    def monitor_end_status(self, status_code, interval=0.5, duration=10):
        end_time = time.time() + duration
        while time.time() < end_time and self.monitoring_active:
            status_command = self.modbus_client.build_read_message(status_code, 4)
            status = self.send_command_and_wait_for_response(status_command, "Monitoring Status", interval)
            if status:
                print("Status: ", status.hex())
            time.sleep(interval)

    def execute_motion_start_sequence(self, commands):
        self.monitoring_active = True
        print(f"execute_motion {commands}")

    def start_motion_sequence(self, commands):
        self.monitoring_active = True
        print(f"execute_motion {commands}")

    def stop_motion_sequence(self):
        self.monitoring_active = False

    def execute_motion_sequence(self, commands):
        print(f"execute_motion {commands}")
        
    def read_PA01_Ctrl_Mode(self):
        print(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        message = self.modbus_client.build_read_message(PA.STY.address, 2)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)

        response_object = ModbusResponse(response)

        print(response_object)

    def write_PA01_Ctrl_Mode(self):
        print(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        config_value = ServoUtility.config_hex_with(1, 0, 1, 0)
        message = self.modbus_client.build_write_message(PA.STY.address, config_value)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)

        response_object = ModbusResponse(response)

        print(response_object)

    # Digital input on/off control source option
    def write_PD_16_Enable_DI_Control(self):
        print(f"Address of PD{PD.SDI.no} {PD.SDI.name}: {hex(PD.SDI.address)}")
        config_value = ServoUtility.config_hex_with(0, 0xF, 0xF, 0xF)
        message = self.modbus_client.build_write_message(PD.SDI.address, config_value)
        print(f"Build Write Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Respnose Message: {response}")

    def read_PD_16(self):
        print(f"Address of PD{PD.SDI.no} {PD.SDI.name}: {hex(PD.SDI.address)}")
        message = self.modbus_client.build_read_message(PD.SDI.address, 1)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

    # Read IO Function From 0x0206
    def read_0x0206_To_0x020B(self):
        print(f"Address of 0x0206: Read Value")
        message = self.modbus_client.build_read_message(0x0206, 6)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

        cnt = 1
        for data in response_object.data:
            print(f"Original data value: {data}")
            for code in DI_Function_Code:
                if code.value == int(data, 16):
                    print(f"DI{cnt} :{code.name}")
                    cnt += 1
                     
        

    # Communication control DI on/off
    def write_PD_25(self):
        print(f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 1)
        message = self.modbus_client.build_write_message(PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

    def read_PD_25(self):
        print(f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        message = self.modbus_client.build_read_message(PD.ITST.address, 1)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")
        response_object = ModbusResponse(response)
        print(response_object)

    def read_PD_01(self):
        print(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        message = self.modbus_client.build_read_message(PD.DIA1.address, 2)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

        response_object = ModbusResponse(response)
        print(response_object)

    def write_PD_01(self):
        print(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        message = self.modbus_client.build_write_message(PD.DIA1.address, 0x1111)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

    def read_PD_08(self):
        print(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_read_message(PD.DI7.address, 2)
        print(f"Build Read Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

        response_object = ModbusResponse(response)
        print(response_object)

    def write_PD_08(self):
        print(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_write_message(PD.DI7.address, 0x02F)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")


    #def read_servo_state(self):
    #    print(f"Addres of 0x0200, 1 word")
    #    message = self.modbus_client.build_read_message(0x0200, 1)
    #    print(f"Build Read Message: {message}")
    #    response = self.modbus_client.send_and_receive(message)
    #    print(f"Response Message: {response}")
#
    #    response_object = ModbusResponse(response)
#
    #    print(response_object)
    #    print(response_object.data)

    def read_control_mode(self):
        print(f"Addres of 0x0201, 1 word")
        message = self.modbus_client.build_read_message(0x0201, 1)
        print(f"Build Read Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

        response_object = ModbusResponse(response)

        print(response_object)

    def read_alarm_msg(self):
        print(f"Address of 0x0100, 1 word")
        message = self.modbus_client.build_read_message(0x0100, 11)
        print(f"Build Read Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

        response_object = ModbusResponse(response)

        print(response_object)

    def read_servo_state(self):
        print(f"Address of 0x0900, 1 word")
        message = self.modbus_client.build_read_message(0x0900, 1)
        print(f"Build Read Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

    



    

