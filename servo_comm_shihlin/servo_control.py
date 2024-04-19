import time
import struct
import inspect
from serial import SerialException
from modbus_ascii_client import ModbusASCIIClient
from modbus_response import ModbusResponse
from servo_utility import ServoUtility
from servo_control_registers import ServoControlRegistry
from status_bit_map import DI_Function_Code
from servo_p_register import PA, PC, PD, PE, PF

PA.init_registers()
PC.init_registers()
PD.init_registers()
PE.init_registers()
PF.init_registers()

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
        

    #  0x0010, 0x0000
    def read_PA01_Ctrl_Mode(self):
        print(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        message = self.modbus_client.build_read_message(PA.STY.address, 2)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)

        response_object = ModbusResponse(response)

        print(response_object)

    def write_PA01_Ctrl_Mode(self):
        print(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 1, 0)
        message = self.modbus_client.build_write_message(PA.STY.address, config_value)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)

        response_object = ModbusResponse(response)

        print(response_object)

    # Digital input on/off control source option
    # # Select IO to Be controlled PD16
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
                     
    # 內部位置命令 p129
    # Pos 1:  000000 PE01/PE02
    # Pos 2:  000001 PE03/PE04
    # ...
    # Pos 63: 111111 PF29/PF30
    
    # Communication control DI on/off
    # 0x0040 (DI7) EMG
    # 0000 0  0  0  0 0 1 0 0 0 0 0 0
    #      12 11 10 9 8 7 6 5 4 3 2 1 
    # 0x0041 (DI7 + DI1) SON
    # 0000 0  0  0  0 0 1 0 0 0 0 0 1
    # 0x0341
    # 0000 0  0  1  1 0 1 0 0 0 0 0 1
    def write_PD_25(self):
        self.write_PD_16_Enable_DI_Control()
        time.sleep(0.1)
        print(f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 4, 1)
        message = self.modbus_client.build_write_message(PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

    def read_PD_25(self):
        self.write_PD_16_Enable_DI_Control()
        time.sleep(0.1)
        print(f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        message = self.modbus_client.build_read_message(PD.ITST.address, 1)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")
        response_object = ModbusResponse(response)
        print(response_object)

    def clear_alarm_12(self):
        self.write_PD_16_Enable_DI_Control()
        time.sleep(0.1)
        print(f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 0)
        message = self.modbus_client.build_write_message(PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)
        print("\n")

    def servo_on(self):
        print(f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 1)
        message = self.modbus_client.build_write_message(PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)
        print("\n")


    def servo_off(self):
        print(f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 4, 0)
        message = self.modbus_client.build_write_message(PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)
        print("\n")

    # Pos mode 0x0000 0x0000
    def read_PD_01(self):
        print(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        message = self.modbus_client.build_read_message(PD.DIA1.address, 2)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

        response_object = ModbusResponse(response)
        print(response_object)

    # 0x0000, 0x0000
    def write_PD_01(self):
        print(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(PD.DIA1.address, config_value)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

        response_object = ModbusResponse(response)
        print(response_object)

    # Config DI Function
    # Pos mode 0x0001 , 0x0000
    def write_PD_02(self):
        print(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        config_value = 1
        message = self.modbus_client.build_write_message(PD.DI1.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)


    # 0x0001 0x0000
    def read_PD_02(self):
        print(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        message = self.modbus_client.build_read_message(PD.DI1.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)


    # initial 0x0012 0x0000 DI7
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


    def read_servo_state(self):
        print(f"Addres of 0x0200, 1 word")
        message = self.modbus_client.build_read_message(0x0200, 1)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")

        response_object = ModbusResponse(response)

        print(response_object)

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

    # Select test mode 0x0004 (Pos test mode)
    def read_test_mode_0x0901(self):
        print(f"Address of 0x0901, 1 word")
        message = self.modbus_client.build_read_message(0x0901, 1)
        print(f"Build Read Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")
        response_object = ModbusResponse(response)

        print(response_object)

    # PR (procedure) sequence control
    def read_PF82(self):
        print(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
        # write value
        origin_return = 0
        excute_PATH = 1 # (1~63)
        stop = 1000
        # Read Value: get the executed PATH situation
        # 3: PATH#3 is being executed
        # 1003: PATH#3 command is completed
        # 2003: PATH#3 positioning is done
        message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

        print(int.from_bytes(response_object.data_bytes, byteorder='big'))

        
        while False:
            message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(response)
            print(response_object)
            flag_value = int(response_object.data, 16)
            time.sleep(0.1)

        print("\n")



    
    # Position Control Test Mode

    def stop_test_mode(self):
        print(f"Address of 0x0901, 1 word")
        config_value = 0x0000
        message = self.modbus_client.build_write_message(0x0901, config_value)
        print(f"Build Read Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")
        response_object = ModbusResponse(response)

        print(response_object)

    def start_test_pos_mode(self):
        print(f"Address of 0x0901, 1 word")
        config_value = 0x0004
        message = self.modbus_client.build_write_message(0x0901, config_value)
        print(f"Build Read Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")
        response_object = ModbusResponse(response)

        print(response_object)

    def config_acc_dec_0x0902(self):
        print(f"Address 0x0902, 1 word")
        config_value = 500
        message = self.modbus_client.build_write_message(0x0902, config_value)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)

        print(response_object)

    def config_speed_0x0903(self):
        print(f"Address 0x0903, 1 word")
        config_value = 60
        message = self.modbus_client.build_write_message(0x0903, config_value)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)

        print(response_object)

    def config_pulses_0x0905_low_byte(self):
        print(f"Address 0x0905, 1 word")
        config_value = 0x0000
        message = self.modbus_client.build_write_message(0x0905, config_value)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)

        print(response_object)
    
    def config_pulses_0x0906_high_byte(self):
        print(f"Address 0x0906, 1 word")
        config_value = 0x0780
        message = self.modbus_client.build_write_message(0x0906, config_value)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)

        print(response_object)

    def read_0x0905_low_byte(self):
        print(f"Address 0x0905, 1 word")
        message = self.modbus_client.build_read_message(0x0905, 2)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

    def pos_motion_start_0x0907(self, value):
        print(f"Address 0x0907, 1 word")
        config_value = value
        message = self.modbus_client.build_write_message(0x0907, config_value)
        print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)

        print(response_object)

    def pos_step_motion_test(self, CW=True):
        self.start_test_pos_mode()
        time.sleep(0.05)
        self.config_acc_dec_0x0902()
        time.sleep(0.05)
        self.config_speed_0x0903()
        time.sleep(0.05)
        self.config_pulses_0x0905_low_byte()
        time.sleep(0.05)
        self.config_pulses_0x0906_high_byte()
        time.sleep(0.05)
        if CW ==True:
            self.pos_motion_start_0x0907(1)
        else:
            self.pos_motion_start_0x0907(2)
        time.sleep(0.05)

    



    

