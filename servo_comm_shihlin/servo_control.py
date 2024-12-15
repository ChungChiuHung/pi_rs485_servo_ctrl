import time
import struct
from threading import Thread, Event
from serial import SerialException
from modbus_ascii_client import ModbusASCIIClient
from modbus_response import ModbusResponse
from servo_utility import ServoUtility
from servo_control_registers import ServoControlRegistry
from status_bit_map import DI_Function_Code
from servo_p_register import PA, PC, PD, PE, PF
import logging

PA.init_registers()
PC.init_registers()
PD.init_registers()
PE.init_registers()
PF.init_registers()

float_error = 0

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')


class ServoController:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.modbus_client = ModbusASCIIClient(
            device_number=1, serial_port_manager=serial_port)
        self.read_thread = None
        self.read_thread_stop_event = Event()
        self.response = ""
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.float_error = 0.0
        self.accumulate_pulse = 0
        # self.completed_tag = False
        # self.check_movement = False

    def delay_ms(self, milliseconds):
        time.sleep(milliseconds / 1000.0)

    def print_byte_array_as_spaced_hex(self, byte_array, data_name):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{data_name}: {hex_string}")

    def start_continuous_reading(self, address=0x0205, interval=0.1):
        if self.read_thread is not None:
            self.stop_continuous_reading()

        self.read_thread_stop_event.clear()
        self.read_thread = Thread(
            target=self._read_continuously, args=(address, interval))
        self.read_thread.start()

    def _read_continuously(self, address, interval):
        while not self.read_thread_stop_event.is_set():
            if not self.serial_port.keep_running:
                logging.info("Reconnection attempts stopped.")
                break
            try:
                message = self.modbus_client.build_read_message(address, 1)
                response = self.modbus_client.send_and_receive(message)
                # response_object = ModbusResponse(response)
                # print(response_object)
                # if self.check_movement:
                #     if self.is_movement_complete(response):
                #         # self.completed_tag = True
                #         self.check_movement = False
                #         logging.info("Movement complete.")
                #     else:
                #         # self.completed_tag = False

                # logging.info(f"servo completed_tag: {self.completed_tag}")
            except Exception as e:
                logging.error(f"Error during read: {e}")
                break
            time.sleep(interval)

    # add completion check

    def is_movement_complete(self, response):
        completion_pattern = bytearray(
            b'\xba\xb0\xb1\xb0\xb3\xb0\xb2\xb0\xb0\xb3\xb3\xc3\xb7\x8d\x8a')
        return response == completion_pattern

    def stop_continuous_reading(self):
        if self.read_thread is not None:
            self.read_thread_stop_event.set()
            self.read_thread.join()
            self.read_thread = None
            print("Continuous reading stopped.")

    #  0x0010, 0x0000


    def read_PA01_Ctrl_Mode(self):
        print(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        message = self.modbus_client.build_read_message(PA.STY.address, 2)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)
    
    def read_PA33_Encoder_ABS_Pos(self):
        message = self.modbus_client.build_read_message(0x0340, 2)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

    def write_PA01_Ctrl_Mode(self):
        print(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 1, 0)
        message = self.modbus_client.build_write_message(
            PA.STY.address, config_value)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

    # Digital input on/off control source option
    # # Select IO to Be controlled PD16
    def write_PD_16_Enable_DI_Control(self):
        print(f"Address of PD{PD.SDI.no} {PD.SDI.name}: {hex(PD.SDI.address)}")
        config_value = ServoUtility.config_hex_with(0, 0xF, 0xF, 0xF)
        message = self.modbus_client.build_write_message(
            PD.SDI.address, config_value)
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


    # Communication control DI on/off [Pt Mode]
    # 0x0040 (DI7) EMG
    # 0000 0  0  0  0 0 1 0 0 0 0 0 0
    #      12 11 10 9 8 7 6 5 4 3 2 1
    #      12 11 10 9 8 7 6 5 4 3 2 1
    # 0x0041 (DI7 + DI1) SON
    # 0000 0  0  0  0 0 1 0 0 0 0 0 1
    # 0x0341 (DI11 + DI10 + DI7 + DI1) : (LSN, LSP, EMG, SON)
    # 0000 0  0  1  1 0 1 0 0 0 0 0 1
    def write_PD_25(self):
        print(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")

        config_value = ServoUtility.config_hex_with(0, 0, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)

        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        print(response_object)

    def read_PD_25(self):
        print(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        message = self.modbus_client.build_read_message(PD.ITST.address, 1)
        print(f"Build Read Message: {message}")
        response = self.modbus_client.send_and_receive(message)
        print(f"Response Message: {response}")
        response_object = ModbusResponse(response)
        print(response_object)

    def clear_alarm(self):
        print(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print("\n")

    def servo_on(self):
        print(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print("\n")

    def clear_alarm_12(self):
        print(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print("\n")

    def servo_off(self):
        # print(
        #    f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        print("Servo Off, Alarm 12 ON!")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        self.response = self.modbus_client.send_and_receive(message)
        time.sleep(0.1)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print("\n")

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
        message = self.modbus_client.build_write_message(
            PD.DIA1.address, config_value)
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
        message = self.modbus_client.build_write_message(
            PD.DI1.address, config_value)
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
        # print(f"Address of 0x0100, 1 word")
        # print(f"Address of 0x0100, 1 word")
        message = self.modbus_client.build_read_message(0x0100, 11)
        # print(f"Build Read Command: {message}")
        self.response = self.modbus_client.send_and_receive(message)
        # print(f"Response Message: {response}")
        response_object = ModbusResponse(self.response)
        print(response_object)
        # print(f"Build Read Command: {message}")

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
        excute_PATH = 1  # (1~63)
        excute_PATH = 1  # (1~63)
        stop = 1000
        # Read Value: get the executed PATH situation
        # 3: PATH#3 is being executed
        # 1003: PATH#3 command is completed
        # 2003: PATH#3 positioning is done
        message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print(int.from_bytes(response_object.data_bytes, byteorder='big'))

        while False:
            message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(response)
            print(response_object)
            flag_value = int(response_object.data, 16)
            time.sleep(0.1)

        print("\n")

    def write_PF82(self, execute_PATH_value=0):
        """
        This method writes and controls the PATH execution.

        Parameters:
            execute_PATH_value (int): The PATH number to execute (1~63)
        """
        print(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
        # goto_origin = 0
        # stop_cmd = 1000
        excute_PATH = execute_PATH_value
        # Read Value: get the executed PATH situation
        # 3: PATH#3 is being executed
        # 1003: PATH#3 command is completed
        # 2003: PATH#3 positioning is done
        # Validate the execute_PATH_value
        if execute_PATH_value < 0 or execute_PATH_value > 9999:
            raise ValueError("execute_PATH_value must be between 0 and 9999.")
        if execute_PATH_value >= 64 and execute_PATH_value < 1000:
            print("Value out of acceptable rnage.")

        message = self.modbus_client.build_write_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)

        # Process the response using ModbusResponse
        try:
            response_object = ModbusResponse(self.response)
            print(f"Parsed Mobus Response: {response_object}")

            if hasattr(response_object, 'data_bytes'):
                # Extract the response value
                response_value = int.from_bytes(response_object.data_bytes, byteorder='big')
                print(f"Response value: {response_value}")

                #Handle the response value logic
                if response_value == execute_PATH_value:
                    print(f"Command {execute_PATH_value} is still being executed.")
                elif response_value == (execute_PATH_value + 10000):
                    print(f"Commnad {execute_PATH_value} has been executed, but motor positioning is not complete.")
                elif response_value == (execute_PATH_value + 20000):
                    print(f"Command {execute_PATH_value} has been executed, and motor position is complete.")

            else:
                print("No data_bytes found in response.")
        except ValueError as e:
            print(f"Failed to process response: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")       

        while False:
            message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(response)
            print(response_object)
            flag_value = int(response_object.data, 16)
            time.sleep(0.1)

        print("\n")


    # Read Position Control related parameters


    def Read_Pos_Related_Paremters(self):
        read_address_array = [PA.STY, PA.HMOV, PA.PLSS,
                              PA.ENR, PA.PO1H, PA.POL,
                              PD.SDI, PD.ITST, PD.MCOK]
        # PD28 MCOK
        # PD16 SDI
        # PD25 ITST
        # PD28 MCOK
        # PD16 SDI
        # PD25 ITST

        # PA01, 2 3, 6, 7, 13, 15
        # PA02, ATUM: Gain tuning mode option
        # PA03, ATUL: Auto-tuning response level setting
        # PA06, CMX : Electronic gear numerator
        # PA07, CDV : Electronic gear denominator
        # PA13, PLSS: Command pulse option
        # PA15, CRSHA: Motor crash protection (time)

        for address in read_address_array:
            print(f"Read {address.no}: {address.name}: {hex(address.address)}")
            message = self.modbus_client.build_read_message(address.address, 1)
            self.response = self.modbus_client.send_and_receive(message)
            # response_object = ModbusResponse(response)
            # print(response_object)
            time.sleep(0.1)


    def Read_Motion_Completed_Signal(self):
        parameter = PD.MCOK
        # print(f"Read {parameter.no}: {parameter.name}: {hex(parameter.address)}")
        message = self.modbus_client.build_read_message(parameter.address, 1)
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)

    # Position Control Test Mode
    def Enable_Position_Mode(self, enable=True):
        address = ServoControlRegistry.CTRL_MODE_SEL.value
        config_value = 0x0000
        # print(f"Address of {address}")
        if enable == True:
            config_value = 0x0004

        message = self.modbus_client.build_write_message(address, config_value)
        # print(f"Build Read Command: {message}")
        self.response = self.modbus_client.send_and_receive(message)
        # print(f"Response Message: {response}")
        # response_object = ModbusResponse(response)
        # print(response_object)

    def Enable_JOG_Mode(self, enable=True):
        address = ServoControlRegistry.CTRL_MODE_SEL.value
        config_value = 0x0000
        if enable:
            config_value = 0x0003

        message = self.modbus_client.build_write_message(address, config_value)
        self.response = self.modbus_client.send_and_receive(message)

    def config_acc_dec_0x0902(self, acc_dec_time):
        # print(f"Address 0x0902, 1 word")
        config_value = acc_dec_time
        message = self.modbus_client.build_write_message(0x0902, config_value)
        # print(f"Build Write Command: {message}")
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print(f"Build Write Command: {message}")
        

    def config_speed_0x0903(self, speed_rpm):
        # print(f"Address 0x0903, 1 word")
        config_value = speed_rpm
        message = self.modbus_client.build_write_message(0x0903, config_value)
        # print(f"Build Write Command: {message}")
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print(f"Build Write Command: {message}")

    def config_pulses_0x0905_low_byte(self, low_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_L.value
        # print(f"Address {address}, 1 word")
        config_value = low_byte
        message = self.modbus_client.build_write_message(address, config_value)
        # print(f"Build Write Command: {message}")
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print(f"Build Write Command: {message}")

    def config_pulses_0x0906_high_byte(self, high_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_H.value
        # print(f"Address {address}, 1 word")
        config_value = high_byte
        message = self.modbus_client.build_write_message(address, config_value)
        # print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)

    def read_0x0905_low_byte(self):
        # print(f"Address 0x0905, 1 word")
        message = self.modbus_client.build_read_message(0x0905, 1)
        # print(f"Build Write Command: {message}")
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print(f"Build Write Command: {message}")

    def read_0x0906_high_byte(self):
        # print(f"Address 0x0906, 1 word")
        message = self.modbus_client.build_read_message(0x0906, 1)
        # print(f"Build Write Command: {message}")
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)
        # print(f"Build Write Command: {message}")

    def pos_motion_start_0x0907(self, value):
        # print(f"Address 0x0907, 1 word")
        config_value = value
        message = self.modbus_client.build_write_message(0x0907, config_value)
        # print(f"Build Write Command: {message}")
        # time.sleep(0.05)
        self.response = self.modbus_client.send_and_receive(message)
        # response_object = ModbusResponse(response)
        # print(response_object)

    def read_encoder_before_gear_ratio(self):
        print(f"Address 0x0000, 1 word")
        message = self.modbus_client.build_read_message(0x0000, 2)
        print(f"Build Read Command: {message}")
        response_object = ModbusResponse(message)
        print(response_object)


    def read_encoder_after_gear_ratio(self):
        print(f"Address 0x0024, 1 word")
        message = self.modbus_client.build_read_message(0x0024, 2)
        print(f"Build Read Command: {message}")
        response_object = ModbusResponse(message)
        print(response_object)

    def pos_step_motion_test(self, CW=True):
        time.sleep(0.1)
        if CW == True:
            self.pos_motion_start_0x0907(1)
        else:
            self.pos_motion_start_0x0907(2)
        #time.sleep(0.1)

    def pos_step_motion_by(self, angle=0.0, acc_dec_time=5000, speed_rpm=10):
        base_pulse_per_degree = 349525.333
        output_pulse = 0

        self.previous_angle = self.current_angle
        self.current_angle = angle
        diff_angle = self.current_angle - self.current_angle
        diff_pulse = diff_angle * base_pulse_per_degree
        fraction_part = diff_pulse - int(diff_pulse)
        self.float_error += fraction_part

    def post_step_motion_by(self, angle=0.0, acc_dec_time=5000, speed_rpm=10):
        # 125829120 pulse/rev
        # 349525 + 1/3 pulse/degree
        # 125829120 pulse/rev
        # 349525 + 1/3 pulse/degree
        base_pulse_per_degree = 349525.333
        output_pulse = 0

        print("\n")
        print(f"Current Angle: {self.current_angle}")
        print(f"Previous Angle: {self.previous_angle}")
        print(f"Set Angle: {angle}")

        self.previous_angle = self.current_angle
        self.current_angle = angle
        diff_angle = self.current_angle - self.previous_angle

        if diff_angle != 0.0:
            total_pulse = base_pulse_per_degree * diff_angle
            integer_pulse = int(total_pulse)
            fractional_pulse = total_pulse - integer_pulse

            # Accumulate fractional part
            self.float_error += fractional_pulse

            if self.float_error >= 1.0:
                integer_error = int(self.float_error)
                integer_pulse += integer_error
                self.float_error -= integer_error

            output_pulse = integer_pulse
            self.accumulate_pulse += output_pulse

        low_byte = abs(output_pulse) & 0xFFFF
        high_byte = (abs(output_pulse) >> 16) & 0xFFFF

        print("\n")
        print(f"Output pulse: {output_pulse}")
        print(f"float error: {self.float_error}")
        # print(f"Current Accumulate Pulse: {self.accumulate_pulse}")
        print(f"{hex(high_byte)}, {hex(low_byte)}")

        self.stop_continuous_reading()
        time.sleep(0.06)

        self.Enable_Position_Mode(True)
        time.sleep(0.06)
        self.config_acc_dec_0x0902(acc_dec_time)
        time.sleep(0.06)
        self.config_speed_0x0903(speed_rpm)
        time.sleep(0.06)
        self.config_pulses_0x0905_low_byte(low_byte)
        time.sleep(0.06)
        self.config_pulses_0x0906_high_byte(high_byte)
        time.sleep(0.06)
        
        self.start_continuous_reading(0x0340)
        time.sleep(0.06)

        if output_pulse > 0:
            print("Running Servo CW")
            self.pos_step_motion_test(True)
        else:
            print("Running Servo CCW")
            self.pos_step_motion_test(False)

    def enable_speed_ctrl(self, speed_rpm):
        self.Enable_JOG_Mode(True)
        time.sleep(0.1)
        self.config_speed_0x0903(speed_rpm)
        time.sleep(0.1)
        self.start_continuous_reading(0x0900, 0.1)

    # 0: Stop
    # 1: CW
    # 2: CCW
    def speed_ctrl_action(self, action_value):
        if action_value == 0:
            print("Servo Stop!")
        elif action_value == 1:
            print("Servo CW")
        elif action_value == 2:
            print("Servo CCW")
        else:
            print("Error Config.")
        time.sleep(0.1)
        address = 0x0904
        message = self.modbus_client.build_write_message(address, action_value)
        # response = self.modbus_client.send_and_receive(message)
        self.modbus_client.send(message)

    def set_home_position(self):
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.float_error = 0.0
        self.accumulate_pulse = 0
        print("home position set!!!")
