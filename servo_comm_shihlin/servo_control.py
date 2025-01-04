import threading
import time
import logging
import json
from typing import Union, Callable
from threading import Thread, Event
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


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class ServoController:
    CONFIG_FILE = "config.json"

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.modbus_client = ModbusASCIIClient.get_instance(
            device_number=1, serial_port_manager=serial_port
            )
        self.read_thread: Union[Thread, None] = None
        self.read_thread_stop_event = threading.Event()
        self.reading_active = False
        self.lock = threading.Lock()
        self.stop_event = Event()
        self.response = ""
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.float_error = 0.0
        self.accumulate_pulse = 0
        self.on_initial_home = False
        self.completed_tag = False
        self.completed_cnt = 0
        #self.abs_home_pos = 1184347
        self.abs_home_pos = self.load_abs_home_pos()
        self._event_listeners = {"on_motion_completed": []}

    def load_abs_home_pos(self) -> int:
        try:
            with open(self.CONFIG_FILE, 'r') as file:
                config = json.load(file)
            return config.get("abs_home_pos", 1184347)
        except FileNotFoundError:
            logging.warning("Config file not found.")
            return 1184347
        except json.JSONDecodeError as e:
            logging.error(f"Error parsing configuration file: {e}.")
            return 1184347
        
    def save_abs_home_pos(self, abs_home_pos: int):
        try:
            with open(self.CONFIG_FILE, 'w') as file:
                json.dump({"abs_home_pos": abs_home_pos}, file)
            logging.info(f"Saved abs_home_pos: {abs_home_pos} to {self.CONFIG_FILE}")
        except Exception as e:
            logging.error(f"Error saving abs_home_pos: {e}")

    def register_event_listener(self, event_name: str, callback: Callable):
        """Register a callback for a specific event."""
        if event_name not in self._event_listeners:
            raise ValueError(f"Event {event_name} is not supported.")
        if not callable(callback):
            raise ValueError("Callback must be callable.")
        self._event_listeners[event_name].append(callback)
        
    def unregister_event_listener(self, event_name: str, callback: callable):
        """Unregister a callback for a specific event."""
        if event_name not in self._event_listeners:
            raise ValueError(f"Event {event_name} is not supported.")
        try:
            self._event_listeners[event_name].remove(callback)
        except ValueError:
            logging.info(f"Callback not found for event {event_name}.")
        
    def _notify_event_listeners(self, event_name, *args, **kwargs):
        """Notify all registered callbacks for a specific event."""
        for callback in self._event_listeners.get(event_name, []):
            callback(*args, **kwargs)

    def delay_ms(self, milliseconds: int) -> None:
        time.sleep(milliseconds / 1000.0)

    def print_byte_array_as_spaced_hex(self, byte_array, data_name) -> None:
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        logger.info(f"{data_name}: {hex_string}")

    # default address = 0x0205
    def start_continuous_reading(self, interval: float = 0.1) -> None:
        with self.lock:
            if self.reading_active:
                self.stop_continuous_reading()
                return

            self.read_thread_stop_event.clear()
            self.read_thread = threading.Thread(target=self._read_continuously, args=(interval,))
            self.reading_active = True
            self.read_thread.start()
            logging.info("Continuous reading started.")
    
    def stop_continuous_reading(self) -> None:
        with self.lock:
            if not self.reading_active:
                logging.warning("Continuous reading is not active; skipping stop.")
                return

            self.reading_active = False
            self.read_thread_stop_event.set()
            if self.read_thread and threading.current_thread() is not self.read_thread:
                self.read_thread.join()

            self.read_thread = None
            self.completed_cnt = 0
            self.completed_tag = False
            if self.on_initial_home:
                self.on_initial_home = False
            logging.info("Motion Completed Signal Reading Stopped.")
            self._notify_event_listeners("on_motion_completed")
            self.stop_event.set()
            

    def _read_continuously(self, interval: float) -> None:
        while not self.read_thread_stop_event.is_set():
            if not self.serial_port.keep_running:
                logging.info("Reconnection attempts stopped.")
                break
            try:
                self.completed_tag = self.Read_Motion_Completed_Signal()
                self.delay_ms(50)
                current_pulse = self.read_encoder_before_gear_ratio()
                logging.info(f"Current Encoder Value: {current_pulse}")

                if self.completed_tag:
                    self.completed_cnt += 1
                    if self.completed_cnt > 6:
                        self.stop_continuous_reading()
                        break
            except Exception as e:
                logging.error(f"Error during read: {e}")
                break
            self.delay_ms(interval * 1000)

    def read_PA01_Ctrl_Mode(self):
        logging.info(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        message = self.modbus_client.build_read_message(PA.STY.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)
    
    def read_PA33_Encoder_ABS_Pos(self):
        message = self.modbus_client.build_read_message(0x0340, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logger.info(response_object)

    def write_PA01_Ctrl_Mode(self):
        logger.info(f"Address of PA{PA.STY.no} {PA.STY.name}: {hex(PA.STY.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 1, 0)
        message = self.modbus_client.build_write_message(
            PA.STY.address, config_value)
        try:
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(response)
            logger.info(response_object)
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def write_PD_16_Enable_DI_Control(self):
        logger.info(f"Address of PD{PD.SDI.no} {PD.SDI.name}: {hex(PD.SDI.address)}")
        config_value = ServoUtility.config_hex_with(0, 0xF, 0xF, 0xF)
        message = self.modbus_client.build_write_message(
            PD.SDI.address, config_value)
        try:
            response = self.modbus_client.send_and_receive(message)
            logger.info(f"Response Message: {response}")
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def read_PD_16(self):
        logger.info(f"Address of PD{PD.SDI.no} {PD.SDI.name}: {hex(PD.SDI.address)}")
        message = self.modbus_client.build_read_message(PD.SDI.address, 1)
        try:
            #print(f"Build Read Message: {message}")
            response = self.modbus_client.send_and_receive(message)
            logger.info(f"Response Message: {response}")
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

    def read_0x0206_To_0x020B(self):
        logger.info(f"Address of 0x0206: Read Value")
        message = self.modbus_client.build_read_message(0x0206, 6)
        try:
            response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(response)
            logger.info(response_object)

            cnt = 1
            for data in response_object.data:
                logger.info(f"Original data value: {data}")
                for code in DI_Function_Code:
                    if code.value == int(data, 16):
                        logger.info(f"DI{cnt} :{code.name}")
                        cnt += 1
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")

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
        logger.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")

        config_value = ServoUtility.config_hex_with(0, 0, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)

        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logger.info(response_object)

    def read_PD_25(self):
        logger.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        message = self.modbus_client.build_read_message(PD.ITST.address, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logger.info(response_object)

    def clear_alarm(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(self.response)
        logging.info(response_object)

    def servo_on(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 3, 4, 1)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)


    def clear_alarm_12(self):
        logging.info(
            f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        config_value = ServoUtility.config_hex_with(0, 0, 4, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def servo_off(self):
        # print(
        #    f"Address of PD{PD.ITST.no} {PD.ITST.name}: {hex(PD.ITST.address)}")
        logger.info("Servo Off, Alarm 12 ON!")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(
            PD.ITST.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logger.info(response_object)
        self.delay_ms(100)

    # Pos mode 0x0000 0x0000
    def read_PD_01(self):
        logger.info(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        message = self.modbus_client.build_read_message(PD.DIA1.address, 2)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(self.response)
        logging.info(response_object)

    # 0x0000, 0x0000
    def write_PD_01(self):
        logging.info(f"Address of PD{PD.DIA1.no} {PD.DIA1.name}: {PD.DIA1.address}")
        config_value = ServoUtility.config_hex_with(0, 0, 0, 0)
        message = self.modbus_client.build_write_message(
            PD.DIA1.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    # Config DI Function
    # Pos mode 0x0001 , 0x0000
    def write_PD_02(self):
        logging.info(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        config_value = 1
        message = self.modbus_client.build_write_message(
            PD.DI1.address, config_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    # 0x0001 0x0000


    def read_PD_02(self):
        logging.info(f"Address of PD{PD.DI1.no} {PD.DI1.name}: {PD.DI1.address}")
        message = self.modbus_client.build_read_message(PD.DI1.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    # initial 0x0012 0x0000 DI7


    def read_PD_08(self):
        logging.info(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_read_message(PD.DI7.address, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def write_PD_08(self):
        logging.info(f"Address of PD{PD.DI7.no} {PD.DI7.name}: {PD.DI7.address}")
        message = self.modbus_client.build_write_message(PD.DI7.address, 0x02F)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def read_servo_state(self):
        message = self.modbus_client.build_read_message(0x0200, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def read_control_mode(self):
        message = self.modbus_client.build_read_message(0x0201, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def read_alarm_msg(self):
        message = self.modbus_client.build_read_message(0x0100, 11)
        self.response = self.modbus_client.send_and_receive(message)
        # print(f"Response Message: {response}")
        response_object = ModbusResponse(self.response)
        logging.info(response_object)

    # Select test mode 0x0004 (Pos test mode)
    def read_test_mode_0x0901(self):
        message = self.modbus_client.build_read_message(0x0901, 1)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    # PR (procedure) sequence control
    def read_PF82(self):
        logging.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
        message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(self.response)
        logger.info(response_object.get_value())

    def write_PF82(self, execute_PATH_value: int = 0):
        """
        This method writes and controls the PATH execution.

        Parameters:
            execute_PATH_value (int): The PATH number to execute (1~63)
        """
        logging.info(f"Address of P{PF.PRCM.no}, {PF.PRCM.name}: {PF.PRCM.address}")
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
            logging.info("Value out of acceptable range.")

        message = self.modbus_client.build_write_message(PF.PRCM.address, 1)
        self.response = self.modbus_client.send_and_receive(message)

        # Process the response using ModbusResponse
        try:
            response_object = ModbusResponse(self.response)
            logging.info(f"Parsed Mobus Response: {response_object}")
        except Exception as e:
            logging.info(f"An unexpected error occurred: {e}")       

    # Read Position Control related parameters


    def Read_Pos_Related_Paremters(self):
        read_address_array = [PA.STY, PA.HMOV, PA.PLSS,
                              PA.ENR, PA.PO1H, PA.POL,
                              PD.SDI, PD.ITST, PD.MCOK]
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
            logger.info(f"Read {address.no}: {address.name}: {hex(address.address)}")
            message = self.modbus_client.build_read_message(address.address, 1)
            self.response = self.modbus_client.send_and_receive(message)
            logger.info(self.response)
            self.delay_ms(100)


    def Read_Motion_Completed_Signal(self) -> bool:
        try:
            message = self.modbus_client.build_read_message(PF.PRCM.address, 1)
            self.response = self.modbus_client.send_and_receive(message)
            response_object = ModbusResponse(self.response)
            return response_object.get_value() != 0
        except SerialException as e:
            logger.error(f"Serial connection error: {e}")
            return False
        except Exception as e:
            logger.error(f"Error during Modbus communication: {e}")
            return False

    # Position Control Test Mode
    def Enable_Position_Mode(self, enable=True):
        address = ServoControlRegistry.CTRL_MODE_SEL.value
        config_value = 0x0000
        # print(f"Address of {address}")
        if enable == True:
            config_value = 0x0004

        message = self.modbus_client.build_write_message(address, config_value)
        # print(f"Build Read Command: {message}")
        # self.response = self.modbus_client.send_and_receive(message)
        self.modbus_client.send(message)


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
        # self.response = self.modbus_client.send_and_receive(message)
        self.modbus_client.send(message)
        

    def config_speed_0x0903(self, speed_rpm):
        # print(f"Address 0x0903, 1 word")
        config_value = speed_rpm
        message = self.modbus_client.build_write_message(0x0903, config_value)
        # print(f"Build Write Command: {message}")
        # self.response = self.modbus_client.send_and_receive(message)
        self.modbus_client.send(message)


    def config_pulses_0x0905_low_byte(self, low_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_L.value
        # print(f"Address {address}, 1 word")
        config_value = low_byte
        message = self.modbus_client.build_write_message(address, config_value)
        # print(f"Build Write Command: {message}")
        # self.response = self.modbus_client.send_and_receive(message)
        self.modbus_client.send(message)


    def config_pulses_0x0906_high_byte(self, high_byte):
        address = ServoControlRegistry.POS_PULSES_CMD_H.value
        # print(f"Address {address}, 1 word")
        config_value = high_byte
        message = self.modbus_client.build_write_message(address, config_value)
        # print(f"Build Write Command: {message}")
        # response = self.modbus_client.send_and_receive(message)
        self.modbus_client.send(message)

    def read_0x0905_low_byte(self):
        # print(f"Address 0x0905, 1 word")
        message = self.modbus_client.build_read_message(0x0905, 1)
        # print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        return response

    def read_0x0906_high_byte(self):
        # print(f"Address 0x0906, 1 word")
        message = self.modbus_client.build_read_message(0x0906, 1)
        # print(f"Build Write Command: {message}")
        response = self.modbus_client.send_and_receive(message)
        return response

    def pos_motion_start_0x0907(self, value):
        # print(f"Address 0x0907, 1 word")
        config_value = value
        message = self.modbus_client.build_write_message(0x0907, config_value)
        self.modbus_client.send(message)

    def read_encoder_before_gear_ratio(self):
        message = self.modbus_client.build_read_message(0x0000, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)
        encoder_value = response_object.get_value()
        if encoder_value is not None:
            return int(encoder_value)
        return None

    def read_encoder_after_gear_ratio(self):
        message = self.modbus_client.build_read_message(0x0024, 2)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def pos_step_motion_test(self, CW=True):
        self.start_continuous_reading()
        self.delay_ms(100)
        if CW == True:
            self.pos_motion_start_0x0907(1)
        else:
            self.pos_motion_start_0x0907(2)

    def pos_step_motion_by(self, target_pulses: int = 0, acc_dec_time=5000, speed_rpm=10):
        # Get Current Pulse Value
        current_pulse = self.read_encoder_before_gear_ratio()
        logger.info(f"Current Pulse Value: {current_pulse}")
        # Set Target Pulse Value
        diff_pulses = target_pulses - current_pulse
        logging.info(f"Diff Pulses: {diff_pulses}")

        low_byte = abs(diff_pulses) & 0xFFFF
        high_byte = (abs(diff_pulses) >> 16) & 0xFFFF

        self._execute_positioning(diff_pulses, low_byte, high_byte, acc_dec_time, speed_rpm)


    def post_step_motion_by(self, angle: float = 0.0, acc_dec_time: int = 5000, speed_rpm: int =10):
        # 125829120 pulse/rev
        # 349525 + 1/3 pulse/degree
        # 125829120 pulse/rev
        # 349525 + 1/3 pulse/degree
        base_pulse_per_degree = 349525.3333333333
        
        self.previous_angle = self.current_angle
        self.current_angle = angle
        diff_angle = self.current_angle - self.previous_angle

        logger.info(f"Performing motion: Current Angle={self.current_angle}, Previous Angle={self.previous_angle}")

        if diff_angle != 0.0:
            total_pulse = base_pulse_per_degree * abs(diff_angle)
            integer_pulse = int(total_pulse)
            fractional_pulse = total_pulse - integer_pulse

            # Accumulate fractional part
            if diff_angle > 0:
                self.float_error += fractional_pulse
            else:
                self.float_error -= fractional_pulse

            if self.float_error >= 1.0:
                integer_error = int(self.float_error)
                integer_pulse += integer_error
                self.float_error -= integer_error

            self.accumulate_pulse += integer_pulse

            low_byte = integer_pulse & 0xFFFF
            high_byte = (integer_pulse >> 16) & 0xFFFF

            logger.info(f"Motion Pulses: {integer_pulse}, float_error: {self.float_error}")

            self._execute_positioning(diff_angle, low_byte, high_byte, acc_dec_time, speed_rpm)
    
    def _execute_positioning(self, angle, low_byte, high_byte, acc_dec_time, speed_rpm):
        # self.stop_continuous_reading()
        self.Enable_Position_Mode(True)
        self.delay_ms(50)
        self.config_acc_dec_0x0902(acc_dec_time)
        self.delay_ms(50)
        self.config_speed_0x0903(speed_rpm)
        self.delay_ms(50)
        self.config_pulses_0x0905_low_byte(low_byte)
        self.delay_ms(50)
        self.config_pulses_0x0906_high_byte(high_byte)
        self.delay_ms(50)
        
        # self.start_continuous_reading(0.2)
        # self.delay_ms(60)

        if angle > 0:
            logger.info("Running Servo CW")
            self.pos_step_motion_test(True)
        else:
            logger.info("Running Servo CCW")
            self.pos_step_motion_test(False)

    def enable_speed_ctrl(self, speed_rpm):
        self.Enable_JOG_Mode(True)
        self.delay_ms(100)
        self.config_speed_0x0903(speed_rpm)
        self.delay_ms(100)
        self.start_continuous_reading(0.1)

    # 0: Stop
    # 1: CW
    # 2: CCW
    def speed_ctrl_action(self, action_value):
        if action_value == 0:
            logging.info("Servo Stop!")
        elif action_value == 1:
            logging.info("Servo CW")
        elif action_value == 2:
            logging.info("Servo CCW")
        else:
            logging.info("Error Config.")
        self.delay_ms(100)
        address = 0x0904
        message = self.modbus_client.build_write_message(address, action_value)
        response = self.modbus_client.send_and_receive(message)
        response_object = ModbusResponse(response)
        logging.info(response_object)

    def set_home_position(self):
        self.current_angle = 0.0
        self.previous_angle = 0.0
        self.float_error = 0.0
        self.accumulate_pulse = 0
        logger.info("home position set!!!")

    def initial_abs_home(self) -> bool:
        if self.on_initial_home:
            logging.info("Initial absolute home now is running.")
            return False
        
        try:
            self.stop_event.clear()
            self.on_initial_home = True
            self.pos_step_motion_by(self.abs_home_pos, 5000, 12)
            
            if self.stop_event.wait(timeout=10):
                logging.info("Stop process completed successfully.")
                return False
            else:
                logging.warning("Timeout while waiting for stop process.")
                self.stop_continuous_reading()
                return False
        except Exception as e:
            logging.error(f"Error in initial_abs_home: {e}")
            self.stop_continuous_reading()
            return False

