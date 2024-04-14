from enum import Enum, auto
from collections import namedtuple

Register = namedtuple('Register', 'address description data_length')

class ServoControlRegistry(Enum):
    MOTOR_FEEDBACK_PULSE = Register(0x0000, "Motor feedback pulse [pulse]", 2)
    COMMAND_PULSES = Register(0x0002, "Command pulses [rev]", 2)
    ACCUMULATIVE_PULSES_ERROR = Register(0x0004, "Accumulative pulses error [pulse]", 2)
    COMMAND_PULSE_FREQUENCY = Register(0x0006, "Command pulse frequency [Hz]", 2)
    MOTOR_SPEED = Register(0x0008, "Motor speed [rpm]", 2)
    ANALOG_SPEED_COMMAND_LIMIT_VOLTAGE = Register(0x000A, "Analog speed command/limit voltage [V]", 2)
    SPEED_INPUT_COMMAND_LIMIT = Register(0x000C, "Speed input command/limit [rpm]", 2)
    ANALOG_TORQUE_COMMAND_LIMIT_VOLTAGE = Register(0x001E, "Analog Torque command/limit voltage [V]", 2)
    TORQUE_INPUT_COMMAND_LIMIT = Register(0x0010, "Torque input command/limit [%]", 2)
    EFFECTIVE_LOAD_RATIO = Register(0x0012, "Effective load ratio [%]", 2)
    PEAK_LOAD_RATIO = Register(0x0014, "Peak load ratio [%]", 2)
    DC_BUS_VOLTAGE = Register(0x0016, "DC bus voltage [V]", 2)
    LOAD_TO_MOTOR_INERTIA_RATIO = Register(0x0018, "Load to motor inertia ratio [times]", 2)
    INSTANTANEOUS_TORQUE = Register(0x001A, "Instantaneous torque [%]", 2)
    REGENERATION_LOAD_RATIO = Register(0x001C, "Regeneration load ratio [%]", 2)
    PULSES_OF_Z_PHASE_REFERENCE_ACKNOWLEDGED = Register(0x0020, "Pulses of Z phase reference acknowledged [pulse]", 2)
    TRANSLATED_COMMAND_PULSES = Register(0x0022, "Translated command pulses [pulse]", 2)
    TRANSLATED_MOTOR_FEEDBACK_PULSES = Register(0x0024, "Translated motor feedback pulses [pulse]", 2)
    TRANSLATED_ACCUMULATIVE_PULSES_ERROR = Register(0x0026, "Translated accumulative pulses error [pulse]", 2)
    DI_STATUS = Register(0x0204, "DI status", 1)
    DO_STATUS = Register(0x0205, "DO status", 1)
    CONTROL_MODE_1 = Register(0x0200, "Servo ready status", 1)
    CONTROL_MODE_2 = Register(0x0201, "Control mode of servo drive", 1)
    DEFAULT_SET = Register(0x0140, "Recover factory settings", 1)
    SERVO_ON = Register(0x0900, "Servo On/Off, alarm Code", 1)
    DO_OUTPUT = Register(0x0901, "Select DO mode", 1)
    JOG_SPEED = Register(0x0903, "JOG speed command [rpm], Setting range: 0~3000", 1)
    JOG_OPERATION = Register(0x0904, "To Select JOG mode", 1)
    POS_SET_ACC = Register(0x0902, "Acceleration/deceleration time[ms]", 1)
    POS_PULSES_CMD_1 = Register(0x0905, "Command pulses", 1)
    POS_PULSES_CMD_2 = Register(0x0906, "Command pulses", 1)
    POS_EXE_MODE = Register(0x0907, "Positioning test operation", 1)
    QUIT_MODE = Register(0x0901, "To quit this mode by 0x0000 written at address", 1)
    SEL_DI_CONTROL_SOURCE = Register(0x061E, "DI on/off control source option", 2)
    DI_PIN_CONTROL = Register(0x0630, "Use bit value to control the corresponding DI on/off state", 2)
    DO_CONTROL = Register(0x0203, "To control DO status by the written data", 1)
    DI_FUNCTION_1_2 = Register(0x0206, "Function of DI1 and DI2", 2)
    DI_FUNCTION_3_4 = Register(0x0207, "Function of DI3 and DI4", 2)
    DI_FUNCTION_5_6 = Register(0x0208, "Function of DI5 and DI6", 2)
    DI_FUNCTION_7_8 = Register(0x0209, "Function of DI7 and DI8", 2)
    DI_FUNCTION_9_10 = Register(0x020A, "Function of DI9 and DI10", 2)
    DI_FUNCTION_11_12 = Register(0x020B, "Function of DI11 and DI12", 2)

    def __new__(cls, address, description, data_length):
        obj = object.__new__(cls)
        obj._value_ = address
        obj.address = address
        obj.description = description
        obj.data_length = data_length
        return obj
    
    @classmethod
    def calculate_dynamic_address(cls, block, index):
        if block not in cls.DYNAMIC_PARAM_INFO():
            raise ValueError(f"Invalid parameter block specified: {block}")
        base_address, count = cls.DYNAMIC_PARAM_INFO()[block]
        if not 1 <= index <= count:
            raise ValueError(f"Parameter index {index} out of allowed range (1-{count}) for block {block}")
        return base_address + (index -1) * 4
    
    @staticmethod
    def DYNAMIC_PARAM_INFO():
        return {
            "PA": (0x0300, 50),
            "PB": (0x0400, 50),
            "PC": (0x0500, 60),
            "PD": (0x0600, 40),
            "PE": (0x0700, 99),
            "PF": (0x0800, 99),
        }
    
    def __str__(self):
        """Returns a string representation of the registry with its description."""
        return f"{self.description} (Address: {hex(self.address)}, Data Length: {self.data_length})"