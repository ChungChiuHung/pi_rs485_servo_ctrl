from enum import Enum

class DeviceStatus(Enum):
    MOTOR_FEEDBACK_PULSE = (0x0000, "Motor feedback pulse [pulse]", 2)
    COMMAND_PULSES = (0x0002, "Command pulses [rev]", 2)
    ACCUMULATIVE_PULSES_ERROR = (0x0004, "Accumulative pulses error [pulse]", 2)
    COMMAND_PULSE_FREQUENCY = (0x0006, "Command pulse frequency [Hz]", 2)
    MOTOR_SPEED = (0x0008, "Motor speed [rpm]", 2)
    ANALOG_SPEED_COMMAND_LIMIT_VOLTAGE = (0x000A, "Analog speed command/limit voltage [V]", 2)
    SPEED_INPUT_COMMAND_LIMIT = (0x000C, "Speed input command/limit [rpm]", 2)
    ANALOG_TORQUE_COMMAND_LIMIT_VOLTAGE = (0x001E, "Analog Torque command/limit voltage [V]", 2)
    TORQUE_INPUT_COMMAND_LIMIT = (0x0010, "Torque input command/limit [%]", 2)
    EFFECTIVE_LOAD_RATIO = (0x0012, "Effective load ratio [%]", 2)
    PEAK_LOAD_RATIO = (0x0014, "Peak load ratio [%]", 2)
    DC_BUS_VOLTAGE = (0x0016, "DC bus voltage [V]", 2)
    LOAD_TO_MOTOR_INERTIA_RATIO = (0x0018, "Load to motor inertia ratio [times]", 2)
    INSTANTANEOUS_TORQUE = (0x001A, "Instantaneous torque [%]", 2)
    REGENERATION_LOAD_RATIO = (0x001C, "Regeneration load ratio [%]", 2)
    PULSES_OF_Z_PHASE_REFERENCE_ACKNOWLEDGED = (0x0020, "Pulses of Z phase reference acknowledged [pulse]", 2)
    TRANSLATED_COMMAND_PULSES = (0x0022, "Translated command pulses [pulse]", 2)
    TRANSLATED_MOTOR_FEEDBACK_PULSES = (0x0024, "Translated motor feedback pulses [pulse]", 2)
    TRANSLATED_ACCUMULATIVE_PULSES_ERROR = (0x0026, "Translated accumulative pulses error [pulse]", 2)

    DI_STATUS = (0x0204, "To show the on/off status of DI.", 1)
    DO_STATUS = (0x0205, "To show the on/off status of DO.", 1)

    

    def __init__(self, address, description, data_length):
        self.address = address
        self.description = description
        self.data_length = data_length




