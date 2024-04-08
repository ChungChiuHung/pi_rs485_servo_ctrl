from enum import Enum

class ServoStatus(Enum):
    # Read only
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

    CONTROL_MODE_1 = (0x0200, "Servo ready status.",1)
    CONTROL_MODE_2 = (0x0201, "Control mode of servo drive.",1)

    ALARM_INFO_1 = (0x0100, "Current alarm.", 1)
    ALARM__INFO_2 = (0x0101, "The last alarm", 1)
    ALARM__INFO_3 = (0x0102, "The 2nd alarm", 1)
    ALARM__INFO_4 = (0x0103, "The 3rd alarm", 1)
    ALARM__INFO_5 = (0x0104, "The 4th alarm", 1)
    ALARM__INFO_6 = (0x0105, "The 5th alarm", 1)
    ALARM__INFO_7 = (0x0106, "The 6th alarm", 1)
    ALARM__INFO_8 = (0x0107, "The 7th alarm", 1)
    ALARM__INFO_9 = (0x0108, "The 8th alarm", 1)
    ALARM__INFO_10 = (0x0109, "The 9th alarm", 1)
    ALARM__INFO_11 = (0x010A, "The 10th alarm", 1)

    ALARM_CLEAR_1 = (0x0130, "Clear current alarm if 0x1EA5 is writen into this address",1)
    ALARM_CLEAR_2 = (0x0131, "Clear all alarm hisotries if 0x1EA5 is written into this address", 1)

    # Readable and wriateble
    PA_PARAMETERS = (0x0300, "PA[][]: 50 parameters", 2)
    PB_PARAMETERS = (0x0400, "PB[][]: 50 parameters", 2)
    PC_PARAMETERS = (0x0500, "PC[][]: 60 parameters", 2)
    PD_PARAMETERS = (0x0600, "PD[][]: 40 parameters", 2)
    PE_PARAMETERS = (0x0700, "PE[][]: 99 parameters", 2)
    PF_PARAMETERS = (0x0800, "PF[][]: 99 parameters", 2)

    DEFAULT_SET = (0x0140, "All parameter would be recovered factory_set 1 second later once 1EA5h being written.", 1)
    # To read this address, the result of "1" means the recovery is processing.
    # "0" means the completion of recovery

    SEL_DI_CONTROL_SOURCE = (0x061E, "DI on/off control source option", 2)
    # 0: The specified DI is controlled by the actual wirings.
    # 1: The specified DI is controlled by communication software.

    DI_PIN_CONTROL = (0x0630, "Use bit value to control the corresponding DI on/off state.", 2)
    # 0: Denotes "off" state.
    # 1: Denotes "on" state.

    SERVO_ON = (0x0900, "Servo On/Off, Alarm Code", 1)
    # 0x0ZYX
    # X=0 : Servo Off
    # X=1 : Servo On
    # ZY : Alarm code

    DO_OUTPUT = (0x0901, "Select DO mode",1)
    # 0x0000: To quit this test mode
    # 0x0001: Reserved
    # 0x0002: DO forced output
    # 0x0003: JOG test
    # 0x0004: Position test

    DO_CONTROL = (0x0203, "To control DO status by the written data.",1)

    # JOG Test
    JOG_SPEED = (0x0903, "JOG speed commnad [rpm], Setting range: 0~3000",1)

    JOG_OPERATION = (0x0904, "To Select JOG mode",1)
    # 0: to stop motor running
    # 1: run the motor at forward rotation(CCW)
    # 2: run the motor at reverse rotation(CW)

    # Position test
    POS_SET_ACC = (0x0902, "Acceleration/deceleration time constant[ms], Setting range: 0~20,000",1)

    POS_SET_SPEED = JOG_SPEED

    POS_PULSES_CMD_1 = (0x0905, "Command pulses are composed of these two registers.",1)
    POS_PULSES_CMD_2 = (0x0906, "Command pulses are composed of these two registers.",1)

    POS_EXE_MODE = (0x0907, "Positioning test opeartion",1)
    # 0: Written 0 to pause/stop motor running.(twice pause command to stop motro running)
    # 1: Written 1 to make motor run forward rotation(CCW)
    # 2: Written 2 to make motor run reverse rotation(CW)

    QUIT_MODE = (0x0901,"To quit this mode by 0x0000 writen at address",1)
    # written 0x0000
    




    def __init__(self, address, description, data_length, di_bits=None, do_bits=None):
        self.address = address
        self.description = description
        self.data_length = data_length

    
    @staticmethod
    def decode_di_status(di_status_byte):
        di_bits = {
            'DI1': (0, "Description of DI1"),
            'DI2': (1, "Description of DI2"),
            'DI3': (2, "Description of DI3"),
            'DI4': (3, "Description of DI4"),
            'DI5': (4, "Description of DI5"),
            'DI6': (5, "Description of DI6"),
            'DI7': (6, "Description of DI7"),
            'DI8': (7, "Description of DI8"),
            'DI8': (8, "Description of DI9"),
            'DI8': (9, "Description of DI10"),
            'DI8': (10, "Description of DI11"),
            'DI8': (11, "Description of DI12"),
        }
        return {pin_name: {'status': bool(di_status_byte & (1 << bit_position)), 'description': bit_description}
                for pin_name, (bit_position, bit_description) in di_bits.items()}
    
    @staticmethod
    def decode_do_status(do_status_byte):
        do_bits ={
            'DO1': (0, "Description of DO1"),
            'DO2': (1, "Description of DO2"),
            'DO3': (2, "Description of DO3"),
            'DO4': (3, "Description of DO4"),
            'DO5': (4, "Description of DO5"),
            'DO6': (5, "Description of DO6"),
        }
        do_status = {}
        for pin_name, (bit_position, bit_description) in do_bits.items():
            status = bool(do_status_byte & (1 << bit_position))
            do_status[pin_name] = {'status': status, 'description': bit_description}
        return do_status
    
    @staticmethod
    def decode_ctrl_mode_1_status(ctrl_mode_1_byte):
        ctrl_mode_1_bits={
            'Servo ON/OFF':(0, "Servo ready status (0: Servo OFF, 1: Servo ON)"),
        }
        return {pin_name: {'status': bool(ctrl_mode_1_byte & (1 << bit_position)), 'description': bit_description}
                for pin_name, (bit_position, bit_description) in ctrl_mode_1_bits.items()}

    @staticmethod
    def decode_ctrl_mode_2_status(ctrl_mode_2_byte):
        ctrl_mode_2_bits={
            'Pt mode': (0, "external pulse-train command"),
            'Pr mode 1': (1, "inner register command in absolute type"),
            'Pr mode 2': (2, "inner register command in incremental type"),
            'S mode': (3, "speed mode"),
            'T mode': (4, "torque mode"),
            'Turret mode': (6, "Turret mode"),
        }
        ctrl_status = {}
        for pin_name, (bit_position, bit_description) in ctrl_mode_2_bits.items():
            # Check if the specific bit position is set (1)
            status = bool(ctrl_mode_2_byte & (1 << bit_position))
            ctrl_status[pin_name] = {'status': status, 'description': bit_description}
        return ctrl_status
