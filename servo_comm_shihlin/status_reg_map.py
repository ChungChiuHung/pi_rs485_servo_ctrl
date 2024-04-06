from enum import Enum 

class RegMap(Enum):
    FEEDBACK_PULSES = [0x0000]
    COMMAND_PULSES = [0x0002]
    DIFF_PULSES = [0x0004]
    COMMAND_FREQ = [0x0006]
    VELOCITY_RPM = [0x0008]
    ANALOG_VOLTAGE = [0x000A]
    SPEED_RPM_LIMIT = [0x000C]
    TORQUE_LIMIT = [0x000E]  # unit: %
    REAL_LOAD_RATIO = [0x0012]
    PEAK_LOAD_RATIO = [0x0014]
    DC_BUS_VOLTAGE = [0x0016]
    LOAD_INERTIA_RATIO = [0x0018]  # 1 decimal place
    INSTANT_TORQUE = [0x001A]  # unit: %
    BACKLOAD_RATIO = [0x001C]  # unit: %
    RELATIVE_Z_ABS_PULSES = [0x0020]
    BEFORE_GEAR_RATIO_PULSES = [0x0022]
    AFTER_GEAR_RATIO_FEEDBACK_PULSES = [0x0024]
    BEFORE_GEAR_RATIO_DIFF_PULSES = [0x0026]
    DI_STATES_1 = [0x0204] # DI1~DI12
    DO_STATES_2 = [0x02, 0x05] # DO1~DO6
    DIO_STATE_1 = [0x02, 0x06] # DI1~DI2

    SERVO_ON = [0x02, 0x00]
    CTRL_MODE = [0x02, 0x01]
    # 0 : PT Mode
    # 1 : ABS Mode
    # 2 : 增量型 PR 位置模式
    # 3 : Speed Mode
    # 4 : Torque Mode
    # 6 : 刀庫控制模式

    # 兩個Z相 脈波命令間隔為10,000 pulse

    ALAM_INFO = [0x0100, 0x010A]

    ALAM_CLEAR = [0x0130, 0x0131]

    DATA_ADDRESS_PA = [0x0300, 0x0363]
    DATA_ADDRESS_PB = [0x0400, 0x0463]
    DATA_ADDRESS_PC = [0x0500, 0x0577]
    DATA_ADDRESS_PD = [0x0600, 0x064F]
    DATA_ADDRESS_PF = [0x0800, 0x08C5]

    SET_DEFAULT_ADDR = [0x014] 
    # Set Value : 0x1EA5, 
    # Reset all default parameter from PA~PF After 3 seconds
    # Response = 1 => 寫入EEPROM 中
    # Response = 0 => 寫入EEPROM 完成

    SEL_INPUT_IO_MODE = 0x061E
    # Bit0~Bit11/DI1~DI12
    # Set Bit as 0 : External Signal
    # Set Bit as 1 : Internal Signal

    SET_INPUT_IO_STATE = 0x0630
    # Config range : 0x0000 ~ 0x0FFF









