from enum import Enum

class BitMap(Enum):
    HOME = 19
    ORG = 20
    PAUSE = 21
    CANCEL = 22
    START1 = 24
    SEL_NO = (26, 29)  # Range for SEL_NO 4 bits
    SVON = 0
    TLSEL1 = 13
    RESET_PCLR = 15
#    PROTO_ID = (6,7)
#    DATA_LEN_BIT = (0,4)
#    DIR_BIT = 7
#    TOGGLE_BIT = 6
#    Error_code = (0,3)
#    COMM_GROUP = (6,7)
#    COMM_CODE = (0,5)

class BitMapOutput(Enum):
    MEND = 20 # Motion complete
    PAUSED = 21 # Pause
    HEND = 22 # Homeing complete
    POINT_SELECT = (24, 27) # Point No. Select
    MEND_T_LIMIT = 30 # Motion complete/Torque limiting
    POSIN = 0 # Position complete
    SREDY = 5 # Servo ready
    T_LIMIT = 6 # Torque Limiting
    MBRK = 8 # Break release 
    SERVO = 11 # Servo status
    ALM = 12 # Alarm output

class BitMapAlarm(Enum):
    SYSTEM_ERROR = 0
    EEPROM_DATA_ERROR = 1
    PRODUCT_CODE_ERROR = 2
    OVER_SPEED_ERROR = 4
    SPEED_DEVIATION_ERROR = 5
    POSITION_DEVIATION_ERROR = 6
    OVERLOAD_ERROR = 7
    COMMAND_OVERSPEED_ERROR = 8
    ENCODER_PLUSE_OUTPUT_FREQUENCY_ERROR = 9
    INTERNAL_POSITION_COMMAND_OVERFLOW_ERROR_HOMING_FAILIURE = 10
    ENCODER_ERROR = 11 # multi-turn counter overflow
    OVERHEAT_ERROR = 12
    OVERVOLTAGE_ERROR = 14
    POWER_SUPPLY_ERROR = 15 # pimary circuit power
    ENCODER_ERROR_Received_data = 16 # Received data
    ENCODER_ERROR_no_response = 17
    ENCODER_ERROR_circuitry = 18
    ENCODER_ERROR_communication = 19
    ENCODER_ERROR_multi_turn_data = 20
    ENCODER_ERROR_voltage_drop = 21
    ENCODER_ERROR_control_power = 22
    SWITCH_CIRCUITRY_ERROR = 23
    OVER_CURRENT_ERROR = 24
    INVERTER_ERROR_1 = 25
    INVERTER_ERROR_2 = 26
    CURRENT_SENSOR_ERROR = 27
    ENCODER_ERROR_Overheat = 28
    VOLTAGE_DROP_insdie_the_amplifier = 29