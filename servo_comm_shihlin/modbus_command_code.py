from enum import Enum

class CmdCode(Enum):
    READ_DATA = 0x03
    WRITE_DATA = 0x06
    Diagnostic_MODE = 0x08
    WRITE_MULTI_DATA=0X10

