from enum import Enum

class CmdCode(Enum):
    READ_DATA = 0x03
    WRITE_DATA = 0x06
    Diagnostic_MODE = 0x08
    WRITE_MULTI_DATA=0X10


class ExceptionCode(Enum):
    COMMAND_CODE_ERROR = 0x01
    PARAMETER_ADDRESS_ERROR = 0x02
    PARAMETER_RANGE_ERROR = 0x03

    # Note on Modbus RTU Error Responses:
    # For errors, the device responds with the function code + 0x80 and a exception code.
    # Example (RTU Mode): ADR (Address) + CMD (Command) + ECP (Exception Code) + CRC_L + CRC_H
    # Example error response: 01 (Address) + 86 (Function Code + 0x80 for error) + 02 (Parameter Address Error) + C3 A1 (CRC)

    