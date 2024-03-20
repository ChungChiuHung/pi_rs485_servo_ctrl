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