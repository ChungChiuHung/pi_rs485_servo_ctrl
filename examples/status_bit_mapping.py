from enum import Enum

class StatusBitPositions(Enum):
    HOME = 19
    ORG = 20
    PCPAUSE = 21
    PCCANCEL = 22
    PCSTART1 = 24
    SEL_NO = (26, 29)  # Range for SEL_NO bits
    SVON = 0
    TLSEL1 = 13
    RESET_PCLR = 15