class BitStatusSetter:
    def __init__(self):
        self.status_bit_positions = {
        "HOME": 19,
        "ORG": 20,
        "PCPAUSE": 21,
        "PCCANCEL": 22,
        "PCSTART1": 24,
        "SEL_NO": (26, 29),  # Range for SEL_NO bits
        "SVON": 0,
        "TLSEL1": 13,
        "RESET/PCLR": 15,
        }

    def set_bit_status(self, status_name, value):
        # Initialize status and mask values.
        status_value = 0
        mask_value = 0

        # Handling SEL_NO which spans multiple bits.
        if status_name == "SEL_NO":
            if not (0<=value <=15):
                raise ValueError("SEL_NO value must be between 0 and 15.")
            # Set value in the correct bit positions and define mask for SEL_NO.
            status_value |= (value << 26)
            mask_value |= (0xF << 26) # 0xF in bits 26 to 29 sets mask to 0x3C000000
        else:  
            # Handling single-bit statuses.
            if status_name not in self.status_bit_positions:
                raise ValueError(f"{status_name} is not a valid status.")
            bit_position = self.status_bit_positions[status_name]
            if value:
                status_value |= (1 << bit_position)
            mask_value |= (1 << bit_position)

        # Convert status and mask to byte arrays.
        status_bytes = status_value.to_bytes(4, byteorder='big')
        mask_bytes = mask_value.to_bytes(4, byteorder='big')

        return status_bytes, mask_bytes