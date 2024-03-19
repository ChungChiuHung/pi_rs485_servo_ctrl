from status_bit_mapping import StatusBitPositions

class SetServoIOStatus:
    def __init__(self):
        pass
    
    def set_bit_status(self, status_name, value):
        # Initialize status and mask values.
        status_value = 0
        mask_value = 0

        # Handling SEL_NO which spans multiple bits.
        if status_name == StatusBitPositions.SEL_NO:
            if not (0<=value <=15):
                raise ValueError("SEL_NO value must be between 0 and 15.")
            # Set value in the correct bit positions and define mask for SEL_NO.
            status_value |= (value << StatusBitPositions.SEL_NO.value[0])
            mask_value |= (0xF << StatusBitPositions.SEL_NO.value[0]) # 0xF in bits 26 to 29 sets mask to 0x3C000000
        else:  
            # Handling single-bit statuses.
            if status_name not in StatusBitPositions:
                raise ValueError(f"{status_name} is not a valid status name.")
            
            bit_position = status_name.value
            if value:
                status_value |= (1 << bit_position)
            mask_value |= (1 << bit_position)

        # Convert status and mask to byte arrays.
        status_bytes = status_value.to_bytes(4, byteorder='big')
        mask_bytes = mask_value.to_bytes(4, byteorder='big')
            
        status_bytes_list = list(status_bytes)
        mask_bytes_list = list(mask_bytes)

        return status_bytes_list + mask_bytes_list