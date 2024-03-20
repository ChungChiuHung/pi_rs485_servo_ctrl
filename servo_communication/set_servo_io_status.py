from status_bit_mapping import BitMap

class SetServoIOStatus:
    def __init__(self):
        pass
    
    def set_bit_status(self, status_name, value):
        status_no = 288
        status_value = 0
        mask_value = 0

        if isinstance(status_name.value, tuple):
            bit_range = status_name.value
            if not (0<= value < 2 ** (bit_range[1] - bit_range[0] + 1)):
                raise ValueError(f"Value for {status_name} must be within the appropriate range.")

            status_value |= (value << bit_range[0])
            mask_value |= ((2 ** (bit_range[1] - bit_range[0] + 1) -1) << bit_range[0])

        else:
            if value not in [0,1]:
                raise ValueError(f"Value for {status_name} must be 0 or 1. ")
            
            status_value |= (value << status_name.value)
            mask_value |= (1 << status_name.value)

        # Convert status number and values to byte arrays
        status_no_bytes = status_no.to_bytes(2, byteorder='big')
        status_bytes = status_value.to_bytes(4, byteorder='big')
        mask_bytes = mask_value.to_bytes(4, byteorder='big')
        
        # Combine and return the full message bytes
        return status_no_bytes + status_bytes + mask_bytes
    
    def set_respone_bit(self, status_name, value):
        # status_no = 288
        status_value = 0
        mask_value = 0

        if isinstance(status_name.value, tuple):
            bit_range = status_name.value
            if not (0<= value < 2 ** (bit_range[1] - bit_range[0] + 1)):
                raise ValueError(f"Value for {status_name} must be within the appropriate range.")

            status_value |= (value << bit_range[0])
            mask_value |= ((2 ** (bit_range[1] - bit_range[0] + 1) -1) << bit_range[0])

        else:
            if value not in [0,1]:
                raise ValueError(f"Value for {status_name} must be 0 or 1. ")
            
            status_value |= (value << status_name.value)
            mask_value |= (1 << status_name.value)

        # Convert status number and values to byte arrays
        # status_no_bytes = status_no.to_bytes(2, byteorder='big')
        status_bytes = status_value.to_bytes(4, byteorder='big')
        mask_bytes = mask_value.to_bytes(4, byteorder='big')

        # Combine and return the full message bytes
        return status_bytes + mask_bytes