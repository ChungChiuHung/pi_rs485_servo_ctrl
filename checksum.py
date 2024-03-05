class CRC16CCITT:
    """
    A class to calculate CRC-16-CCITT checksums and append it to the input data,
    with the option to return the result as a list of integers.
    """
    def __init__(self, poly=0x1021, init_val=0xFFFF):
        """
        Initializes the CRC calculator instance.
        
        :param poly: Polynomial used for CRC calculation. Default is 0x1021 (CRC-16-CCITT).
        :param init_val: Initial value of the CRC. Default is 0xFFFF.
        """
        self.poly = poly
        self.init_val = init_val

    def append_crc(self, data):
        """
        Calculate the CRC-16-CCITT checksum of input data and append it to the input.
        
        :param data: Input data as bytes or bytearray.
        :return: A new byte array containing the input data followed by the CRC checksum.
        """
        crc = self.init_val
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ self.poly
                else:
                    crc <<= 1
            crc &= 0xFFFF  # Trim the CRC to 16 bits
        
        return data + crc.to_bytes(2, 'big')

    def append_crc_as_list(self, data):
        """
        Calculate the CRC-16-CCITT checksum of input data, append it to the input,
        and return the result as a list of integers.
        
        :param data: Input data as a list of integers.
        :return: A list of integers containing the input data followed by the CRC checksum.
        """
        data_bytes = bytes(data)
        result_with_crc = self.append_crc(data_bytes)
        return list(result_with_crc)

# Example usage:
# crc_calculator = CRC16CCITT()
# test_data_list = [0x24, 0x01, 0x00, 0x11, 0x01, 0x28]
# result_data_with_crc_list = crc_calculator.append_crc_as_list(test_data_list)

# print("Original data as list:", test_data_list)
# print("Data with CRC-16-CCITT appended as list:", result_data_with_crc_list)
