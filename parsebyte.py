class HexParser:
    def __init__(self):
        self.error_mapping = {
             0: "Error 0",
            1: "Error 1",
            2: "Error 2",
            3: "Error 3",
            4: "Error 4",
            5: "Error 5",
            6: "Error 6",
            7: "Error 7",
            8: "Error 8",
            9: "Error 9"
        }

    def parse_byte(self, hex_value):
        # Convert hex to binary
        bin_value = bin(hex_value)[2:].zfill(8)
        # Extract bits
        bit_7 = "On" if bin_value[0] == '1' else "Off"
        direction = "CW" if bin_value[1] == '1' else "CCW"
        position = int(bin_value[2:4], 2)
        error_code = int(bin_value[4:], 2)
        error_msg = self.error_mapping.get(error_code, "Unknown error")

        return {
            "7th Bit": bit_7,
            "Direction" : direction,
            "Position" : position,
            "Error Code": error_msg
        }
    
    @staticmethod
    def test_function():
        hex_parser = HexParser()
        test_value=0x2A # Binary: 00101010
        expected_output = {
            "7th Bit": "Off",
            "Direction": "CCW",
            "Position":2,
            "Error Code": "Unknown error"
        }

        assert hex_parser.parse_byte(test_value) == expected_output, "Test failed!"
        print("Test passed successfully.")