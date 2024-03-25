class ServoControlUtils:
    @staticmethod
    def delay_ms(milliseconds):
        import time
        time.sleep(milliseconds / 1000.0)

    @staticmethod
    def print_byte_array_as_spaced_hex(byte_array, description="Data"):
        hex_string = ' '.join(f"{byte:02X}" for byte in byte_array)
        print(f"{description}: {hex_string}")