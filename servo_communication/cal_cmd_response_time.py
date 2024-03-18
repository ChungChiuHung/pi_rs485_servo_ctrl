class CommAnalyzer:
    def calculate_transmission_time_ms(self, byte_array, transmission_speed_bps):
        command_length_bits = len(byte_array)*8
        return (command_length_bits/transmission_speed_bps)*1000