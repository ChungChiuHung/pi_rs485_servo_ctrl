from time import sleep
class CmdDelayTime:
    def __init__(self, transmission_speed_bps):
        self.transmission_speed_bps = transmission_speed_bps

    def calculate_transmission_time_ms(self, byte_array):
        message_length_bits = len(byte_array)*8
        delay_time_sec = message_length_bits/self.transmission_speed_bps
        delay_time_ms = delay_time_sec*1000
        return delay_time_ms