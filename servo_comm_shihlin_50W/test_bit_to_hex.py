class MotionController:
    def __init__(self):
        self.float_error= 0
        self.accumulate_pulse = 0
        self.current_angle = 0
        self.previous_angle = 0

    def config_bit(self,**kwargs):
        bit_positions = {
            'DI1': 0, 'DI2': 1, 'DI3': 2, 'DI4': 3, 'DI5': 4, 'DI6': 5,
            'DI7': 6, 'DI8': 7, 'DI9': 8, 'DI10': 9, 'DI11': 10, 'DI12': 11
        }
        value = 0
        for bit, position in bit_positions.items():
            if kwargs.get(bit, 0) == 1:
                value |= 1 << position

        return hex(value)

    def post_step_motion_by(self, angle=0):
        # 125829120 pulse/rev
        # 349525 + 1/3 pulse/degree
        base_pulse_per_degree = 345625
        output_pulse = 0

        print("\n")
        print(f"Current Angle: {self.current_angle}")
        print(f"Previous Angle: {self.previous_angle}")
        print(f"Set Angle: {angle}")
        
        self.previous_angle = self.current_angle
        self.current_angle = angle
        diff_angle = self.current_angle - self.previous_angle

        if angle == 0:
            output_pulse = self.accumulate_pulse
            self.accumulate_pulse = 0
            self.float_error = 0
        else:
            if diff_angle != 0:
                current_fraction_part = diff_angle
                total_fraction_part = self.float_error + current_fraction_part
                output_pulse = (base_pulse_per_degree * diff_angle) + (total_fraction_part//3)

                the_left_fraction_part = current_fraction_part % 3
                self.float_error = the_left_fraction_part
                self.accumulate_pulse += output_pulse

        low_byte = abs(output_pulse) & 0xFFFF
        high_byte = (abs(output_pulse) >> 16) & 0xFFFF

        if output_pulse > 0:
            print("Running Servo CW")
        else:
            print("Running Servo CCW")
        
        print("\n")
        print(f"Output pulse: {output_pulse}")
        print(f"float error: {self.float_error}")
        print(f"Current Accumulate Pulse: {self.accumulate_pulse}")
        print (f"{hex(high_byte)}, {hex(low_byte)}")

        return (high_byte, low_byte)

    
    def math_test(self):
        value = 5//3
        value_2 = 5/3
        value_3 = 5 % 3
        print(f"5//3 = {value}")
        print(f"5/3 = {value_2}")
        print(f"5%3 = {value_3}")

if __name__ =="__main__":
    mc = MotionController()

    value_1 = mc.config_bit(DI1=1, DI2=1, DI3=1) # the value_1 = 111
    value_2 = mc.config_bit(DI4=1) # the value_2 = 1111
    value_3 = mc.config_bit(DI= 0, DI2 = 0, DI3 = 0)

    print("Value 1 (hex):", value_1)  # the value = 0x7
    print("Value 2 (hex):", value_2)  # the value = 0xF
    print("Value 3 (hex):", value_3) # the value = 0x8

    mc.math_test()

    print("Test Angle To Pulse")
    byte_value = mc.post_step_motion_by(50)
    print(byte_value)
    byte_value = mc.post_step_motion_by(100)
    print(byte_value)
    byte_value = mc.post_step_motion_by(150)
    print(byte_value)
    byte_value = mc.post_step_motion_by(100)
    print(byte_value)