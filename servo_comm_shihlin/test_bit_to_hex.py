class MotionController:
    def __init__(self):
        self.float_error= 0

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
        # 349525 + 1/3
        pulse_value = 345625 * angle + (angle + self.float_error) // 3

        fraction_part = angle % 3
        if fraction_part !=0:
            self.float_error += fraction_part
        else:
            self.float_error = 0

        print(f"Angle: {angle} [degree]")
        print(f"Total Pulse: {pulse_value}")
        print(f"float_error: {self.float_error}")
        low_byte = pulse_value & 0xFFFF
        high_byte = (pulse_value >> 16) & 0xFFFF
        print (f"Config Bytes: {hex(high_byte)}, {hex(low_byte)}\n")
        

if __name__ =="__main__":
    mc = MotionController()

    value_1 = mc.config_bit(DI1=1, DI2=1, DI3=1) # the value_1 = 111
    value_2 = mc.config_bit(DI4=1) # the value_2 = 1111
    value_3 = mc.config_bit(DI= 0, DI2 = 0, DI3 = 0)

    print("Value 1 (hex):", value_1)  # the value = 0x7
    print("Value 2 (hex):", value_2)  # the value = 0xF
    print("Value 3 (hex):", value_3) # the value = 0x8

    print("Test Angle To Pulse")
    mc.post_step_motion_by(0)
    mc.post_step_motion_by(1)
    mc.post_step_motion_by(2)
    mc.post_step_motion_by(3)
    mc.post_step_motion_by(4)
    mc.post_step_motion_by(5)
    mc.post_step_motion_by(6)
    mc.post_step_motion_by(360)