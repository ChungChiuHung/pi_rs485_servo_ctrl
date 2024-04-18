def config_bit(**kwargs):
    bit_positions = {
        'DI1': 0, 'DI2': 1, 'DI3': 2, 'DI4': 3, 'DI5': 4, 'DI6': 5,
        'DI7': 6, 'DI8': 7, 'DI9': 8, 'DI10': 9, 'DI11': 10, 'DI12': 11
    }
    value = 0
    for bit, position in bit_positions.items():
        if kwargs.get(bit, 0) == 1:
            value |= 1 << position

    return hex(value)


value_1 = config_bit(DI1=1, DI2=1, DI3=1) # the value_1 = 111
value_2 = config_bit(DI4=1) # the value_2 = 1111

print("Value 1 (hex):", value_1)  # the value = 0x7
print("Value 2 (hex):", value_2)  # the value = 0xF

value_3 = config_bit(DI= 0, DI2 = 0, DI3 = 0)

print("Value 3 (hex):", value_3) # the value = 0x8