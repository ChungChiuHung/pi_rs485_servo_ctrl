class ServoUtility:
    @staticmethod
    def decode_di_status(di_status_byte):
        di_bits ={
            'DI1':(0, "Description of DI1"),
            'DI2':(1, "Description of DI2"),
            'DI3':(2, "Description of DI3"),
            'DI4':(3, "Description of DI4"),
            'DI5':(4, "Description of DI5"),
            'DI6':(5, "Description of DI6"),
            'DI7':(6, "Description of DI7"),
            'DI8':(7, "Description of DI8"),
            'DI9':(8, "Description of DI9"),
            'DI10':(9, "Description of DI10"),
            'DI11':(10, "Description of DI11"),
            'DI12':(11, "Description of DI12"),
        }
        di_statuses={}
        for pin_name, (bit_position, bit_description) in di_bits.items():
            status = bool(di_status_byte & (1 << bit_position))
            di_statuses[pin_name] = {'status': status, 'description': bit_description}
        return di_statuses
    
    @staticmethod
    def decode_do_status(do_status_byte):
        do_bits = {
            'DO1': (0, "Description of DO1"),
            'DO2': (1, "Description of DO2"),
            'DO3': (2, "Description of DO3"),
            'DO4': (3, "Description of DO4"),
            'DO5': (4, "Description of DO5"),
            'DO6': (5, "Description of DO6"),
        }

        do_statuses={}
        for pin_name, (bit_position, bit_description) in do_bits.items():
            status = bool(do_status_byte & (1 << bit_position))
            do_statuses[pin_name] = {'status':status, 'description':bit_description}
        return do_statuses
    
    @staticmethod
    def decode_control_mode(ctrl_mode_byte):
        ctrl_mode_bits={
            'Pt mode': (0, "External pulse-train command"),
            'Pr_Abs mode': (1, "Absolute type"),
            'Pr_Inc mode':(2, "Incremental type"),
            'S mode':(3, "Speed mode"),
            'T mode':(4, "Torque mode"),
            'Turret mode':(6, "Turret mode"),
        }
        ctrl_modes = {}
        for mode_name, (bit_position, mode_description) in ctrl_mode_bits.items():
            mode_active = bool(ctrl_mode_byte & (1 << bit_position))
            ctrl_modes[mode_name] = {'active':mode_active, 'description': mode_description}
        return ctrl_modes
    
    @staticmethod
    def get_alarm_description(alarm_code):
        alarms = {

        }
        return alarms.get(alarm_code, "Unknown Alarm")
    
    @staticmethod
    def decode_di_function(di_function_word):
        """
        Decode the DI function values encoded in a word
        Each DI_FUNCTION register encodes the functions for two DIs.
        -The hight byte (15th to 8th bits) encodes the function for the first DI.
        -The low byte (7th to 0th bits) encodes the function for the second DI.
        Function value range from 0x00 to 0x2F

        :param di_function_word: The word read from a DI_FUNCTION register.
        :return A tuple (di1_function, di2_function) representing the function values.
        """
        di1_function = (di_function_word >> 8) & 0xFF
        di2_function = di_function_word & 0xFF
        return di1_function, di2_function
    
    @staticmethod
    def config_hex_with(x, y, z, u):
        if any(not(0 <= a < 16) for a in [x,y,z,u]):
            raise ValueError("All inputs must be within the range 0 to 15 (inclusive).") 
        value = (x << 12) | (y << 8)| (z << 4) | u

        print(f"Config value: {hex(value)}")
