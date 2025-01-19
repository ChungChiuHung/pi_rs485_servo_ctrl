class Register:
    def __init__(self, no, name, description, value, address):
        self.no = no
        self.name = name
        self.description = description
        self.address = address
        self.value = value

    def read_value(self):
        print(f"Reading value from hardware for {self.name} at address {hex(self.address)}")
    
    def write_value(self, new_value):
        self.value = new_value
        print(f"Written {new_value} to {self.name} at address {hex(self.address)}")

    def __str__(self):
        return f"{self.no}: {self.name}, {self.description}, Start Address: {hex(self.address)}, Value: {self.value if self.value is not None else 'Not Set'}"
    
class PA:
    _start_address = 0x0300
    _register_size = 2

    @classmethod
    def calculate_address(cls, no):
        return cls._start_address + (no -1) * cls._register_size
    @classmethod
    def init_registers(cls):
        cls.STY = Register(1, "STY", "Control mode option", 0x1000, cls.calculate_address(1))
        cls.ATUM = Register(2, "ATUM", "Gain tuning mode option", 0x0002, cls.calculate_address(2))
        cls.ATUL = Register(3, "ATUL", "Auto-tuning mode option", 10, cls.calculate_address(3))
        cls.HMOV = Register(4, "HMOV", "Home moving option", 0x0000, cls.calculate_address(4))
        cls.TL1 = Register(5, "TL1", "Inner torque limit 1 [%]", 100, cls.calculate_address(5))
        cls.CMX = Register(6, "CMX", "Electornic gear numerator", 1, cls.calculate_address(6))
        cls.CDV = Register(7, "CDV", "Electornic gear denominator", 1, cls.calculate_address(7))
        cls.PLSS = Register(13, "PLSS", "Command pulse option", 0x0000, cls.calculate_address(13))
        cls.ENR = Register(14, "ENR", "Encode output pulse [pulse/rev]", 10000, cls.calculate_address(14))
        cls.PO1H = Register(15, "PO1H", "Revolution of inner position command 1 [rev]", 0, cls.calculate_address(15))
        cls.POL = Register(39, "POL", "Motor rotary direction option", 0x0000, cls.calculate_address(39))
        #------------Digital I/O setting related parameters-----------
        cls.INP = Register(12, "INP", "In-position range [pulse]", 100, cls.calculate_address(12))

    @classmethod
    def encode_HMOV(cls, z, y, x):
        if any(not (0 <= a <= 15) for a in (z, y, x)):
            raise ValueError("One or more paramter values are out of expected range")
        return (z << 8) | (y << 4) | x
    
    @classmethod
    def decode_HMOV(cls, value):
        z = (value // 0x0100) % 0x10
        y = (value // 0x10) % 0x10
        x = value % 0x10
        return z,y,x
    
    @classmethod
    def explain_HMOV(cls, value):
        # Using English version, I could not understand the manual in chinese...why? I am a netive chinese speaker
        z,y,x = cls.decode_HMOV(value)
        explanations = {
            'z': f"origin recognized completion option: {'Decelerates to stop then return to the mechanism origin' if z == 0 else 'decelerates to stop'}",
            'y': "Origin attained shortcut moving option: " + ["Turn back to last Z pulse to attain", "goes ahead to next Z pulse to attain", "origin recognized right away"][y % 3],
            'x': "Origin detector and rotation option: " + [
                "Running in CCW rotation and LSP is as a trigger",
                "Running in CW rotation and LSN is as a trigger",
                "Running in CCW rotation and ORGP ↑ is as a trigger",
                "Running in CW rotation and ORGP ↑ is as a trigger",
                "Running in CCW rotation and Encoder Z pulse as a trigger",
                "Running in CW rotation and Encoder Z pulse as a trigger",
                "Running in CCW rotation and ORGP ↓  is as a trigger",
                "Rotate CCW to zero point the ORGP ↓  from ON to OFF",
                "Define current position as a origin"
            ][x % 9]
        }
        return f"z (Error Handling): {explanations['z']}, y (Return Behavior): {explanations['y']}, x (Zero Point Definition): {explanations['x']}"

    @classmethod
    def set_HMOV(cls,z,y,x):
        value = cls.encode_HMOV(z,y,x)
        cls.HMOV.write_value(value)
        print(f"Register set to {hex(value)} ({cls.explain_HMOV(value)})")
        
    

class PC:
    _start_address = 0x0500
    _register_size = 2

    @classmethod
    def calculate_address(cls, no):
        return cls._start_address + (no -1) * cls._register_size
    @classmethod
    def init_registers(cls):
        cls.TL2 = Register(25, "TL2", "Inner torque limit 2 [%]", 100, cls.calculate_address(25))
        cls.CMX2 = Register(32, "CMX2", "Electronic gear numerator 2", 1, cls.calculate_address(32))
        cls.CMX3 = Register(33, "CMX3", "Electronic gear numerator 3", 1, cls.calculate_address(33))
        cls.CMX4 = Register(34, "CMX4", "Electronic gear numerator 4", 1, cls.calculate_address(34))

        #------------Digital I/O setting related parameters-----------
        cls.MBR = Register(16, "MBR", "Electromagnetic brake output delay time [pulse]", 100, cls.calculate_address(16) )
        cls.ZSP = Register(17, "ZSP", "Zero speed acknowledged range [ms]", 100, cls.calculate_address(17))

        #------------Communication related parameters----------
        cls.SNO = Register(20, "SNO", "Communication device number", 1, cls.calculate_address(20))
        cls.CMS = Register(21, "CMS", "Communication mode option", 0x0010, cls.calculate_address(21))
        cls.BPS = Register(22, "BPS", "Communication protocol option", 0x0010, cls.calculate_address(22))
        cls.SIC = Register(23, "SIC", "Communication time-out process option", 0, cls.calculate_address(23))

class PD:
    _start_address = 0x0600
    _register_size = 2

    @classmethod
    def calculate_address(cls, no):
        return cls._start_address + (no -1) * cls._register_size
    
    @classmethod
    def init_registers(cls):
        cls.DIA1 = Register(1, "DIA1", "Digital inpurt signal auto-ON option 1 [rpm]", 50, cls.calculate_address(1))
        cls.DIA2 = Register(21, "DIA2", "Digital input signal auto-No option 2", 0x0000, cls.calculate_address(21))

        cls.DI1 = Register(2, "DI1", "Digital input 1 option(CN1-14)", 0x0001, cls.calculate_address(2))
        cls.DI2 = Register(3, "DI2", "Digital input 2 option(CN1-15)", 0x000D, cls.calculate_address(3))
        cls.DI3 = Register(4, "DI3", "Digital input 3 option(CN1-16)", 0x0003, cls.calculate_address(4))
        cls.DI4 = Register(5, "DI4", "Digital input 4 option(CN1-17)", 0x0004, cls.calculate_address(5))
        cls.DI5 = Register(6, "DI5", "Digital input 5 option(CN1-18)", 0x0002, cls.calculate_address(6))
        cls.DI6 = Register(7, "DI6", "Digital input 6 option(CN1-19)", 0x0012, cls.calculate_address(7))
        cls.DI7 = Register(8, "DI7", "Digital input 7 option(CN1-20)", 0x0011, cls.calculate_address(8))
        cls.DI8 = Register(9, "DI8", "Digital input 7 option(CN1-21)", 0x0011, cls.calculate_address(9))

        cls.DI10 = Register(22, "DI10", "Digital input 10 option", 0x0000, cls.calculate_address(22))
        cls.DI11 = Register(23, "DI11", "Digital input 11 option", 0x0000, cls.calculate_address(23))
        cls.DI12 = Register(24, "DI12", "Digital input 12 option", 0x0000, cls.calculate_address(24))
        cls.DID = Register(29, "DID", "DI signal contact definition", 0x0000, cls.calculate_address(29))

        cls.DO1 = Register(10, "DI8", "Digital output 1 option(CN1-41)", 0x0003, cls.calculate_address(10))
        cls.DO2 = Register(11, "DI8", "Digital input 2 option(CN1-42)", 0x0008, cls.calculate_address(11))
        cls.DO3 = Register(12, "DI8", "Digital input 3 option(CN1-43)", 0x0007, cls.calculate_address(12))
        cls.DO4 = Register(13, "DI8", "Digital input 4 option(CN1-44)", 0x0005, cls.calculate_address(13))
        cls.DO5 = Register(14, "DI8", "Digital input 5 option(CN1-45)", 0x0001, cls.calculate_address(14))
        cls.DO6 = Register(26, "DO6", "Digital output 6 option", 0x0002, cls.calculate_address(26))
        cls.DOD = Register(27, "DOD", "DO signal contact definition", 0x0000, cls.calculate_address(27))

        cls.DIF = Register(15, "DIF", "Digital inpurt filter time option", 0x0002, cls.calculate_address(15))
        cls.IOS = Register(16, "DIF", "Digital input on/off control source option", 0x0000, cls.calculate_address(16))
        cls.DOP1 = Register(17, "DOP1", "LSP/LSN triggered stop option", 0x0000, cls.calculate_address(17))
        cls.DOP2 = Register(18, "DOP2", "CR signal clear option", 0x0000, cls.calculate_address(18))
        cls.DOP3 = Register(19, "DOP3", "Alarm code output option", 0x0000, cls.calculate_address(19))
        cls.DOP4 = Register(20, "DOP4", "Alarm reset triggered process", 0x0000, cls.calculate_address(20))

        cls.MCOK = Register(28, "MCOK", "Motion completion option", 0x0000, cls.calculate_address(28))

        cls.SDI = Register(16, "SDI", "數位輸入接點來源控制開關", 0x0000, cls.calculate_address(16))
        cls.ITST = Register(25, "ITST", "通訊控制數位輸入接點狀態", 0x0000, cls.calculate_address(25))

class PE:
    _start_address = 0x0700
    _register_size = 2

    @classmethod
    def calculate_address(cls, no):
        return cls._start_address + (no -1) * cls._register_size
    
    @classmethod
    def init_registers(cls):
        cls.PE01 = Register(1, "ODEF", "Origin return definition", 0x0000, cls.calculate_address(1))
        cls.PE02 = Register(2, "ODAT", "Origin offset value definition", 0, cls.calculate_address(2))
    

class PF:
    _start_address = 0x0800
    _register_size = 2 # word

    @classmethod
    def calculate_address(cls, no):
        return cls._start_address + (no -1) * cls._register_size
    
    @classmethod
    def init_registers(cls):
        cls.PRCM = Register(82, "PRCM", "PR trigger register: (0~1000)", 0, cls.calculate_address(82))

    
