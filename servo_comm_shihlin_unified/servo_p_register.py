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
        #------------Absolute encoder related parameters (manual §8 "Servo absolute system")-----------
        # PA28 (ABS): 0 = incremental mode, 1 = absolute mode. PA32(APR)/PA33(APP)
        # are only valid when this is 1 (manual p.88-89) — must be confirmed on real
        # hardware before relying on PA32/PA33 for encoder-overflow-safe position reads.
        # PA23 (MCS), Chinese manual V1.07 (the English manuals list only 0/1):
        # 0 = parameters written to EEPROM; 1 = no EEPROM writes, and PA23
        # itself reverts to 0 after power-off; 2 = no EEPROM writes but PA23=2
        # persists (firmware >= 106 only). EEPROM life is ~100,000 writes.
        cls.MCS = Register(23, "MCS", "Memory write-inhibit function (0=EEPROM writable, 1=RAM only, resets at power-off; 2=RAM only, persists, fw>=106)", 0, cls.calculate_address(23))
        cls.ABS = Register(28, "ABS", "Absolute encoder settings", 0x0000, cls.calculate_address(28))
        # PA29-PA33 (manual p.88-89): only meaningful when PA28 == 1.
        cls.CAP = Register(29, "CAP", "Absolute homing position (write 1 = current position becomes origin)", 0x0000, cls.calculate_address(29))
        cls.UAP = Register(30, "UAP", "Update encoder absolute position (1 = refresh PA31~PA33; 2 = also clear position error)", 0, cls.calculate_address(30))
        cls.APST = Register(31, "APST", "ABS position status (read only)", 0x0000, cls.calculate_address(31))
        # The two manuals DISAGREE on which of PA32/PA33 is which: Chinese
        # V1.07 says PA32 = pulses within a revolution (0~4194303) and PA33 =
        # signed revolutions (-32768~32767); the English manuals say the
        # reverse. ServoController.abs_rev_register selects the layout.
        cls.APR = Register(32, "APR", "Encoder absolute position word 1 (read only; pulses or revolutions, see abs_rev_register)", 0, cls.calculate_address(32))
        cls.APP = Register(33, "APP", "Encoder absolute position word 2 (read only; revolutions or pulses, see abs_rev_register)", 0, cls.calculate_address(33))

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

    # STY/PLSS/POL are all packed as hex nibbles (manual pp.83-90), same
    # shape as HMOV above -- decoded here purely for the "GET STATE VALUE"
    # diagnostic read (Read_Pos_Related_Paremters()), not used by any
    # control path.
    @classmethod
    def explain_STY(cls, value):
        x = value % 0x10
        y = (value // 0x10) % 0x10
        z = (value // 0x100) % 0x10
        u = (value // 0x1000) % 0x10
        control_modes = ["position", "position/speed", "speed", "speed/torque",
                          "torque", "torque/position", "turret"]
        position_cmd_sources = ["external input", "inner register", "Pt-Pr switched (SDE-P only)"]
        x_desc = control_modes[x] if x < len(control_modes) else f"unknown ({x})"
        y_desc = position_cmd_sources[y] if y < len(position_cmd_sources) else f"unknown ({y})"
        z_desc = "enabled (motor has electromagnetic brake)" if z else "disabled"
        u_desc = "DI/DO functions vary with control mode" if u else "DI/DO functions fixed"
        return f"control mode={x_desc}; position command source={y_desc}; brake={z_desc}; DI/DO={u_desc}"

    @classmethod
    def explain_PLSS(cls, value):
        x = value % 0x10
        y = (value // 0x10) % 0x10
        z = (value // 0x100) % 0x10
        pulse_formats = ["forward/reverse rotation pulse train", "pulse train + sign", "A/B phase pulse train"]
        ack_logics = ["positive logic", "negative logic"]
        filter_options = ["<=500kpps", "<=200kpps", "<=2Mpps", "<=4Mpps"]
        x_desc = pulse_formats[x] if x < len(pulse_formats) else f"unknown ({x})"
        y_desc = ack_logics[y] if y < len(ack_logics) else f"unknown ({y})"
        z_desc = filter_options[z] if z < len(filter_options) else f"unknown ({z})"
        return f"pulse-train format={x_desc}; ack logic={y_desc}; input pulse filter={z_desc}"

    @classmethod
    def explain_POL(cls, value):
        x = value % 0x10
        z = (value // 0x100) % 0x10
        direction_options = [
            "forward pulse-train -> CCW, reverse pulse-train -> CW",
            "forward pulse-train -> CW, reverse pulse-train -> CCW",
        ]
        encoder_output_options = ["output pulse count (PA14=ENR is pulses/rev)",
                                   "output division ratio (PA14=ENR is the divisor)"]
        x_desc = direction_options[x] if x < len(direction_options) else f"unknown ({x})"
        z_desc = encoder_output_options[z] if z < len(encoder_output_options) else f"unknown ({z})"
        # The manual's y-nibble table (motor rotation vs. encoder pulse
        # output relationship, PA39) is a diagram, not text -- no textual
        # description exists to decode it from, so it's omitted here.
        return f"input pulse/motor direction={x_desc}; encoder output={z_desc}"

    @classmethod
    def explain_MCS(cls, value):
        return {
            0: "EEPROM writable -- every parameter write wears the EEPROM (~100,000 write life)",
            1: "EEPROM write-inhibited (RAM only); reverts to 0 at the next power-off",
            2: "EEPROM write-inhibited (RAM only); persists across power-off (firmware >= 106)",
        }.get(value, f"unexpected value {value}")

    @classmethod
    def explain_ABS(cls, value):
        if value == 0:
            return "incremental mode (an absolute-encoder motor is operated as incremental)"
        if value == 1:
            return "absolute mode (valid only with an absolute-encoder motor; otherwise AL.24)"
        return f"unexpected value {value} (manual documents only 0 or 1)"

    # PA31 (APST) bit layout, manual p.88. Any of bit0/bit1/bit2/bit4 set
    # means PA32/PA33 must not be trusted as a position reference.
    APST_BAD_BITS = {
        0: "absolute position lost",
        1: "battery low voltage",
        2: "overflow",
        4: "absolute coordinate system not yet set",
    }

    @classmethod
    def decode_APST(cls, value):
        """Returns the list of active fault descriptions (empty = all clear)."""
        return [text for bit, text in cls.APST_BAD_BITS.items() if (value >> bit) & 1]

    @classmethod
    def explain_APST(cls, value):
        faults = cls.decode_APST(value)
        return "normal" if not faults else "; ".join(faults)


class PC:
    _start_address = 0x0500
    _register_size = 2

    @classmethod
    def calculate_address(cls, no):
        return cls._start_address + (no -1) * cls._register_size
    @classmethod
    def init_registers(cls):
        cls.STA = Register(1, "STA", "Acceleration time constant", 1, cls.calculate_address(1))
        cls.STB = Register(2, "STB", "Deceleration time constant", 1, cls.calculate_address(2))
        cls.JOG = Register(3, "JOG", "Jog speed command [rpm]", 1, cls.calculate_address(3))
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

    # SDI/ITST are 12-bit masks, bit0-11 = DI1-DI12 (manual p.105/107).
    # MCOK is nibble-packed like PA's STY/PLSS/POL. Decoded here purely
    # for the "GET STATE VALUE" diagnostic read.
    @classmethod
    def explain_SDI(cls, value):
        controlled = [f"DI{i + 1}" for i in range(12) if (value >> i) & 1]
        return ("Communication-controlled: " + ", ".join(controlled)) if controlled \
            else "All DI controlled by hardware wiring"

    @classmethod
    def explain_ITST(cls, value):
        # Only meaningful for DIs that SDI marks as communication-controlled
        # -- see PD16/PD25 interaction example in the manual (p.107).
        on_bits = [f"DI{i + 1}" for i in range(12) if (value >> i) & 1]
        return ("Virtual ON: " + ", ".join(on_bits)) if on_bits else "All virtual DI bits OFF"

    @classmethod
    def explain_MCOK(cls, value):
        x = value % 0x10
        y = (value // 0x10) % 0x10
        x_desc = "MC_OK held until next move" if x else "MC_OK pulses briefly (not held)"
        y_desc = "AL1B (position error) enabled" if y else "AL1B (position error) invalid"
        return f"{x_desc}; {y_desc}"


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
        cls.PE03 = Register(3, "PDEF1", "PATH#1 definition", 0x0000, cls.calculate_address(3))
        cls.PE04 = Register(4, "PDAT1", "PATH#1 data", 0x0000, cls.calculate_address(4))
    

class PF:
    _start_address = 0x0800
    _register_size = 2 # word

    @classmethod
    def calculate_address(cls, no):
        return cls._start_address + (no -1) * cls._register_size
    
    @classmethod
    def init_registers(cls):
        cls.PRCM = Register(82, "PRCM", "PR trigger register: (0~1000)", 0, cls.calculate_address(82))

    
