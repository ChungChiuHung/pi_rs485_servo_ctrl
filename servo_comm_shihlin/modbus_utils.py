class ModbusUtils:
    def __init__(self, poly=0x1021, init_val=0xFFFF):
        self.poly = poly
        self.init_val = init_val

    @staticmethod
    def calclulate_lrc(data_bytes):
        lrc = sum(data_bytes) % 256
        lrc = (-lrc) & 0xFF
        return lrc
    
    def calculate_crc(self, data_bytes):
        crc = self.init_val
        for byte in data_bytes:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ self.poly
                else:
                    crc <<= 1
            crc &= 0xFFFF
        return crc.to_bytes(2, 'big')