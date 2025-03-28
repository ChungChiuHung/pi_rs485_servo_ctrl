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
        for pos in data_bytes:
            crc ^= pos
            for _ in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='little')