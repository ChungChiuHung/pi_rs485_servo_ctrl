from modbus_utils import ModbusUtils

ADR = 0x01
CMD = 0x03
START_ADDRESS = [0x01 , 0x04]
Data_COUNT = [0x00, 0x02]

data_bytes = bytes([ADR, CMD]) + bytes(START_ADDRESS) + bytes(Data_COUNT)

get_lrc = ModbusUtils()

lrc_byte = get_lrc.calclulate_lrc(data_bytes)

print (hex(lrc_byte))