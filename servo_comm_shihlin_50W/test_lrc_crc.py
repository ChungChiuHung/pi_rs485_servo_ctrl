from modbus_utils import ModbusUtils

ADR = 0x01
CMD = 0x03
START_ADDRESS = [0x01 , 0x04]
Data_COUNT = [0x00, 0x02]

data_bytes = bytes([ADR, CMD]) + bytes(START_ADDRESS) + bytes(Data_COUNT)

get_lrc_crc = ModbusUtils()

lrc_byte = get_lrc_crc.calclulate_lrc(data_bytes)

print (hex(lrc_byte))

ADR = 0x01
CMD = 0x03
START_ADDRESS = [0x01, 0x01]
WRITTEN_DATA = [0x00, 0x02]
data_bytes = bytes([ADR, CMD]) + bytes(START_ADDRESS) + bytes(WRITTEN_DATA)

crc_bytes = get_lrc_crc.calculate_crc(data_bytes)

total_command = data_bytes + crc_bytes

crc_hex_string = ' '.join(f'{byte:02X}' for byte in data_bytes)
data_hex_string = ' '.join(f'{byte:02X}' for byte in crc_bytes)
total_hex_string = ' '.join(f'{byte:02X}' for byte in total_command)

print (data_hex_string)
print(crc_hex_string)
print(total_hex_string)

ADR = 0x01
CMD = 0x86
ECP = 0x02

data_bytes = bytes([ADR, CMD, ECP])
crc_bytes = get_lrc_crc.calculate_crc(data_bytes)
total_command = data_bytes + crc_bytes

crc_hex_string = ' '.join(f'{byte:02X}' for byte in data_bytes)
data_hex_string = ' '.join(f'{byte:02X}' for byte in crc_bytes)
total_hex_string = ' '.join(f'{byte:02X}' for byte in total_command)

print (data_hex_string)
print(crc_hex_string)
print(total_hex_string)
