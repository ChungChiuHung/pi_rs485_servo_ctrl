from modbus_command_code import ExceptionCode
from status_registers import DeviceStatus

exception_code_value = 0x02
exception_code = ExceptionCode(exception_code_value)

print(exception_code.name)

for status in DeviceStatus:
     print(f"Address: {hex(status.address)}, Description: {status.description}, Data Length: {status.data_length} words")
