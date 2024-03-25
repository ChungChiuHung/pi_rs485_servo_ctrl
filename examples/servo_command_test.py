from servo_serial_protocol_handler import SerialPotocolHandler
from command_code import CmdCode

command_code = CmdCode.NOP
command_format = SerialPotocolHandler()
nop_command = command_format.construct_packet(1,command_code, b'', is_response=True)
print(f"{command_code.name} Command: ", nop_command.hex())

nop_response = command_format.construct_packet(1,command_code, b'', is_response=False)
print(f"{command_code.name} Response: ", nop_response.hex())

command_code = CmdCode.GET_PARAM_2
get_param_2 = command_format.construct_packet(1, command_code, b'\x00\x20', is_response=False)
print(f"{command_code.name} Command:", get_param_2.hex())

get_param_2 = command_format.construct_packet(1, command_code, b'\x00\x20', is_response=True)
print(f"{command_code.name} Response:", get_param_2.hex())

command_code = CmdCode.GET_PARAM_4
get_param_4 = command_format.construct_packet(1, command_code, b'\x02\x8B', is_response=False)
print(f"{command_code.name} Command:", get_param_4.hex())

get_param_4 = command_format.construct_packet(1, command_code, b'\x02\x8B', is_response=True)
print(f"{command_code.name} Response:", get_param_4.hex())