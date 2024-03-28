from servo_serial_protocol_handler import SerialPotocolHandler
from servo_command_code import CmdCode
from status_bit_mapping import BitMap

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

#=========================================
command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
get_io_input_command = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 3, False)
print(f"{command_code.name} Command:", get_io_input_command)
print(f"{command_code.name} Command:", get_io_input_command.hex())

#=========================================
# SET_PARAM_2 command        
command_code = CmdCode.SET_PARAM_2
command_format = SerialPotocolHandler()
set_param_2_command = command_format.construct_packet(1,command_code, b'\x00\x09\x00\x01', is_response=False)
print(f"{command_code.name} Command: ", set_param_2_command.hex())

# SERVO_ON Command
command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
command_format = SerialPotocolHandler()
servo_on_command = command_format.construct_packet(1,command_code, b'\x01\x20\x00\x00\x00\x01\x00\x00\x00\x01', is_response=False)
print(f"{command_code.name} Command: ", servo_on_command.hex())
