import struct
from servo_control import ServoController
from serial_port_manager import SerialPortManager
from modbus_command_code import CmdCode

if __name__ =="__main__":
    from serial_port_manager import SerialPortManager
    serial_manager = SerialPortManager()
    controller = ServoController(serial_manager)

    commands = [
        {'code': CmdCode.WRITE_DATA, 'data': struct.pack('>H', 0x1234)},
        {'code': CmdCode.WRITE_MULTI_DATA, 'data': struct.pack('>H', 0x5678)}
    ]
    controller.start_motion_sequence(commands)
    