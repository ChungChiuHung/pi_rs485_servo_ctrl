import struct
from servo_control import ServoController
from serial_port_manager import SerialPortManager
from modbus_command_code import CmdCode

if __name__ =="__main__":
    from serial_port_manager import SerialPortManager
    serial_manager = SerialPortManager()
    controller = ServoController(serial_manager)

    controller.set_pd01_value(1,0,0,0)
    

    