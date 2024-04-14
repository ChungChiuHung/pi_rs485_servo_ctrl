import struct
from servo_control import ServoController
from serial_port_manager import SerialPortManager
from modbus_command_code import CmdCode

if __name__ =="__main__":
    from serial_port_manager import SerialPortManager
    serial_manager = SerialPortManager()
    controller = ServoController(serial_manager)

    try:
        controller.set_pd01_value(x=1,y=0,z=1,u=1)
    except Exception as e:
        print(f"An error occurred: {e}")
    