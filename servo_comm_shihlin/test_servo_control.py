from servo_control import ServoController
from serial_port_manager import SerialPortManager


if __name__ =="__main__":
    from serial_port_manager import SerialPortManager
    serial_manager = SerialPortManager()
    controller = ServoController(serial_manager)

    value = controller.enable_control_DI(1,2,3,4)
    

    