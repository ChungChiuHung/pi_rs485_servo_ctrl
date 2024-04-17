import time
from servo_control import ServoController
from serial_port_manager import SerialPortManager


if __name__ =="__main__":
    from serial_port_manager import SerialPortManager
    serial_manager = SerialPortManager()
    controller = ServoController(serial_manager)

    # controller.enable_di_control()
    # controller.read_PD_01()
    controller.read_servo_state()
    time.sleep(0.05)
    print("\n")
    controller.read_control_mode()
    time.sleep(0.05)
    print("\n")
    controller.read_alarm_msg()
    time.sleep(0.05)
    print("\n")
    controller.read_servo_state()
    
    

    