import time
from servo_control import ServoController
from serial_port_manager import SerialPortManager


if __name__ =="__main__":
    from serial_port_manager import SerialPortManager
    serial_manager = SerialPortManager()
    controller = ServoController(serial_manager)

    print("\n")
    print("START Communication With AC Servo Driver")
    print("\n")

    controller.read_PA01_Ctrl_Mode()
    time.sleep(0.05)
    print("\n")

    controller.read_servo_state()
    time.sleep(0.05)
    print("\n")

    #controller.write_PA01_Ctrl_Mode()
    #time.sleep(0.05)
    #print("\n")

    #controller.write_PD_16_Enable_DI_Control()
    #time.sleep(0.05)
    #print("\n")
#
    controller.read_PD_16()
    time.sleep(0.05)
    print("\n")

    #controller.write_PD_25()
    #time.sleep(0.05)
    #print("\n")

    controller.read_PD_25()
    time.sleep(0.05)
    print("\n")

    #controller.write_PD_01()
    #time.sleep(0.05)
    #print("\n")
#
    #controller.read_PD_01()
    #time.sleep(0.05)
    #print("\n")

    controller.write_PD_02()
    time.sleep(0.05)
    print("\n")

    controller.read_PD_02()
    time.sleep(0.05)
    print("\n")

    #controller.read_PD_08()
    #time.sleep(0.05)
    #print("\n")

    #controller.write_PD_08()
    #time.sleep(0.05)
    #print("\n")

    #controller.read_0x0206_To_0x020B()
    #time.sleep(0.05)
    #print("\n")

    
    

    