import time
from servo_control import ServoController
from serial_port_manager import SerialPortManager

if __name__ =="__main__":
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

    controller.read_PD_01()
    time.sleep(0.05)
    print("\n")

    #controller.write_PD_02()
    #time.sleep(0.05)
    #print("\n")

    controller.read_PD_02()
    time.sleep(0.05)
    print("\n")

    controller.read_PD_08()
    time.sleep(0.05)
    print("\n")

    #controller.write_PD_08()
    #time.sleep(0.05)
    #print("\n")

    #controller.read_0x0206_To_0x020B()
    #time.sleep(0.05)
    #print("\n")

    controller.read_test_mode_0x0901()
    time.sleep(0.05)
    print("\n")

    controller.read_PF82()
    time.sleep(0.1)
    print("\n")

    #controller.start_test_pos_mode()
    #time.sleep(0.05)
    #print("\n")

    #controller.stop_test_mode()
    #time.sleep(0.05)
    #print("\n")

    controller.write_PF82(1)
    time.sleep(0.1)
    print("\n")



