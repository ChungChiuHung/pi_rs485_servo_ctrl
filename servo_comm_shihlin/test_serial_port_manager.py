from serial_port_manager import SerialPortManager

if __name__ == "__main__":
    serial_manager = SerialPortManager()
    serial_manager.connect()
    if serial_manager.get_serial_instance():
        response = serial_manager.send_and_receive(b'0x01010011')
        print(response)
    serial_manager.disconnect()