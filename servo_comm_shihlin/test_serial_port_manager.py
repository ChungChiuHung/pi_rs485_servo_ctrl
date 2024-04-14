from serial_port_manager import SerialPortManager

def test_serial_port_manager():
    print("Creating SerailPortManager instance...")
    serial_manager = SerialPortManager()

    print("Attempting to connect to a serial port...")
    if serial_manager.connect():
        print("Connection established.")
    else:
        print("Failed to establish connection. Exiting test.")
        return
    
    try:
        # Sending a test message
        test_message = "Hello, Serial Port!"
        print(f"Sending message: {test_message}")
        response = serial_manager.send_and_receive(test_message)

        if response:
            print(f"Response received: {response.decode('utf-8')}")
        else:
            print("No response received or connection error.")
    
    finally:
        print("Disconnecting from serial port...")
        serial_manager.disconnect()
        print("Disconnected successfully.")

if __name__ == "__main__":
    test_serial_port_manager()

if __name__ == "__main__":
    serial_manager = SerialPortManager()
    serial_manager.connect()
    if serial_manager.get_serial_instance():
        response = serial_manager.send_and_receive(b'0x01010011')
        print(response)
    serial_manager.disconnect()