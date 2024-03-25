import serial

class SerialPortConfig:
    def __init__(self, baud_rate=57600, timeout=1):
        self.baud_rate = baud_rate
        self.timeout = timeout
        # List of potential serial ports
        self.available_ports = ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]

    def configure_serial_port(self):
        """
        Attempts to open a serial connection on a list of possible ports.
        Returns the configured serial port instance if successful, None otherwise.
        """
        for port in self.available_ports:
            try:
                serial_port = serial.Serial(port, self.baud_rate, timeout=self.timeout)
                serial_port.bytesize = serial.EIGHTBITS
                serial_port.parity = serial.PARITY_NONE
                serial_port.stopbits = serial.STOPBITS_ONE
                print(f"Successfully connected to {port}")
                return serial_port
            except (serial.SerialException, OSError) as e:
                print(f"Failed to open serial port {port}: {e}")
        print("Failed to open any serial port.")
        return None
