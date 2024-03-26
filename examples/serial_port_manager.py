import serial
from serial.serialutil import SerialException

class SerialPortManager:
    def __init__(self,port=None, baud_rate=57600, timeout=1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_instance = None
        # List of potential serial ports
        self.available_ports = ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]

    def connect(self):
        """
        Attempts to open a serial connection on a list of possible ports.
        Returns the configured serial port instance if successful, None otherwise.
        """
        for port in self.available_ports:
            try:
                self.serial_instance = serial.Serial(port, self.baud_rate, timeout=self.timeout)
                self.port = port
                self.serial_instance.bytesize = serial.EIGHTBITS
                self.serial_instance.parity = serial.PARITY_NONE
                self.serial_instance.stopbits = serial.STOPBITS_ONE
                
                print(f"Connected to {port} at {self.baud_rate} baud.")
                break
            except (SerialException, OSError) as e:
                print(f"Failed to open serial port {port}: {e}")
        else:
            print("Failed to open any serial port.")

    def get_serial_instance(self):
        if self.serial_instance and self.serial_instance.is_open:
            return self.serial_instance
        else:
            print("Serial port is not open. Attempting to reconnect...")
            self.connect()
            return self.serial_instance if self.serial_instance and self.serial_instance.is_open else None
        
    def disconnect(self):
        if self.serial_instance and self.serial_instance.is_open:
            self.serial_instance.close()
            print(f"Disconnected from {self.port}.")

    def get_baud_rate(self):
        return self.baud_rate

    def get_connected_port(self):
        return self.port if self.serial_instance and self.serial_instance.is_open else None    