import serial
import time
from serial.serialutil import SerialException
import serial.tools.list_ports # For dynamically listing available ports

class SerialPortManager:
    def __init__(self,port=None, baud_rate=57600, timeout=1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_instance = None
        # Dynamically list available serial ports
        self.available_port = self.list_available_ports()
        # List of potential serial ports
        # self.available_ports = ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]

    def list_available_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports if ports else ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]

    def connect(self):
        """
        Attempts to open a serial connection on a list of possible ports.
        """
        for port in self.available_ports:
            try:
                self.serial_instance = serial.Serial(port, self.baud_rate, timeout=self.timeout)
                self.serial_instance.bytesize = serial.EIGHTBITS
                self.serial_instance.parity = serial.PARITY_NONE
                self.serial_instance.stopbits = serial.STOPBITS_ONE
                self.port = port
                
                print(f"Connected to {port} at {self.baud_rate} baud.")
                return
            except (SerialException, OSError) as e:
                print(f"Failed to open serial port {port}: {e}")

        print("Failed to open any serial port.")

    def get_serial_instance(self):
        if self.serial_instance and self.serial_instance.is_open:
            return self.serial_instance

        print("Serial port is not open. Attempting to reconnect...")
        self.connect()
        return self.serial_instance if self.serial_instance and self.serial_instance.is_open else None
        
    def disconnect(self):
        if self.serial_instance and self.serial_instance.is_open:
            self.serial_instance.close()
            print(f"Disconnected from {self.port}.")

    def send_and_receive(self, message, read_timeout=0.5):
        serial_conn = self.get_serial_instance()
        if serial_conn:
            serial_conn.write(message)
            print(f"Send: {message}")
            time.sleep(read_timeout)
            response = serial_conn.read_all()
            print(f"Received: {response}")
            return response
        else:
            print("No activate serail connection.")
            return None

    def get_baud_rate(self):
        return self.baud_rate

    def get_connected_port(self):
        return self.port if self.serial_instance and self.serial_instance.is_open else None    