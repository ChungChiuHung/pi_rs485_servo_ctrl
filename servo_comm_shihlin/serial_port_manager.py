import serial
import time
from serial.serialutil import SerialException
import serial.tools.list_ports # For dynamically listing available ports

class SerialPortManager:
    def __init__(self,port=None, baud_rate=9600, timeout=1, bytesize=serial.SEVENBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_TWO):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.serial_instance = None
        self.available_port = self.list_available_ports()
        
    def list_available_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        return ports if ports else ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]

    def connect(self):
        """
        Attempts to open a serial connection on a list of possible ports.
        """
        if self.port:
            try:
                self.serial_instance = serial.Serial(self.port, self.baud_rate, timeout=self.timeout,
                                                     bytesize = self.bytesize, parity=self.parity, stopbits=self.stopbits)
                print(f"Connected to {self.port} at {self.baud_rate} baud.")
                return
            except (SerialException, OSError) as e:
                print(f"Failed to open serial port {self.port}: {e}")

        print("Trying available ports...")
        for port in self.list_available_ports():
            try:
                self.serial_instance = serial.Serial(port, self.baud_rate, timeout=self.timeout,
                                                     bytesize=self.bytesize, parity=self.parity, stopbits=self.stopbits )
                self.port = port
                print(f"Connected to {port} at {self.baud_rate} baud.")
                return True
            except (SerialException, OSError) as e:
                print(f"Failed to connect on {port}: {e}")

        print("Failed to open any serial port.")
        return False

    def get_serial_instance(self):
        if self.serial_instance and self.serial_instance.is_open:
            return self.serial_instance
        else:
            print("Serial port is not open. Attempting to reconnect...")
            if self.connect():
                return self.serial_instance
            return None
        
    def disconnect(self):
        if self.serial_instance and self.serial_instance.is_open:
            self.serial_instance.close()
            print(f"Disconnected from {self.port}.")

    def send_and_receive(self, message, read_timeout=0.5):
        serial_conn = self.get_serial_instance()
        if serial_conn:
            serial_conn.write(message.encode('utf-8')) # Ensure message is in bytes
            print(f"Send: {message}")
            time.sleep(read_timeout) # Wait to receive the  full message
            response = serial_conn.read_all()
            print(f"Received: {response.decode('utf-8')}")
            return response
        else:
            print("No activate serail connection.")
            return None

    def get_baud_rate(self):
        return self.baud_rate

    def get_connected_port(self):
        return self.port if self.serial_instance and self.serial_instance.is_open else "Not connected"