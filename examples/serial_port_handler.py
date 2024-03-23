import serial

class SerialPortHandler:
    def __init__(self, baud_rate=57600, timeout=1):
        self.serial_port = None
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.available_ports = ["/dev/ttyS0", "/dev/ttyAMA0", "/dev/serial0", "/dev/ttyUSB0"]

    def initialize_serial(self):
        for port in self.available_ports:
            try:
                self.serial_port = serial.Serial(port, self.baud_rate, timeout=self.timeout)
                self.serial_port.bytesize = serial.EIGHTBITS
                self.serial_port.parity = serial.PARITY_NONE
                self.serial_port.stopbits = serial.STOPBITS_ONE
                print(f"Successfully connected to {port}")
                break
            except (OSError, serial.SerialException):
                continue
    
    def close_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")
    
    def write(self, data):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(data)

    def read(self):
        if self.serial_port and self.serial_port.is_open:
            return self.serial_port.read(self.serial_port.in_waiting)
        return None