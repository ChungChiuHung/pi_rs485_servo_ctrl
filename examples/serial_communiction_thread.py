import threading
import serial
import time

class SerialCommThreaded:
    def __init__(self, port, baud_rate, command, response_parser):
        self.ser_port = serial.Serial(port, baud_rate, timeout=1)
        self.command = command
        self.response_parser = response_parser
        self.running = False
        self.latest_status = {}
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()

    def run(self):
        while self.running:
            # Write the command to the serial port
            self.ser_port.write(self.command)

            # Write for a response
            response = self.ser_port.read_until()

            if response:
                with self.lock:
                    self.latest_status = self.response_parser(response)

            time.sleep(1)

    def get_latest_status(self):
        with self.lock:
            return self.latest_status
        
    


