from threading import Thread, Event
from modbus_ascii_client import ModbusASCIIClient
import time

class ModbusPoller:
    def __init__(self, modbus_client, address, count, poll_interval=0.1):
        self.modbus_client = modbus_client
        self.address = address
        self.count = count
        self.poll_interval = poll_interval
        self.stop_evnet = Event()
        self.thread = Thread(target=self.run)

    def start(self):
        if not self.modbus_client:
            print("Failed to connect to the Modbus server")
            return
        print("Connected to Modbus server, starting poller")
        self.thread.start()

