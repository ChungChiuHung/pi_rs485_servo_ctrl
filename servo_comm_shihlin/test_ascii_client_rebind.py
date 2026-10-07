"""
The Modbus ASCII client is a singleton. After a reconnect (a new
SerialPortManager because the port was missing at start, or was replugged) it
must talk through the NEW manager, not the closed one it was first created
with. The serial port is a mock; nothing touches hardware. Run from inside
this directory: python -m unittest test_ascii_client_rebind
"""
import unittest
from unittest.mock import MagicMock

from modbus_ascii_client import ModbusASCIIClient


class ModbusClientRebindTests(unittest.TestCase):
    def setUp(self):
        self.saved = ModbusASCIIClient._instance
        ModbusASCIIClient._instance = None
        self.addCleanup(setattr, ModbusASCIIClient, "_instance", self.saved)

    def test_a_new_manager_replaces_the_old_one(self):
        first, second = MagicMock(), MagicMock()
        client = ModbusASCIIClient.get_instance(device_number=1, serial_port_manager=first)
        self.assertIs(client.serial_port_manager, first)
        same_client = ModbusASCIIClient.get_instance(device_number=1, serial_port_manager=second)
        self.assertIs(same_client, client)
        self.assertIs(client.serial_port_manager, second)

    def test_the_same_manager_is_left_alone(self):
        manager = MagicMock()
        client = ModbusASCIIClient.get_instance(device_number=1, serial_port_manager=manager)
        ModbusASCIIClient.get_instance(device_number=1, serial_port_manager=manager)
        self.assertIs(client.serial_port_manager, manager)


if __name__ == "__main__":
    unittest.main()
