"""
The Modbus ASCII client remembers the last frame it sent and received, for the
web UI's "Last RS-485 transaction" panel. The serial port is a mock; nothing
touches hardware. Run from inside this directory:
python -m unittest test_ascii_client_traffic
"""
import unittest
from unittest.mock import MagicMock

from modbus_ascii_client import ModbusASCIIClient


class AsciiClientTrafficTests(unittest.TestCase):

    def setUp(self):
        self.saved_instance = ModbusASCIIClient._instance
        ModbusASCIIClient._instance = None
        self.addCleanup(setattr, ModbusASCIIClient, "_instance", self.saved_instance)
        self.manager = MagicMock()
        self.serial = self.manager.get_serial_instance.return_value
        self.serial.in_waiting = 0
        self.client = ModbusASCIIClient.get_instance(device_number=1, serial_port_manager=self.manager)

    def test_nothing_recorded_before_the_first_transaction(self):
        self.assertIsNone(self.client.last_sent)
        self.assertIsNone(self.client.last_received)
        self.assertEqual(self.client.format_frame(None), "")

    def test_send_records_the_frame(self):
        message = self.client.build_read_message(0x0100, 1)
        self.client.send(message)
        self.assertEqual(self.client.last_sent, message)
        self.assertTrue(self.client.format_frame(self.client.last_sent).startswith(":01"))

    def test_receive_records_the_reply(self):
        reply = b":01030200FF00\r\n"
        self.serial.in_waiting = len(reply)
        self.serial.read.return_value = reply

        def drain(_):
            self.serial.in_waiting = 0
            return reply
        self.serial.read.side_effect = drain
        self.assertEqual(bytes(self.client.receive(timeout=0.05)), reply)
        self.assertEqual(self.client.format_frame(self.client.last_received), ":01030200FF00")

    def test_format_frame_strips_line_endings_and_accepts_text_or_bytes(self):
        self.assertEqual(self.client.format_frame(b":0103\r\n"), ":0103")
        self.assertEqual(self.client.format_frame(bytearray(b":0103\r\n")), ":0103")
        self.assertEqual(self.client.format_frame(":0103\r\n"), ":0103")


if __name__ == "__main__":
    unittest.main()
