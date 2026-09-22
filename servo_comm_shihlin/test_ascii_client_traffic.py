"""
The Modbus ASCII client remembers the last frame it sent and received, for the
web UI's "Last RS-485 transaction" panel. The serial port is a mock; nothing
touches hardware. Run from inside this directory:
python -m unittest test_ascii_client_traffic
"""
import threading
import time
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

    def test_infer_expected_length_for_write_data(self):
        message = self.client.build_write_message(0x0130, 0x1EA5)
        self.assertEqual(self.client._infer_expected_length(message), 17)
        # Cross-check against the project's own worked example.
        self.assertEqual(message.strip(), b":010601301EA505")
        self.assertEqual(len(message.strip()), 15)  # +2 for \r\n = 17

    def test_infer_expected_length_for_read_data(self):
        one_word = self.client.build_read_message(0x0100, 1)
        two_words = self.client.build_read_message(0x0000, 2)
        self.assertEqual(self.client._infer_expected_length(one_word), 15)
        self.assertEqual(self.client._infer_expected_length(two_words), 19)

    def test_infer_expected_length_is_none_for_unrecognized_or_malformed_messages(self):
        self.assertIsNone(self.client._infer_expected_length(b":0110001002000200EE\r\n"))  # write-multi (0x10)
        self.assertIsNone(self.client._infer_expected_length(b""))
        self.assertIsNone(self.client._infer_expected_length(b":01"))
        self.assertIsNone(self.client._infer_expected_length(b"not ascii-hex at all"))

    def test_receive_stops_as_soon_as_the_inferred_length_arrives_not_the_full_timeout(self):
        """The whole point of inferring expected_length: receive() should
        return the moment the exact frame is complete, not wait out
        timeout's "quiet period" -- confirmed live 2026-09-22 that this
        wait, repeated across ~7 transactions per button press, was the
        dominant cost behind a ~1.9s positioning-test move."""
        message = self.client.build_write_message(0x0130, 0x1EA5)
        reply = message  # the drive echoes the write back verbatim
        self.serial.in_waiting = len(reply)
        self.serial.read.side_effect = lambda n: (
            setattr(self.serial, "in_waiting", 0) or reply
        )
        started = time.time()
        result = self.client.send_and_receive(message, timeout=5.0)
        elapsed = time.time() - started
        self.assertEqual(bytes(result), reply)
        self.assertLess(elapsed, 1.0, "receive() waited far longer than the 17 bytes needed")

    def test_receive_cannot_loop_forever_on_a_continuous_trickle(self):
        """Regression test for a real freeze found via the web UI
        2026-09-22: receive()'s "quiet period" timer resets every time
        in_waiting is nonzero, so a continuous trickle of incoming bytes
        (noise, an echo, a slow/garbled response) kept the old code
        spinning forever, holding _transaction_lock and freezing every
        other Modbus user (the continuous-reading background thread AND
        every /action request) permanently. Simulates that trickle (one
        byte "arrives" every call, response never reaches expected_length)
        and asserts receive() still returns within a bounded time -- run on
        its own thread with a hard join timeout so a regression fails this
        test instead of hanging the whole test process."""
        self.serial.in_waiting = 1
        self.serial.read.side_effect = lambda n: b"\x00"

        result = {}

        def call_receive():
            # No expected_length: the only way out is the timeout-based
            # "quiet period" check, which is exactly what a continuous
            # trickle defeats (in_waiting is never falsy) without the
            # absolute hard_deadline.
            result["value"] = self.client.receive(timeout=0.05)
            result["returned"] = True

        t = threading.Thread(target=call_receive, daemon=True)
        started = time.time()
        t.start()
        t.join(timeout=5.0)
        elapsed = time.time() - started

        self.assertFalse(t.is_alive(),
                          "receive() never returned against a continuous trickle "
                          "(it needs an absolute deadline, not just a quiet-period timer)")
        self.assertTrue(result.get("returned"))
        self.assertLess(elapsed, 3.0, "receive() took far longer than its ~1.1s hard deadline")


if __name__ == "__main__":
    unittest.main()
