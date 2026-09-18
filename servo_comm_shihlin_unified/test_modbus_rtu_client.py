"""
Tests for ModbusRTUClient, focused on the fix for the concatenated-frame
bug found via a real production log (two valid, correctly-CRC'd RTU
responses glued together into one receive() call): a write's driver-echo
response was never drained (fire-and-forget send()), so it sat in the
input buffer until a later, unrelated send_and_receive() call scooped it
up together with its own real response. Fixed two ways, both covered here:
1. send_and_receive() now infers expected_length from the outgoing message
   when the caller doesn't pass one, so receive() knows exactly when a
   response is complete instead of accumulating until the line goes quiet.
2. send() flushes the input buffer before transmitting, as defense in depth
   against any leftover unread bytes regardless of their source.

No real hardware -- a fake serial object simulates in_waiting/read/write.
"""
import struct
import unittest
from unittest.mock import MagicMock

from modbus_rtu_client import ModbusRTUClient
from modbus_command_code import CmdCode
from modbus_utils import ModbusUtils


def build_frame(device_number, command_code, payload_after_cmd: bytes) -> bytes:
    body = struct.pack('B', device_number) + struct.pack('B', command_code) + payload_after_cmd
    crc = ModbusUtils().calculate_crc(body)
    return body + crc


class FakeSerial:
    """Minimal stand-in for pyserial's Serial. `response_after_write`
    becomes readable only once write() is called, mirroring how a real
    driver only replies after receiving a request -- so reset_input_buffer()
    (called by send() before writing) clears any pre-existing
    `stale_bytes`, not the response that hasn't "arrived" yet."""

    def __init__(self, response_after_write: bytes = b'', stale_bytes: bytes = b''):
        self._incoming = bytearray(stale_bytes)
        self._response_after_write = bytes(response_after_write)
        self.written = bytearray()
        self.reset_input_buffer_call_count = 0

    @property
    def in_waiting(self):
        return len(self._incoming)

    def read(self, n):
        n = min(n, len(self._incoming))
        chunk = bytes(self._incoming[:n])
        del self._incoming[:n]
        return chunk

    def write(self, data):
        self.written.extend(data)
        self._incoming.extend(self._response_after_write)
        self._response_after_write = b''  # only "arrives" once

    def reset_input_buffer(self):
        self.reset_input_buffer_call_count += 1
        self._incoming.clear()


def make_client_with_fake_serial(response_after_write: bytes = b'', stale_bytes: bytes = b''):
    fake_serial = FakeSerial(response_after_write, stale_bytes)
    port_manager = MagicMock()
    port_manager.get_serial_instance.return_value = fake_serial
    client = ModbusRTUClient(device_number=1, serial_port_manager=port_manager)
    return client, fake_serial


class TestInferExpectedLength(unittest.TestCase):

    def test_read_message_infers_length_from_word_length_in_request(self):
        client, _ = make_client_with_fake_serial()
        message = client.build_read_message(0x0100, 1)
        self.assertEqual(
            client._infer_expected_length(message),
            ModbusRTUClient.expected_read_response_length(1)
        )

    def test_read_message_with_multiple_words(self):
        client, _ = make_client_with_fake_serial()
        message = client.build_read_message(0x0000, 2)
        self.assertEqual(
            client._infer_expected_length(message),
            ModbusRTUClient.expected_read_response_length(2)
        )

    def test_write_message_infers_fixed_echo_length(self):
        client, _ = make_client_with_fake_serial()
        message = client.build_write_message(0x061E, 0x0FFF)
        self.assertEqual(
            client._infer_expected_length(message),
            ModbusRTUClient.expected_write_response_length()
        )

    def test_too_short_message_returns_none(self):
        client, _ = make_client_with_fake_serial()
        self.assertIsNone(client._infer_expected_length(b'\x01\x03'))


class TestSendAndReceiveUsesInferredLength(unittest.TestCase):

    def test_stops_exactly_at_inferred_length_ignoring_trailing_leftover_bytes(self):
        """Reproduces the exact bug: a genuine response (frame1) is
        immediately followed in the buffer by unrelated leftover bytes
        (frame2, e.g. from an earlier undrained write echo). Without
        expected_length, receive() would swallow both. With inferred
        expected_length, it must stop after frame1 and leave frame2 in the
        buffer for whatever reads it next."""
        frame1 = build_frame(1, CmdCode.WRITE_DATA.value, struct.pack('>HH', 0x061E, 0x0FFF))
        frame2 = build_frame(1, CmdCode.READ_DATA.value, b'\x02' + struct.pack('>H', 1000))
        client, fake_serial = make_client_with_fake_serial(response_after_write=frame1 + frame2)

        message = client.build_write_message(0x061E, 0x0FFF)
        response = client.send_and_receive(message)

        self.assertEqual(response, frame1)
        self.assertEqual(bytes(fake_serial._incoming), frame2)

    def test_explicit_expected_length_overrides_inference(self):
        frame = build_frame(1, CmdCode.READ_DATA.value, b'\x02' + struct.pack('>H', 42))
        client, fake_serial = make_client_with_fake_serial(response_after_write=frame)
        message = client.build_read_message(0x0100, 1)

        response = client.send_and_receive(message, expected_length=len(frame))

        self.assertEqual(response, frame)


class TestSendFlushesInputBufferFirst(unittest.TestCase):

    def test_reset_input_buffer_called_before_write(self):
        client, fake_serial = make_client_with_fake_serial(stale_bytes=b'stale-leftover-bytes')
        client.send(b'\x01\x03\x01\x00\x00\x01\x00\x00')
        self.assertEqual(fake_serial.reset_input_buffer_call_count, 1)
        # the stale bytes were discarded, not left for the next receive()
        self.assertEqual(len(fake_serial._incoming), 0)


if __name__ == "__main__":
    unittest.main()
