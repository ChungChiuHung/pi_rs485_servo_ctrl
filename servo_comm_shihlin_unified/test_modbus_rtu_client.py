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
import threading
import time
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


class RequestEchoingFakeSerial:
    """Answers every write() with a response tailored to THAT specific
    request (encodes the requested address into the reply), unlike a fixed
    canned response -- if two threads' transactions interleave, a thread
    can end up reading bytes that actually belong to a DIFFERENT thread's
    response, and an identical-for-everyone canned reply couldn't tell that
    apart from a clean one. Deliberately has NO internal locking of its
    own: any interleaving between threads' write()/read()/
    reset_input_buffer() calls should only be prevented by
    ModbusRTUClient's own _transaction_lock, so this fake can actually
    expose a real race if that lock were missing."""

    def __init__(self, write_delay: float = 0.0):
        self._incoming = bytearray()
        self._write_delay = write_delay
        self.write_count = 0

    @property
    def in_waiting(self):
        return len(self._incoming)

    def read(self, n):
        n = min(n, len(self._incoming))
        chunk = bytes(self._incoming[:n])
        del self._incoming[:n]
        return chunk

    def write(self, data):
        if self._write_delay:
            time.sleep(self._write_delay)
        self.write_count += 1
        address = struct.unpack('>H', data[2:4])[0]
        response = build_frame(1, CmdCode.READ_DATA.value, b'\x02' + struct.pack('>H', address))
        self._incoming.extend(response)

    def reset_input_buffer(self):
        self._incoming.clear()


class TestSendAndReceiveIsThreadSafe(unittest.TestCase):
    """Regression coverage for the 2026-09-18 real-hardware finding: the web
    UI's periodic /status poll (reads) and an in-progress /action handler's
    own sequence of writes both call send_and_receive() on the SAME
    ModbusRTUClient from different threads (ServoController.lock doesn't
    cover most write methods, e.g. Enable_Position_Mode/config_*/
    clear_alarm_12 never acquire it). Without serializing at this level,
    one thread's send() -- which calls reset_input_buffer() -- can run
    concurrently with another thread's write-then-wait-for-response window,
    wiping or corrupting that other transaction's response. Confirmed via
    real CRC-mismatch errors in production logs during exactly this
    collision (an ENABLE POS MODE action running while /status polled)."""

    def test_concurrent_calls_each_get_their_own_clean_response(self):
        # A real delay inside write() -- without _transaction_lock, this is
        # exactly the window where another thread's send() could call
        # reset_input_buffer() and wipe out, or splice into, a response
        # that's about to "arrive" for the thread currently in write().
        fake_serial = RequestEchoingFakeSerial(write_delay=0.02)
        port_manager = MagicMock()
        port_manager.get_serial_instance.return_value = fake_serial
        client = ModbusRTUClient(device_number=1, serial_port_manager=port_manager)

        results = {}
        results_lock = threading.Lock()
        addresses = list(range(0x0100, 0x0108))
        barrier = threading.Barrier(len(addresses))

        def worker(address):
            message = client.build_read_message(address, 1)
            barrier.wait(timeout=5)  # force all threads to call send_and_receive at once
            response = client.send_and_receive(message)
            with results_lock:
                results[address] = response

        threads = [threading.Thread(target=worker, args=(addr,)) for addr in addresses]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5)

        self.assertEqual(len(results), len(addresses))
        for addr in addresses:
            expected = build_frame(1, CmdCode.READ_DATA.value, b'\x02' + struct.pack('>H', addr))
            self.assertEqual(
                results[addr], expected,
                f"address 0x{addr:04X} got a missing/corrupted response -- "
                "likely interleaved with another thread's transaction"
            )
        self.assertEqual(fake_serial.write_count, len(addresses))


if __name__ == "__main__":
    unittest.main()
