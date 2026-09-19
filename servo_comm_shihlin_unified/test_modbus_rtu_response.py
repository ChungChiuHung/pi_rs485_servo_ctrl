import unittest

from modbus_rtu_response import ModbusRTUResponse
from modbus_utils import ModbusUtils


def read_response(data_bytes: bytes) -> bytes:
    body = bytes([1, 0x03, len(data_bytes)]) + data_bytes
    return body + ModbusUtils().calculate_crc(body)


class TestGetValueSigned(unittest.TestCase):
    """PA32 (APR, absolute-encoder revolutions) is documented as signed
    (-32768~32767); get_value() must be able to decode it, while staying
    unsigned by default for every existing caller."""

    def test_unsigned_by_default(self):
        # Two-word value 0xFFFFFFFF, words swapped by the driver's layout.
        response = ModbusRTUResponse(read_response(b'\xff\xff\xff\xff'))
        self.assertEqual(response.get_value(), 0xFFFFFFFF)

    def test_signed_negative_two_word_value(self):
        response = ModbusRTUResponse(read_response(b'\xff\xfd\xff\xff'))  # low word FFFD, high FFFF
        self.assertEqual(response.get_value(signed=True), -3)

    def test_signed_positive_value_unchanged(self):
        response = ModbusRTUResponse(read_response(b'\x00\x05\x00\x00'))
        self.assertEqual(response.get_value(signed=True), 5)

    def test_signed_single_word(self):
        response = ModbusRTUResponse(read_response(b'\xff\xfe'))
        self.assertEqual(response.get_value(signed=True), -2)


if __name__ == "__main__":
    unittest.main()
