import unittest

from modbus_rtu_response import ModbusRTUResponse, ModbusExceptionResponse
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


class TestExceptionResponse(unittest.TestCase):

    def _exception_frame(self, function, code):
        body = bytes([1, function | 0x80, code])
        return body + ModbusUtils().calculate_crc(body)

    def test_exception_frame_raises_a_typed_error_carrying_the_code(self):
        with self.assertRaises(ModbusExceptionResponse) as ctx:
            ModbusRTUResponse(self._exception_frame(0x06, 0x02))
        self.assertEqual(ctx.exception.function_code, 0x06)
        self.assertEqual(ctx.exception.exception_code, 0x02)

    def test_it_is_still_a_value_error_for_existing_callers(self):
        self.assertTrue(issubclass(ModbusExceptionResponse, ValueError))

    def test_a_bad_crc_is_not_reported_as_a_drive_exception(self):
        frame = bytearray(self._exception_frame(0x06, 0x02))
        frame[-1] ^= 0xFF
        with self.assertRaises(ValueError) as ctx:
            ModbusRTUResponse(bytes(frame))
        self.assertNotIsInstance(ctx.exception, ModbusExceptionResponse)


if __name__ == "__main__":
    unittest.main()
