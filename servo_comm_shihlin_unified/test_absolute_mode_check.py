import unittest
from unittest.mock import MagicMock, patch

from absolute_mode_check import check_absolute_mode


class TestCheckAbsoluteMode(unittest.TestCase):

    def _make_modbus_client(self, send_and_receive_return=b"dummy-response"):
        client = MagicMock()
        client.build_read_message.return_value = b"dummy-message"
        client.send_and_receive.return_value = send_and_receive_return
        return client

    @patch("absolute_mode_check.ModbusResponse")
    def test_pa28_equals_1_returns_true(self, mock_response_cls):
        mock_response_cls.return_value.get_value.return_value = 1
        client = self._make_modbus_client()

        result = check_absolute_mode(client)

        self.assertTrue(result)

    @patch("absolute_mode_check.ModbusResponse")
    def test_pa28_equals_0_returns_false(self, mock_response_cls):
        mock_response_cls.return_value.get_value.return_value = 0
        client = self._make_modbus_client()

        result = check_absolute_mode(client)

        self.assertFalse(result)

    @patch("absolute_mode_check.ModbusResponse")
    def test_pa28_unexpected_value_returns_false(self, mock_response_cls):
        # Manual only documents 0 or 1 for PA28; anything else must not be
        # silently treated as "safe".
        mock_response_cls.return_value.get_value.return_value = 5

        client = self._make_modbus_client()

        result = check_absolute_mode(client)

        self.assertFalse(result)

    def test_no_response_returns_none(self):
        # send_and_receive() returning None means a communication failure --
        # must be treated as UNKNOWN, not as "safe" (False would be misread
        # as "confirmed incremental mode", which is a different failure mode
        # than "we don't actually know").
        client = self._make_modbus_client(send_and_receive_return=None)

        result = check_absolute_mode(client)

        self.assertIsNone(result)

    @patch("absolute_mode_check.ModbusResponse")
    def test_response_parse_exception_returns_none(self, mock_response_cls):
        mock_response_cls.side_effect = ValueError("malformed response")
        client = self._make_modbus_client()

        result = check_absolute_mode(client)

        self.assertIsNone(result)

    @patch("absolute_mode_check.ModbusResponse")
    def test_get_value_returns_none_treated_as_unknown(self, mock_response_cls):
        mock_response_cls.return_value.get_value.return_value = None
        client = self._make_modbus_client()

        result = check_absolute_mode(client)

        self.assertIsNone(result)

    def test_reads_pa28_address(self):
        # PA28 = register no. 28, start_address 0x0300, size 2 per register
        # -> 0x0300 + (28-1)*2 = 0x0336, matches the manual's address table.
        client = self._make_modbus_client()
        with patch("absolute_mode_check.ModbusResponse"):
            check_absolute_mode(client)

        client.build_read_message.assert_called_once_with(0x0336, 2)


if __name__ == "__main__":
    unittest.main()
