import json
import unittest
from servo_serial_protocol_handler import SerialProtocolHandler
from status_bit_mapping import BitMapOutput
from servo_command_code import CmdCode

class TestSerialProtocolHandler(unittest.TestCase):
    def setUp(self):
        self.handler = SerialProtocolHandler()

    def test_constrcut_header(self):
        header = self.handler.construct_header(15,5)
        self.assertEqual(header, b'\x1f\x05', "Header construction failed with valid inputs.")

        with self.assertRaises(ValueError):
            self.handler.construct_header(32,5)
        with self.assertRaises(ValueError):
            self.handler.construct_header(15,0)

    def test_set_bit_status(self):
        result = self.handler.set_bit_status(BitMapOutput.MEND,1)
        expected_result = b'\x00\x01\x00\x00\x00\x10'
        self.assertEqual(result, expected_result, "Setting single bit status failed")

    def test_parse_bit_statuses(self):
        parameter_data = b'\x00\x00\x00\x10'  # Example parameter data, adjust as needed
        expected_statuses = {'MEND': True}  # Expected statuses, adjust as needed
        bit_statuses = self.handler.parse_bit_statuses(parameter_data)
        self.assertEqual(bit_statuses, expected_statuses, "Parsing bit statuses failed")

    def test_construct_packet(self):
        destination_address = 5
        command_code = CmdCode.GET_STATE_VALUE_4
        data = b'\x00\x01'
        packet = self.handler.construct_packet(destination_address, command_code, data, is_response=True)

        expected_result = b'\x00'
        self.assertEqual(packet[:4], expected_result[:4], "Packet construction failed")

    def test_response_parser(self):
        mock_data = b'\x1f\x05\x80\x04\x00\x00\x00\x10' + b'\x00\x00'  # Adjust CRC
        expected_response = {
            'header': 0x1f,
            'destination_address': 5,
            'control_code': 0x80,
            'command_code': CmdCode.GET_STATE_VALUE_4.value,
            'parameter_data': [0, 0, 0, 16],
            'bit_statuses': {'MEND': True}  # Assuming this matches your logic
        }
        response = json.loads(self.handler.response_parser(CmdCode.GET_STATE_VALUE_4.value, mock_data))
        self.assertEqual(response, expected_response, "Response parsing failed")

if __name__ == "__main__":
    unittest.main()