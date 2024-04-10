import struct
import unittest
from unittest.mock import MagicMock, patch
from modbus_rtu_client import ModbusRTUClient
from modbus_command_code import CmdCode
from servo_control_registers import ServoControlRegistry

class MockSerialPortManager:
    def __init__(self):
        pass
    def get_serial_instance(self):
        return True
    def write(self, message):
        print(f"Mock write: {message.hex()}")
    def read_all(self):
        return bytes.fromhex('01020304')
    def inWaiting(self):
        return 4

class TestModbusRTUClient(unittest.TestCase):
    def setUp(self):
        self.mock_serial_manager = MockSerialPortManager()
        self.modbus_client = ModbusRTUClient(device_number=1, serial_port_manager=self.mock_serial_manager)
        self.modbus_client.send = MagicMock()

    def test_build_read_message(self):
        read_message = self.modbus_client.build_read_message(ServoControlRegistry.MOTOR_FEEDBACK_PULSE, word_length=2)
        print("Read Message:", read_message.hex())

    def test_build_write_message(self):
        sample_data = struct.pack('>H', 1000)
        write_message = self.modbus_client.build_write_message(ServoControlRegistry.MOTOR_SPEED, sample_data)
        print("Write Message: ", write_message.hex())

    def test_build_write_message_2(self):
        test_cases = [
            (ServoControlRegistry.SEL_DI_CONTROL_SOURCE, struct.pack('>H', 0xFFFF)),  # All bits set
            (ServoControlRegistry.DI_PIN_CONTROL, struct.pack('>H', 0x0001)),  # First bit set
            (ServoControlRegistry.JOG_SPEED, struct.pack('>H', 1500)),  # Mid-range speed
            (ServoControlRegistry.POS_PULSES_CMD_1, struct.pack('>H', 0x1234)),  # Example pulse count
        ]

        for registry, data in test_cases:
            message = self.modbus_client.build_write_message(registry, data)
            print(f"Test with {registry.name}: Message = {message.hex()}")
    
    @patch('modbus_rtu_client.ModbusRTUClient.set_positioning_test_mode')
    @patch('modbus_rtu_client.ModbusRTUClient.set_acc_dec_time')
    @patch('modbus_rtu_client.ModbusRTUClient.set_jog_speed')
    @patch('modbus_rtu_client.ModbusRTUClient.set_command_pulses')
    @patch('modbus_rtu_client.ModbusRTUClient.start_positioning_operation')
    @patch('modbus_rtu_client.ModbusRTUClient.exit_positioning_mode')

    def test_positioning(self, mock_exit, mock_start, mock_set_pulses, mock_set_speed, mock_set_acc, mock_set_test_mode):
        acc_dec_time = 1000
        jog_speed = 1500
        command_pulses = 123456
        direction = 1

        success = self.modbus_client.positioning(acc_dec_time, jog_speed, command_pulses, direction)

        self.assertTrue(success, "Positioning method failed to complete successfully.")
        mock_set_test_mode.assert_called_once()
        mock_set_acc.assert_called_once_with(acc_dec_time)
        mock_set_speed.assert_called_once_with(jog_speed)
        mock_set_pulses.assert_called_once_with(command_pulses)
        mock_start.assert_called_once_with(direction)
        mock_exit.assert_called_once()
    

if __name__ == "__main__":
   unittest.main()
