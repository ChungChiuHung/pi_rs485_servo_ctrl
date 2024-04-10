import struct
from unittest import MagicMock, patch
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

def test_build_read_message():
    mock_serial_manager = MockSerialPortManager()
    modbus_client = ModbusRTUClient(device_number=1, serial_port_manager=mock_serial_manager)
    read_message = modbus_client.build_read_message(ServoControlRegistry.MOTOR_FEEDBACK_PULSE, word_length=2)
    print("Read Message:", read_message.hex())

def test_build_write_message():
    mock_serial_manager = MockSerialPortManager()
    modbus_client = ModbusRTUClient(device_number=1, serial_port_manager=mock_serial_manager)
    sample_data = struct.pack('>H', 1000)
    write_message = modbus_client.build_write_message(ServoControlRegistry.MOTOR_SPEED, sample_data)
    print("Write Message: ", write_message.hex())

def test_build_write_message_2():
    mock_serial_maanger = MockSerialPortManager()
    modbus_client = ModbusRTUClient(device_number=1, serial_port_manager=mock_serial_maanger)
    test_cases = [
        (ServoControlRegistry.SEL_DI_CONTROL_SOURCE, struct.pack('>H', 0xFFFF)),  # All bits set
        (ServoControlRegistry.DI_PIN_CONTROL, struct.pack('>H', 0x0001)),  # First bit set
        (ServoControlRegistry.JOG_SPEED, struct.pack('>H', 1500)),  # Mid-range speed
        (ServoControlRegistry.POS_PULSES_CMD_1, struct.pack('>H', 0x1234)),  # Example pulse count
    ]

    for registry, data in test_cases:
        message = modbus_client.build_write_message(registry, data)
        print(f"Test with {registry.name}: Message = {message.hex()}")

def test_positioning():
    mock_serial_maanger = MockSerialPortManager()
    modbus_client = ModbusRTUClient(device_number=1, serial_port_manager=mock_serial_maanger)

    acc_dec_time = 1000
    jog_speed = 1500
    command_pulses = 123456
    direction = 1

    success = modbus_client.positioning(acc_dec_time, jog_speed, command_pulses, direction)

    # Verify that messages were sent correctly
    calls = [
        patch('modbus_client.set_positioning_test_mode'),
        patch('modbus_client.set_acc_dec_time', args=(acc_dec_time,)),
        patch('modbus_client.set_jog_speed', args=(jog_speed,)),
        patch('modbus_client.set_command_pulses', args=(command_pulses,)),
        patch('modbus_client.start_positioning_operation', args=(direction,)),
        patch('modbus_client.exit_positioning_mode')
    ]
    for call in calls:
        modbus_client.send.assert_any_call(call)
    

if __name__ == "__main__":
    test_build_read_message()
    test_build_write_message()
    test_build_write_message_2()
    test_positioning()
