import struct
from modbus_rtu_client import ModbusRTUClient
from modbus_command_code import CmdCode
from servo_control_registers import ServoControlRegistry

class MockSerialPortManager:
    def get_serial_instance(self):
        return True

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


if __name__ == "__main__":
    modbus_rtu = ModbusRTUClient(device_number=1)

    message = modbus_rtu.build_message(CmdCode.READ_DATA, ServoControlRegistry.MOTOR_FEEDBACK_PULSE, word_length=2)

    print("Constructed Modbut RTU Message:", message.hex())
    
    test_build_read_message()
    test_build_write_message()
