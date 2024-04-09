from modbus_rtu_client import ModbusRTUClient
from modbus_command_code import CmdCode
from servo_control_registers import ServoControlRegistry

if __name__ == "__main__":
    modbus_rtu = ModbusRTUClient(device_number=1)

    message = modbus_rtu.build_message(CmdCode.READ_DATA, ServoControlRegistry.MOTOR_FEEDBACK_PULSE, word_length=2)

    print("Constructed Modbut RTU Message:", message.hex())