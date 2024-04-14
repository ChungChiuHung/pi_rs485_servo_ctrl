from serial_port_manager import SerialPortManager
from modbus_ascii_client import ModbusASCIIClient

def test_modbus_ascii_client():
    print("Initializing SerialPortManager...")
    serial_manager = SerialPortManager()

    print("Creating ModbusASCIIClient instance...")
    modbus_client = ModbusASCIIClient(device_number=1, serial_port_manager=serial_manager)

    print("Testing connection ...")
    if not modbus_client.ensure_connection():
        print("Failed to connect to the device")
        return
    
    try:
        print("Setting DI Control Source...")
        modbus_client.set_di_control_source(0x00FF)
        response = modbus_client.receive()
        if response:
            print("DI Control Source set successfully.")

        
        # Testing DI state setting
        print("Setting DI state ...")
        modbus_client.set_di_state(0x00AA)
        response = modbus_client.receive()
        if response:
            print("DI State set successfully.")

        # Positioning sequence
        print("Executing positioning sequence...")
        success = modbus_client.positioning(acc_dec_time = 5000, jog_speed=1000, command_pulses = 100000, direction=1)
        if success:
            print("Positioning sequence completed successfully.")
    
    except Exception as e:
        print(f"An error occurred during testing: {e}")

    finally:
        print("Cleaning up ...")
        modbus_client.exit_positioning_mode()
        serial_manager.disconnect()
        print("Test complteted and port disconnected.")


if __name__ == "__main__":
    test_modbus_ascii_client()
