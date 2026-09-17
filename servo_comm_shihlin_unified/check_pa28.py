"""
Standalone diagnostic: connect to the real servo driver over RS485 and
report whether PA28 (absolute-encoder mode) is set to 1.

Run this directly on the Raspberry Pi, with the driver connected:

    cd servo_comm_shihlin_unified/
    python3 check_pa28.py

This does NOT move, enable, or write anything to the motor -- it only reads
a status parameter (read-only, no hardware-safety confirmation needed).

Exit codes:
    0 -- PA28 == 1, absolute mode confirmed.
    1 -- PA28 == 0 or an unexpected value; NOT confirmed absolute mode.
    2 -- Could not read PA28 at all (check wiring / port / baud rate).
"""
import sys

from serial_port_manager import SerialPortManager
from modbus_ascii_client import ModbusASCIIClient
from absolute_mode_check import check_absolute_mode


def main():
    # NOTE: baud rate here matches servo_comm_shihlin_50W's default (9600).
    # If checking against the 400W driver instead, pass baud_rate=115200.
    port_manager = SerialPortManager(baud_rate=9600)
    port_manager.connect()

    if not port_manager.get_serial_instance():
        print("Could not open any serial port. Check wiring/permissions and retry.")
        return 2

    print(f"Connected port: {port_manager.get_connected_port()}")
    print(f"Baud rate: {port_manager.get_baud_rate()}")

    modbus_client = ModbusASCIIClient.get_instance(
        device_number=1, serial_port_manager=port_manager
    )

    result = check_absolute_mode(modbus_client)

    if result is True:
        print("\nRESULT: PA28 = 1 -- absolute mode confirmed. Safe to proceed "
              "with the PA32/PA33-based encoder-overflow fix.")
        return 0
    elif result is False:
        print("\nRESULT: PA28 is NOT 1 -- absolute mode NOT confirmed. Do not "
              "proceed with the PA32/PA33-based fix until this is corrected "
              "on the driver.")
        return 1
    else:
        print("\nRESULT: Could not read PA28 at all. Check the connection and "
              "retry before concluding anything.")
        return 2


if __name__ == "__main__":
    sys.exit(main())
