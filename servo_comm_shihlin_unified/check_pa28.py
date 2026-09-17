"""
Standalone diagnostic: connect to the real servo driver over RS485 and
report whether PA28 (absolute-encoder mode) is set to 1.

Run this directly on the Raspberry Pi, with the driver connected:

    cd servo_comm_shihlin_unified/
    python3 check_pa28.py

This does NOT move, enable, or write anything to the motor -- it only reads
a status parameter (read-only, no hardware-safety confirmation needed).

By default this speaks Modbus ASCII (matching servo_comm_shihlin_50W's
communication settings). Pass --protocol rtu to instead try Modbus RTU
(binary framing + CRC16) -- this exists to test the hypothesis that the
physical driver's PC22 protocol option is actually set to one of the RTU
variants (6/7/8) rather than an ASCII variant (0-5); see
docs/en_manual.txt SS9.2 and the diagnostic report in this project's history
(82 ASCII-framed attempts across every documented baud/station-number
combination got zero response).

Exit codes:
    0 -- PA28 == 1, absolute mode confirmed.
    1 -- PA28 == 0 or an unexpected value; NOT confirmed absolute mode.
    2 -- Could not read PA28 at all (check wiring / port / baud rate).
"""
import argparse
import sys

from serial_port_manager import SerialPortManager
from modbus_ascii_client import ModbusASCIIClient
from modbus_response import ModbusResponse
from modbus_rtu_client import ModbusRTUClient
from modbus_rtu_response import ModbusRTUResponse
from absolute_mode_check import check_absolute_mode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--protocol", choices=["ascii", "rtu"], default="ascii",
        help="Modbus framing to use (default: ascii, matching the rest of "
             "this project's assumptions)."
    )
    # NOTE: default baud matches servo_comm_shihlin_50W's default (9600).
    # If checking against the 400W driver instead, pass --baud 115200.
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--device-number", type=int, default=1)
    args = parser.parse_args()

    port_manager = SerialPortManager(baud_rate=args.baud)
    port_manager.connect()

    if not port_manager.get_serial_instance():
        print("Could not open any serial port. Check wiring/permissions and retry.")
        return 2

    print(f"Connected port: {port_manager.get_connected_port()}")
    print(f"Baud rate: {port_manager.get_baud_rate()}")
    print(f"Protocol: {args.protocol}")

    if args.protocol == "rtu":
        modbus_client = ModbusRTUClient(
            device_number=args.device_number, serial_port_manager=port_manager
        )
        result = check_absolute_mode(modbus_client, response_parser=ModbusRTUResponse)
    else:
        modbus_client = ModbusASCIIClient.get_instance(
            device_number=args.device_number, serial_port_manager=port_manager
        )
        result = check_absolute_mode(modbus_client, response_parser=ModbusResponse)

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
