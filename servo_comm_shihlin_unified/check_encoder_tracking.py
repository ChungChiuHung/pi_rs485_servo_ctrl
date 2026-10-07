"""
Standalone diagnostic: read the raw 32-bit "motor feedback pulses" register
(0x0000) repeatedly over RS-485 (RTU, confirmed working parameters -- see
docs/servo_comm_shihlin_merge_design.md's 2026-09-18 hardware confirmation
note: RTU, 115200 baud, device number 1) and feed each reading through
EncoderPulseTracker, printing raw vs. cumulative side by side.

This does NOT move, enable, or write anything to the motor -- read-only,
same safety class as check_pa28.py.

Usage:
    python check_encoder_tracking.py [--count N] [--interval SECONDS]
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from modbus_rtu_client import ModbusRTUClient
from modbus_rtu_response import ModbusRTUResponse
from encoder_pulse_tracker import EncoderPulseTracker

RAW_ENCODER_ADDRESS = 0x0000  # "Motor feedback pulses", 2-word register


def read_raw_encoder(client: ModbusRTUClient):
    message = client.build_read_message(RAW_ENCODER_ADDRESS, 2)
    response = client.send_and_receive(
        message, expected_length=ModbusRTUClient.expected_read_response_length(2)
    )
    if response is None:
        return None
    try:
        return ModbusRTUResponse(response).get_value()
    except Exception as e:
        print(f"  (failed to parse response: {e})")
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--device-number", type=int, default=1)
    args = parser.parse_args()

    port_manager = SerialPortManager(baud_rate=args.baud)
    port_manager.connect()
    if not port_manager.get_serial_instance():
        print("Could not open any serial port.")
        return 2

    print(f"Connected port: {port_manager.get_connected_port()} @ {args.baud} baud (RTU, device {args.device_number})")

    client = ModbusRTUClient(device_number=args.device_number, serial_port_manager=port_manager)
    tracker = EncoderPulseTracker()

    failures = 0
    for i in range(args.count):
        raw = read_raw_encoder(client)
        if raw is None:
            failures += 1
            print(f"[{i+1}/{args.count}] No response reading encoder.")
        else:
            cumulative = tracker.update(raw)
            print(f"[{i+1}/{args.count}] raw=0x{raw:08X} ({raw}) -> cumulative={cumulative}")
        time.sleep(args.interval)

    if failures == args.count:
        print("\nRESULT: every read failed -- check connection.")
        return 2
    if failures:
        print(f"\nRESULT: {failures}/{args.count} reads failed; see above.")
    else:
        print("\nRESULT: all reads succeeded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
