"""
Standalone diagnostic: measure how tightly-spaced consecutive Modbus RTU
requests can be sent to the real driver before responses start failing
(timeout / CRC mismatch / no response), so app.py can enforce a debounce
interval that's actually justified by real hardware behavior instead of a
guessed number.

Read-only: only sends READ_DATA requests against the current-alarm-code
register (0x0100, 1 word) -- never moves or writes anything. Same safety
class as check_pa28.py / check_encoder_tracking.py.

Usage:
    python check_response_timing.py [--samples N]
"""
import argparse
import sys
import time

from serial_port_manager import SerialPortManager
from modbus_rtu_client import ModbusRTUClient
from modbus_rtu_response import ModbusRTUResponse

ALARM_CODE_ADDRESS = 0x0100

# Gaps to try, in seconds, from generous to aggressive.
INTERVALS_MS = [500, 300, 200, 150, 100, 75, 50, 30, 20, 10, 0]


def try_one_read(client: ModbusRTUClient) -> bool:
    message = client.build_read_message(ALARM_CODE_ADDRESS, 1)
    # expected_length matters a lot here: without it, receive()'s read loop
    # doesn't know it's done as soon as the full response has arrived, and
    # instead waits out its entire timeout window of silence before
    # returning -- inflating "round trip" time by ~500ms regardless of the
    # real wire/driver latency. This was found by this script's own first
    # run: every interval measured ~520ms average round-trip, an
    # unmistakable timeout artifact rather than a real hardware number.
    expected_length = ModbusRTUClient.expected_read_response_length(1)
    response = client.send_and_receive(message, expected_length=expected_length)
    if response is None:
        return False
    try:
        ModbusRTUResponse(response)  # validates CRC / frame shape
        return True
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=20,
                         help="Reads attempted per interval (default: 20)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--device-number", type=int, default=1)
    args = parser.parse_args()

    port_manager = SerialPortManager(baud_rate=args.baud)
    port_manager.connect()
    if not port_manager.get_serial_instance():
        print("Could not open any serial port.")
        return 2

    print(f"Connected: {port_manager.get_connected_port()} @ {args.baud} baud (RTU, device {args.device_number})")
    client = ModbusRTUClient(device_number=args.device_number, serial_port_manager=port_manager)

    print(f"\n{'Interval (ms)':>14} | {'Success':>8} | {'Fail':>5} | {'Success rate':>13} | {'Avg round-trip (ms)':>20}")
    print("-" * 75)

    results = []
    for interval_ms in INTERVALS_MS:
        successes = 0
        failures = 0
        round_trip_times = []
        for _ in range(args.samples):
            start = time.perf_counter()
            ok = try_one_read(client)
            round_trip_times.append((time.perf_counter() - start) * 1000)
            if ok:
                successes += 1
            else:
                failures += 1
            time.sleep(interval_ms / 1000.0)

        rate = successes / args.samples
        avg_rt = sum(round_trip_times) / len(round_trip_times)
        results.append((interval_ms, successes, failures, rate, avg_rt))
        print(f"{interval_ms:>14} | {successes:>8} | {failures:>5} | {rate*100:>12.0f}% | {avg_rt:>20.1f}")

    # Find the smallest interval with a 100% success rate, scanning from the
    # most conservative (largest) end so we report the first failure point.
    safe_interval = None
    for interval_ms, successes, failures, rate, avg_rt in sorted(results, key=lambda r: -r[0]):
        if rate == 1.0:
            safe_interval = interval_ms
        else:
            break

    print()
    if safe_interval is not None:
        print(f"RESULT: smallest interval with 100% success in this run: {safe_interval} ms")
    else:
        print("RESULT: no interval achieved 100% success -- check connection before trusting any timing number.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
