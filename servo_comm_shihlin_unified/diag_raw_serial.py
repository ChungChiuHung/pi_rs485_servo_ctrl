"""
Standalone RS485 diagnostic (NOT part of the check_pa28.py path).

Read-only: only sends Modbus ASCII READ_DATA (0x03) requests, never write
commands. Prints raw hex of everything sent/received so a "no response" can
be told apart from "sent but nothing came back" vs "got bytes but wrong
framing".

Usage:
    python3 diag_raw_serial.py                      # sweep default addr/baud combos
    python3 diag_raw_serial.py --port /dev/ttyUSB0 --baud 9600 --addr 1
    python3 diag_raw_serial.py --loopback --port /dev/ttyUSB0 --baud 9600
"""
import argparse
import struct
import sys
import time

import serial


def build_pa28_read_frame(device_number: int) -> bytes:
    """Same framing as ModbusASCIIClient.build_read_message(PA.ABS.address, 2)."""
    pa28_address = 0x0336  # 0x0300 + (28 - 1) * 2, see servo_p_register.py
    adr = f"{device_number:02X}"
    cmd = f"{0x03:02X}"
    data = struct.pack(">HH", pa28_address, 2)
    msg_no_lrc = f":{adr}{cmd}{data.hex().upper()}"
    body = bytes.fromhex(msg_no_lrc[1:])
    lrc = (-sum(body)) & 0xFF
    return f"{msg_no_lrc}{lrc:02X}\r\n".encode("ascii")


def hexdump(label: str, data: bytes) -> None:
    if not data:
        print(f"  {label}: (empty, 0 bytes)")
        return
    print(f"  {label} ({len(data)} bytes): {data.hex(' ')}")
    try:
        print(f"  {label} (ascii): {data!r}")
    except Exception:
        pass


def try_one(port: str, baud: int, device_number: int, read_window: float = 1.0, bits: int = 8):
    frame = build_pa28_read_frame(device_number)
    bytesize = serial.SEVENBITS if bits == 7 else serial.EIGHTBITS
    print(f"\n--- port={port} baud={baud} device_number={device_number} bits={bits} ---")
    try:
        with serial.Serial(
            port, baud, timeout=0.2,
            bytesize=bytesize, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
        ) as ser:
            ser.reset_input_buffer()
            hexdump("TX", frame)
            ser.write(frame)
            ser.flush()

            deadline = time.time() + read_window
            response = bytearray()
            while time.time() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    response.extend(chunk)
                    deadline = time.time() + 0.2  # keep reading a bit after last byte
            hexdump("RX", bytes(response))
            return bytes(response)
    except serial.SerialException as e:
        print(f"  SERIAL ERROR: {e}")
        return None


def loopback_test(port: str, baud: int):
    """Requires the RS485 A/B terminals to be jumpered together by hand first."""
    print(f"\n=== Loopback test on {port} @ {baud} ===")
    print("Make sure A and B (or TX/RX) are physically bridged before this runs.")
    payload = b"LOOPBACK-TEST-1234567890\r\n"
    try:
        with serial.Serial(
            port, baud, timeout=0.2,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
        ) as ser:
            ser.reset_input_buffer()
            hexdump("TX", payload)
            ser.write(payload)
            ser.flush()
            time.sleep(0.3)
            response = ser.read(ser.in_waiting or 1)
            hexdump("RX", response)
            if response == payload:
                print("  RESULT: PASS -- adapter transmits and receives on its own bus.")
            elif response:
                print("  RESULT: PARTIAL -- got bytes back but they don't match exactly "
                      "(could be timing/framing, not necessarily broken).")
            else:
                print("  RESULT: FAIL -- nothing looped back. Either A/B aren't bridged, "
                      "or the transceiver doesn't echo while transmitting (some auto "
                      "direction-controlled adapters disable RX while driving TX -- "
                      "this is inconclusive on its own if that's the design).")
    except serial.SerialException as e:
        print(f"  SERIAL ERROR: {e}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=None,
                         help="If omitted, sweeps 9600/19200/38400/115200")
    parser.add_argument("--addr", type=int, default=None,
                         help="If omitted, sweeps device_number 1..10")
    parser.add_argument("--bits", type=int, choices=[7, 8], default=8)
    parser.add_argument("--loopback", action="store_true")
    args = parser.parse_args()

    if args.loopback:
        loopback_test(args.port, args.baud or 9600)
        return

    bauds = [args.baud] if args.baud else [9600, 19200, 38400, 115200]
    addrs = [args.addr] if args.addr else list(range(1, 11))

    results = []
    for baud in bauds:
        for addr in addrs:
            resp = try_one(args.port, baud, addr, bits=args.bits)
            results.append((baud, addr, bool(resp)))
            time.sleep(0.05)

    print("\n=== Summary ===")
    any_hit = False
    for baud, addr, got in results:
        if got:
            any_hit = True
        print(f"  baud={baud:<6} device_number={addr:<3} response={'YES' if got else 'no'}")
    if not any_hit:
        print("\n  No combination produced any response.")


if __name__ == "__main__":
    sys.exit(main())
