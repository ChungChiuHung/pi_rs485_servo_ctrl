"""
PC setup report for start_server.bat: Python version, the packages in
requirements_pc.txt (OK / MISSING / TOO OLD), a note that the Raspberry Pi
GPIO package is not needed on a PC, and the serial ports Windows can see.
Reads installed-package metadata only -- no network, no pip, no serial I/O
(ports are listed, never opened).

Exit code: 0 = every package is installed at the listed version, 1 =
something has to be installed. Understands `name`, `name>=x`, `name~=x` (as a
minimum) and `name==x`; any other line counts as "needs install" so pip gets
the final say -- except with --installed (pip has just succeeded), where such
lines are trusted to pip instead of failing the re-check forever.

Usage: python check_pc_setup.py [--installed] [requirements_pc.txt]
"""
import os
import re
import sys

try:
    from importlib.metadata import PackageNotFoundError, version as installed_version
except ImportError:  # Python < 3.8: can't check, let pip decide
    print("Python 3.8 or newer is needed to check the installed packages.")
    sys.exit(1)

MIN_PYTHON = (3, 8)  # Flask 2.3 (requirements_pc.txt's floor) needs 3.8+
REQUIREMENT = re.compile(r"^([A-Za-z0-9][A-Za-z0-9._-]*)\s*(?:(>=|~=|==)\s*([0-9][0-9A-Za-z.]*))?$")


def version_tuple(text):
    """Leading numeric release parts only: '3.1.2rc1' -> (3, 1, 2)."""
    parts = []
    for piece in text.split("."):
        digits = re.match(r"\d+", piece)
        if not digits:
            break
        parts.append(int(digits.group()))
    return tuple(parts)


def package_status(lines):
    """One (status, name, installed, wanted) row per requirement line, where
    wanted is e.g. ">= 2.3" or None. Status is OK, MISSING, TOO OLD,
    WRONG VERSION (an == pin that doesn't match) or UNCHECKED (a line this
    can't parse)."""
    rows = []
    for raw in lines:
        line = raw.split("#", 1)[0].split(";", 1)[0].strip()
        if not line:
            continue
        match = REQUIREMENT.match(line)
        if not match:
            rows.append(("UNCHECKED", line, None, None))
            continue
        name, operator, wanted = match.groups()
        wanted_text = f"{operator} {wanted}" if wanted else None
        try:
            have = installed_version(name)
        except PackageNotFoundError:
            rows.append(("MISSING", name, None, wanted_text))
            continue
        status = "OK"
        if operator == "==" and version_tuple(have) != version_tuple(wanted):
            status = "WRONG VERSION"
        elif operator in (">=", "~=") and version_tuple(have) < version_tuple(wanted):
            status = "TOO OLD"
        rows.append((status, name, have, wanted_text))
    return rows


def serial_ports():
    """(device, description) for each port, Bluetooth ones left out the same
    way serial_port_manager.py does; None if pyserial isn't installed."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    return [(p.device, p.description) for p in sorted(list_ports.comports(), key=lambda p: p.device)
            if "bluetooth" not in p.device.lower()]


def report(requirements_path, installed=False):
    """Prints the report; returns the exit code. installed=True: pip has just
    succeeded, so lines this can't check (UNCHECKED) don't fail it."""
    version = ".".join(map(str, sys.version_info[:3]))
    python_ok = sys.version_info[:2] >= MIN_PYTHON
    print(f"Python      : {version}  {'OK' if python_ok else 'TOO OLD'}"
          f" (needs {MIN_PYTHON[0]}.{MIN_PYTHON[1]} or newer)")
    print(f"              {sys.executable}")

    try:
        with open(requirements_path, encoding="utf-8") as f:
            rows = package_status(f.readlines())
    except OSError as e:
        print(f"Cannot read {requirements_path}: {e}")
        return 1
    print(f"Packages    : from {os.path.basename(requirements_path)}")
    for status, name, have, wanted in rows:
        needs = f"needs {wanted}" if wanted else "any version"
        if status == "UNCHECKED":
            needs = "left to pip" if installed else "can't check this line; pip will"
        print(f"  {status:<13} {name:<12} {have or '-':<10} ({needs})")
    print("  (RPi.GPIO is for the Raspberry Pi only -- not needed on a PC.)")

    ports = serial_ports()
    print("Serial ports:")
    if ports is None:
        print("  (shown once pyserial is installed)")
    elif not ports:
        print("  none found -- plug in the RS-485 USB adapter. If it is plugged in,")
        print("  install its driver (CH340 / FTDI / CP210x) and look under")
        print("  Device Manager > Ports (COM & LPT).")
    else:
        for device, description in ports:
            print(f"  {device:<8} {description}")
    forced = os.getenv("SERVO_SERIAL_PORT")
    if forced:
        print(f"  SERVO_SERIAL_PORT is set: the server tries {forced} first.")

    accepted = {"OK", "UNCHECKED"} if installed else {"OK"}
    return 0 if python_ok and all(row[0] in accepted for row in rows) else 1


def main(argv):
    args = argv[1:]
    installed = "--installed" in args
    paths = [a for a in args if a != "--installed"]
    return report(paths[0] if paths else "requirements_pc.txt", installed=installed)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
