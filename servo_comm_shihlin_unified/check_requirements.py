"""
Exit 0 if every package in a requirements file is installed at (at least)
the listed minimum version, 1 otherwise -- so a launcher can skip `pip
install` when nothing is missing. Reads only installed-package metadata: no
network, no pip.

Understands the simple lines requirements_pc.txt uses (`name` or
`name>=version`); comments, blank lines and environment markers are ignored
for the decision (anything it can't parse counts as "needs install", so pip
gets the final say).

Usage: python check_requirements.py [requirements_pc.txt]
"""
import re
import sys

try:
    from importlib.metadata import PackageNotFoundError, version as installed_version
except ImportError:  # Python < 3.8: can't check, let pip decide
    sys.exit(1)

REQUIREMENT = re.compile(r"^([A-Za-z0-9][A-Za-z0-9._-]*)\s*(?:>=\s*([0-9][0-9A-Za-z.]*))?$")


def version_tuple(text):
    """Leading numeric release parts only: '3.1.2rc1' -> (3, 1, 2)."""
    parts = []
    for piece in text.split("."):
        digits = re.match(r"\d+", piece)
        if not digits:
            break
        parts.append(int(digits.group()))
    return tuple(parts)


def problems(lines):
    """Human-readable reasons the requirements aren't met (empty = all met)."""
    found = []
    for raw in lines:
        line = raw.split("#", 1)[0].split(";", 1)[0].strip()
        if not line:
            continue
        match = REQUIREMENT.match(line)
        if not match:
            found.append(f"{line} (can't check this line)")
            continue
        name, minimum = match.groups()
        try:
            have = installed_version(name)
        except PackageNotFoundError:
            found.append(f"{name} is not installed")
            continue
        if minimum and version_tuple(have) < version_tuple(minimum):
            found.append(f"{name} {have} is older than {minimum}")
    return found


def main(argv):
    path = argv[1] if len(argv) > 1 else "requirements_pc.txt"
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()
    except OSError as e:
        print(f"Cannot read {path}: {e}")
        return 1
    missing = problems(lines)
    for reason in missing:
        print(f"  - {reason}")
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
