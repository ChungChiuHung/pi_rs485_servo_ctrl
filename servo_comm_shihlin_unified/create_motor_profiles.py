"""
Creates motor_profiles.json from motor_profiles.example.json when it is
missing -- called by start_server.bat on a new PC or a fresh clone.

motor_profiles.json is tuned per rig and kept out of git (see .gitignore), so
a clone never has it and app.py cannot start without it. The template is the
configuration of the rig in use (default motor: its active_profile,
shihlin_400W); --profile makes another of its motors the default instead.
active_profile is the motor used at every start -- switching motors in the
web UI is not saved to the file.

Never overwrites an existing motor_profiles.json, never talks to the drive.

Usage:
  python create_motor_profiles.py                       default motor
  python create_motor_profiles.py --profile shihlin_50W
Exit code: 0 = the file exists (created now or already there), 1 = not created.
"""
import json
import os
import sys

from motor_profile import resolve_profile

EXAMPLE_FILE = "motor_profiles.example.json"
TARGET_FILE = "motor_profiles.json"


def describe(profiles, name):
    profile = profiles["profiles"][name]
    return f"{name} (gear ratio {profile['gear_ratio']}:1, {profile['baud_rate']} baud)"


def create(example_path=EXAMPLE_FILE, target_path=TARGET_FILE, profile_name=None):
    """Returns the exit code (see the module docstring)."""
    if os.path.exists(target_path):
        print(f"{target_path} already exists -- left unchanged.")
        return 0
    try:
        with open(example_path, encoding="utf-8") as f:
            profiles = json.load(f)
    except (OSError, ValueError) as e:
        print(f"ERROR: cannot read the template {example_path}: {e}")
        return 1

    if profile_name is None:
        profile_name = profiles["active_profile"]
    elif profile_name not in profiles["profiles"]:
        print(f"ERROR: unknown profile {profile_name!r}; the template has: "
              f"{', '.join(profiles['profiles'])}")
        return 1

    profiles["active_profile"] = profile_name
    resolve_profile(profiles, profile_name)  # fails loudly if the template is malformed
    with open(target_path, "w", encoding="utf-8") as f:
        json.dump(profiles, f, indent=2)
        f.write("\n")
    print(f"Created {target_path} -- default motor: {describe(profiles, profile_name)}.")
    others = [name for name in profiles["profiles"] if name != profile_name]
    if others:
        print(f"Another motor? Pick {', '.join(others)} in the web page's Motor Profile list,")
        print(f"or delete {target_path} and run: python create_motor_profiles.py --profile {others[0]}")
    print("Press SET HOME in the web page before trusting any absolute angle.")
    return 0


def main(argv):
    if len(argv) == 1:
        return create()
    if len(argv) == 3 and argv[1] == "--profile":
        return create(profile_name=argv[2])
    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
