"""
Loads motor_profiles.json and resolves one named profile into the concrete
values ServoController needs (baud_rate, gear_ratio, a default abs_home_pos,
and base_pulse_per_degree computed -- never hardcoded -- from
encoder_pulses_per_rev * gear_ratio / 360).

See docs/servo_comm_shihlin_merge_design.md §3 for the schema rationale:
encoder_pulses_per_rev is a fixed property of the encoder/driver shared by
both motor variants, so it lives once at the top level; gear_ratio and
abs_home_pos differ per motor and live under "profiles".
"""
import json

MOTOR_PROFILES_FILE = "motor_profiles.json"


class ProfileNotFoundError(KeyError):
    pass


def load_profiles(path: str = MOTOR_PROFILES_FILE) -> dict:
    with open(path, "r") as f:
        return json.load(f)


def resolve_profile(profiles: dict, name: str) -> dict:
    """Merge the named profile with the shared top-level fields and compute
    base_pulse_per_degree. Raises ProfileNotFoundError for an unknown name."""
    try:
        profile = profiles["profiles"][name]
    except KeyError:
        raise ProfileNotFoundError(f"Unknown motor profile: {name!r}") from None

    encoder_pulses_per_rev = profiles["encoder_pulses_per_rev"]
    gear_ratio = profile["gear_ratio"]

    return {
        "name": name,
        "baud_rate": profile["baud_rate"],
        "gear_ratio": gear_ratio,
        "abs_home_pos_default": profile["abs_home_pos"],
        "encoder_pulses_per_rev": encoder_pulses_per_rev,
        "base_pulse_per_degree": encoder_pulses_per_rev * gear_ratio / 360,
        "modbus_device_number": profiles.get("modbus_device_number", 1),
    }


def load_active_profile(path: str = MOTOR_PROFILES_FILE) -> dict:
    profiles = load_profiles(path)
    return resolve_profile(profiles, profiles["active_profile"])
