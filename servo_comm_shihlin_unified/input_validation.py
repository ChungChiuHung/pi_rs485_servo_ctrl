"""
Pure request-payload validation, split out from app.py so it's unit-testable
without importing app.py (which opens a real serial connection at module
load time -- see hardware_lock.py's docstring for the same reasoning).
"""


def validate_int_range(value, min_value, max_value, field_name):
    """Returns (int_value, None) on success, or (None, error_message) if
    `value` isn't a whole number within [min_value, max_value]."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None, f"'{field_name}' must be a number."
    if isinstance(value, float) and not value.is_integer():
        return None, f"'{field_name}' must be a whole number."
    int_value = int(value)
    if not (min_value <= int_value <= max_value):
        return None, f"'{field_name}' must be between {min_value} and {max_value}."
    return int_value, None
