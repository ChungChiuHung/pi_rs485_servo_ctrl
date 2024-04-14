def combine_bytes_to_hex(a, b, c, d):
    """ Combines four byte-values into a single hexadecimal value 0xabcd. """
    # Ensure all inputs are within the hexadecimal digit range
    if any(not (0 <= x <= 15) for x in [a, b, c, d]):
        raise ValueError("All inputs must be within the range 0 to 15 (inclusive).")

    # Combine inputs into a single hexadecimal value
    result = (a << 12) | (b << 8) | (c << 4) | d
    return result

# Example usage:
a, b, c, d = 0, 1, 1, 1
combined_hex = combine_bytes_to_hex(a, b, c, d)
print(f"Combined Hex: {combined_hex:04X}")  # Output should be ABCD