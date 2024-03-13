class CommunicationAnalyzer:
    def __init__(self):
        """Initialize the Communication Analyzer class."""
        pass


    def calculate_command_response_time(self, command_length_bits, transmission_speed_bps)
        """
        Calculate the command transmission time

        Parameters:
        command_length_bits (int): The length of the command in bits.
        transmission_speed_bps (int): The transmission speed in bits per second.

        Returns:
        float: The command transmission time in milliseconds.
        """

        # Calculdate the transmission time in seconds
        transmission_time_seconds = command_length_bits / transmission_speed_bps

        # Convert the transmission time to milliseconds
        transmission_time_milliseconds = transmission_time_seconds * 1000

        return transmission_time_milliseconds


    def get_bit_length(self, byte_array)
        """
        Get the bit length of a byte array

        Parameters:
        byte_array (bytes): The byte array for which to calculate the bit length.

        Returns:
        int: The bit length of the byte array

        """
        return len(byte_array) * 8

    
