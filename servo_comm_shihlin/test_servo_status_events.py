import time
from servo_status_events import StatusMonitor

def di_bit_change_callback(bit_position, new_value):
     print(f"DI Bit {bit_position} changed to {new_value}")

def do_bit_change_callback(bit_position, new_value):
    print(f"DO Bit {bit_position} changed to {new_value}")

monitor = StatusMonitor()

# Register interest in specific DI and DO bits
monitor.register_di_monitor(3, di_bit_change_callback)  # Monitor DI bit 3
monitor.register_do_monitor(2, do_bit_change_callback)  # Monitor DO bit 2

# Simulate an update in status
monitor.update_di_status(0b00001000)  # Expected to trigger callback for DI bit 3
monitor.update_do_status(0b00000100)  # Expected to trigger callback for DO bit 2


