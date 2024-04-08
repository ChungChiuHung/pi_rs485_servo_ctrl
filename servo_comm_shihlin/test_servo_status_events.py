import time
from servo_status_events import ServoDIStatusMonitor

def di_bit_change_listener(bit_name, status):
     print(f"Change detected in {bit_name}. New status: {'ON' if status else 'OFF'}")

def simulate_servo_di_status_monitor():
     di_monitor = ServoDIStatusMonitor()

     di_monitor.add_bit_changed_listener('DI1', di_bit_change_listener)
     di_monitor.add_bit_changed_listener('DI2', di_bit_change_listener)

     print("Starting simulation of DI status changes...")
     for _ in range(5):
          di_monitor.update_di_status(0b00000001)
          time.sleep(1)
          di_monitor.update_di_status(0b00000000)
          time.sleep(1)

if __name__ == "__main__":
     simulate_servo_di_status_monitor()
