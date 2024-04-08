from status_registers import ServoStatus

class ServoDIStatusMonitor:
    def __init__(self):
        self.di_status_byte = None
        self.bit_change_listeners = {bit: [] for bit in ServoStatus.decode_di_status(0)}

    def add_bit_changed_listener(self, bit_name, callback):
        if bit_name in self.bit_change_listeners:
            self.bit_change_listeners[bit_name].append(callback)
        else:
            print(f"Bit name {bit_name} does not exist.")
    
    def notify_bit_change(self, bit_name, new_status):
        for callback in self.bit_change_listeners.get(bit_name, []):
            callback(bit_name, new_status)

    def update_di_status(self, new_di_status_byte):
        old_status = self.di_status_byte
        new_status = ServoStatus.decode_di_status(new_di_status_byte)

        for bit_name, bit_info in new_status.items():
            if old_status is not None and bit_info['status'] != ServoStatus.decode_di_status(old_status).get(bit_name, {}).get('status'):
                self.notify_bit_change(bit_name, bit_info['status'])

        self.di_status_byte = new_di_status_byte
