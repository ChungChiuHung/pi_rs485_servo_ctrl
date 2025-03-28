class StatusMonitor:
    def __init__(self):
        self.di_watchlist = {} # Format: {bit_position: callback_function, ...}
        self.do_watchlist = {}
        self.previous_di_status = 0
        self.previous_do_status = 0

    def register_di_monitor(self, bit_position, callback):
        self.di_watchlist[bit_position] = callback

    def register_do_monitor(self, bit_position, callback):
        self.do_watchlist[bit_position] = callback

    def update_di_status(self, di_status_byte):
        for bit_position, callback in self.di_watchlist.items():
            if self._has_bit_changed(self.previous_di_status, di_status_byte, bit_position):
                callback(bit_position, self._get_bit_value(di_status_byte, bit_position))
        self.previous_di_status = di_status_byte

    def update_do_status(self, do_status_byte):
        for bit_position, callback in self.do_watchlist.items():
            if self._has_bit_changed(self.previous_do_status, do_status_byte, bit_position):
                callback(bit_position, self._get_bit_value(do_status_byte, bit_position))
        self.previous_do_status = do_status_byte
    
    @staticmethod
    def _has_bit_changed(previous_status, current_status, bit_position):
        previous_bit = (previous_status >> bit_position) & 1
        current_bit = (current_status >> bit_position) & 1
        return previous_bit != current_bit
    
    @staticmethod
    def _get_bit_value(status_byte, bit_position):
        return (status_byte >> bit_position) & 1
    
