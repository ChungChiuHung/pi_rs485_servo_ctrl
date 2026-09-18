import RPi.GPIO as GPIO

class GPIOUtils:
    def __init__(self):
        self.pins_config = {
            "RS485_ENABLE": {"pin_number": 4, "mode": GPIO.OUT, "initial": GPIO.HIGH},
            "LED_RED": {"pin_number": 13, "mode": GPIO.OUT, "initial": GPIO.LOW},
            "LED_YLW": {"pin_number": 19, "mode": GPIO.OUT, "initial": GPIO.LOW},
            "LED_GRN": {"pin_number": 26, "mode": GPIO.OUT, "initial": GPIO.LOW},
            }
        self.initialize_gpio()
    
    def initialize_gpio(self):
        GPIO.setmode(GPIO.BCM)  # Use Broadcom pin numbering
        GPIO.setwarnings(False)
        for pin_name, config in self.pins_config.items():
            GPIO.setup(config["pin_number"], config["mode"], initial=config["initial"])


    def cleanup_gpio(self):
        GPIO.cleanup()
    
    def set_led_state(self, led_name, state):
        pin_config = self.pins_config.get(led_name)
        if pin_config:
            GPIO.output(pin_config["pin_number"], GPIO.HIGH if state else GPIO.LOW)
        else:
            print(f"LED name '{led_name}' is not recognized.")