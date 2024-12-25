import os
import time
import argparse
import logging
from functools import wraps
from threading import Thread, Event
from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server, udp_client
from gpio_utils import GPIOUtils
from serial_port_manager import SerialPortManager
from servo_control import ServoController

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Instantiate GPIOUtils for GPIO opeartions
gpio_utils = GPIOUtils()

# Configure the serial port
serial_manager = SerialPortManager()

def configure_serial_port():
    try:
        if serial_manager.get_serial_instance():
            logging.info(f"Connected port: {serial_manager.get_connected_port()}")
            logging.info(f"Current baudrate: {serial_manager.get_baud_rate()}")
            return ServoController(serial_manager)
        else:
            logging.error("Could not configure any serial port. Exiting...")
            exit()
    except Exception as e:
        logging.error(f"Error in configuring serial port: {e}")
        exit()

servo_ctrller = configure_serial_port()

# Enable the Digital I/O Writable
servo_ctrller.write_PD_16_Enable_DI_Control()

# Initialize global variables for RS485 messages
START, STOP = False, False
RS485_send, RS485_read = "00 00 FF FF", "FF FF 00 00"

# Duplicate message check
previous_data = None

# Setup OSC Client
TOUCHDESIGNER_IP = "10.12.1.164"
TOUCHDESIGNER_PORT = 5008
osc_client = udp_client.SimpleUDPClient(TOUCHDESIGNER_IP, TOUCHDESIGNER_PORT)

def send_to_touchdesigner(address, *args):
    try:
        osc_client.send_message(address, args)
        logging.info(f"Send to TouchDesigner: {address} {args}")
    except Exception as e:
        logging.error(f"Error sending message to TouchDesigner: {e}")

def check_duplicated(data):
    global previous_data
    try:
        if previous_data != data:
            previous_data = data
            return False
        return True
    except Exception as e:
        logging.error(f"Error in checking duplicated data: {e}")
    return True

# Handlers
def servo_handler(unused_addr, args, data):
    try:
        if not check_duplicated(data):
            logging.info(f"Received: {data}")
            if data == 1.0:
                servo_ctrller.servo_on()
                send_to_touchdesigner("/servo_on", "on")
                logging.info("Servo turned on.")
            elif data == 0.0:
                servo_ctrller.servo_off()
                send_to_touchdesigner("/servo_off", "off")
                logging.info("Servo turned off.")
    except Exception as e:
        logging.error(f"Error in servo_handler: {e}")

def clear_handler(unused_addr, args, clear):
    try:
        if not check_duplicated(clear):
            if clear == 1.0:
                servo_ctrller.clear_alarm_12()
                send_to_touchdesigner("/clear", "cleared")
                logging.info("Clear Alarm 12.")
    except Exception as e:
        logging.error(f"Error in clear_handler: {e}")

def set_point_handler(unused_addr, args, angle, acc_time, rpm):
    try:
        if not check_duplicated(angle):
            servo_ctrller.post_step_motion_by(angle, acc_time, rpm)
            send_to_touchdesigner("/set_point", angle, acc_time, rpm)
            logging.info(f"Set point to {angle} degrees. Acc time: {acc_time} RPM: {rpm}")
    except Exception as e:
        logging.error(f"Error in set_point_handler: {e}")

def back_home_handler(unused_addr, args, state):
    try:
        if not check_duplicated(state):
            if state == 1.0:
                servo_ctrller.initial_abs_home()
                send_to_touchdesigner("/back_home", "back_home")
                logging.info("Back to home position.")
    except Exception as e:
        logging.error(f"Error in back_home_handler: {e}")

def set_home_position_handler(unused_addr, args, state):
    try:
        if not check_duplicated(state):
            if state == 1.0:
                servo_ctrller.set_home_position()
                send_to_touchdesigner("/set_home_position", "set_home_position")
                logging.info("Set home position.")
    except Exception as e:
        logging.error(f"Error in set_home_position_handler: {e}")

def main():
    server = None
    try:
        gpio_utils.initialize_gpio()
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", default="10.12.1.107", help="The ip to listen on")
        parser.add_argument("--port_receive", type=int, default=5005, help="The port to listen on")
        args = parser.parse_args()

        dispatcher = Dispatcher()
        dispatcher.map("/servo", servo_handler, "data")
        dispatcher.map("/clear", clear_handler, "clear")
        dispatcher.map("/set_point", set_point_handler, "angle", "acc_time", "rpm")
        dispatcher.map("/back_home", back_home_handler, "state")
        dispatcher.map("/set_home", set_home_position_handler, "state")

        server = osc_server.ThreadingOSCUDPServer((args.ip, args.port_receive), dispatcher)
        logging.info(f"Serving on {server.server_address}")

        server.serve_forever()

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received. Exiting...")
    except Exception as e:
        logging.error(f"Error in main: {e}")
    finally:
        if server:
            server.shutdown()
            logging.info("OSC server shutdown.")
        gpio_utils.cleanup_gpio()
        serial_manager.disconnect()
        logging.info("Cleanup completed. Exiting.")


if __name__ == "__main__":
    main()