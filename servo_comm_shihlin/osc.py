import os
import time
import argparse
from functools import wraps
import traceback
from threading import Thread, Event
from gpio_utils import GPIOUtils
from serial_port_manager import SerialPortManager
from modbus_command_code import CmdCode
from servo_control import ServoController

from pythonosc.dispatcher import Dispatcher
from pythonosc import osc_server
from pythonosc import udp_client


# # Instantiate GPIOUtils for GPIO opeartions
gpio_utils = GPIOUtils()

# Configure the serial port
serial_manager = SerialPortManager()
if serial_manager.get_serial_instance():
    print(f"Connected port: {serial_manager.get_connected_port()}")
    print(f"Current baud rate: {serial_manager.get_baud_rate()}")
    servo_ctrller = ServoController(serial_manager)
else:
    print("Could not configure any serial port. Exiting.")
    exit()

# Initailize global variables for RS485 messages
START, STOP = False, False
RS485_send, RS485_read = "00 00 FF FF", "FF FF 00 00"


def convert_bytes_to_hex(data):
    return data.hex() if isinstance(data, bytes) else data


# duplicate msg check
previous_data = None


def check_duplicated(data):
    global previous_data
    if previous_data != data:
        # print("Not duplicated")
        previous_data = data  # Update previous_data
        return False
    else:
        # print("Duplicated")
        return True


def clear_handler(unused_addr, args, clear):
    if not check_duplicated(clear):
        if clear == 1.0:
            servo_ctrller.clear_alarm_12()


def set_point_handler(unused_addr, args, angle, acc_time, rpm):
    if not check_duplicated(angle):
        servo_ctrller.post_step_motion_by(angle, acc_time, rpm)


def back_home_handler(unused_addr, args, state):
    if not check_duplicated(state):
        if state == 1.0:
            servo_ctrller.post_step_motion_by(0, 5000, 10)


def set_home_position_handler(unused_addr, args, state):
    if not check_duplicated(state):
        if state == 1.0:
            servo_ctrller.set_home_position()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip",
                        default="0.0.0.0", help="The ip to listen on")
    parser.add_argument("--port_recieve",
                        type=int, default=5005, help="The port to listen on")
    parser.add_argument("--port_send",
                        type=int, default=5008, help="The port to send to")

    args = parser.parse_args()

    dispatcher = Dispatcher()
    dispatcher.map("/clear", clear_handler, "clear")
    dispatcher.map("/set_point", set_point_handler, "angle", "acc_time", "rpm")
    dispatcher.map("/back_home", back_home_handler, "state")
    dispatcher.map("/set_home", set_home_position_handler, "state")

    server = osc_server.ThreadingOSCUDPServer(
        (args.ip, args.port_recieve), dispatcher)
    print("Serving on {}".format(server.server_address))
    server.serve_forever()
