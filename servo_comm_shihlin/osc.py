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

client = udp_client.SimpleUDPClient("0.0.0.0", 5008)

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


# print RS485 read/send messages
def print_RS485():
  client.send_message("/RS485_send", "RS485_send")
  client.send_message("/RS485_read", "RS485_read")

# catch any address or message to print RS485
def default_handler(unused_addr, args):
  client.send_message("/RS485_send", "RS485_send")
  client.send_message("/RS485_read", "RS485_read")


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

  print_RS485()

def enable_handler(unused_addr, args, enable):
  if not check_duplicated(enable):
      if enable == 1.0:
        servo_ctrller.Enable_Position_Mode(True)
        time.sleep(0.05)
        servo_ctrller.config_acc_dec_0x0902(500)
        time.sleep(0.05)
        servo_ctrller.config_speed_0x0903(600)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0905_low_byte(0x0001)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0906_high_byte(0x0780)
        time.sleep(0.05)
        servo_ctrller.start_continuous_reading(0x0900, 0.1)
        time.sleep(0.1)

  print_RS485()


def oneDirection_handler(unused_addr, args, direction):
  if not check_duplicated(direction):
    if direction == 1.0:
        servo_ctrller.pos_step_motion_test(CW=False)
    elif direction == 0.0:
        servo_ctrller.pos_step_motion_test(CW=True)

  print_RS485()


if __name__ == "__main__":

    # try:
    #     # Initialize GPIO
    #     gpio_utils.initialize_gpio()
    # except Exception as e:
    #     print(f"Error initializing GPIO: {e}")
    #     exit()
    # finally:
    #     gpio_utils.cleanup_gpio()

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip",
                        default="0.0.0.0", help="The ip to listen on")
    parser.add_argument("--port_recieve",
                        type=int, default=5005, help="The port to listen on")
    parser.add_argument("--port_send",
                        type=int, default=5008, help="The port to send to")

    args = parser.parse_args()

    client = udp_client.SimpleUDPClient(args.ip, args.port_send)

    dispatcher = Dispatcher()
    dispatcher.map("/print", print)
    dispatcher.map("/clear", clear_handler, "clear")
    dispatcher.map("/enable", enable_handler, "enable")
    dispatcher.map("/oneDirection", oneDirection_handler, "direction")
    server = osc_server.ThreadingOSCUDPServer(
        (args.ip, args.port_recieve), dispatcher)
    print("Serving on {}".format(server.server_address))
    server.serve_forever()
