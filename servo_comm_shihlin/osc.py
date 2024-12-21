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
from modbus_command_code import CmdCode
from servo_control import ServoController

# Configure logging
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')

# Instantiate GPIOUtils for GPIO operations
gpio_utils = GPIOUtils()

# Configure the serial port
serial_manager = SerialPortManager()


def configure_serial_port():
    try:
        if serial_manager.get_serial_instance():
            logging.info(
                f"Connected port: {serial_manager.get_connected_port()}")
            logging.info(
                f"Current baud rate: {serial_manager.get_baud_rate()}")
            return ServoController(serial_manager)
        else:
            logging.error("Could not configure any serial port. Exiting.")
            exit()
    except Exception as e:
        logging.error(
            f"Exception occurred while configuring the serial port: {e}")
        exit()


servo_ctrller = configure_serial_port()

# Enable the Digital I/O Writable
servo_ctrller.write_PD_16_Enable_DI_Control()


# Initialize global variables for RS485 messages
START, STOP = False, False
RS485_send, RS485_read = "00 00 FF FF", "FF FF 00 00"


def convert_bytes_to_hex(data):
    try:
        return data.hex() if isinstance(data, bytes) else data
    except Exception as e:
        logging.error(f"Error converting bytes to hex: {e}")
    return None


# Duplicate message check
previous_data = None


def check_duplicated(data):
    global previous_data
    try:
        if previous_data != data:
            previous_data = data
            return False
        return True
    except Exception as e:
        logging.error(f"Error checking duplicated data: {e}")
    return True


def servo_handler(unused_addr, args, data):
    try:
        if not check_duplicated(data):
            logging.info(f"Received: {data}")
            if data == 1.0:
                servo_ctrller.servo_on()
                logging.info("Servo turned on.")
            elif data == 0.0:
                servo_ctrller.servo_off()
                logging.info("Servo turned off.")
    except Exception as e:
        logging.error(f"Error in servo_handler: {e}")


def clear_handler(unused_addr, args, clear):
    try:
        if not check_duplicated(clear):
            if clear == 1.0:
                servo_ctrller.clear_alarm_12()
                logging.info("Cleared alarm 12.")
    except Exception as e:
        logging.error(f"Error in clear_handler: {e}")


def set_point_handler(unused_addr, args, angle, acc_time, rpm):
    # if servo_ctrller.check_movement:
    #     logging.info("Movement is still running.")
    #     return
    # else:
    try:
        if not check_duplicated(angle):
            logging.info("Sending command.")
            servo_ctrller.post_step_motion_by(angle, acc_time, rpm)
            logging.info(
                f"Set point to {angle} degrees. Acc time: {acc_time} ms. RPM: {rpm}")
            # servo_ctrller.check_movement = True

    except Exception as e:
        logging.error(f"Error in set_point_handler: {e}")
        # servo_ctrller.check_movement = False

def set_point_handler_2(unused_addr, args, angle, acc_time, rpm):
    try:
        if not check_duplicated(angle):
            logging.info("Sending command.")
            servo_ctrller.post_step_motion_by(angle, acc_time, rpm)
            logging.info(
                f"Set point to {angle} degrees. Acc time: {acc_time} ms. RPM: {rpm}"
            )
    except Exception as e:
        logging.error(f"Error in set_point_handler_2: {e}")


def back_home_handler(unused_addr, args, state):
    try:
        if not check_duplicated(state):
            if state == 1.0:
                servo_ctrller.post_step_motion_by(0, 5000, 10)
                logging.info("Back to home position.")
    except Exception as e:
        logging.error(f"Error in back_home_handler: {e}")

def pr_mode_ctrl_handler(unused_addr, args, path_numb=0):
    try:
        if not check_duplicated(path_numb):
            logging.info("Sending Command. ")
            servo_ctrller.write_PF82(path_numb)
            logging.info(
                f"Use Pr Mode to goto configed postion number: {path_numb}. "
            )
    except Exception as e:
        logging.error(f"Error in pr_mode_ctrl_handler")



def set_home_position_handler(unused_addr, args, state):
    try:
        if not check_duplicated(state):
            if state == 1.0:
                servo_ctrller.set_home_position()
                logging.info("Home position set.")
    except Exception as e:
        logging.error(f"Error in set_home_position_handler: {e}")


def main():
    server = None
    try:
        
        gpio_utils.initialize_gpio()
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", default="0.0.0.0",
                            help="The IP to listen on")
        parser.add_argument("--port_receive", type=int,
                            default=5005, help="The port to listen on")
        parser.add_argument("--port_send", type=int,
                            default=5008, help="The port to send to")
        args = parser.parse_args()

        dispatcher = Dispatcher()
        dispatcher.map("/servo", servo_handler, "data")
        dispatcher.map("/clear", clear_handler, "clear")
        dispatcher.map("/set_point", set_point_handler, "angle", "acc_time", "rpm")
        dispatcher.map("/set_point_2", set_point_handler_2, "angle", "acc_time", "rpm")
        dispatcher.map("/back_home", back_home_handler, "state")
        dispatcher.map("/pr_step_path", pr_mode_ctrl_handler, "path_number")
        dispatcher.map("/set_home", set_home_position_handler, "state")

        server = osc_server.ThreadingOSCUDPServer((args.ip, args.port_receive), dispatcher)
        logging.info(f"Serving on {server.server_address}")

        server.serve_forever()
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received. Exiting.")
        if server:
            logging.info("Shutting down the OSC server.")
            server.shutdown()
        gpio_utils.cleanup_gpio()
        serial_manager.disconnect()
        if servo_ctrller.read_thread:
            servo_ctrller.read_thread_stop_event.set()
            servo_ctrller.read_thread.join()
        logging.info("Cleaning up GPIO and disconnecting serial port.")
        logging.info("Exiting and done.")
    except Exception as e:
        logging.error(f"Exception in main: {e}")
        if server:
            server.shutdown()
        gpio_utils.cleanup_gpio()
        serial_manager.disconnect()
        if servo_ctrller.read_thread:
            servo_ctrller.read_thread_stop_event.set()
            servo_ctrller.read_thread.join()
    finally:
        if server:
            logging.info("Closing the OSC server.")
            server.server_close()
        exit()


if __name__ == "__main__":
    main()
