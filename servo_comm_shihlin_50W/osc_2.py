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

TOUCHDESIGNER_IP = "10.12.1.164"
TOUCHDESIGNER_PORT = 5008
osc_client = udp_client.SimpleUDPClient(TOUCHDESIGNER_IP, TOUCHDESIGNER_PORT)
previous_data = None # for duplicate message check

def configure_serial_port():
    try:
        if serial_manager.get_serial_instance():
            logging.info(f"Connected port: {serial_manager.get_connected_port()}")
            logging.info(f"Current baudrate: {serial_manager.get_baud_rate()}")
            return ServoController(serial_manager)
        else:
            logging.error("Could not configure any serial port. Exiting...")
            exit(1)
    except Exception as e:
        logging.error(f"Error in configuring serial port: {e}")
        exit(1)

# Instantiate ServoController
servo_ctrller = configure_serial_port()

# Enable the Digital I/O Writable
servo_ctrller.write_PD_16_Enable_DI_Control()

def send_to_touchdesigner(address, *args):
    try:
        osc_client.send_message(address, args)
        logging.info(f"Send to TouchDesigner: {address} {args}")
    except Exception as e:
        logging.error(f"Error sending message to TouchDesigner: {e}")

def check_duplicated(data):
    global previous_data
    if data != previous_data:
        previous_data = data
        return False
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

def clear_handler(unused_addr, *args):
    try:
        servo_ctrller.clear_alarm_12()
        send_to_touchdesigner("/clear", "cleared")
        logging.info("Clear Alarm 12.")
    except Exception as e:
        logging.error(f"Error in clear_handler: {e}")

def set_countinuous_motion_handler(unused_addr, args, speed_rpm, acc_time, enable= True):
    try:
        speed_rpm = int(speed_rpm)
        acc_time = int(acc_time)
        servo_ctrller.enable_speed_ctrl(speed_rpm, acc_time, enable)
        send_to_touchdesigner("/continuous_mode_start", speed_rpm, acc_time)
        logging.info(f"Continuous motion started with speed: {speed_rpm} RPM, Acc time: {acc_time} ms")
    except Exception as e:
        logging.error(f"Error in set_countinuous_motion_handler: {e}")

def ctrl_continuous_motion_handler(unused_addr, args, action, CW_CCW):
    try:
        if action == "stop":
            servo_ctrller.speed_ctrl_action(0)
            send_to_touchdesigner("/continuous_mode_stop", "stop")
            logging.info("Continuous motion stopped.")
        elif action == "start":
            if CW_CCW == "CW":
                servo_ctrller.speed_ctrl_action(1)
            elif CW_CCW == "CCW":
                servo_ctrller.speed_ctrl_action(2)
            servo_ctrller.speed_ctrl_action("CW")
            send_to_touchdesigner("/continuous_mode_start", CW_CCW)
            logging.info(f"Continuous motion started in {CW_CCW} direction.")
    except Exception as e:
        logging.error(f"Error in ctrl_continuous_motion_handler: {e}")


def set_point_handler(unused_addr, args, angle, acc_time, rpm):
    try:
        angle = float(angle)
        acc_time = int(acc_time)
        rpm = int(rpm)

        if not check_duplicated(angle):
            servo_ctrller.post_step_motion_by(angle, acc_time, rpm)
            send_to_touchdesigner("/set_point", angle, acc_time, rpm)
            logging.info(f"Set point to {angle} degrees. Acc time: {acc_time} RPM: {rpm}")
    except Exception as e:
        logging.error(f"Error in set_point_handler: {e}")

def reset_initial_abs_position_handler(unused_addr, *args):
    try:
        servo_ctrller.write_PA29_Initial_Abs_Pos()
        send_to_touchdesigner("/reset_initial_abs_position", "reset")
        logging.info("Reset initial absolute position.")
    except Exception as e:
        logging.error(f"Error in reset_initial_abs_position_handler: {e}")

def set_initial_abs_position_handler(unused_addr, *args):
    try:
        servo_ctrller.set_initial_abs_position()
        send_to_touchdesigner("/set_initial_abs_position", "set_initial_abs_position")
        logging.info("Set initial absolute position.")
    except Exception as e:
        logging.error(f"Error in set_initial_abs_position_handler: {e}")

def back_home_handler(unused_addr, *args):
    try:
        servo_ctrller.initial_abs_home()
        send_to_touchdesigner("/back_home", "back_home")
        logging.info("Back to home position.")
    except Exception as e:
        logging.error(f"Error in back_home_handler: {e}")

def set_home_position_handler(unused_addr, *args):
    try:
        servo_ctrller.set_home_position()
        send_to_touchdesigner("/set_home_position", "set_home_position")
        logging.info("Set home position.")
    except Exception as e:
        logging.error(f"Error in set_home_position_handler: {e}")


def cancel_loop_handler(unused_addr, *args):
    try:
        servo_ctrller.cancel_continuous_reading()
        send_to_touchdesigner("/cancel_loop", servo_ctrller.current_angle)
        logging.info(f"Cancel loop. Current angle: {servo_ctrller.current_angle}")
    except Exception as e:
        logging.error(f"Error in cancel_loop_handler: {e}")

def on_motion_completed():
    try:
        send_to_touchdesigner("/motion_complete", "complete")
        logging.info("Motion complete.")
    except Exception as e:
        logging.error(f"Error in on_motion_complete: {e}")

def on_moving(diff_angle):
    try:
        send_to_touchdesigner("/moving", diff_angle)
        #logging.info(f"Moving. Diff Angle: {diff_angle}")
    except Exception as e:
        logging.error(f"Error in on_moving: {e}")

def main():
    server = None
    try:
        gpio_utils.initialize_gpio()
        parser = argparse.ArgumentParser()
        parser.add_argument("--ip", default="10.12.1.107", help="The ip to listen on")
        parser.add_argument("--port_receive", type=int, default=5005, help="The port to listen on")
        args = parser.parse_args()

        servo_ctrller.register_event_listener("on_motion_completed", on_motion_completed)
        servo_ctrller.register_event_listener("on_moving", on_moving)

        dispatcher = Dispatcher()
        dispatcher.map("/servo", servo_handler, "data")
        dispatcher.map("/clear", clear_handler, "clear")
        dispatcher.map("/set_point", set_point_handler, "angle", "acc_time", "rpm")
        dispatcher.map("/back_home", back_home_handler)
        dispatcher.map("/set_home", set_home_position_handler)
        dispatcher.map("/reset_initial_abs_position", reset_initial_abs_position_handler)
        dispatcher.map("/set_continous_motion", set_countinuous_motion_handler, "speed_rpm", "acc_time", "enable")
        dispatcher.map("/ctrl_continuous_motion", ctrl_continuous_motion_handler, "action", "CW_CCW")
        dispatcher.map("/cancel_loop", cancel_loop_handler)

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