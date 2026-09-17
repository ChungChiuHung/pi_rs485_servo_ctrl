import os
import time
import logging
from datetime import datetime, timezone
from flask import Flask, render_template, request, jsonify, Response
from functools import wraps
import traceback
from threading import Thread, Event
from gpio_utils import GPIOUtils
from serial_port_manager import SerialPortManager
from modbus_command_code import CmdCode
from servo_control import ServoController

app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'your keys')

# Dedicated logger for the /alarm/clear endpoint (CLAUDE.md hardware-safety
# section: every call that can write live state to the motor must be logged).
alarm_logger = logging.getLogger("alarm_clear")

# Instantiate GPIOUtils for GPIO opeartions
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

SET_POINT_1 = 0
SET_POINT_2 = 0
SET_POINT_3 = 0
SET_HOME = 0

def convert_bytes_to_hex(data):
      return data.hex() if isinstance(data, bytes) else data

def json_response(f):
      @wraps(f)
      def decorated_function(*args, **kwargs):
            try:
                  result = f(*args, **kwargs)
                  if isinstance(result, Response):
                        return result
                  return jsonify(Response)
            except Exception as e:
                  traceback.print_exc()
                  return jsonify({"error": "An error occurred", "details":str(e)}),500
      return decorated_function


@app.route('/')
def home():
    return render_template('home.html')
    return render_template('home.html')



@app.route('/index')
def index():
    return render_template('index.html', title='Servo Control Panel', RS485_read=RS485_read, RS485_send=RS485_send)
    return render_template('index.html', title='Servo Control Panel', RS485_read=RS485_read, RS485_send=RS485_send)



@app.route('/alarm/clear', methods=['POST'])
def clear_alarm_12_endpoint():
    """Clear Alarm 12 (AL.12, Emergency stop) only. Not a general-purpose
    Modbus write endpoint -- see README for the safety precondition this
    requires before calling it.
    """
    caller_ip = request.remote_addr
    started_at = datetime.now(timezone.utc).isoformat()

    payload = request.get_json(silent=True) or {}
    if payload.get('confirm') is not True:
        alarm_logger.warning(
            "Rejected /alarm/clear from %s at %s: missing confirm=true.",
            caller_ip, started_at
        )
        return jsonify({
            "status": "error",
            "message": 'Missing or false "confirm" field. POST {"confirm": true} to proceed.'
        }), 400

    before_code = servo_ctrller.read_current_alarm_code()
    alarm_logger.info(
        "Alarm-12 clear requested by %s at %s. Alarm code before: %s",
        caller_ip, started_at, before_code
    )

    if before_code is None:
        alarm_logger.error(
            "Could not read alarm status before clearing (comm failure) -- "
            "aborting, no clear command was sent."
        )
        return jsonify({
            "status": "error",
            "message": "Could not read current alarm status (communication failure). No clear command was sent.",
            "before_alarm_code": None,
            "caller_ip": caller_ip,
            "timestamp": started_at,
        }), 503

    # Primary mechanism: existing clear_alarm_12(). NOTE -- this switches the
    # drive's DI control source to communication mode (PD16) and writes the
    # virtual EMG DI bit to its "released" state. If a physical E-Stop
    # circuit is still engaged, this can make the drive treat EMG as
    # released without the physical condition actually being resolved.
    # See README "Alarm 12 clear -- safety precondition" before using this.
    servo_ctrller.write_PD_16_Enable_DI_Control()
    servo_ctrller.clear_alarm_12()
    time.sleep(0.1)
    after_code = servo_ctrller.read_current_alarm_code()
    mechanism_used = "clear_alarm_12"

    if after_code != 0:
        # Fallback: official 0x0130 "Alarm clearance" register (write
        # 0x1EA5). Does not touch DI control source / virtual EMG state.
        servo_ctrller.clear_alarm_via_register()
        time.sleep(0.1)
        after_code = servo_ctrller.read_current_alarm_code()
        mechanism_used = "clear_alarm_12+0x0130_fallback"

    success = after_code == 0
    alarm_logger.info(
        "Alarm-12 clear result for %s: before=%s after=%s success=%s mechanism=%s",
        caller_ip, before_code, after_code, success, mechanism_used
    )

    return jsonify({
        "status": "success" if success else "failed",
        "before_alarm_code": before_code,
        "after_alarm_code": after_code,
        "mechanism_used": mechanism_used,
        "caller_ip": caller_ip,
        "timestamp": started_at,
    }), (200 if success else 502)


@app.route('/action', methods=['POST'])
@json_response
def handle_action():
    global START, STOP, RS485_send, RS485_read

    data = request.json
    action = data.get('action')
    response = {"status": "success", "action": action}

    print(f"Received action: {action}")

    # Enable the Digital I/O Writable
    servo_ctrller.write_PD_16_Enable_DI_Control()

    # Perform the raspi action here based on action type
    if action == "start":
        print("start")
    elif action == "stop":
        print("stop")
    elif action == "servoOn":
        # SET_PARAM_2 command
        servo_ctrller.clear_alarm_12()
        time.sleep(0.1)
        servo_ctrller.servo_on()
        time.sleep(0.1)

    elif action == "servoOff":

        servo_ctrller.servo_off()

    elif action == "getMsg":

        servo_ctrller.Read_Pos_Related_Paremters()

    elif action == "clearAlarm12":
        servo_ctrller.clear_alarm_12()

    elif action == "enablePosMode":
        servo_ctrller.Enable_Position_Mode(True)
        time.sleep(0.05)
        servo_ctrller.config_acc_dec_0x0902(0)
        time.sleep(0.05)
        servo_ctrller.config_speed_0x0903(10)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0905_low_byte(0x0000)
        time.sleep(0.05)
        servo_ctrller.config_pulses_0x0906_high_byte(0x0780)
        time.sleep(0.05)
        servo_ctrller.start_continuous_reading(0x0900, 0.1)

    elif action == "posTestStart_CW":
        servo_ctrller.pos_step_motion_test(CW=True)
    elif action == "posTestStart_CW":
        servo_ctrller.pos_step_motion_test(CW=True)

    elif action == "posTestStart_CCW":
        servo_ctrller.pos_step_motion_test(CW=False)

    elif action == "setPoint_1":

        print("set point")
        servo_ctrller.post_step_motion_by(90)

    elif action == "setPoint_2":

        print("set point")
        servo_ctrller.post_step_motion_by(180)

    elif action == "Home":
           
        print("HOME")
        servo_ctrller.post_step_motion_by(0)

    elif action == "enableSpeedCtrlMode":
        servo_ctrller.enable_speed_ctrl(100)

    elif action == "motionStart_CW":
        servo_ctrller.speed_ctrl_action(1)
      
    elif action == "motionStart_CCW":
        servo_ctrller.speed_ctrl_action(2)

    elif action == "motionPause":

        print("motion pause")
        servo_ctrller.speed_ctrl_action(0)

    elif action == "motionCancel":
            
        print("motion cancel")
        servo_ctrller.stop_continuous_reading()
        servo_ctrller.Enable_Position_Mode(False)
    else:
        response['error'] = "Action not recognized."

    return jsonify({
        "status": "success",
        "action": action,
        "RS485_send": RS485_send,
        "RS485_read": RS485_read,
        "message":f"Action {action} completed successfully."  
      })

if __name__ == "__main__":
   gpio_utils.initialize_gpio()
   try:
        app.run(host='0.0.0.0', port=5000, debug = True)
   finally:
        gpio_utils.cleanup_gpio()
