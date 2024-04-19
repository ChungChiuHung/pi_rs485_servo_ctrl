import os
import time
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

@app.route('/index')
def index():
      return render_template('index.html', title='Servo Control Panel', RS485_read=RS485_read, RS485_send=RS485_send)

@app.route('/action', methods=['POST'])
@json_response
def handle_action():
      global START, STOP, RS485_send, RS485_read

      data = request.json
      action = data.get('action')
      response = {"status":"success","action":action}

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
            servo_ctrller.config_acc_dec_0x0902(500)
            time.sleep(0.05)
            servo_ctrller.config_speed_0x0903(600)
            time.sleep(0.05)
            servo_ctrller.config_pulses_0x0905_low_byte(0x0001)
            time.sleep(0.05)
            servo_ctrller.config_pulses_0x0906_high_byte(0x0780)
            time.sleep(0.05)
            #servo_ctrller.start_continuous_reading(0x0204, 2)
            #time.sleep(0.1)
            
            

      elif action == "posTestStart_CW":
            servo_ctrller.pos_step_motion_test(CW=True)

      elif action == "posTestStart_CCW":
            servo_ctrller.pos_step_motion_test(CW=False)

      elif action == "setPoint_1":

            print("set point")

      elif action == "setPoint_2":
            
            print("set point")

      elif action == "Home":
           
           print("HOME")
            
      elif action == "motionStart":
            print("START")
      

      elif action == "motionPause":

           print("motion pause")

      elif action == "motionCancel":
            
            print("motion cancel")
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
