import os
from flask import Flask, render_template, request, session, jsonify
from gpio_utils import GPIOUtils
from serial_port_config import SerialPortConfig

from servo_params import ServoParams
from base_msg_generator import BaseMsgGenerator, MessageCommander
from response_parsing import ResponseMsgParser
from command_code import CmdCode
from set_servo_io_status import SetServoIOStatus
from set_servo_io_status import BitMap
from io_status_fetcher import IOStatusFetcher
from serial_communication import SerialCommunication

app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'JOJO0912956011')


# Instantiate GPIOUtils for GPIO opeartions
gpio_utils = GPIOUtils()

# Configure the serial port
serial_config = SerialPortConfig(baud_rate=57600, timeout=2)
serial_port = serial_config.configure_serial_port()

if not serial_port:
      print("Could not configure any serial port. Exiting.")
      exit()

# Initailize global variables for RS485 messages
RS485_send = "00 00 FF FF"
RS485_read = "FF FF 00 00"

START = False
STOP = False
SERVO_ON = False
SERVO_OFF = False
GET_MSG = False
GET_IO_OUTPUT = False
SET_POINT_1 = False
SET_POINT_2 = False
SET_POINT_HOME = False
MOTION_START = False
MOTION_PAUSE = False

cmd_generator = BaseMsgGenerator()
setter = SetServoIOStatus()
parser = ResponseMsgParser()

# Config the waiting reponse timeout
delay_time_before_read_ms = 50
timeout = 1 # Timeout in second
      
pause_toggle_bit = True
protocol_id=1
destination_address = 1
dir_bit = 0
error_code = 0

@app.route('/')
def home():
      return render_template('home.html')

@app.route('/index')
def index():
      templateData = {
        'title': 'Servo Control Panel',
        'START':START,
        'STOP':STOP,
        'RS485_read': RS485_send,
        'RS485_send': RS485_read,
        'SERVO_ON' : SERVO_ON,
        'SERVO_OFF' : SERVO_OFF,
        'GET_MSG' : GET_MSG,
        'GET_IO_OUTPUT' : GET_IO_OUTPUT,
        'SET_POINT_1' : SET_POINT_1,
        'SET_POINT_2' : SET_POINT_2,
        'SET_POINT_HOME' : SET_POINT_HOME,
        'MOTION_START' : MOTION_START,
        'MOTION_PAUSE' : MOTION_PAUSE,
      }
      return render_template('index.html', **templateData) 

@app.route('/action', methods=['POST'])
def handle_action():
      global SERVO_ON, SERVO_OFF, GET_MSG, GET_IO_OUTPUT, SET_POINT_1, SET_POINT_2, SET_POINT_HOME, MOTION_START, MOTION_PAUSE
      global START, STOP

      data = request.json
      action = data.get('action')
      response = {"status":"success","action":action}

      print(f"Received action: {action}")

      # Perform the raspi action here based on action type

      if action == "start":
            START = True
            STOP = False
            response['message']=f"START:{START}/STOP:{STOP} successfully."
      elif action == "stop":
            STOP = True
            START = False
            response['message']=f"START:{START}/STOP:{STOP} successfully."

      if action == "servoOn":
            # SET_PARAM_2 command        
            set_param_2_command = ServoParams.SET_PARAM_2      

            # SERVO_ON Command
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SVON, 1)
            servo_on_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
  
            response['message']="Servo turned on successfully."

      elif action == "servoOff":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SVON, 0)
            servo_off_command = cmd_generator.generate_message(
                  protocol_id,
                  destination_address,
                  dir_bit, error_code, cmd_code, parameter_data)
            
            response['message'] = "Servo turned off successfully."
      
      elif action == "getMsg":
            get_state_value_command = ServoParams.GET_STATE_VALUE_4

            response['message'] = "Get Servo IO Input Status Value."

      elif action == "getIOOutput":
            
            response['message'] = "Get Servo IO Output Status Value."

      elif action == "setPoint_1":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.SEL_NO, 5)

            response['message'] = "Set the Postion in Point 1."

      elif action == "setPoint_2":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            response['message'] = "Set the Postion in Point 2."

      elif action == "Home":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            response['message'] = "Set the Postion in HOME."
            
      elif action == "motionStart":
            print("START")
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.START1, 0)
            
            parameter_data = setter.set_bit_status(BitMap.START1, 1)
            
            response['message'] = "Motion Start."


      elif action == "motionPause":
            cmd_code = CmdCode.SET_STATE_VALUE_WITHMASK_4.value
            parameter_data = setter.set_bit_status(BitMap.PAUSE, 1)
            response['message'] = "Motion Pause."
      else:
            response['error'] = "Action not recognized."

      return jsonify(response)

if __name__ == "__main__":
   gpio_utils.initialize_gpio()
   try:
         app.run(host='0.0.0.0', port=5000, debug = True)
   finally:
         gpio_utils.cleanup_gpio()
