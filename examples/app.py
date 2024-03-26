import os
from flask import Flask, render_template, request, session, jsonify
from gpio_utils import GPIOUtils
from serial_port_manager import SerialPortManager
from command_code import CmdCode
from servo_serial_protocol_handler import SerialPortHandler
from servo_control import ServoCntroller

app = Flask(__name__)
app.secret_key = os.getenv('FLASK_SECRET_KEY', 'JOJO0912956011')


# Instantiate GPIOUtils for GPIO opeartions
gpio_utils = GPIOUtils()

# Configure the serial port
port_manager = SerialPortManager(baud_rate=57600)
port_manager.connect()
if port_manager.get_serial_instance():
        print(f"Connected port: {port_manager.get_connected_port()}")
        print(f"Current baud rate: {port_manager.get_baud_rate()}")


servo_ctrller = ServoCntroller(port_manager)

if not port_manager:
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
            command_code = CmdCode.NOP
            command_format = SerialProtocolHander()
            nop_command = command_format.construct_packet(1,command_code, b'', is_response=True)
            print(f"{command_code.name} Command: ", nop_command.hex())

            servo_ctrller.send_command_and_wait_for_response(nop_command, f"{command_code.name}")
            # SERVO_ON Command
            
  
            response['message']="Servo turned on successfully."

      elif action == "servoOff":
            
            response['message'] = "Servo turned off successfully."
      
      elif action == "getMsg":

            response['message'] = "Get Servo IO Input Status Value."

      elif action == "getIOOutput":
            
            response['message'] = "Get Servo IO Output Status Value."

      elif action == "setPoint_1":

            response['message'] = "Set the Postion in Point 1."

      elif action == "setPoint_2":
            response['message'] = "Set the Postion in Point 2."

      elif action == "Home":
            response['message'] = "Set the Postion in HOME."
            
      elif action == "motionStart":
            print("START")
            response['message'] = "Motion Start."


      elif action == "motionPause":
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
