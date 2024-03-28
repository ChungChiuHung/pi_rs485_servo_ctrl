import os
from flask import Flask, render_template, request, session, jsonify
from gpio_utils import GPIOUtils
from serial_port_manager import SerialPortManager
from servo_command_code import CmdCode
from status_bit_mapping import BitMap
from servo_serial_protocol_handler import SerialPotocolHandler
from servo_control import ServoController


app = Flask(__name__)


# Instantiate GPIOUtils for GPIO opeartions
gpio_utils = GPIOUtils()

# Configure the serial port
port_manager = SerialPortManager(baud_rate=57600)
port_manager.connect()
if port_manager.get_serial_instance():
      print(f"Connected port: {port_manager.get_connected_port()}")
      print(f"Current baud rate: {port_manager.get_baud_rate()}")
      servo_ctrller = ServoController(port_manager.get_serial_instance())
else:
      print("Could not configure any serial port. Exiting.")
      exit()



app.secret_key = os.getenv('FLASK_SECRET_KEY', 'JOJO0912956011')

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
            command_code = CmdCode.SET_PARAM_2
            command_format = SerialPotocolHandler()
            set_param_2_command = command_format.construct_packet(1,command_code, b'\x00\x09\x00\x01', is_response=False)
            print(f"{command_code.name} Command: ", set_param_2_command.hex())

            servo_ctrller.send_command_and_wait_for_response(set_param_2_command, f"{command_code.name}", 0.05)
            # SERVO_ON Command
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            command_format = SerialPotocolHandler()
            servo_on_command = command_format.construct_packet(1,command_code, b'\x01\x20\x00\x00\x00\x01\x00\x00\x00\x01', is_response=False)
            print(f"{command_code.name} Command: ", servo_on_command.hex())

            servo_ctrller.send_command_and_wait_for_response(servo_on_command, f"{command_code.name}", 0.05)
  
            response['message']="Servo turned on successfully."

      elif action == "servoOff":
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            command_format = SerialPotocolHandler()
            servo_off_command = command_format.construct_packet(1,command_code, b'\x01\x20\x00\x00\x00\x00\x00\x00\x00\x01', is_response=False)
            print(f"{command_code.name} Command: ", servo_off_command.hex())

            servo_ctrller.send_command_and_wait_for_response(servo_off_command, f"{command_code.name}", 0.05)

            
            response['message'] = "Servo turned off successfully."
      
      elif action == "getMsg":

            # Alarm
            command_code = CmdCode.GET_STATE_VALUE_4
            command_format = SerialPotocolHandler()
            get_io_alarm_state = command_format.construct_packet(1,command_code, b'\x00\x00', is_response=False)
            print(f"{command_code.name} Command: ", get_io_alarm_state.hex())

            servo_ctrller.send_command_and_wait_for_response(get_io_alarm_state, f"{command_code.name}", 0.05)

            # Logic I/O Input
            command_code = CmdCode.GET_STATE_VALUE_4
            command_format = SerialPotocolHandler()
            get_io_input_state = command_format.construct_packet(1,command_code, b'\x01\x20', is_response=False)
            print(f"{command_code.name} Command: ", get_io_input_state.hex())

            servo_ctrller.send_command_and_wait_for_response(get_io_input_state, f"{command_code.name}", 0.05)

            # Logic I/O Output
            command_code = CmdCode.GET_STATE_VALUE_4
            command_format = SerialPotocolHandler()
            get_io_output_state = command_format.construct_packet(1,command_code, b'\x01\x28', is_response=False)
            print(f"{command_code.name} Command: ", get_io_output_state.hex())

            servo_ctrller.send_command_and_wait_for_response(get_io_output_state, f"{command_code.name}", 0.05)


            response['message'] = "Get Servo IO Input Status Value."

      elif action == "getIOOutput":

            # Logic I/O Output
            command_code = CmdCode.GET_STATE_VALUE_4
            command_format = SerialPotocolHandler()
            get_io_output_state = command_format.construct_packet(1,command_code, b'\x01\x28', is_response=False)
            print(f"{command_code.name} Command: ", get_io_output_state.hex())

            servo_ctrller.send_command_and_wait_for_response(get_io_output_state, f"{command_code.name}", 0.05)
            
            response['message'] = "Get Servo IO Output Status Value."

      elif action == "setPoint_1":

            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            command_format = SerialPotocolHandler()
            set_point_1 = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 1, is_response=False)
            print(f"{command_code.name} Command:", set_point_1.hex())

            servo_ctrller.send_command_and_wait_for_response(set_point_1, f"{command_code.name}", 0.05)

            response['message'] = "Set the Postion in Point 1."

      elif action == "setPoint_2":
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            command_format = SerialPotocolHandler()
            set_point_2 = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 2, is_response=False)
            print(f"{command_code.name} Command:", set_point_1.hex())

            servo_ctrller.send_command_and_wait_for_response(set_point_2, f"{command_code.name}", 0.05)

            response['message'] = "Set the Postion in Point 2."

      elif action == "Home":
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            command_format = SerialPotocolHandler()
            set_home_position = command_format.construct_packet(1, command_code,b'', BitMap.SEL_NO, 3, is_response=False)
            print(f"{command_code.name} Command:", set_home_position.hex())

            servo_ctrller.send_command_and_wait_for_response(set_home_position, f"{command_code.name}", 0.05)

            response['message'] = "Set the Postion in HOME."
            
      elif action == "motionStart":
            print("START")
            command_code = CmdCode.SET_STATE_VALUE_WITHMASK_4
            command_format = SerialPotocolHandler()
            set_home_position = command_format.construct_packet(1, command_code,b'', BitMap.START1, 0, is_response=False)
            print(f"{command_code.name} Command:", set_home_position.hex())

            servo_ctrller.send_command_and_wait_for_response(set_home_position, f"{command_code.name}", 0.05)

            set_home_position = command_format.construct_packet(1, command_code,b'', BitMap.START1, 1, is_response=False)
            print(f"{command_code.name} Command:", set_home_position.hex())

            servo_ctrller.send_command_and_wait_for_response(set_home_position, f"{command_code.name}", 0.05)



            response['message'] = "Motion Start."


      elif action == "motionPause":
            response['message'] = "Motion Pause."

      elif action == "motionCancel":
            response['message']   
      else:
            response['error'] = "Action not recognized."

      return jsonify(response)

if __name__ == "__main__":
   gpio_utils.initialize_gpio()
   try:
         app.run(host='0.0.0.0', port=5000, debug = True)
   finally:
         gpio_utils.cleanup_gpio()
